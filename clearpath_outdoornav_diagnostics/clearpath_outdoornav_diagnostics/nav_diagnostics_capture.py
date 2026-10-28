#!/usr/bin/env python3
"""Capture a diagnostics snapshot of OutdoorNav running on a robot.

On the robot, source its ROS/workspace environment, then run:

    sudo -v
    ros2 run clearpath_outdoornav_diagnostics nav_diagnostics_capture

On a remote PC, connect first (connect_robot.sh is in the API repository's middleware/
directory) and request remote service logs:

    source middleware/connect_robot.sh zenoh <robot_ip> <robot_hostname>
    ros2 run clearpath_outdoornav_diagnostics nav_diagnostics_capture \
        --remote-service-logs --remote-docker-logs

Results are written to onav_diagnostics_<timestamp>.tar.gz; the capture directory is
removed once archived (keep it with --keep-directory, or skip the archive with --no-archive).
Extract the archive and open summary.md at the capture root; supporting files are grouped
in navigation/, ros/, tf/, logs/, timeline/, environment/, parameters/ and, when
recording, bag/.
Trajectory, localization and velocity plots are always written; missing data is
annotated in the plots rather than suppressing their output.
timeline/ contains a unified live-capture event stream (JSON/CSV) and plot:
goal changes/results, localization threshold crossings, stop/collision states,
diagnostic changes and significant ROS logs. Events use capture-host receipt
time; publisher timestamps are retained separately. This is not a causal diagnosis
or a timeline of historical Docker/service logs.
environment/fingerprint/ contains robot software/image provenance and sanitized
configuration snapshots. --remote-fingerprint (also enabled by either remote-log
option) collects these on the robot over SSH; --ssh-port overrides SSH config.
Snapshots use descriptive basenames (with hash suffixes for conflicting names).
Identical sanitized contents are saved once; fingerprint.json lists every source.
Configuration snapshots preserve commands, redacting recognizable credential
keys, arguments and URLs only. Unlabeled positional credentials and raw logs or
middleware files may still contain sensitive data. Review before sharing.
Docker logs for every container are downloaded from Dozzle as a ZIP, covering all
history still retained by Docker's logging driver. For a remote capture, Dozzle
must be reachable at http://<robot_ip>:5200/_/logs or set --dozzle-url.
The latest service invocation's clearpath-platform-extras journal is read locally
by default. Use --remote-service-logs to retrieve it over SSH as robot (override
with --ssh-user/--ssh-host), or disable with --skip-service-logs.
Remote retrieval requires trusted SSH host keys, key/agent authentication and
passwordless sudo. Local retrieval requires passwordless sudo or cached credentials.
Press Ctrl-C to end the capture window early; results are still written.
"""

import argparse
import array
import bisect
import csv
import hashlib
import importlib.metadata
import inspect
import json
import math
import os
import platform
import re
import shutil
import signal
import socket
import statistics
import subprocess
import sys
import tarfile
import time
import urllib.error
import urllib.parse
import urllib.request
import zipfile
import xml.etree.ElementTree as ET
from collections import Counter
from datetime import datetime, timezone
from itertools import pairwise
from pathlib import Path

import rclpy
import yaml
from action_msgs.msg import GoalStatusArray
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lifecycle_msgs.srv import GetState
from nav_msgs.msg import Odometry, Path as NavPath
from rcl_interfaces.msg import Log
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from rcl_interfaces.srv import GetParameters, ListParameters
from rclpy.action import get_action_names_and_types
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import parameter_value_to_python
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from rclpy.time import Time
from rosidl_runtime_py.utilities import get_action, get_message
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer

PACKAGE_NAME = 'clearpath_outdoornav_diagnostics'
CONFIG_NAME = 'nav_diagnostics_config.yaml'


def config_path():
    """Locate the config in the source checkout, falling back to the installed share directory."""
    source_config = Path(__file__).resolve().parents[1] / 'config' / CONFIG_NAME
    if source_config.is_file():
        return source_config
    try:
        return Path(get_package_share_directory(PACKAGE_NAME)) / 'config' / CONFIG_NAME
    except PackageNotFoundError:
        return source_config


CONFIG_PATH = config_path()
SECRET_KEY = re.compile(
    r'password|passwd|passphrase|secret|token|credential|private.?key|api.?key|'
    r'authorization|cookie|(^|[_.-])(psk|pwd|auth)([_.-]|$)', re.IGNORECASE)
BUILD_VARIABLE = re.compile(r'^(BRANCH|COMMIT)_CPR_[A-Z0-9_]+$')


def load_config():
    config_path = CONFIG_PATH
    try:
        config = yaml.safe_load(config_path.read_text())
    except OSError as exc:
        raise RuntimeError(f'Cannot read diagnostics config {config_path}: {exc}') from exc
    except yaml.YAMLError as exc:
        raise RuntimeError(f'Invalid diagnostics config {config_path}: {exc}') from exc
    if not isinstance(config, dict):
        raise RuntimeError(f'Diagnostics config {config_path} must contain a YAML mapping')
    required = {
        'heavy_types', 'nav_nodes', 'localization_nodes', 'scan_candidates',
        'key_topics', 'localization_topics', 'tf_lookups', 'log_levels', 'diagnostic_levels',
        'diagnostic_severity', 'environment_variables', 'namespace_exclude',
        'clock_skew_limit_s', 'goal_status', 'terminal_goal_status', 'navigation_actions',
        'feedback_actions', 'result_request_timeout_s', 'velocity_chain',
        'goal_pose_match_before_s', 'goal_pose_match_after_s',
        'capture_startup_status_window_s',
        'velocity_stage_checks', 'teleop_inputs', 'stop_locks', 'measured_velocity_topic',
        'collision_state_topic', 'velocity_extractors', 'collision_actions',
        'twist_mux_timeout_s', 'zero_velocity', 'moving_velocity', 'stalled_velocity',
        'min_command_samples', 'pass_through_limit_percent', 'velocity_match_tolerance_s',
        'stalled_command_percent', 'localization_std_xy_limit_m',
        'localization_std_yaw_limit_deg', 'map_odom_jump_limit_m', 'map_odom_jump_limit_deg',
    }
    missing = sorted(required - config.keys())
    if missing:
        raise RuntimeError(f'Diagnostics config {config_path} is missing: {", ".join(missing)}')
    list_fields = {
        'heavy_types', 'nav_nodes', 'localization_nodes', 'scan_candidates',
        'tf_lookups', 'environment_variables', 'namespace_exclude', 'terminal_goal_status',
        'navigation_actions', 'feedback_actions', 'velocity_chain',
    }
    mapping_fields = {
        'key_topics', 'localization_topics', 'log_levels', 'diagnostic_levels',
        'diagnostic_severity', 'goal_status', 'velocity_stage_checks', 'teleop_inputs',
        'stop_locks', 'velocity_extractors', 'collision_actions',
    }
    invalid = sorted(
        key for key in list_fields if not isinstance(config[key], list)
    ) + sorted(key for key in mapping_fields if not isinstance(config[key], dict))
    if invalid:
        raise RuntimeError(
            f'Diagnostics config {config_path} has invalid field types: {", ".join(invalid)}')
    return config


CONFIG = load_config()
HEAVY_TYPES = set(CONFIG['heavy_types'])
NAV_NODES = CONFIG['nav_nodes']
LOCALIZATION_NODES = CONFIG['localization_nodes']
SCAN_CANDIDATES = CONFIG['scan_candidates']
KEY_TOPICS = CONFIG['key_topics']
LOCALIZATION_TOPICS = CONFIG['localization_topics']
TF_LOOKUPS = [tuple(lookup) for lookup in CONFIG['tf_lookups']]
LOG_LEVELS = CONFIG['log_levels']
DIAG_LEVELS = CONFIG['diagnostic_levels']
DIAG_SEVERITY = CONFIG['diagnostic_severity']
ENV_VARS = CONFIG['environment_variables']
NAMESPACE_EXCLUDE = set(CONFIG['namespace_exclude'])
CLOCK_SKEW_LIMIT_S = CONFIG['clock_skew_limit_s']
GOAL_STATUS = CONFIG['goal_status']
TERMINAL_GOAL_STATUS = set(CONFIG['terminal_goal_status'])
NAVIGATION_ACTIONS = tuple(CONFIG['navigation_actions'])
FEEDBACK_ACTIONS = set(CONFIG['feedback_actions'])
RESULT_REQUEST_TIMEOUT_S = CONFIG['result_request_timeout_s']
GOAL_POSE_MATCH_BEFORE_S = CONFIG['goal_pose_match_before_s']
GOAL_POSE_MATCH_AFTER_S = CONFIG['goal_pose_match_after_s']
CAPTURE_STARTUP_STATUS_WINDOW_S = CONFIG['capture_startup_status_window_s']
VELOCITY_CHAIN = [tuple(stage) for stage in CONFIG['velocity_chain']]
VELOCITY_STAGE_CHECKS = CONFIG['velocity_stage_checks']
TELEOP_INPUTS = CONFIG['teleop_inputs']
STOP_LOCKS = CONFIG['stop_locks']
MEASURED_VELOCITY_TOPIC = CONFIG['measured_velocity_topic']
COLLISION_STATE_TOPIC = CONFIG['collision_state_topic']
VELOCITY_EXTRACTORS = CONFIG['velocity_extractors']
COLLISION_ACTIONS = CONFIG['collision_actions']
TWIST_MUX_TIMEOUT_S = CONFIG['twist_mux_timeout_s']
ZERO_VELOCITY = CONFIG['zero_velocity']
MOVING_VELOCITY = CONFIG['moving_velocity']
STALLED_VELOCITY = CONFIG['stalled_velocity']
MIN_COMMAND_SAMPLES = CONFIG['min_command_samples']
PASS_THROUGH_LIMIT_PERCENT = CONFIG['pass_through_limit_percent']
VELOCITY_MATCH_TOLERANCE_S = CONFIG['velocity_match_tolerance_s']
STALLED_COMMAND_PERCENT = CONFIG['stalled_command_percent']
LOCALIZATION_STD_XY_LIMIT_M = CONFIG['localization_std_xy_limit_m']
LOCALIZATION_STD_YAW_LIMIT_DEG = CONFIG['localization_std_yaw_limit_deg']
MAP_ODOM_JUMP_LIMIT_M = CONFIG['map_odom_jump_limit_m']
MAP_ODOM_JUMP_LIMIT_DEG = CONFIG['map_odom_jump_limit_deg']


def extract_velocity(message, field_path):
    for field in field_path.split('.'):
        if field != 'self':
            message = getattr(message, field)
    return message


class ParameterDumper(yaml.SafeDumper):

    def represent_list(self, data):
        return self.represent_sequence('tag:yaml.org,2002:seq', data, flow_style=True)


ParameterDumper.add_representer(list, ParameterDumper.represent_list)


def as_int(value):
    return value if isinstance(value, int) else int.from_bytes(value, 'little')


def full_name(namespace, name):
    return f"{namespace.rstrip('/')}/{name}"


def to_plain(value):
    if isinstance(value, (array.array, list, tuple)):
        return [to_plain(v) for v in value]
    if isinstance(value, bytes):
        return list(value)
    return value


def endpoint(info):
    qos = info.qos_profile
    return {
        'node': full_name(info.node_namespace, info.node_name),
        'reliability': qos.reliability.name,
        'durability': qos.durability.name,
        'depth': qos.depth,
    }


def transform_xy(x, y, transform):
    q = transform.rotation
    tx, ty, tz = 2.0 * (q.y * 0.0 - q.z * y), 2.0 * (q.z * x - q.x * 0.0), \
        2.0 * (q.x * y - q.y * x)
    rx = x + q.w * tx + q.y * tz - q.z * ty
    ry = y + q.w * ty + q.z * tx - q.x * tz
    return rx + transform.translation.x, ry + transform.translation.y


def project_to_path(x, y, path):
    best_distance = float('inf')
    best_progress = 0.0
    distance_before = 0.0
    for start, end in pairwise(path):
        dx, dy = end[0] - start[0], end[1] - start[1]
        length_squared = dx * dx + dy * dy
        length = length_squared ** 0.5
        if length_squared == 0:
            continue
        fraction = max(0.0, min(1.0, ((x - start[0]) * dx + (y - start[1]) * dy)
                                      / length_squared))
        distance = ((x - start[0] - fraction * dx) ** 2
                    + (y - start[1] - fraction * dy) ** 2) ** 0.5
        if distance < best_distance:
            best_distance = distance
            best_progress = distance_before + fraction * length
        distance_before += length
    return best_distance, best_progress


def trajectory_statistics(path, trajectory):
    if not path or len(path) < 2:
        return {'status': 'unavailable', 'reason': 'No global path was received.'}
    if len(trajectory) < 2:
        return {'status': 'unavailable', 'reason': 'Fewer than two robot poses were captured.'}

    segment_lengths = [((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2) ** 0.5
                       for a, b in pairwise(path)]
    path_length = sum(segment_lengths)
    distances = [project_to_path(x, y, path)[0] for _, x, y in trajectory]
    progress = [project_to_path(x, y, path)[1] for _, x, y in trajectory]
    speeds = []
    for previous, current in pairwise(trajectory):
        elapsed = current[0] - previous[0]
        if elapsed > 0:
            distance = ((current[1] - previous[1]) ** 2
                        + (current[2] - previous[2]) ** 2) ** 0.5
            speeds.append(distance / elapsed)

    return {
        'status': 'ok',
        'path_frame': None,
        'path_points': len(path),
        'trajectory_samples': len(trajectory),
        'path_length_m': round(path_length, 3),
        'distance_traveled_m': round(sum(
            ((b[1] - a[1]) ** 2 + (b[2] - a[2]) ** 2) ** 0.5
            for a, b in pairwise(trajectory)), 3),
        'duration_s': round(trajectory[-1][0] - trajectory[0][0], 3),
        'cross_track_error_mean_m': round(statistics.fmean(distances), 3),
        'cross_track_error_rms_m': round(statistics.fmean(d * d for d in distances) ** 0.5, 3),
        'cross_track_error_max_m': round(max(distances), 3),
        'path_progress_percent': round(min(100.0, max(progress) / path_length * 100.0), 1)
        if path_length > 0 else None,
        'path_remaining_m': round(max(0.0, path_length - max(progress)), 3),
        'mean_speed_mps': round(statistics.fmean(speeds), 3) if speeds else None,
        'max_speed_mps': round(max(speeds), 3) if speeds else None,
    }


def save_trajectory_outputs(out_dir, path, path_frame, trajectory, errors):
    stats = trajectory_statistics(path, trajectory)
    stats['path_frame'] = path_frame
    stats['transform_errors'] = dict(errors)
    csv_path = out_dir / 'trajectory.csv'
    with csv_path.open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['series', 'elapsed_s', 'x_m', 'y_m'])
        for x, y in path or []:
            writer.writerow(['global_path', '', x, y])
        if trajectory:
            start = trajectory[0][0]
            for timestamp, x, y in trajectory:
                writer.writerow(['robot_trajectory', round(timestamp - start, 3), x, y])

    plt = pyplot()
    figure, axis = plt.subplots(figsize=(9, 7))
    if path:
        path_x, path_y = zip(*path)
        axis.plot(path_x, path_y, color='#2676b8', linewidth=2.2, label='Global path')
    if trajectory:
        trajectory_x = [point[1] for point in trajectory]
        trajectory_y = [point[2] for point in trajectory]
        axis.plot(trajectory_x, trajectory_y, color='#e66b32', linewidth=1.8,
                  label='Robot trajectory')
        axis.scatter(trajectory_x[0], trajectory_y[0], color='#27864b', marker='o',
                     s=55, label='Start', zorder=3)
        axis.scatter(trajectory_x[-1], trajectory_y[-1], color='#b32e37', marker='X',
                     s=70, label='Current position', zorder=4)
    axis.set_title('Global path and robot trajectory')
    axis.set_xlabel(f'X ({path_frame or "map"}) [m]')
    axis.set_ylabel(f'Y ({path_frame or "map"}) [m]')
    axis.set_aspect('equal', adjustable='datalim')
    axis.grid(True, alpha=0.25)
    if path or trajectory:
        axis.legend(loc='best')
    if stats['status'] == 'ok':
        axis.text(0.02, 0.98,
                  f'Progress: {stats["path_progress_percent"]}%\n'
                  f'Cross-track mean / max: {stats["cross_track_error_mean_m"]} / '
                  f'{stats["cross_track_error_max_m"]} m\n'
                  f'Distance traveled: {stats["distance_traveled_m"]} m\n'
                  f'Mean / max speed: {stats["mean_speed_mps"]} / '
                  f'{stats["max_speed_mps"]} m/s',
                  transform=axis.transAxes, va='top', fontsize=9,
                  bbox={'facecolor': 'white', 'alpha': 0.85, 'edgecolor': '#cccccc'})
    else:
        axis.text(0.02, 0.98, stats['reason'], transform=axis.transAxes, va='top')
    figure.tight_layout()
    figure.savefig(out_dir / 'trajectory_plot.png', dpi=150)
    plt.close(figure)
    stats['plot'] = 'trajectory_plot.png'
    write_json(out_dir / 'trajectory_stats.json', stats)
    return stats


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_difference(a, b):
    return math.atan2(math.sin(a - b), math.cos(a - b))


def uuid_hex(goal_id):
    return bytes(bytearray(goal_id.uuid)).hex()


def rounded(value, digits=3):
    return None if value is None else round(value, digits)


def clock_time(seconds):
    if not seconds:
        return '-'
    return datetime.fromtimestamp(seconds).strftime('%H:%M:%S.%f')[:-3]


def pyplot():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    return plt


def nav2_error_code_names():
    try:
        from nav2_msgs import action as nav2_actions
    except ImportError:
        return {}
    names = {}
    for action_name in dir(nav2_actions):
        result = getattr(getattr(nav2_actions, action_name), 'Result', None)
        if result is None:
            continue
        for constant in dir(result):
            value = getattr(result, constant)
            if constant.isupper() and type(value) is int and value != 0:
                names.setdefault(value, set()).add(f'{action_name}.{constant}')
    return {code: ' / '.join(sorted(found)) for code, found in names.items()}


def goal_results(goals, feedback, goal_poses, capture_end):
    error_names = nav2_error_code_names()
    unmatched_poses = list(goal_poses)
    rows = []
    for key, goal in sorted(goals.items(), key=lambda item: item[1]['first_seen']):
        end = goal.get('ended')
        code = goal.get('error_code')
        finished_before_capture = goal['first_status'] in TERMINAL_GOAL_STATUS
        row = {
            'action': goal['action'],
            'type': goal['type'],
            'goal_id': goal['goal_id'],
            'accepted_stamp': goal['accepted_stamp'],
            'first_seen': goal['first_seen'],
            'started_before_capture': goal['started_before_capture'],
            'finished_before_capture': finished_before_capture,
            'status': goal['status'],
            'transitions': [{'time': t, 'status': s} for t, s in goal['transitions']],
            'ended': end,
            'observed_duration_s': None if finished_before_capture
            else rounded((end or capture_end) - goal['first_seen'], 2),
            'result_status': goal.get('result_status'),
            'result_received_time': goal.get('result_received_time'),
            'error_code': code,
            'error_name': error_names.get(code) if code else None,
            'error_msg': goal.get('error_msg') or None,
            'result_error': goal.get('result_error'),
            'feedback': feedback.get(key, {}),
            'goal_pose': None,
        }
        if goal['action'].rsplit('/', 1)[-1] in NAVIGATION_ACTIONS:
            candidates = [p for p in unmatched_poses
                          if goal['first_seen'] - GOAL_POSE_MATCH_BEFORE_S
                          <= p['time'] <= goal['first_seen'] + GOAL_POSE_MATCH_AFTER_S]
            if candidates:
                row['goal_pose'] = candidates[-1]
                unmatched_poses.remove(candidates[-1])
        rows.append(row)
    return rows


def save_goal_outputs(out_dir, goals, feedback, goal_poses, monitored_actions, capture_end):
    rows = goal_results(goals, feedback, goal_poses, capture_end)
    data = {'monitored_actions': monitored_actions, 'goals': rows, 'goal_pose_messages': goal_poses}
    write_json(out_dir / 'navigation_goals.json', data)
    return data


def localization_results(localization_poses, map_odom, initial_poses, capture_end):
    result = {
        'localization_pose_messages': len(localization_poses),
        'initial_pose_events': initial_poses,
    }
    if localization_poses:
        std_xy = [sample['std_xy_m'] for sample in localization_poses]
        std_yaw = [sample['std_yaw_deg'] for sample in localization_poses]
        last = localization_poses[-1]
        result.update(
            std_xy_mean_m=rounded(statistics.fmean(std_xy)),
            std_xy_max_m=rounded(max(std_xy)),
            std_xy_last_m=rounded(std_xy[-1]),
            std_yaw_mean_deg=rounded(statistics.fmean(std_yaw), 2),
            std_yaw_max_deg=rounded(max(std_yaw), 2),
            std_yaw_last_deg=rounded(std_yaw[-1], 2),
            last_pose={'x': rounded(last['x']), 'y': rounded(last['y']),
                       'yaw_deg': rounded(math.degrees(last['yaw']), 1)},
            last_update_age_s=rounded(capture_end - last['time'], 2),
        )
    corrections = [{
        'time': current['time'],
        'translation_m': rounded(math.hypot(current['x'] - previous['x'],
                                            current['y'] - previous['y'])),
        'rotation_deg': rounded(abs(math.degrees(
            angle_difference(current['yaw'], previous['yaw']))), 2),
    } for previous, current in pairwise(map_odom)]
    result['map_odom_corrections'] = len(corrections)
    if corrections:
        result['map_odom_max_translation_m'] = max(c['translation_m'] for c in corrections)
        result['map_odom_max_rotation_deg'] = max(c['rotation_deg'] for c in corrections)
    result['map_odom_jumps'] = [
        c for c in corrections
        if c['translation_m'] > MAP_ODOM_JUMP_LIMIT_M or c['rotation_deg'] > MAP_ODOM_JUMP_LIMIT_DEG
    ]
    result['correction_history'] = corrections
    return result


def save_localization_outputs(out_dir, localization_poses, map_odom, initial_poses,
                              capture_start, capture_end):
    result = localization_results(localization_poses, map_odom, initial_poses, capture_end)
    with (out_dir / 'localization.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['elapsed_s', 'x_m', 'y_m', 'yaw_deg', 'std_xy_m', 'std_yaw_deg'])
        for sample in localization_poses:
            writer.writerow([round(sample['time'] - capture_start, 3), round(sample['x'], 4),
                             round(sample['y'], 4), round(math.degrees(sample['yaw']), 2),
                             round(sample['std_xy_m'], 4), round(sample['std_yaw_deg'], 3)])

    plt = pyplot()
    figure, (uncertainty, corrections) = plt.subplots(2, 1, figsize=(10, 7), sharex=True)
    if localization_poses:
        times = [sample['time'] - capture_start for sample in localization_poses]
        uncertainty.step(times, [s['std_xy_m'] for s in localization_poses], where='post',
                         color='#2676b8', label='Position σ (m)')
        uncertainty.axhline(LOCALIZATION_STD_XY_LIMIT_M, color='#2676b8', linestyle=':',
                            linewidth=1)
        yaw_axis = uncertainty.twinx()
        yaw_axis.step(times, [s['std_yaw_deg'] for s in localization_poses], where='post',
                      color='#e66b32', label='Yaw σ (deg)')
        yaw_axis.set_ylabel('Yaw σ [deg]')
        lines = uncertainty.get_lines()[:1] + yaw_axis.get_lines()
        uncertainty.legend(lines, [line.get_label() for line in lines], loc='upper left')
    else:
        uncertainty.text(0.5, 0.5, 'No localization poses received',
                         transform=uncertainty.transAxes, ha='center')
    uncertainty.set_ylabel('Position σ [m]')
    uncertainty.set_title('Localization pose uncertainty')
    uncertainty.grid(True, alpha=0.25)
    history = result['correction_history']
    if history:
        corrections.bar([c['time'] - capture_start for c in history],
                        [c['translation_m'] for c in history], width=0.15,
                        color='#7a5195', label='map→odom translation change')
        corrections.axhline(MAP_ODOM_JUMP_LIMIT_M, color='#b32e37', linestyle=':',
                            linewidth=1, label='Jump threshold')
        corrections.legend(loc='upper left')
    else:
        corrections.text(0.5, 0.5, 'No map-to-odom changes recorded',
                         transform=corrections.transAxes, ha='center')
    for event in initial_poses:
        for axis in (uncertainty, corrections):
            axis.axvline(event['time'] - capture_start, color='#27864b', linestyle='--',
                         linewidth=1)
    corrections.set_title('Localization corrections (green dashes: initial pose set)')
    corrections.set_xlabel('Capture time [s]')
    corrections.set_ylabel('Change [m]')
    corrections.grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(out_dir / 'localization_plot.png', dpi=150)
    plt.close(figure)
    result['plot'] = 'localization_plot.png'
    write_json(out_dir / 'localization.json', result)
    return result


def is_moving(sample, threshold=ZERO_VELOCITY):
    return any(abs(value) > threshold for value in sample[1:])


def velocity_summary(samples, window):
    moving = [sample for sample in samples if is_moving(sample)]
    summary = {
        'messages': len(samples),
        'rate_hz': rounded(len(samples) / window, 2),
        'non_zero_messages': len(moving),
        'non_zero_percent': rounded(100.0 * len(moving) / len(samples), 1) if samples else None,
    }
    if samples:
        summary.update(
            max_abs_vx=rounded(max(abs(s[1]) for s in samples)),
            max_abs_vy=rounded(max(abs(s[2]) for s in samples)),
            max_abs_wz=rounded(max(abs(s[3]) for s in samples)),
            last={'vx': rounded(samples[-1][1]), 'vy': rounded(samples[-1][2]),
                  'wz': rounded(samples[-1][3])},
        )
    if moving:
        summary['mean_abs_vx_non_zero'] = rounded(statistics.fmean(abs(s[1]) for s in moving))
        summary['last_non_zero_time'] = moving[-1][0]
    return summary


def pass_through_percent(upstream, downstream):
    commands = [s[0] for s in upstream if is_moving(s, MOVING_VELOCITY)]
    if len(commands) < MIN_COMMAND_SAMPLES:
        return None
    outputs = sorted(s[0] for s in downstream if is_moving(s))
    passed = 0
    for stamp in commands:
        index = bisect.bisect_left(outputs, stamp - VELOCITY_MATCH_TOLERANCE_S)
        if index < len(outputs) and outputs[index] <= stamp + TWIST_MUX_TIMEOUT_S:
            passed += 1
    return rounded(100.0 * passed / len(commands), 1)


def overlapping_commands(source, navigation):
    times = sorted(s[0] for s in navigation if is_moving(s))
    count = 0
    for sample in source:
        if not is_moving(sample):
            continue
        index = bisect.bisect_left(times, sample[0] - TWIST_MUX_TIMEOUT_S)
        if index < len(times) and times[index] <= sample[0] + TWIST_MUX_TIMEOUT_S:
            count += 1
    return count


def tracking_statistics(commands, measured):
    command_times = [s[0] for s in commands]
    vx_errors, wz_errors = [], []
    stalled = 0
    for stamp, vx, vy, wz in measured:
        index = bisect.bisect_right(command_times, stamp) - 1
        if index < 0 or stamp - command_times[index] > TWIST_MUX_TIMEOUT_S:
            continue
        command = commands[index]
        if not is_moving(command, MOVING_VELOCITY):
            continue
        vx_errors.append(abs(command[1] - vx))
        wz_errors.append(abs(command[3] - wz))
        if not is_moving((stamp, vx, vy, wz), STALLED_VELOCITY):
            stalled += 1
    if not vx_errors:
        return {'moving_command_samples': 0}
    return {
        'moving_command_samples': len(vx_errors),
        'vx_error_mean_mps': rounded(statistics.fmean(vx_errors)),
        'vx_error_max_mps': rounded(max(vx_errors)),
        'wz_error_mean_radps': rounded(statistics.fmean(wz_errors)),
        'wz_error_max_radps': rounded(max(wz_errors)),
        'stalled_samples': stalled,
        'stalled_percent': rounded(100.0 * stalled / len(vx_errors), 1),
    }


def active_intervals(events, capture_end):
    intervals, start = [], None
    for stamp, active in events:
        if active and start is None:
            start = stamp
        elif not active and start is not None:
            intervals.append((start, stamp))
            start = None
    if start is not None:
        intervals.append((start, capture_end))
    return intervals


def velocity_chain_results(samples, topics, locks, collision_events, window, capture_end):
    stages = []
    for index, (relative, description) in enumerate(VELOCITY_CHAIN):
        info = topics.get(relative)
        stage = {'relative': relative, 'description': description, 'present': info is not None}
        if info:
            stage.update(info)
            stage.update(velocity_summary(samples.get(relative, []), window))
        if index + 1 < len(VELOCITY_CHAIN):
            following = VELOCITY_CHAIN[index + 1][0]
            if info and topics.get(following):
                stage['passed_to_next_percent'] = pass_through_percent(
                    samples.get(relative, []), samples.get(following, []))
        stages.append(stage)

    navigation = samples.get('cmd_vel', [])
    teleop = {}
    for relative, label in TELEOP_INPUTS.items():
        if relative in topics:
            teleop[relative] = {
                'label': label, **topics[relative],
                **velocity_summary(samples.get(relative, []), window),
                'overlapping_navigation_commands': overlapping_commands(
                    samples.get(relative, []), navigation),
            }

    stop_locks = {}
    for relative, label in STOP_LOCKS.items():
        if relative not in topics:
            continue
        events = locks.get(relative, [])
        intervals = active_intervals(events, capture_end)
        stop_locks[relative] = {
            'label': label, **topics[relative],
            'messages': len(events),
            'engaged_messages': sum(1 for _, engaged in events if engaged),
            'engaged_at_end': bool(events and events[-1][1]),
            'engaged_s': rounded(sum(end - start for start, end in intervals), 2),
            'intervals': intervals,
        }

    collision = None
    if COLLISION_STATE_TOPIC in topics:
        intervals = active_intervals(
            [(e['time'], e['action'] != 'DO_NOTHING') for e in collision_events], capture_end)
        collision = {
            **topics[COLLISION_STATE_TOPIC],
            'events': collision_events,
            'counts': dict(Counter(
                f'{e["action"]} ({e["polygon"] or "unnamed"})' for e in collision_events
                if e['action'] != 'DO_NOTHING').most_common()),
            'active_s': rounded(sum(end - start for start, end in intervals), 2),
            'intervals': intervals,
        }

    measured = samples.get(MEASURED_VELOCITY_TOPIC, [])
    result = {
        'stages': stages,
        'teleop_inputs': teleop,
        'stop_locks': stop_locks,
        'collision_monitor': collision,
        'measured': ({**topics[MEASURED_VELOCITY_TOPIC], **velocity_summary(measured, window)}
                     if MEASURED_VELOCITY_TOPIC in topics else None),
    }
    if 'platform/cmd_vel' in topics and MEASURED_VELOCITY_TOPIC in topics:
        result['tracking'] = tracking_statistics(samples.get('platform/cmd_vel', []), measured)
    return result


def save_velocity_outputs(out_dir, samples, topics, locks, collision_events,
                          capture_start, capture_end):
    window = max(capture_end - capture_start, 1e-3)
    result = velocity_chain_results(samples, topics, locks, collision_events, window,
                                    capture_end)
    with (out_dir / 'velocity_chain.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(['topic', 'elapsed_s', 'vx_mps', 'vy_mps', 'wz_radps'])
        for relative, series in samples.items():
            for stamp, vx, vy, wz in series:
                writer.writerow([relative, round(stamp - capture_start, 3), round(vx, 4),
                                 round(vy, 4), round(wz, 4)])

    plotted = [relative for relative, _ in VELOCITY_CHAIN] + list(TELEOP_INPUTS) + \
        [MEASURED_VELOCITY_TOPIC]
    plt = pyplot()
    figure, (linear, angular) = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
    for relative in plotted:
        series = samples.get(relative)
        if not series:
            continue
        times = [s[0] - capture_start for s in series]
        style = {'linewidth': 2.2, 'color': 'black'} \
            if relative == MEASURED_VELOCITY_TOPIC else {'linewidth': 1.3}
        label = f'{relative} (measured)' if relative == MEASURED_VELOCITY_TOPIC else relative
        linear.step(times, [s[1] for s in series], where='post', label=label, **style)
        angular.step(times, [s[3] for s in series], where='post', label=label, **style)
    shaded = [(lock['intervals'], '#b32e37', lock['label'])
              for lock in result['stop_locks'].values()]
    if result['collision_monitor']:
        shaded.append((result['collision_monitor']['intervals'], '#f2a541',
                       'Collision monitor active'))
    for intervals, color, label in shaded:
        for number, (start, end) in enumerate(intervals):
            for axis in (linear, angular):
                axis.axvspan(start - capture_start, end - capture_start, color=color,
                             alpha=0.18, label=label if number == 0 and axis is linear
                             else None)
    linear.set_ylabel('Linear x [m/s]')
    angular.set_ylabel('Angular z [rad/s]')
    angular.set_xlabel('Capture time [s]')
    linear.set_title('Velocity command chain')
    for axis in (linear, angular):
        axis.grid(True, alpha=0.25)
    if linear.get_legend_handles_labels()[0]:
        linear.legend(loc='upper left', fontsize=8, ncol=2)
    if not any(samples.get(relative) for relative in plotted):
        for axis in (linear, angular):
            axis.text(0.5, 0.5, 'No velocity samples received',
                      transform=axis.transAxes, ha='center')
    figure.tight_layout()
    figure.savefig(out_dir / 'velocity_chain_plot.png', dpi=150)
    plt.close(figure)
    result['plot'] = 'velocity_chain_plot.png'
    write_json(out_dir / 'velocity_chain.json', result)
    return result


def timeline_events(capture_start, capture_end, goals, localization, poses, locks,
                    collision_events, diagnostics, logs):
    events = []

    def add(stamp, category, severity, source, message, evidence,
            classification='observation', source_time=None):
        if not capture_start <= stamp <= capture_end:
            return
        events.append({
            'time': stamp,
            'utc': datetime.fromtimestamp(stamp, timezone.utc).isoformat(timespec='milliseconds'),
            'elapsed_s': round(stamp - capture_start, 6),
            'category': category, 'severity': severity, 'classification': classification,
            'source': source, 'message': sanitize(message), 'evidence': evidence,
            'source_time': source_time,
        })

    for goal in goals['goals']:
        for index, transition in enumerate(goal['transitions']):
            status = transition['status']
            initial = index == 0 and goal['started_before_capture']
            severity = 'WARN' if status == 'CANCELED' else 'ERROR' if status == 'ABORTED' else 'INFO'
            message = f'Goal {goal["goal_id"]}: {status}'
            if initial:
                message += ' (initial observed state; goal predates capture)'
                severity = 'INFO'
            add(transition['time'], 'Navigation', severity, goal['action'], message,
                'navigation/navigation_goals.json')
        if goal.get('result_received_time') is not None:
            message = f'Goal {goal["goal_id"]} result: {goal.get("result_status")}'
            severity = 'INFO'
            if goal.get('error_code') or goal.get('result_status') == 'ABORTED':
                severity = 'ERROR'
            elif goal.get('result_status') == 'CANCELED':
                severity = 'WARN'
            if goal.get('finished_before_capture'):
                message += ' (goal finished before capture)'
                severity = 'INFO'
            if goal.get('error_code'):
                message += f'; error {goal["error_code"]}: {goal.get("error_msg") or ""}'
            if goal.get('result_error'):
                message += f'; retrieval error: {goal["result_error"]}'
                severity = 'WARN'
            add(goal['result_received_time'], 'Navigation', severity, goal['action'], message,
                'navigation/navigation_goals.json')

    for category, source, samples, evidence in (
        ('Navigation', 'goal_pose', goals.get('goal_pose_messages', []),
         'navigation/navigation_goals.json'),
        ('Localization', 'initialpose', localization.get('initial_pose_events', []),
         'navigation/localization.json'),
    ):
        for sample in samples:
            add(sample['time'], category, 'INFO', source,
                f'{source} message: ({sample["x"]}, {sample["y"]}), '
                f'yaw {sample["yaw_deg"]} deg in {sample["frame"]}', evidence)

    for field, limit, label in (
        ('std_xy_m', LOCALIZATION_STD_XY_LIMIT_M, 'Position uncertainty [m]'),
        ('std_yaw_deg', LOCALIZATION_STD_YAW_LIMIT_DEG, 'Yaw uncertainty [deg]'),
    ):
        previous = None
        for sample in poses:
            high = sample[field] > limit
            if high != previous and (high or previous is not None):
                message = (
                    f'{label} {"exceeds threshold" if high else "within threshold"}: '
                    f'{sample[field]:.3f} (limit {limit})')
                add(sample['time'], 'Localization', 'WARN' if high else 'INFO',
                    'localization/odom', message, 'navigation/localization.csv',
                    classification='threshold')
            previous = high
    for jump in localization['map_odom_jumps']:
        add(jump['time'], 'Localization', 'WARN', 'map -> odom',
            f'Transform jump: {jump["translation_m"]} m / {jump["rotation_deg"]} deg',
            'navigation/localization.json', classification='threshold')

    for source, samples in locks.items():
        previous = None
        for stamp, engaged in samples:
            if engaged != previous:
                add(stamp, 'Stop locks', 'WARN' if engaged else 'INFO', source,
                    f'{"Engaged" if engaged else "Released"}'
                    + (' (initial observed state)' if previous is None else ''),
                    'timeline/state_events.json')
                previous = engaged
    previous = None
    for event in collision_events:
        state = (event['action'], event['polygon'])
        if state != previous:
            add(event['time'], 'Collision monitor',
                'INFO' if event['action'] == 'DO_NOTHING' else 'WARN',
                COLLISION_STATE_TOPIC, f'{event["action"]}: {event["polygon"] or "unnamed"}',
                'timeline/state_events.json')
            previous = state
    for source, diagnostic in diagnostics.items():
        for event in diagnostic.get('transitions', []):
            add(event['time'], 'Diagnostics',
                'WARN' if event['level'] == 'STALE' else
                'INFO' if event['level'] == 'OK' else event['level'],
                source, f'{event["level"]}: {event["message"]}', 'ros/diagnostics.json',
                source_time=event['source_time'])
    for entry in logs:
        if entry['level'] in ('WARN', 'ERROR', 'FATAL'):
            add(entry['received_time'], 'ROS logs', entry['level'], entry['logger'],
                entry['msg'], 'logs/rosout.log', source_time=entry['time'])
    return sorted(events, key=lambda event: event['time'])


def save_timeline_outputs(out_dir, capture_start, capture_end, goals, localization,
                          poses, locks, collision_events, diagnostics, logs):
    events = timeline_events(capture_start, capture_end, goals, localization, poses,
                             locks, collision_events, diagnostics, logs)
    data = {
        'schema_version': 1,
        'capture_start': capture_start, 'capture_end': capture_end,
        'time_basis': 'Capture host wall-clock receipt time (UTC). Publisher timestamps '
                      'are separate source_time fields and are not used for ordering.',
        'interpretation': 'Observations and threshold crossings only; temporal proximity '
                          'does not establish causation. Initial states may predate capture.',
        'scope': 'Events observed during the capture; historical Docker/service logs '
                 'are retained separately, not parsed into this timeline.',
        'events': events, 'event_count': len(events),
        'plot': 'timeline/timeline_plot.png',
        'json': 'timeline/timeline.json', 'csv': 'timeline/timeline.csv',
    }
    write_json(out_dir / 'timeline.json', data)
    write_json(out_dir / 'state_events.json', sanitize({
        'stop_locks': locks, 'collision_monitor': collision_events,
    }))
    with (out_dir / 'timeline.csv').open('w', newline='') as stream:
        columns = ['utc', 'elapsed_s', 'category', 'severity', 'classification',
                   'source', 'message', 'evidence', 'source_time']
        writer = csv.DictWriter(stream, fieldnames=columns, extrasaction='ignore')
        writer.writeheader()
        writer.writerows(events)
    plt = pyplot()
    lanes = ['Navigation', 'Localization', 'Collision monitor', 'Stop locks',
             'Diagnostics', 'ROS logs']
    figure, axis = plt.subplots(figsize=(12, 5))
    for severity, color in (('INFO', '#2676b8'), ('WARN', '#f2a541'),
                            ('ERROR', '#b32e37'), ('FATAL', '#7a5195')):
        selected = [event for event in events if event['severity'] == severity]
        if selected:
            axis.scatter([event['elapsed_s'] for event in selected],
                         [lanes.index(event['category']) for event in selected],
                         label=severity, color=color, s=25, alpha=0.7)
    if events:
        axis.legend(loc='upper right')
    else:
        axis.text(0.5, 0.5, 'No timeline events observed during capture',
                  transform=axis.transAxes, ha='center')
    axis.set_yticks(range(len(lanes)), labels=lanes)
    axis.set_xlim(0, max(capture_end - capture_start, 0.001))
    axis.set_ylim(-0.5, len(lanes) - 0.5)
    axis.set_xlabel('Capture time [s] (receipt time)')
    axis.set_title('Observed incident timeline (not a causal diagnosis)')
    axis.grid(True, alpha=0.25)
    figure.tight_layout()
    figure.savefig(out_dir / 'timeline_plot.png', dpi=150)
    plt.close(figure)
    return data


class TopicStats:

    def __init__(self, latched):
        self.latched = latched
        self.arrivals = []
        self.total_bytes = 0

    def on_msg(self, data):
        self.arrivals.append(time.monotonic())
        self.total_bytes += len(data)

    def result(self, window):
        count = len(self.arrivals)
        out = {
            'messages': count,
            'latched': self.latched,
            'bandwidth_kBps': round(self.total_bytes / window / 1000.0, 2),
        }
        if count >= 2:
            gaps = [b - a for a, b in zip(self.arrivals, self.arrivals[1:])]
            mean = statistics.fmean(gaps)
            out.update(
                rate_hz=round(1.0 / mean, 2) if mean > 0 else None,
                min_gap_s=round(min(gaps), 4),
                max_gap_s=round(max(gaps), 4),
                gap_stddev_s=round(statistics.pstdev(gaps), 4),
            )
        return out


class Capture(Node):

    def __init__(self):
        super().__init__(f'nav_diagnostics_capture_{os.getpid()}')
        self.rates = {}
        self.rate_errors = {}
        self.logs = []
        self.diagnostics = {}
        self.tf_edges = {}
        self.tf_buffer = Buffer(cache_time=Duration(seconds=120))
        self.global_path = None
        self.global_path_frame = None
        self.trajectory = []
        self.trajectory_errors = Counter()
        self.goals = {}
        self.goal_feedback = {}
        self.goal_poses = []
        self.goal_status_seen = set()
        self.capture_start = None
        self.monitored_actions = {}
        self.result_clients = {}
        self.pending_results = []
        self.result_timer = None
        self.localization_poses = []
        self.initial_poses = []
        self.map_odom = []
        self.track_map_odom = False
        self.velocity_samples = {}
        self.velocity_topics = {}
        self.lock_events = {}
        self.collision_events = []

    def topic_info(self, graph, relative, namespace, types):
        topic = resolve(namespace, relative)
        info = graph['topics'].get(topic)
        type_name = next((t for t in info['types'] if t in types), None) if info else None
        if not type_name:
            return None
        return {
            'topic': topic,
            'type': type_name,
            'publishers': [p['node'] for p in info['publishers']],
        }

    # ---------- graph ----------
    def snapshot_graph(self):
        me = self.get_fully_qualified_name()
        nodes = [full_name(ns, n) for n, ns in self.get_node_names_and_namespaces()]
        nodes = [n for n in nodes if n != me]
        topics = {}
        for name, types in self.get_topic_names_and_types():
            topics[name] = {
                'types': types,
                'publishers': [endpoint(i) for i in self.get_publishers_info_by_topic(name)],
                'subscribers': [
                    endpoint(i) for i in self.get_subscriptions_info_by_topic(name)
                    if full_name(i.node_namespace, i.node_name) != me
                ],
            }
        return {
            'nodes': sorted(nodes),
            'duplicate_nodes': sorted(n for n, c in Counter(nodes).items() if c > 1),
            'topics': topics,
            'services': {n: t for n, t in self.get_service_names_and_types()},
            'actions': {n: t for n, t in get_action_names_and_types(self)},
        }

    def qos_for(self, topic):
        infos = self.get_publishers_info_by_topic(topic)
        reliable = bool(infos) and all(
            i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
        latched = bool(infos) and all(
            i.qos_profile.durability == DurabilityPolicy.TRANSIENT_LOCAL for i in infos)
        qos = QoSProfile(
            depth=100 if latched else 10,
            reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
            durability=(DurabilityPolicy.TRANSIENT_LOCAL if latched
                        else DurabilityPolicy.VOLATILE),
        )
        return qos, latched

    # ---------- subscriptions ----------
    def monitor_rates(self, topics, graph):
        for topic in topics:
            type_name = graph['topics'][topic]['types'][0]
            try:
                msg_type = get_message(type_name)
            except Exception as exc:  # message package not installed on this PC
                self.rate_errors[topic] = f'cannot load {type_name}: {exc}'
                continue
            qos, latched = self.qos_for(topic)
            stats = TopicStats(latched)
            self.create_subscription(msg_type, topic, stats.on_msg, qos, raw=True)
            self.rates[topic] = stats

    def monitor_logs(self, topics):
        for topic in topics:
            self.create_subscription(Log, topic, self.on_log, self.qos_for(topic)[0])

    def monitor_diagnostics(self, topics):
        for topic in topics:
            self.create_subscription(
                DiagnosticArray, topic, lambda m, t=topic: self.on_diagnostics(t, m),
                self.qos_for(topic)[0])

    def monitor_tf(self, topics):
        for topic in topics:
            static = topic.endswith('tf_static')
            self.create_subscription(
                TFMessage, topic, lambda m, s=static: self.on_tf(m, s), self.qos_for(topic)[0])

    def monitor_trajectory(self, graph, namespace):
        path_topic = f'/{namespace}/plan'
        odom_topic = f'/{namespace}/localization/odom'
        if path_topic in graph['topics'] and 'nav_msgs/msg/Path' in graph['topics'][path_topic]['types']:
            self.create_subscription(NavPath, path_topic, self.on_global_path,
                                     self.qos_for(path_topic)[0])
        if odom_topic in graph['topics'] and \
                'nav_msgs/msg/Odometry' in graph['topics'][odom_topic]['types']:
            self.create_subscription(Odometry, odom_topic, self.on_odometry,
                                     self.qos_for(odom_topic)[0])

    def monitor_navigation_goals(self, graph, namespace):
        goal_topic = f'/{namespace}/goal_pose'
        if 'geometry_msgs/msg/PoseStamped' in graph['topics'].get(goal_topic, {}).get('types', []):
            self.create_subscription(PoseStamped, goal_topic, self.on_goal_pose,
                                     QoSProfile(depth=10,
                                                reliability=ReliabilityPolicy.BEST_EFFORT))
        for action, types in sorted(graph['actions'].items()):
            if not action.startswith(f'/{namespace}/') or not types:
                continue
            action_type = types[0]
            try:
                interface = get_action(action_type)
            except Exception as exc:  # action package not installed on this PC
                self.monitored_actions[action] = f'cannot load {action_type}: {exc}'
                continue
            self.monitored_actions[action] = action_type
            status_topic = f'{action}/_action/status'
            status_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(
                GoalStatusArray, status_topic,
                lambda msg, a=action, t=action_type: self.on_goal_status(a, t, msg), status_qos)
            self.result_clients[action] = self.create_client(
                interface.Impl.GetResultService, f'{action}/_action/get_result')
            if action.rsplit('/', 1)[-1] in FEEDBACK_ACTIONS:
                self.create_subscription(
                    interface.Impl.FeedbackMessage, f'{action}/_action/feedback',
                    lambda msg, a=action: self.on_goal_feedback(a, msg), QoSProfile(depth=10))
        if self.result_clients:
            self.result_timer = self.create_timer(0.2, self.request_pending_results)

    def monitor_localization(self, graph, namespace):
        self.track_map_odom = True
        topic = f'/{namespace}/localization/odom'
        if 'nav_msgs/msg/Odometry' in graph['topics'].get(topic, {}).get('types', []):
            self.create_subscription(Odometry, topic, self.on_localization_odom,
                                     self.qos_for(topic)[0])

    def monitor_velocity_chain(self, graph, namespace):
        velocity_topics = [relative for relative, _ in VELOCITY_CHAIN] + list(TELEOP_INPUTS) + \
            [MEASURED_VELOCITY_TOPIC]
        for relative in velocity_topics:
            info = self.topic_info(graph, relative, namespace, VELOCITY_EXTRACTORS)
            if not info:
                continue
            self.velocity_topics[relative] = info
            samples = self.velocity_samples.setdefault(relative, [])
            field_path = VELOCITY_EXTRACTORS[info['type']]
            self.create_subscription(
                get_message(info['type']), info['topic'],
                lambda msg, s=samples, p=field_path:
                    self.on_velocity(s, extract_velocity(msg, p)),
                self.qos_for(info['topic'])[0])
        for relative in STOP_LOCKS:
            info = self.topic_info(graph, relative, namespace, {'std_msgs/msg/Bool'})
            if not info:
                continue
            self.velocity_topics[relative] = info
            events = self.lock_events.setdefault(relative, [])
            self.create_subscription(
                get_message(info['type']), info['topic'],
                lambda msg, e=events: e.append((time.time(), bool(msg.data))),
                self.qos_for(info['topic'])[0])
        info = self.topic_info(graph, COLLISION_STATE_TOPIC, namespace,
                               {'nav2_msgs/msg/CollisionMonitorState'})
        if info:
            self.velocity_topics[COLLISION_STATE_TOPIC] = info
            self.create_subscription(get_message(info['type']), info['topic'],
                                     self.on_collision_state, self.qos_for(info['topic'])[0])

    def stop_goal_tracking(self):
        if self.result_timer:
            self.destroy_timer(self.result_timer)
            self.result_timer = None
        for client in self.result_clients.values():
            self.destroy_client(client)
        self.result_clients.clear()
        for request in self.pending_results:
            request['goal'].setdefault('result_error', 'result request not sent before capture end')
        self.pending_results.clear()

    def on_goal_pose(self, msg):
        pose = msg.pose
        self.goal_poses.append({
            'time': time.time(),
            'frame': msg.header.frame_id,
            'x': round(pose.position.x, 3),
            'y': round(pose.position.y, 3),
            'yaw_deg': round(math.degrees(yaw_from_quaternion(pose.orientation)), 1),
        })

    def on_goal_status(self, action, action_type, msg):
        now = time.time()
        first_message = action not in self.goal_status_seen
        self.goal_status_seen.add(action)
        for status in msg.status_list:
            goal_id = uuid_hex(status.goal_info.goal_id)
            key = f'{action}|{goal_id}'
            goal = self.goals.get(key)
            name = GOAL_STATUS.get(status.status, str(status.status))
            if goal is None:
                # Latched status arrives right after subscribing; goals already finished or
                # running then started before the capture window.
                just_subscribed = self.capture_start is None or \
                    now - self.capture_start < CAPTURE_STARTUP_STATUS_WINDOW_S
                goal = self.goals[key] = {
                    'action': action,
                    'type': action_type,
                    'goal_id': goal_id,
                    'accepted_stamp': stamp_seconds(status.goal_info.stamp),
                    'first_seen': now,
                    'first_status': name,
                    'started_before_capture': name in TERMINAL_GOAL_STATUS
                    or (first_message and just_subscribed),
                    'status': None,
                    'transitions': [],
                }
                self.pending_results.append({'action': action, 'goal': goal,
                                             'goal_id': status.goal_info.goal_id,
                                             'queued': time.monotonic()})
            if goal['status'] != name:
                goal['status'] = name
                goal['transitions'].append((now, name))
                if name in TERMINAL_GOAL_STATUS:
                    goal['ended'] = now

    def request_pending_results(self):
        waiting = []
        for request in self.pending_results:
            client = self.result_clients.get(request['action'])
            if client is None:
                continue
            if not client.service_is_ready():
                if time.monotonic() - request['queued'] > RESULT_REQUEST_TIMEOUT_S:
                    request['goal']['result_error'] = 'result service unavailable'
                else:
                    waiting.append(request)
                continue
            future = client.call_async(client.srv_type.Request(goal_id=request['goal_id']))
            future.add_done_callback(lambda f, g=request['goal']: self.on_goal_result(g, f))
        self.pending_results = waiting

    def on_goal_result(self, goal, future):
        goal['result_received_time'] = time.time()
        try:
            response = future.result()
        except Exception as exc:
            goal['result_error'] = str(exc)
            return
        if response is None:
            return
        status = GOAL_STATUS.get(response.status, str(response.status))
        goal['result_status'] = status
        if status == 'UNKNOWN':
            goal['result_error'] = 'result expired on the server before it was retrieved'
            return
        goal['error_code'] = getattr(response.result, 'error_code', None)
        goal['error_msg'] = getattr(response.result, 'error_msg', None)
        if status in TERMINAL_GOAL_STATUS and goal['status'] not in TERMINAL_GOAL_STATUS:
            goal['status'] = status
            goal['transitions'].append((time.time(), status))
            goal['ended'] = time.time()

    def on_goal_feedback(self, action, msg):
        key = f'{action}|{uuid_hex(msg.goal_id)}'
        record = self.goal_feedback.setdefault(key, {'messages': 0})
        record['messages'] += 1
        for field, field_type in msg.feedback.get_fields_and_field_types().items():
            value = getattr(msg.feedback, field)
            if field_type == 'builtin_interfaces/Duration':
                value = stamp_seconds(value)
            elif isinstance(value, bool) or not isinstance(value, (int, float)):
                continue
            record[field] = round(value, 3) if isinstance(value, float) else value
            if field == 'number_of_recoveries':
                record['max_recoveries'] = max(record.get('max_recoveries', 0), value)

    def on_localization_odom(self, msg):
        pose = msg.pose.pose
        covariance = msg.pose.covariance
        self.localization_poses.append({
            'time': time.time(),
            'stamp': stamp_seconds(msg.header.stamp),
            'x': pose.position.x,
            'y': pose.position.y,
            'yaw': yaw_from_quaternion(pose.orientation),
            'std_xy_m': math.sqrt(max(covariance[0], 0.0) + max(covariance[7], 0.0)),
            'std_yaw_deg': math.degrees(math.sqrt(max(covariance[35], 0.0))),
        })

    def on_initial_pose(self, msg):
        pose = msg.pose.pose
        self.initial_poses.append({
            'time': time.time(),
            'frame': msg.header.frame_id,
            'x': round(pose.position.x, 3),
            'y': round(pose.position.y, 3),
            'yaw_deg': round(math.degrees(yaw_from_quaternion(pose.orientation)), 1),
        })

    def record_map_odom(self, transform):
        x, y = transform.translation.x, transform.translation.y
        yaw = yaw_from_quaternion(transform.rotation)
        if self.map_odom:
            last = self.map_odom[-1]
            if abs(x - last['x']) < 1e-4 and abs(y - last['y']) < 1e-4 and \
                    abs(angle_difference(yaw, last['yaw'])) < 1e-4:
                return
        self.map_odom.append({'time': time.time(), 'x': x, 'y': y, 'yaw': yaw})

    def on_velocity(self, samples, twist):
        samples.append((time.time(), twist.linear.x, twist.linear.y, twist.angular.z))

    def on_collision_state(self, msg):
        self.collision_events.append({
            'time': time.time(),
            'action': COLLISION_ACTIONS.get(msg.action_type, str(msg.action_type)),
            'polygon': msg.polygon_name,
        })

    def on_global_path(self, msg):
        points = [(pose.pose.position.x, pose.pose.position.y) for pose in msg.poses]
        if len(points) >= 2:
            self.global_path = points
            self.global_path_frame = msg.header.frame_id

    def on_odometry(self, msg):
        if self.global_path is None:
            return
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        source_frame = msg.header.frame_id
        if source_frame != self.global_path_frame:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.global_path_frame, source_frame, Time.from_msg(msg.header.stamp),
                    timeout=Duration(seconds=0.05))
                x, y = transform_xy(x, y, transform.transform)
            except Exception as exc:
                self.trajectory_errors[type(exc).__name__] += 1
                return
        self.trajectory.append((time.monotonic(), x, y))

    def on_log(self, msg):
        level = as_int(msg.level)
        self.logs.append({
            'time': msg.stamp.sec + msg.stamp.nanosec * 1e-9,
            'received_time': time.time(),
            'level': LOG_LEVELS.get(level, str(level)),
            'logger': msg.name,
            'msg': msg.msg,
            'location': f'{msg.file}:{msg.line} ({msg.function})',
        })

    def on_diagnostics(self, topic, msg):
        received_time = time.time()
        for status in msg.status:
            level = as_int(status.level)
            rec = self.diagnostics.setdefault(f'{topic} | {status.name}', {
                'hardware_id': status.hardware_id,
                'counts': Counter(),
                'worst_level': 'OK',
                'worst_message': '',
                '_worst': 0,
                'transitions': [],
            })
            name = DIAG_LEVELS.get(level, str(level))
            previous = rec.get('last')
            if previous is None or (previous['level'], previous['message']) != (name, status.message):
                rec['transitions'].append({
                    'time': received_time, 'source_time': stamp_seconds(msg.header.stamp),
                    'level': name, 'message': status.message,
                })
            rec['counts'][name] += 1
            if DIAG_SEVERITY.get(level, 3) > rec['_worst']:
                rec['_worst'] = DIAG_SEVERITY.get(level, 3)
                rec['worst_level'] = name
                rec['worst_message'] = status.message
            rec['last'] = {
                'level': name,
                'message': status.message,
                'values': {kv.key: kv.value for kv in status.values},
            }

    def on_tf(self, msg, static):
        now_ns = self.get_clock().now().nanoseconds
        for t in msg.transforms:
            if static:
                self.tf_buffer.set_transform_static(t, 'diagnostics_capture')
            else:
                self.tf_buffer.set_transform(t, 'diagnostics_capture')
            edge = self.tf_edges.setdefault(
                f'{t.header.frame_id} -> {t.child_frame_id}',
                {'static': static, 'count': 0, 'ages_s': []})
            edge['count'] += 1
            if not static:
                edge['ages_s'].append((now_ns - Time.from_msg(t.header.stamp).nanoseconds) / 1e9)
                if self.track_map_odom and t.header.frame_id.lstrip('/') == 'map' and \
                        t.child_frame_id.lstrip('/') == 'odom':
                    self.record_map_odom(t.transform)

    # ---------- services ----------
    def call(self, srv_type, name, request, timeout):
        client = self.create_client(srv_type, name)
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return None, 'service unavailable'
            future = client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
            if not future.done():
                future.cancel()
                return None, 'timeout'
            return future.result(), None
        except Exception as exc:
            return None, str(exc)
        finally:
            self.destroy_client(client)

    def lifecycle_states(self, services, timeout):
        states = {}
        for srv, types in sorted(services.items()):
            if srv.endswith('/get_state') and 'lifecycle_msgs/srv/GetState' in types:
                node = srv[:-len('/get_state')]
                res, err = self.call(GetState, srv, GetState.Request(), timeout)
                states[node] = res.current_state.label if res else f'<{err}>'
        return states

    def parameters(self, node, timeout):
        req = ListParameters.Request(depth=ListParameters.Request.DEPTH_RECURSIVE)
        res, err = self.call(ListParameters, f'{node}/list_parameters', req, timeout)
        if not res:
            return None, err
        names = sorted(res.result.names)
        res, err = self.call(GetParameters, f'{node}/get_parameters',
                             GetParameters.Request(names=names), timeout)
        if not res:
            return None, err
        return {n: to_plain(parameter_value_to_python(v)) for n, v in zip(names, res.values)}, None


def spin_for(node, seconds):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def wait_for_graph(node, prefix, timeout):
    last, stable_since = -1, time.monotonic()
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
        count = sum(1 for n, _ in node.get_topic_names_and_types() if n.startswith(prefix))
        if count != last:
            last, stable_since = count, time.monotonic()
        elif count > 0 and time.monotonic() - stable_since >= 3.0:
            break
    return max(last, 0)


def detect_namespace(topic_names):
    counts = Counter(
        parts[1] for parts in (t.split('/') for t in topic_names)
        if len(parts) > 2 and parts[1] and parts[1] not in NAMESPACE_EXCLUDE)
    return counts.most_common(1)[0][0] if counts else None


def resolve(ns, relative):
    return relative if relative.startswith('/') else f'/{ns}/{relative}'


def sanitize(value):
    if isinstance(value, dict):
        return {
            str(key): '[REDACTED]' if SECRET_KEY.search(str(key)) else sanitize(item)
            for key, item in value.items()
        }
    if isinstance(value, list):
        sanitized = []
        for item in value:
            if (isinstance(item, str) and '=' in item
                    and re.fullmatch(r'[A-Za-z_][\w.-]*', item.split('=', 1)[0])
                    and SECRET_KEY.search(item.split('=', 1)[0])):
                sanitized.append(item.split('=', 1)[0] + '=[REDACTED]')
            else:
                sanitized.append(sanitize(item))
        return sanitized
    if isinstance(value, str):
        value = re.sub(r'([a-zA-Z][a-zA-Z0-9+.-]*://)[^/\s@]+@',
                       r'\1[REDACTED]@', value)
        value = re.sub(
            r'([?&](?:password|token|key|api_key|secret|access_token)=)[^&#\s]*',
            r'\1[REDACTED]', value, flags=re.IGNORECASE)
    return value


def sanitize_command(value):
    if isinstance(value, list):
        result = []
        redact_next = False
        for item in value:
            result.append('[REDACTED]' if redact_next else sanitize_command(item))
            redact_next = (isinstance(item, str)
                           and re.fullmatch(r'--?[A-Za-z_][\w.-]*', item) is not None
                           and SECRET_KEY.search(item) is not None)
        return result
    if not isinstance(value, str):
        return sanitize(value)

    value = sanitize(value)
    result = []
    end = 0
    for match in re.finditer(
        r'(?<![\w./-])(?P<key>--?[A-Za-z_][\w.-]*|[A-Za-z_][\w.-]*(?=\s*:?\s*=))'
        r'(?P<separator>:\s*=\s*|\s*=\s*|\s+)', value,
    ):
        if match.start() < end or not SECRET_KEY.search(match['key']):
            continue
        argument = re.match(
            r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|[^\s;|&]+', value[match.end():])
        if argument is not None:
            result.extend((value[end:match.end()], '[REDACTED]'))
            end = match.end() + argument.end()
    result.append(value[end:])
    return ''.join(result)


def run_command(command, errors, timeout=30, input_text=None):
    try:
        result = subprocess.run(command, input=input_text, capture_output=True,
                                text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired) as exc:
        errors.append(sanitize(f'{command[0]} failed: {exc}'))
        return None
    if result.returncode:
        errors.append(sanitize(
            f'{command[0]} exited {result.returncode}: {result.stderr.strip()[:1000]}'))
        return None
    return result.stdout


def command_json(command, errors):
    output = run_command(command, errors)
    if output is None:
        return None
    try:
        return json.loads(output)
    except json.JSONDecodeError as exc:
        errors.append(f'{command[0]} returned invalid JSON: {exc}')
        return None


def git_revision(directory, errors):
    if not (directory / '.git').exists():
        return {'status': 'unavailable', 'reason': 'No Git metadata in deployed directory'}
    commit = run_command(['git', '-C', str(directory), 'rev-parse', 'HEAD'], errors)
    dirty = run_command(
        ['git', '-C', str(directory), 'status', '--porcelain', '--untracked-files=no'], errors)
    return {
        'commit': commit.strip() if commit is not None else None,
        'tracked_changes': bool(dirty) if dirty is not None else None,
    }


def snapshot_file(path, errors):
    try:
        with path.open('rb') as stream:
            raw = stream.read(2 * 1024 * 1024 + 1)
        if len(raw) > 2 * 1024 * 1024:
            errors.append(f'Configuration exceeds 2 MiB snapshot limit: {path}')
            return None
        data = sanitize(yaml.safe_load(raw))
        if isinstance(data, dict) and isinstance(data.get('services'), dict):
            for service in data['services'].values():
                if isinstance(service, dict):
                    for key in ('command', 'entrypoint'):
                        if key in service:
                            service[key] = sanitize_command(service[key])
        text = yaml.safe_dump(data, sort_keys=True)
    except (OSError, yaml.YAMLError, UnicodeError, ValueError) as exc:
        errors.append(f'Cannot snapshot {path}: {type(exc).__name__}')
        return None
    return {
        'source': str(path), 'content': text,
        'sanitized_sha256': hashlib.sha256(text.encode()).hexdigest(),
    }


def parameter_files(value):
    if isinstance(value, dict):
        for key, item in value.items():
            if key in ('param_file', 'params_file') and isinstance(item, str):
                yield Path(item)
            else:
                yield from parameter_files(item)
    elif isinstance(value, list):
        for item in value:
            yield from parameter_files(item)


def runtime_software():
    errors = []
    packages = {}
    roots = list(Path('/opt/ros').glob('*/share')) + list(Path('/opt').glob('*ws/install/share'))
    roots += list(Path('/opt').glob('*ws/install/*/share'))
    for root in roots:
        for manifest in sorted(root.glob('*/package.xml')):
            name = manifest.parent.name
            if not name.startswith(('rmw_', 'rcl', 'nav2_', 'onav_', 'clearpath_', 'zenoh',
                                    'cyclonedds')):
                continue
            try:
                package = ET.parse(manifest).getroot()
                packages[str(manifest)] = package.findtext('version')
            except (OSError, ET.ParseError) as exc:
                errors.append(f'Cannot read {manifest}: {type(exc).__name__}')
    if not packages:
        errors.append('No ROS package manifests found in standard /opt install locations')
    sources = {}
    for workspace in (Path('/opt/base_ws'), Path('/opt/overlay_ws')):
        for repository in sorted((workspace / 'src').glob('*')):
            if (repository / '.git').exists():
                sources[str(repository)] = git_revision(repository, errors)
    manifests = []
    for path in Path('/opt').glob('*ws/*.repos'):
        snapshot = snapshot_file(path, errors)
        if snapshot:
            manifests.append(snapshot)
    return {
        'environment': {key: value for key, value in os.environ.items()
                        if key in ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID')
                        or BUILD_VARIABLE.fullmatch(key)},
        'installed_package_versions': packages,
        'source_revisions': sources,
        'declared_dependency_manifests': manifests,
        'errors': errors,
    }


def fingerprint_probe_source():
    functions = (sanitize, sanitize_command, run_command, command_json, git_revision, snapshot_file,
                 parameter_files, runtime_software, probe_robot)
    source = (
        'import hashlib, json, os, platform, re, socket, subprocess, sys\n'
        'import xml.etree.ElementTree as ET\n'
        'from datetime import datetime, timezone\n'
        'from pathlib import Path\n'
        'import yaml\n'
        f'SECRET_KEY = re.compile({SECRET_KEY.pattern!r}, re.IGNORECASE)\n'
        f'BUILD_VARIABLE = re.compile({BUILD_VARIABLE.pattern!r})\n'
    )
    source += '\n\n'.join(inspect.getsource(function) for function in functions)
    source += (
        '\nprint(json.dumps(runtime_software() if "--runtime" in sys.argv '
        'else probe_robot(PROBE_SOURCE), default=str))\n'
    )
    return 'PROBE_SOURCE = ' + repr(source) + '\n' + source


def probe_robot(probe_source=None):
    errors = []
    data = {
        'schema_version': 1,
        'collected_at': datetime.now(timezone.utc).isoformat(),
        'robot_host': {'hostname': socket.gethostname(), 'platform': platform.platform()},
        'containers': [], 'images': [], 'configurations': [], 'deployments': [],
        'errors': errors,
    }
    configs = {Path('/etc/clearpath/robot.yaml'), Path('/opt/onav/config/outdoornav.yaml')}
    config_roots = {Path('/opt/onav/config')}
    app_roots = {Path('/opt/onav/app')} if Path('/opt/onav/app').is_dir() else set()
    ids = run_command(['docker', 'ps', '--all', '--quiet', '--no-trunc'], errors)
    if ids is not None:
        image_ids = set()
        for container_id in ids.split():
            if not re.fullmatch(r'[0-9a-f]{64}', container_id):
                errors.append('Docker returned an invalid container ID')
                continue
            inspected = command_json(['docker', 'inspect', container_id], errors)
            if not isinstance(inspected, list) or not inspected:
                errors.append(f'No inspect data for container {container_id}')
                continue
            container = inspected[0]
            config = container.get('Config') or {}
            labels = config.get('Labels') or {}
            service = labels.get('com.docker.compose.service', '')
            env = dict(item.split('=', 1) for item in config.get('Env', []) if '=' in item)
            record = {
                'name': container.get('Name', '').lstrip('/'), 'id': container_id,
                'image_id': container.get('Image'), 'image_reference': config.get('Image'),
                'service': service,
                'build_environment': {
                    key: value for key, value in env.items()
                    if key in ('ROS_DISTRO', 'RMW_IMPLEMENTATION', 'ROS_DOMAIN_ID')
                    or BUILD_VARIABLE.fullmatch(key)
                },
            }
            data['containers'].append(record)
            if container.get('Image'):
                image_ids.add(container['Image'])
            if not service.startswith('onav-'):
                continue
            working_dir = labels.get('com.docker.compose.project.working_dir')
            if working_dir:
                app_roots.add(Path(working_dir))
            for source in labels.get('com.docker.compose.project.config_files', '').split(','):
                if source:
                    configs.add(Path(source))
            for mount in container.get('Mounts', []):
                destination = mount.get('Destination')
                if destination == '/opt/onav/app':
                    app_roots.add(Path(mount['Source']))
                elif destination == '/opt/onav/config':
                    config_roots.add(Path(mount['Source']))
            if service == 'onav-autonomy' and container.get('State', {}).get('Running'):
                probe = run_command(
                    ['docker', 'exec', '-i', container_id, 'python3', '-', '--runtime'],
                    errors, timeout=60, input_text=probe_source if probe_source is not None
                    else fingerprint_probe_source())
                if probe is not None:
                    try:
                        record['runtime'] = json.loads(probe)
                        errors.extend(record['runtime'].get('errors', []))
                    except json.JSONDecodeError:
                        errors.append(f'Invalid runtime fingerprint from {container_id}')
        for image_id in sorted(image_ids):
            inspected = command_json(['docker', 'image', 'inspect', image_id], errors)
            if not isinstance(inspected, list) or not inspected:
                errors.append(f'No image metadata for {image_id}')
                continue
            image = inspected[0]
            labels = (image.get('Config') or {}).get('Labels') or {}
            data['images'].append({
                'id': image.get('Id'), 'repo_digests': image.get('RepoDigests', []),
                'created': image.get('Created'), 'architecture': image.get('Architecture'),
                'version': labels.get('org.opencontainers.image.version'),
                'revision': labels.get('org.opencontainers.image.revision'),
            })
    if not any('runtime' in container for container in data['containers']):
        errors.append('Running onav-autonomy container unavailable; collecting host ROS versions instead')
        data['host_runtime'] = runtime_software()
        errors.extend(data['host_runtime']['errors'])
    for root in sorted(config_roots):
        configs.update(root.rglob('*.yaml'))
        configs.update(root.rglob('*.yml'))
    for path in sorted(configs):
        snapshot = snapshot_file(path, errors)
        if snapshot:
            data['configurations'].append(snapshot)
    robot_configs = []
    onav_configs = []
    for snapshot in data['configurations']:
        config = yaml.safe_load(snapshot['content'])
        if not isinstance(config, dict):
            continue
        name = Path(snapshot['source']).name
        if name == 'robot.yaml':
            serial = config.get('serial_number')
            system = config.get('system')
            if not isinstance(system, dict):
                errors.append(f'Robot system configuration is not a mapping: {snapshot["source"]}')
            robot_configs.append({
                'source': snapshot['source'], 'serial_number': serial,
                'model': serial.split('-')[0] if isinstance(serial, str) else None,
                'ros2': system.get('ros2', {}) if isinstance(system, dict) else None,
                'sensors': config.get('sensors', {}),
            })
        elif name == 'outdoornav.yaml':
            onav_configs.append({'source': snapshot['source'], 'version': config.get('version')})
    data['robot_identity'] = robot_configs
    data['outdoornav_versions'] = onav_configs
    if not robot_configs:
        errors.append('Robot identity unavailable: no readable robot.yaml mapping')
    if not onav_configs:
        errors.append('OutdoorNav version unavailable: no readable outdoornav.yaml mapping')
    overrides = set()
    for snapshot in data['configurations']:
        overrides.update(parameter_files(yaml.safe_load(snapshot['content'])))
    for path in sorted(overrides):
        if not path.is_absolute():
            errors.append(f'Cannot resolve relative parameter override without runtime working directory: {path}')
            continue
        candidates = [path]
        if path.is_relative_to('/opt/onav/app'):
            candidates = [root / path.relative_to('/opt/onav/app') for root in sorted(app_roots)]
        elif path.is_relative_to('/opt/onav/config'):
            candidates = [root / path.relative_to('/opt/onav/config') for root in sorted(config_roots)]
        existing = [candidate for candidate in candidates if candidate.is_file()]
        if existing:
            candidates = existing
        if not candidates:
            errors.append(f'Cannot resolve parameter override {path} on robot host')
        for candidate in candidates:
            snapshot = snapshot_file(candidate, errors)
            if snapshot:
                data['configurations'].append(snapshot)
    for root in sorted(app_roots):
        data['deployments'].append({'path': str(root), **git_revision(root, errors)})
        model_configs = []
        for snapshot in data['configurations']:
            if Path(snapshot['source']).name == 'robot.yaml':
                robot = yaml.safe_load(snapshot['content'])
                if isinstance(robot, dict) and isinstance(robot.get('serial_number'), str):
                    model = robot['serial_number'].split('-')[0]
                    if re.fullmatch(r'[a-zA-Z0-9_]+', model):
                        model_configs.extend((root / 'autonomy' / 'params' / model).rglob('*.yaml'))
        for path in sorted(set(model_configs) | {root / 'docker-compose.yml'}):
            snapshot = snapshot_file(path, errors)
            if snapshot:
                data['configurations'].append(snapshot)
        hashes = {}
        for folder in (root / 'docker' / 'launch', root / 'autonomy' / 'launch'):
            for path in sorted(folder.glob('*.py')):
                try:
                    hashes[str(path)] = hashlib.sha256(path.read_bytes()).hexdigest()
                except OSError as exc:
                    errors.append(f'Cannot hash {path}: {type(exc).__name__}')
        data['deployments'][-1]['launch_file_sha256'] = hashes
    data['status'] = 'partial' if errors else 'ok'
    return sanitize(data)


def save_fingerprint_configurations(directory, snapshots):
    grouped = {}
    for snapshot in sorted(snapshots, key=lambda item: (item['source'], item['content'])):
        content = snapshot['content']
        sources = grouped.setdefault(content, [])
        if snapshot['source'] not in sources:
            sources.append(snapshot['source'])
    configurations = []
    used_names = set()
    for content, sources in grouped.items():
        digest = hashlib.sha256(content.encode()).hexdigest()
        stem = re.sub(r'[^a-zA-Z0-9_.-]+', '_', Path(sources[0]).stem).strip('._-')
        stem = stem[:100] or 'configuration'
        name = f'{stem}.yaml'
        if name in used_names:
            name = f'{stem}_{digest[:12]}.yaml'
            number = 2
            while name in used_names:
                name = f'{stem}_{digest[:12]}_{number}.yaml'
                number += 1
        used_names.add(name)
        (directory / name).write_text(content)
        configurations.append({
            'source': sources[0], 'sources': sources,
            'sanitized_sha256': digest, 'file': name,
        })
    return configurations


def collector_repository(script_dir):
    """Return the Git checkout containing this script, or the script directory if installed."""
    for directory in (script_dir, *script_dir.parents):
        if (directory / '.git').exists():
            return directory
    return script_dir


def collector_version():
    try:
        return importlib.metadata.version(PACKAGE_NAME)
    except importlib.metadata.PackageNotFoundError:
        return None


def collect_fingerprint(out_dir, host, user, remote=False, port=None):
    directory = out_dir / 'fingerprint'
    directory.mkdir(exist_ok=True)
    if remote:
        errors = []
        if not host:
            errors.append('Remote fingerprint requires --ssh-host or ROBOT_IP/ROBOT_HOST')
            output = None
        else:
            command = [
                'ssh', '-T', *(['-p', str(port)] if port is not None else []),
                '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
                '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=5',
                '-o', 'ServerAliveCountMax=2', '-l', user, '--', host, 'python3 -',
            ]
            output = run_command(command, errors, timeout=300,
                                 input_text=fingerprint_probe_source())
        if output is not None:
            try:
                data = json.loads(output)
            except json.JSONDecodeError:
                errors.append('Remote fingerprint returned invalid JSON')
                data = {}
            else:
                if (not isinstance(data, dict) or data.get('status') not in ('ok', 'partial')
                        or not isinstance(data.get('errors'), list)
                        or not all(isinstance(error, str) for error in data['errors'])
                        or not isinstance(data.get('configurations'), list)
                        or not all(isinstance(snapshot, dict)
                                   and isinstance(snapshot.get('content'), str)
                                   and isinstance(snapshot.get('source'), str)
                                   for snapshot in data['configurations'])):
                    errors.append('Remote fingerprint returned an invalid payload structure')
                    data = {}
        else:
            data = {}
        if errors:
            data.update(status='error', errors=errors)
    else:
        data = probe_robot()
    data['target'] = {'transport': 'SSH' if remote else 'local',
                      'host': host if remote else socket.gethostname()}
    data['configurations'] = save_fingerprint_configurations(
        directory, data.get('configurations', []))
    data['schema_version'] = 2
    script_path = Path(__file__).resolve()
    data['collector'] = {
        'git': git_revision(collector_repository(script_path.parent),
                            data.setdefault('errors', [])),
        'version': collector_version(),
        'sha256': {
            path.name: hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (script_path, CONFIG_PATH)
        },
    }
    if data.get('errors') and data.get('status') == 'ok':
        data['status'] = 'partial'
    (directory / 'fingerprint.json').write_text(
        json.dumps(data, indent=2, sort_keys=True, default=str))
    if data.get('errors'):
        print('WARNING: Software/configuration fingerprint incomplete; '
              'see environment/fingerprint/fingerprint.json', file=sys.stderr)
    return {'status': data['status'], 'errors': data.get('errors', []),
            'file': 'environment/fingerprint/fingerprint.json'}


def copy_middleware_config(out_dir):
    for var in ('ZENOH_SESSION_CONFIG_URI', 'CYCLONEDDS_URI'):
        path = os.environ.get(var, '').removeprefix('file://')
        if path and Path(path).is_file():
            shutil.copy(path, out_dir / Path(path).name)


def run_ping(out_dir):
    ip = os.environ.get('ROBOT_IP')
    if not ip or not shutil.which('ping'):
        return None
    try:
        proc = subprocess.run(['ping', '-c', '10', '-i', '0.2', '-W', '1', ip],
                              capture_output=True, text=True, timeout=20)
        output = proc.stdout + proc.stderr
    except subprocess.TimeoutExpired:
        output = 'ping timed out'
    (out_dir / 'ping.txt').write_text(output)
    summary = [line for line in output.splitlines() if 'packet loss' in line or 'rtt' in line]
    return ' | '.join(summary) or output.strip()


def collect_platform_extras_logs(out_dir, host, user, local=True, ssh_port=None):
    transport = 'local' if local else 'SSH'
    result = {
        'status': 'error',
        'host': socket.gethostname() if local else host,
        'user': None if local else user,
        'transport': transport,
        'unit': 'clearpath-platform-extras.service',
        'scope': 'latest service start',
        'log': 'clearpath-platform-extras.log',
        'stderr': 'clearpath-platform-extras.stderr.txt',
    }
    journal_command = (
        'invocation_id=$(systemctl show clearpath-platform-extras.service '
        '--property=InvocationID --value) || '
        '{ echo "Could not query service invocation ID" >&2; exit 1; }; '
        'if [ "${#invocation_id}" -ne 32 ]; then '
        'echo "No valid service invocation ID available" >&2; exit 1; fi; '
        'case "$invocation_id" in *[!0-9a-f]*|00000000000000000000000000000000) '
        'echo "No valid service invocation ID available" >&2; exit 1;; esac; '
        'sudo -n journalctl --unit=clearpath-platform-extras.service '
        '"_SYSTEMD_INVOCATION_ID=$invocation_id" --no-pager --output=short-iso'
    )
    command = ['/bin/sh', '-c', journal_command] if local else [
        'ssh', '-T', *(['-p', str(ssh_port)] if ssh_port is not None else []),
        '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
        '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=5',
        '-o', 'ServerAliveCountMax=2', '-l', user, '--', host or '', journal_command,
    ]
    with (out_dir / result['log']).open('w') as log, \
            (out_dir / result['stderr']).open('w') as errors:
        if not local and not host:
            result['error'] = 'No robot address: set ROBOT_IP/ROBOT_HOST or pass --ssh-host.'
        else:
            try:
                proc = subprocess.run(command, stdout=log, stderr=errors,
                                      stdin=subprocess.DEVNULL, timeout=60)
                result['returncode'] = proc.returncode
                if proc.returncode:
                    result['error'] = (
                        f'{transport} journal dump exited with code {proc.returncode}; '
                        f'see {result["stderr"]}. Check '
                        f'{"local journal permissions" if local else "SSH access"} '
                        'and passwordless sudo.')
                else:
                    result['status'] = 'ok'
            except subprocess.TimeoutExpired:
                result['error'] = f'{transport} journal dump timed out after 60s; logs may be partial.'
            except OSError as exc:
                result['error'] = f'Could not run {transport} journal dump: {exc}'
        if result['status'] == 'error':
            errors.write(f'\n{result["error"]}\n')

    if result['status'] == 'ok':
        output = (out_dir / result['log']).read_text()
        if not output.strip() or output.strip() == '-- No entries --':
            result['status'] = 'empty'
            result['error'] = (
                'No clearpath-platform-extras journal entries found for the latest service start.')
    if result['status'] != 'ok':
        print(f'WARNING: {result["error"]}', file=sys.stderr)
    return result


def collect_docker_logs(out_dir, url, dozzle_host, host, user, remote=False, timeout=600,
                        ssh_port=None):
    result = {
        'status': 'error',
        'transport': 'SSH' if remote else 'local',
        'dozzle_url': url,
        'dozzle_host': dozzle_host,
        'scope': 'all containers and all history retained by Docker',
        'archive': 'docker_logs.zip',
        'stderr': 'docker_logs.stderr.txt',
        'container_count': 0,
        'log_file_count': 0,
    }
    containers = []
    if remote and not host:
        result['error'] = 'No robot address: set ROBOT_IP/ROBOT_HOST or pass --ssh-host.'
        (out_dir / result['stderr']).write_text(result['error'] + '\n')
        print(f'WARNING: {result["error"]}', file=sys.stderr)
        return result

    docker_command = [
        'docker', 'ps', '--all', '--no-trunc',
        '--format', '{{.ID}}|{{.Names}}|{{.Status}}',
    ]
    command = docker_command if not remote else [
        'ssh', '-T', *(['-p', str(ssh_port)] if ssh_port is not None else []),
        '-o', 'BatchMode=yes', '-o', 'ConnectTimeout=10',
        '-o', 'StrictHostKeyChecking=yes', '-o', 'ServerAliveInterval=5',
        '-o', 'ServerAliveCountMax=2', '-l', user, '--', host,
        "docker ps --all --no-trunc --format '{{.ID}}|{{.Names}}|{{.Status}}'",
    ]
    try:
        proc = subprocess.run(command, capture_output=True, text=True,
                              stdin=subprocess.DEVNULL, timeout=60)
    except (OSError, subprocess.TimeoutExpired) as exc:
        result['error'] = f'Could not list Docker containers: {exc}'
    else:
        if proc.returncode:
            result['error'] = (
                f'Docker container listing exited with code {proc.returncode}: '
                f'{proc.stderr.strip() or "no error output"}')
        else:
            invalid_lines = []
            for line in proc.stdout.splitlines():
                fields = line.split('|', 2)
                if len(fields) != 3 or not re.fullmatch(r'[0-9a-fA-F]{12,64}', fields[0]):
                    invalid_lines.append(line)
                    continue
                container_id, name, status = fields
                containers.append({
                    'id': container_id,
                    'name': name,
                    'status': status,
                })
            result['container_count'] = len(containers)
            if invalid_lines:
                result['error'] = 'Docker returned container entries in an unexpected format.'
            elif not containers:
                result['status'] = 'empty'
                result['error'] = 'Docker reported no containers.'
            else:
                data = [
                    ('format', 'zip'),
                    ('stdout', 'true'),
                    ('stderr', 'true'),
                ]
                data.extend(
                    ('container', f'{dozzle_host}/{container["id"]}')
                    for container in containers
                )
                temporary_archive = out_dir / (result['archive'] + '.part')
                archive_path = out_dir / result['archive']
                try:
                    request = urllib.request.Request(
                        url.rstrip('/') + '/api/containers/logs',
                        data=urllib.parse.urlencode(data).encode(),
                        headers={'Content-Type': 'application/x-www-form-urlencoded'},
                        method='POST',
                    )
                    with urllib.request.urlopen(request, timeout=timeout) as response, \
                            temporary_archive.open('wb') as archive:
                        while chunk := response.read(1024 * 1024):
                            archive.write(chunk)
                    with zipfile.ZipFile(temporary_archive) as archive:
                        log_files = [name for name in archive.namelist()
                                     if not name.endswith('/')]
                        if not log_files:
                            raise RuntimeError('Dozzle returned an empty log archive.')
                    temporary_archive.replace(archive_path)
                    result['status'] = (
                        'ok' if len(log_files) == len(containers) else 'partial')
                    result['log_file_count'] = len(log_files)
                    result['log_files'] = log_files
                    if result['status'] == 'partial':
                        result['error'] = (
                            f'Dozzle returned {len(log_files)} log files for '
                            f'{len(containers)} containers; some container logs may be unavailable.')
                except urllib.error.HTTPError as exc:
                    detail = exc.read(4096).decode(errors='replace').strip()
                    result['error'] = (
                        f'Dozzle log download returned HTTP {exc.code}'
                        f'{": " + detail if detail else ""}.')
                except (urllib.error.URLError, TimeoutError, OSError,
                        zipfile.BadZipFile, RuntimeError, ValueError) as exc:
                    result['error'] = f'Could not download valid Docker logs from Dozzle: {exc}'
                finally:
                    temporary_archive.unlink(missing_ok=True)

    if result['status'] == 'error':
        (out_dir / result['stderr']).write_text(result['error'] + '\n')
    if result['status'] in ('error', 'partial', 'empty'):
        print(f'WARNING: {result["error"]}', file=sys.stderr)
    return result


def start_bag(out_dir, topics):
    if not shutil.which('ros2'):
        print('WARNING: ros2 CLI not found; skipping bag recording.', file=sys.stderr)
        return None, None
    log = open(out_dir / 'logs' / 'bag_record.log', 'w')
    proc = subprocess.Popen(
        ['ros2', 'bag', 'record', '-o', str(out_dir / 'bag'), '--topics', *topics],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    return proc, log


def stop_bag(proc, log):
    if not proc:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=20)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait()
    log.close()


def write_json(path, data):
    path.write_text(json.dumps(data, indent=2, sort_keys=True, default=str))


def summarize_goals(goals, add):
    lines = ['', '## Navigation goals', '']
    if goals is None:
        return lines + ['- Not captured.']
    actions = goals['monitored_actions']
    lines.append(f'- Monitored {len(actions)} action servers; details in '
                 '[navigation_goals.json](navigation/navigation_goals.json)')
    unavailable = {a: t for a, t in actions.items() if t.startswith('cannot load')}
    for action, reason in unavailable.items():
        add('INFO', f'Action `{action}` not monitored: {reason}')
    if not goals['goals']:
        lines.append('- No goals were active or completed during the capture window.')
    else:
        lines += ['',
                  '| action | goal | first seen | status | observed (s) | recoveries | '
                  'distance left (m) | goal pose | error |',
                  '|---|---|---|---|---:|---:|---:|---|---|']
    sub_action_failures = Counter()
    for goal in goals['goals']:
        action = goal['action'].rsplit('/', 1)[-1]
        feedback = goal['feedback']
        recoveries = feedback.get('max_recoveries', feedback.get('number_of_recoveries', '-'))
        remaining = feedback.get('distance_remaining', feedback.get('distance_to_goal', '-'))
        error = goal['error_name'] or (f'code {goal["error_code"]}' if goal['error_code'] else '')
        if goal['error_msg']:
            error = f'{error}: {goal["error_msg"]}' if error else goal['error_msg']
        if goal['result_error'] and not error:
            error = f'_{goal["result_error"]}_'
        error = error.replace('|', '\\|').replace('\n', ' ')
        pose = goal['goal_pose']
        pose_text = f'({pose["x"]}, {pose["y"]}, {pose["yaw_deg"]}°)' if pose else '-'
        seen = clock_time(goal['first_seen'])
        if goal['finished_before_capture']:
            seen += ' (finished before capture)'
        elif goal['started_before_capture']:
            seen += ' (started before capture)'
        status = goal['status'] if goal['status'] in TERMINAL_GOAL_STATUS \
            else f'{goal["status"]} (in progress at end)'
        duration = '-' if goal['observed_duration_s'] is None else goal['observed_duration_s']
        lines.append(f'| {action} | `{goal["goal_id"][:8]}` | {seen} | {status} | '
                     f'{duration} | {recoveries} | {remaining} | '
                     f'{pose_text} | {error or "-"} |')

        label = f'`{action}` goal `{goal["goal_id"][:8]}`'
        if action in NAVIGATION_ACTIONS:
            if goal['status'] == 'ABORTED':
                add('ERROR', f'{label} ABORTED{": " + error if error else ""}')
            elif goal['status'] == 'CANCELED':
                add('INFO', f'{label} was canceled')
            if isinstance(recoveries, int) and recoveries > 0:
                add('WARN', f'{label} needed {recoveries} recovery behavior(s)')
        elif goal['status'] == 'ABORTED':
            sub_action_failures[(action, error or 'no error details')] += 1
    for (action, error), count in sorted(sub_action_failures.items()):
        add('INFO', f'`{action}` aborted {count} time(s) ({error}); navigation may have '
                    'recovered by replanning')

    if goals['goal_pose_messages']:
        lines += ['', 'Goal poses received on `goal_pose` (RViz/Foxglove):', '']
        lines += [f'- {clock_time(p["time"])}: ({p["x"]}, {p["y"]}, {p["yaw_deg"]}°) '
                  f'in `{p["frame"]}`' for p in goals['goal_pose_messages']]
    return lines


def summarize_localization(localization, add):
    lines = ['', '## Localization quality', '']
    if localization is None:
        return lines + ['- Localization data unavailable.']
    if localization.get('plot'):
        lines.append(f'- Plot: [{localization["plot"]}](navigation/{localization["plot"]})')
    if not localization['localization_pose_messages']:
        add('WARN', 'No odometry pose received from `localization/odom`')
        lines.append('- No pose messages received from `localization/odom`.')
    else:
        pose = localization['last_pose']
        lines += [
            f'- `localization/odom` messages: {localization["localization_pose_messages"]}; '
            f'last received {localization["last_update_age_s"]} s before capture end',
            f'- Last pose: ({pose["x"]}, {pose["y"]}, {pose["yaw_deg"]}°)',
            (f'- Position σ mean / max / last: {localization["std_xy_mean_m"]} / '
             f'{localization["std_xy_max_m"]} / {localization["std_xy_last_m"]} m'),
            (f'- Yaw σ mean / max / last: {localization["std_yaw_mean_deg"]} / '
             f'{localization["std_yaw_max_deg"]} / {localization["std_yaw_last_deg"]}°'),
        ]
        if localization['std_xy_last_m'] > LOCALIZATION_STD_XY_LIMIT_M:
            add('WARN', f'Localization position uncertainty is high at capture end '
                        f'(σ {localization["std_xy_last_m"]} m > '
                        f'{LOCALIZATION_STD_XY_LIMIT_M} m); robot may be mislocalized')
        elif localization['std_xy_max_m'] > LOCALIZATION_STD_XY_LIMIT_M:
            add('WARN', f'Localization position uncertainty peaked at {localization["std_xy_max_m"]} m '
                        f'(> {LOCALIZATION_STD_XY_LIMIT_M} m) during the capture')
        if localization['std_yaw_max_deg'] > LOCALIZATION_STD_YAW_LIMIT_DEG:
            add('WARN', f'Localization yaw uncertainty peaked at {localization["std_yaw_max_deg"]}° '
                        f'(> {LOCALIZATION_STD_YAW_LIMIT_DEG}°)')

    corrections = localization['map_odom_corrections']
    lines.append(f'- map→odom corrections: {corrections}'
                 + (f'; largest {localization["map_odom_max_translation_m"]} m / '
                    f'{localization["map_odom_max_rotation_deg"]}°' if corrections else ''))
    jumps = localization['map_odom_jumps']
    if jumps:
        add('WARN', f'{len(jumps)} map→odom jump(s) larger than {MAP_ODOM_JUMP_LIMIT_M} m or '
                    f'{MAP_ODOM_JUMP_LIMIT_DEG}° (localization snapped to a new pose)')
        lines += [f'  - {clock_time(j["time"])}: {j["translation_m"]} m, {j["rotation_deg"]}°'
                  for j in jumps]
    for event in localization['initial_pose_events']:
        add('INFO', f'Initial pose set to ({event["x"]}, {event["y"]}, {event["yaw_deg"]}°) at '
                    f'{clock_time(event["time"])}')
        lines.append(f'- Initial pose set at {clock_time(event["time"])}: '
                     f'({event["x"]}, {event["y"]}, {event["yaw_deg"]}°)')
    return lines


def summarize_velocity(velocity, add):
    lines = ['', '## Velocity command chain', '']
    if velocity is None:
        return lines + ['- Not captured.']
    if velocity.get('plot'):
        lines.append(f'- Plot: [{velocity["plot"]}](navigation/{velocity["plot"]}); details in '
                     '[velocity_chain.json](navigation/velocity_chain.json)')
    lines += ['',
              '| stage | topic | publishers | msgs | rate (Hz) | non-zero % | max \\|vx\\| | '
              'max \\|wz\\| | reached next stage % |',
              '|---|---|---|---:|---:|---:|---:|---:|---:|']
    stages = velocity['stages']
    for index, stage in enumerate(stages):
        if not stage['present']:
            lines.append(f'| {stage["description"]} | `{stage["relative"]}` | missing | '
                         '- | - | - | - | - | - |')
            continue
        publishers = ', '.join(f'`{p}`' for p in stage['publishers']) or '-'
        passed = stage.get('passed_to_next_percent')
        lines.append(
            f'| {stage["description"]} | `{stage["topic"]}` | {publishers} | '
            f'{stage["messages"]} | {stage["rate_hz"]} | '
            f'{"-" if stage["non_zero_percent"] is None else stage["non_zero_percent"]} | '
            f'{stage.get("max_abs_vx", "-")} | {stage.get("max_abs_wz", "-")} | '
            f'{"-" if passed is None else passed} |')
        if index + 1 >= len(stages):
            continue
        following = stages[index + 1]
        if not following['present']:
            add('WARN', f'Velocity stage `{following["relative"]}` ({following["description"]}) '
                        f'does not exist; commands on `{stage["topic"]}` cannot reach the robot')
        elif passed is not None and passed < PASS_THROUGH_LIMIT_PERCENT:
            add('WARN', f'Only {passed}% of moving commands on `{stage["topic"]}` reached '
                        f'`{following["topic"]}`; check '
                        f'{VELOCITY_STAGE_CHECKS[following["relative"]]}')

    measured = velocity['measured']
    if measured:
        lines.append(f'- Measured (`{measured["topic"]}`): {measured["messages"]} msgs, '
                     f'max |vx| {measured.get("max_abs_vx", "-")} m/s, '
                     f'max |wz| {measured.get("max_abs_wz", "-")} rad/s')
    tracking = velocity.get('tracking')
    if tracking and tracking['moving_command_samples']:
        lines.append(
            f'- Command tracking (`platform/cmd_vel` vs measured, '
            f'{tracking["moving_command_samples"]} samples): vx error mean / max '
            f'{tracking["vx_error_mean_mps"]} / {tracking["vx_error_max_mps"]} m/s, '
            f'wz error mean / max {tracking["wz_error_mean_radps"]} / '
            f'{tracking["wz_error_max_radps"]} rad/s; robot stationary in '
            f'{tracking["stalled_percent"]}% of moving-command samples')
        if tracking['moving_command_samples'] >= MIN_COMMAND_SAMPLES and \
                tracking['stalled_percent'] >= STALLED_COMMAND_PERCENT:
            add('WARN', f'Robot was commanded to move but wheel odometry showed no motion in '
                        f'{tracking["stalled_percent"]}% of samples (check e-stop, motor '
                        'faults and the platform controller)')

    for lock in velocity['stop_locks'].values():
        state = 'ENGAGED at capture end' if lock['engaged_at_end'] else 'released at capture end'
        lines.append(f'- {lock["label"]} (`{lock["topic"]}`): engaged {lock["engaged_s"]} s, '
                     f'{state} ({lock["messages"]} msgs)')
        if lock['engaged_messages']:
            add('WARN', f'{lock["label"]} (`{lock["topic"]}`) engaged for {lock["engaged_s"]} s; '
                        'twist_mux blocks all motion while it is engaged')
    for teleop in velocity['teleop_inputs'].values():
        if not teleop['non_zero_messages']:
            continue
        overlap = teleop['overlapping_navigation_commands']
        lines.append(f'- {teleop["label"]} (`{teleop["topic"]}`): '
                     f'{teleop["non_zero_messages"]} non-zero commands, {overlap} while '
                     'navigation was commanding motion')
        if overlap:
            add('WARN', f'{teleop["label"]} overrode navigation commands {overlap} time(s) '
                        '(higher twist_mux priority)')
        else:
            add('INFO', f'{teleop["label"]} sent {teleop["non_zero_messages"]} non-zero commands')
    collision = velocity['collision_monitor']
    if collision:
        if collision['counts']:
            lines.append(f'- Collision monitor active {collision["active_s"]} s: ' + ', '.join(
                f'{name} ×{count}' for name, count in collision['counts'].items()))
            add('WARN', f'Collision monitor limited or stopped motion for '
                        f'{collision["active_s"]} s ('
                        + ', '.join(collision['counts']) + ')')
        else:
            lines.append('- Collision monitor: no STOP/SLOWDOWN/APPROACH/LIMIT events.')
    return lines


def build_summary(ctx):
    ns = ctx['namespace']
    graph = ctx['graph']
    rates = ctx['rates']
    findings = []

    def add(level, text):
        findings.append((level, text))

    env = ctx['environment']['env']
    if not env.get('RMW_IMPLEMENTATION'):
        add('WARN', 'RMW_IMPLEMENTATION is not set; was connect_robot.sh sourced?')

    fingerprint = ctx.get('fingerprint')
    if fingerprint and fingerprint['status'] in ('partial', 'error'):
        add('WARN', 'Software/configuration fingerprint incomplete; see fingerprint report')

    service_logs = ctx.get('service_logs')
    if service_logs and service_logs['status'] in ('error', 'empty'):
        add('WARN', f'clearpath-platform-extras logs: {service_logs["error"]}')

    docker_logs = ctx.get('docker_logs')
    if docker_logs and docker_logs['status'] in ('error', 'empty', 'partial'):
        add('WARN', f'Docker container logs: {docker_logs["error"]}')

    for node in graph['duplicate_nodes']:
        add('ERROR', f'Duplicate node name `{node}` (two instances running?)')

    expected = NAV_NODES + LOCALIZATION_NODES
    for node in expected:
        if f'/{ns}/{node}' not in graph['nodes']:
            add('ERROR', f'Expected node `/{ns}/{node}` is not running')

    for node, state in ctx['lifecycle'].items():
        if state != 'active':
            add('ERROR', f'Lifecycle node `{node}` is `{state}`, expected `active`')

    key_topics = dict(KEY_TOPICS, **LOCALIZATION_TOPICS)
    scan = next((s for s in SCAN_CANDIDATES if f'/{ns}/{s}' in graph['topics']), None)
    if scan:
        key_topics[scan] = False
    else:
        add('ERROR', f'No filtered scan topic found (looked for {", ".join(SCAN_CANDIDATES)})')

    topic_rows = []
    for rel, event_driven in key_topics.items():
        topic = resolve(ns, rel)
        info = graph['topics'].get(topic)
        if not info:
            add('ERROR', f'Key topic `{topic}` does not exist')
            topic_rows.append((topic, '-', '-', '-', '-', '-', 'missing'))
            continue
        stats = rates.get(topic, {})
        publishers = info['publishers']
        subscribers = info['subscribers']
        publisher_names = ', '.join(f'`{endpoint["node"]}`' for endpoint in publishers) or '-'
        subscriber_names = ', '.join(f'`{endpoint["node"]}`' for endpoint in subscribers) or '-'
        pubs, subs = len(publishers), len(subscribers)
        msgs = stats.get('messages')
        if pubs == 0:
            add('ERROR', f'Key topic `{topic}` has no publishers')
        elif msgs == 0 and not event_driven:
            add('ERROR', f'Key topic `{topic}` has {pubs} publisher(s) but no data arrived')
        if topic in ctx['rate_errors']:
            add('INFO', f'`{topic}`: {ctx["rate_errors"][topic]}')
        topic_rows.append((topic, pubs, publisher_names, subs, subscriber_names,
                   stats.get('rate_hz', '-'), '-' if msgs is None else msgs))
    if scan and not graph['topics'][f'/{ns}/{scan}']['subscribers']:
        add('WARN', f'Scan topic `/{ns}/{scan}` has no subscribers (costmaps not consuming it)')

    median_age = ctx['tf']['median_age_s']
    if median_age is not None:
        if median_age < -CLOCK_SKEW_LIMIT_S or median_age > CLOCK_SKEW_LIMIT_S:
            add('WARN', f'Median TF age is {median_age:.3f}s; PC and robot clocks may be out of '
                        'sync (check chrony/NTP) or the link is very slow')
    for lookup, res in ctx['tf']['lookups'].items():
        if not res['ok']:
            add('ERROR', f'TF lookup {lookup} failed: {res["error"]}')

    for key, rec in ctx['diagnostics'].items():
        if rec['worst_level'] in ('ERROR', 'STALE'):
            add('ERROR' if rec['worst_level'] == 'ERROR' else 'WARN',
                f'Diagnostic `{key}` reached {rec["worst_level"]}: {rec["worst_message"]}')

    log_counts = {}
    for entry in ctx['logs']:
        if entry['level'] in ('WARN', 'ERROR', 'FATAL'):
            log_counts.setdefault(entry['logger'], Counter())[entry['level']] += 1
    for logger, counts in sorted(log_counts.items()):
        if counts['ERROR'] or counts['FATAL']:
            add('ERROR', f'Logger `{logger}` emitted {counts["ERROR"]} ERROR / '
                         f'{counts["FATAL"]} FATAL messages')

    goal_lines = summarize_goals(ctx.get('goals'), add)
    localization_lines = summarize_localization(ctx.get('localization'), add)
    velocity_lines = summarize_velocity(ctx.get('velocity'), add)

    env_info = ctx['environment']
    lines = [
        '# OutdoorNav diagnostics capture',
        '',
        f'- Captured: {env_info["captured_at"]} from `{env_info["host"]}`',
        f'- Robot: `{ns}` @ {env.get("ROBOT_HOST") or "?"} ({env.get("ROBOT_IP") or "?"}), '
        f'domain {env.get("ROS_DOMAIN_ID") or 0}',
        f'- Middleware: {env.get("ROBOT_MIDDLEWARE") or "?"} ({env.get("RMW_IMPLEMENTATION") or "default"})',
        f'- Capture window: {ctx["window"]:.1f}s, {len(graph["nodes"])} nodes, '
        f'{len(graph["topics"])} topics',
        f'- Ping: {env_info.get("ping") or "n/a"}',
        '',
        '## Findings',
        '',
    ]
    order = {'ERROR': 0, 'WARN': 1, 'INFO': 2}
    lines += [f'- **{lvl}** {text}' for lvl, text in sorted(findings, key=lambda f: order[f[0]])]
    if not findings:
        lines.append('- No problems detected.')

    if fingerprint:
        lines += ['', '## Software and configuration fingerprint', '',
                  f'- Status: {fingerprint["status"]}.']
        if fingerprint.get('file'):
            lines.append(f'- [Robot software, images and configuration]({fingerprint["file"]})')
        lines.append('- Robot evidence is separate from the capture-PC environment. '
                     'Configuration hashes describe sanitized snapshots; '
                     'dependency manifests describe declared, not resolved, versions.')

    timeline = ctx.get('timeline')
    if timeline:
        lines += [
            '', '## Unified incident timeline', '',
            f'- {timeline["event_count"]} observed events; '
            f'[plot]({timeline["plot"]}), [JSON]({timeline["json"]}), [CSV]({timeline["csv"]}).',
            '- Ordered by capture-host receipt time, not robot timestamps. '
            'Historical Docker/service logs are not included in the event stream.',
            '- Threshold crossings are labeled separately from direct observations. '
            'Temporal proximity does not establish causation.',
        ]
        notable = [event for event in timeline['events']
                   if event['severity'] in ('WARN', 'ERROR', 'FATAL')]
        if notable:
            lines += ['', '| elapsed (s) | severity | source | event |',
                      '|---:|---|---|---|']
            for event in notable[:20]:
                message = event['message'].replace('|', r'\|').replace('\n', ' ')
                source = event['source'].replace('|', r'\|').replace('\n', ' ')
                lines.append(
                    f'| {event["elapsed_s"]:.3f} | {event["severity"]} | {source} | '
                    f'[{message}]({event["evidence"]}) |')
            if len(notable) > 20:
                lines.append(f'- {len(notable) - 20} additional significant events in the timeline.')

    if service_logs:
        lines += ['', '## clearpath-platform-extras service logs', '']
        if service_logs['status'] == 'skipped':
            lines.append('- Skipped with `--skip-service-logs`.')
        else:
            lines += [
                f'- Status: {service_logs["status"]}; scope: {service_logs["scope"]}.',
                f'- Journal: [{service_logs["log"]}](logs/{service_logs["log"]})',
                f'- Journal retrieval stderr: [{service_logs["stderr"]}](logs/{service_logs["stderr"]})',
            ]
            if service_logs.get('error'):
                lines.append(f'- {service_logs["error"]}')

    if docker_logs:
        lines += ['', '## Docker container logs', '']
        if docker_logs['status'] == 'skipped':
            lines.append('- Skipped with `--skip-docker-logs`.')
        else:
            lines.append(
                f'- Status: {docker_logs["status"]}; '
                f'{docker_logs["log_file_count"]} log files for '
                f'{docker_logs["container_count"]} containers.')
            if docker_logs['status'] in ('ok', 'partial') and docker_logs.get('archive'):
                lines.append(f'- Dozzle archive: [{docker_logs["archive"]}]'
                             f'(logs/{docker_logs["archive"]})')
            lines.append(
                f'- Scope: all history retained by Docker; Dozzle does not keep logs '
                f'beyond the Docker logging driver retention policy.')
            if docker_logs.get('error'):
                lines.append(f'- {docker_logs["error"]}')

    lines += ['', '## Lifecycle nodes', '', '| node | state |', '|---|---|']
    lines += [f'| {n} | {s} |' for n, s in ctx['lifecycle'].items()]

    lines += ['', '## Key topics', '',
              '| topic | pub count | publisher nodes | sub count | subscriber nodes | rate (Hz) | msgs |',
              '|---|---:|---|---:|---|---:|---:|']
    lines += [f'| {t} | {pc} | {pn} | {sc} | {sn} | {r} | {m} |'
              for t, pc, pn, sc, sn, r, m in topic_rows]

    lines += goal_lines
    lines += localization_lines

    trajectory = ctx.get('trajectory')
    if trajectory:
        lines += ['', '## Path-following performance', '']
        lines.append('- Plot: [trajectory_plot.png](navigation/trajectory_plot.png)')
        if trajectory['status'] == 'ok':
            lines += [
                f'- Path frame: `{trajectory["path_frame"] or "unknown"}`',
                (f'- Path length: {trajectory["path_length_m"]} m; '
                 f'distance traveled: {trajectory["distance_traveled_m"]} m'),
                (f'- Progress: {trajectory["path_progress_percent"]}%; '
                 f'remaining: {trajectory["path_remaining_m"]} m'),
                (f'- Cross-track error mean / RMS / max: '
                 f'{trajectory["cross_track_error_mean_m"]} / '
                 f'{trajectory["cross_track_error_rms_m"]} / '
                 f'{trajectory["cross_track_error_max_m"]} m'),
                (f'- Speed mean / max: {trajectory["mean_speed_mps"]} / '
                 f'{trajectory["max_speed_mps"]} m/s'),
                f'- Trajectory points: {trajectory["trajectory_samples"]}',
            ]
        else:
            lines.append(f'- Not available: {trajectory["reason"]}')
        if trajectory.get('transform_errors'):
            lines.append(f'- Pose transform failures: {trajectory["transform_errors"]}')

    lines += velocity_lines

    lines += ['', '## TF lookups', '', '| lookup | result | age (s) |', '|---|---|---|']
    for lookup, res in ctx['tf']['lookups'].items():
        lines.append(f'| {lookup} | {"ok" if res["ok"] else res["error"]} | '
                     f'{res.get("age_s", "-")} |')

    non_ok = {k: v for k, v in ctx['diagnostics'].items() if v['worst_level'] != 'OK'}
    lines += ['', '## Diagnostics (worst level not OK)', '']
    lines += [f'- `{k}`: {v["worst_level"]} - {v["worst_message"]} (last: {v["last"]["level"]})'
              for k, v in sorted(non_ok.items())] or ['- All diagnostic statuses OK.']

    lines += ['', '## rosout warnings/errors', '', '| logger | WARN | ERROR | FATAL |',
              '|---|---|---|---|']
    lines += [f'| {lg} | {c["WARN"]} | {c["ERROR"]} | {c["FATAL"]} |'
              for lg, c in sorted(log_counts.items())]
    return '\n'.join(lines) + '\n', findings


def main():
    parser = argparse.ArgumentParser(
        description='Capture diagnostics from OutdoorNav locally or remotely. '
                    'On a remote PC, source middleware/connect_robot.sh first and use '
                    '--remote-service-logs for the robot journal.')
    parser.add_argument('--namespace', default=os.environ.get('ROBOT_NAMESPACE'),
                        help='Robot namespace (default: $ROBOT_NAMESPACE or auto-detect)')
    parser.add_argument('--duration', type=float, default=60.0,
                        help='Seconds to monitor topics, diagnostics and logs (default: 60)')
    parser.add_argument('--discovery-timeout', type=float, default=20.0,
                        help='Max seconds to wait for the ROS graph (default: 20)')
    parser.add_argument('--service-timeout', type=float, default=5.0,
                        help='Timeout per service call (default: 5)')
    parser.add_argument('--output', type=Path, help='Output directory')
    parser.add_argument('--topics', nargs='*', default=[],
                        help='Extra topics to rate-monitor (relative to the namespace or absolute)')
    parser.add_argument('--include-heavy', action='store_true',
                        help='Also monitor point clouds and images (uses a lot of bandwidth)')
    parser.add_argument('--skip-params', action='store_true', help='Do not dump node parameters')
    parser.add_argument('--ssh-host', default=os.environ.get('ROBOT_IP') or
                        os.environ.get('ROBOT_HOST'),
                        help='SSH host for logs/fingerprint (default: $ROBOT_IP or $ROBOT_HOST)')
    parser.add_argument('--ssh-user', default='robot',
                        help='SSH user for logs/fingerprint (default: robot)')
    parser.add_argument('--ssh-port', type=int,
                        help='SSH port for logs and fingerprint (default: SSH configuration)')
    fingerprint_options = parser.add_mutually_exclusive_group()
    fingerprint_options.add_argument('--remote-fingerprint', action='store_true',
                                     help='Collect robot software/configuration over SSH '
                                          '(automatic with remote log options)')
    fingerprint_options.add_argument('--skip-fingerprint', action='store_true',
                                     help='Do not collect software/configuration fingerprint')
    docker_log_options = parser.add_mutually_exclusive_group()
    docker_log_options.add_argument('--remote-docker-logs', action='store_true',
                                    help='List robot containers over SSH and download their logs')
    docker_log_options.add_argument('--skip-docker-logs', action='store_true',
                                    help='Do not download Docker container logs from Dozzle')
    parser.add_argument('--dozzle-url', default=os.environ.get('DOZZLE_URL'),
                        help='Dozzle base URL (default: local port 5200, or robot port 5200 '
                             'with --remote-docker-logs)')
    parser.add_argument('--dozzle-host', default=os.environ.get('DOZZLE_HOSTNAME', 'localhost'),
                        help='Dozzle host label for container IDs (default: localhost)')
    parser.add_argument('--docker-logs-timeout', type=float, default=600.0,
                        help='Timeout in seconds for the Dozzle archive download (default: 600)')
    service_log_options = parser.add_mutually_exclusive_group()
    service_log_options.add_argument('--remote-service-logs', action='store_true',
                                     help='Retrieve the robot journal over SSH (default: local journal)')
    service_log_options.add_argument('--skip-service-logs', action='store_true',
                                     help='Do not retrieve clearpath-platform-extras journal')
    parser.add_argument('--bag', action='store_true',
                        help='Also record the monitored topics with ros2 bag')
    parser.add_argument('--no-archive', action='store_true',
                        help='Do not create a .tar.gz; keep the capture directory instead')
    parser.add_argument('--keep-directory', action='store_true',
                        help='Keep the capture directory next to the .tar.gz')
    args = parser.parse_args()
    if args.ssh_port is not None and not 1 <= args.ssh_port <= 65535:
        parser.error('--ssh-port must be between 1 and 65535')

    if not os.environ.get('RMW_IMPLEMENTATION'):
        print('WARNING: RMW_IMPLEMENTATION is not set. Source middleware/connect_robot.sh '
              'first so this PC can see the robot.', file=sys.stderr)

    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Capture()
    try:
        prefix = f'/{args.namespace}/' if args.namespace else '/'
        print(f'Waiting for ROS graph (up to {args.discovery_timeout:.0f}s)...')
        count = wait_for_graph(node, prefix, args.discovery_timeout)
        ns = args.namespace or detect_namespace(n for n, _ in node.get_topic_names_and_types())
        if not ns or count == 0:
            print(f'ERROR: no topics found under {prefix}. Is the robot reachable and running '
                  'the same middleware/domain?', file=sys.stderr)
            return 1
        print(f'Found {count} topics under /{ns}/.')

        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        out_dir = args.output or Path.cwd() / f'onav_diagnostics_{stamp}'
        # Only a directory created by this capture is removed after archiving
        created_out_dir = not out_dir.exists()
        out_dir.mkdir(parents=True, exist_ok=True)
        nav_dir = out_dir / 'navigation'
        ros_dir = out_dir / 'ros'
        tf_dir = out_dir / 'tf'
        logs_dir = out_dir / 'logs'
        env_dir = out_dir / 'environment'
        timeline_dir = out_dir / 'timeline'
        for directory in (nav_dir, ros_dir, tf_dir, logs_dir, env_dir, timeline_dir):
            directory.mkdir(exist_ok=True)

        graph = node.snapshot_graph()
        topics = graph['topics']

        def visible(name):
            return '/_' not in name and not name.endswith(('/parameter_events', '/rosout'))

        rate_topics = {
            name for name, info in topics.items()
            if name.startswith(f'/{ns}/') and visible(name)
            and (args.include_heavy or info['types'][0] not in HEAVY_TYPES)
        }
        rate_topics |= {resolve(ns, t) for t in args.topics if resolve(ns, t) in topics}
        log_topics = [n for n, i in topics.items()
                      if n.endswith('/rosout') and 'rcl_interfaces/msg/Log' in i['types']]
        diag_topics = [n for n, i in topics.items() if visible(n)
                       and 'diagnostic_msgs/msg/DiagnosticArray' in i['types']]
        tf_topics = [t for t in (f'/{ns}/tf', f'/{ns}/tf_static', '/tf', '/tf_static')
                     if t in topics]

        node.monitor_rates(sorted(rate_topics), graph)
        node.monitor_logs(log_topics)
        node.monitor_diagnostics(diag_topics)
        node.monitor_tf(tf_topics)
        node.monitor_trajectory(graph, ns)
        node.monitor_localization(graph, ns)
        node.monitor_navigation_goals(graph, ns)
        node.monitor_velocity_chain(graph, ns)

        bag_proc, bag_log = (start_bag(out_dir, sorted(set(rate_topics) | set(log_topics)
                                                       | set(tf_topics)))
                             if args.bag else (None, None))

        print(f'Monitoring {len(node.rates)} topics for {args.duration:.0f}s '
              '(Ctrl-C to stop early)...')
        start = time.monotonic()
        capture_start = time.time()
        node.capture_start = capture_start
        try:
            spin_for(node, args.duration)
        except KeyboardInterrupt:
            print('\nCapture interrupted; writing results.')
        window = max(time.monotonic() - start, 1e-3)
        capture_end = time.time()
        node.stop_goal_tracking()
        stop_bag(bag_proc, bag_log)

        trajectory = save_trajectory_outputs(
            nav_dir, node.global_path, node.global_path_frame, node.trajectory,
            node.trajectory_errors)
        localization = save_localization_outputs(
            nav_dir, node.localization_poses, node.map_odom, node.initial_poses,
            capture_start, capture_end)
        goals = save_goal_outputs(nav_dir, node.goals, node.goal_feedback, node.goal_poses,
                                  node.monitored_actions, capture_end)
        velocity = save_velocity_outputs(
            nav_dir, node.velocity_samples, node.velocity_topics, node.lock_events,
            node.collision_events, capture_start, capture_end)

        for sub in list(node.subscriptions):
            node.destroy_subscription(sub)

        now = node.get_clock().now()
        lookups = {}
        for target, source in TF_LOOKUPS:
            key = f'{target} -> {source}'
            try:
                tf = node.tf_buffer.lookup_transform(target, source, Time())
                stamp_ns = Time.from_msg(tf.header.stamp).nanoseconds
                lookups[key] = {'ok': True}
                if stamp_ns:
                    lookups[key]['age_s'] = round((now.nanoseconds - stamp_ns) / 1e9, 3)
            except Exception as exc:
                lookups[key] = {'ok': False, 'error': type(exc).__name__ + ': ' + str(exc)}
        all_ages = []
        for edge in node.tf_edges.values():
            ages = edge.pop('ages_s')
            all_ages += ages
            if ages:
                edge.update(rate_hz=round(len(ages) / window, 2),
                            median_age_s=round(statistics.median(ages), 4),
                            max_age_s=round(max(ages), 4))
        tf_info = {
            'edges': node.tf_edges,
            'lookups': lookups,
            'median_age_s': round(statistics.median(all_ages), 4) if all_ages else None,
        }

        print('Querying lifecycle states...')
        lifecycle = node.lifecycle_states(graph['services'], args.service_timeout)

        param_errors = {}
        if not args.skip_params:
            print('Dumping node parameters...')
            param_dir = out_dir / 'parameters'
            param_dir.mkdir(exist_ok=True)
            for name in graph['nodes']:
                if not name.startswith(f'/{ns}/') or \
                        f'{name}/list_parameters' not in graph['services']:
                    continue
                params, err = node.parameters(name, args.service_timeout)
                if params is None:
                    param_errors[name] = err
                    continue
                (param_dir / (name.strip('/').replace('/', '__') + '.yaml')).write_text(
                    yaml.dump(sanitize({name: {'ros__parameters': params}}), Dumper=ParameterDumper,
                              sort_keys=True))

        fingerprint = {'status': 'skipped'}
        if not args.skip_fingerprint:
            print('Collecting robot software and configuration fingerprint...')
            fingerprint = collect_fingerprint(
                env_dir, args.ssh_host, args.ssh_user,
                remote=args.remote_fingerprint or args.remote_service_logs
                or args.remote_docker_logs, port=args.ssh_port)

        service_logs = {'status': 'skipped'}
        if not args.skip_service_logs:
            transport = 'over SSH' if args.remote_service_logs else 'locally'
            print(f'Dumping clearpath-platform-extras service logs {transport}...')
            service_logs = collect_platform_extras_logs(
                logs_dir, args.ssh_host, args.ssh_user, local=not args.remote_service_logs,
                ssh_port=args.ssh_port)
        write_json(logs_dir / 'service_logs.json', service_logs)

        docker_logs = {'status': 'skipped'}
        if not args.skip_docker_logs:
            dozzle_url = args.dozzle_url or (
                f'http://{args.ssh_host}:5200/_/logs' if args.remote_docker_logs
                else 'http://127.0.0.1:5200/_/logs')
            transport = 'over SSH' if args.remote_docker_logs else 'locally'
            print(f'Downloading all Docker container logs from Dozzle {transport}...')
            docker_logs = collect_docker_logs(
                logs_dir, dozzle_url, args.dozzle_host, args.ssh_host, args.ssh_user,
                remote=args.remote_docker_logs, timeout=args.docker_logs_timeout,
                ssh_port=args.ssh_port)
        write_json(logs_dir / 'docker_logs.json', docker_logs)

        environment = {
            'captured_at': datetime.now().astimezone().isoformat(timespec='seconds'),
            'host': socket.gethostname(),
            'platform': platform.platform(),
            'python': sys.version.split()[0],
            'env': {v: os.environ.get(v) for v in ENV_VARS},
            'namespace': ns,
            'duration_s': round(window, 2),
            'ping': run_ping(env_dir),
        }
        rates = {t: s.result(window) for t, s in sorted(node.rates.items())}
        diagnostics = {}
        for key, rec in sorted(node.diagnostics.items()):
            rec.pop('_worst')
            rec['counts'] = dict(rec['counts'])
            diagnostics[key] = rec
        logs = sorted(node.logs, key=lambda e: e['time'])
        timeline = save_timeline_outputs(
            timeline_dir, capture_start, capture_end, goals, localization,
            node.localization_poses, node.lock_events, node.collision_events, diagnostics, logs)

        copy_middleware_config(env_dir)
        shutil.copy2(CONFIG_PATH, env_dir / CONFIG_PATH.name)
        write_json(env_dir / 'environment.json', environment)
        write_json(ros_dir / 'graph.json', graph)
        write_json(ros_dir / 'topic_rates.json', {'rates': rates, 'errors': node.rate_errors})
        write_json(ros_dir / 'lifecycle.json', lifecycle)
        write_json(ros_dir / 'diagnostics.json', diagnostics)
        write_json(tf_dir / 'tf.json', tf_info)
        if param_errors:
            write_json(out_dir / 'parameters' / 'parameter_errors.json', param_errors)
        (tf_dir / 'tf_frames.yaml').write_text(node.tf_buffer.all_frames_as_yaml())
        (ros_dir / 'topic_list.txt').write_text(''.join(
            f'{n} [{", ".join(i["types"])}] pubs={len(i["publishers"])} '
            f'subs={len(i["subscribers"])}\n' for n, i in sorted(topics.items())))
        (logs_dir / 'rosout.log').write_text(''.join(
            f'{datetime.fromtimestamp(e["time"]).isoformat(timespec="milliseconds")} '
            f'[{e["level"]}] [{e["logger"]}] {e["msg"]}  <{e["location"]}>\n' for e in logs))

        summary, findings = build_summary({
            'namespace': ns, 'graph': graph, 'rates': rates, 'rate_errors': node.rate_errors,
            'lifecycle': lifecycle, 'tf': tf_info, 'diagnostics': diagnostics, 'logs': logs,
            'environment': environment, 'window': window, 'trajectory': trajectory,
            'service_logs': service_logs, 'docker_logs': docker_logs,
            'fingerprint': fingerprint,
            'timeline': timeline,
            'goals': goals, 'localization': localization,
            'velocity': velocity,
        })
        (out_dir / 'summary.md').write_text(summary)
        print()
        print(summary)

        if not args.no_archive:
            archive = out_dir.with_name(out_dir.name + '.tar.gz')
            # Write to a temporary name so a failed archive never replaces the capture files
            partial_archive = archive.with_name(archive.name + '.partial')
            with tarfile.open(partial_archive, 'w:gz') as tar:
                tar.add(out_dir, arcname=out_dir.name)
            partial_archive.replace(archive)
            print(f'Archive: {archive}')
            if created_out_dir and not args.keep_directory:
                shutil.rmtree(out_dir)
        if out_dir.exists():
            print(f'Capture directory: {out_dir}')
        return 1 if any(level == 'ERROR' for level, _ in findings) else 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
