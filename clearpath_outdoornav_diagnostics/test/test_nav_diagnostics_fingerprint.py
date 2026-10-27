import csv
import hashlib
import io
import json
import subprocess
import sys
import tempfile
import tarfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import yaml

from clearpath_outdoornav_diagnostics import nav_diagnostics_capture as fingerprint


class FingerprintTest(unittest.TestCase):

    def test_snapshot_files_are_deduplicated_with_all_sources_and_readable_names(self):
        snapshots = [
            {'source': '/opt/onav/ros2/app/docker-compose.yml', 'content': 'services: {}\n'},
            {'source': '/opt/onav/app/docker-compose.yml', 'content': 'services: {}\n'},
            {'source': '/opt/onav/ros2/app/docker-compose.yml', 'content': 'services: {}\n'},
            {'source': '/etc/clearpath/robot.yaml', 'content': 'serial_number: a300-12345\n'},
            {'source': '/app/navigation/nav2_params.yaml', 'content': 'frequency: 20\n'},
            {'source': '/other/navigation/nav2_params.yaml', 'content': 'frequency: 30\n'},
            {'source': '/unsafe/../../fingerprint.json', 'content': 'example: true\n'},
        ]
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            result = fingerprint.save_fingerprint_configurations(output, snapshots)
            self.assertEqual(len(result), 5)
            self.assertEqual(len(list(output.glob('*.yaml'))), 5)
            compose = next(item for item in result if item['file'] == 'docker-compose.yaml')
            self.assertEqual(compose['sources'], [
                '/opt/onav/app/docker-compose.yml', '/opt/onav/ros2/app/docker-compose.yml'])
            self.assertEqual(compose['source'], compose['sources'][0])
            self.assertTrue((output / 'robot.yaml').is_file())
            self.assertTrue((output / 'nav2_params.yaml').is_file())
            different = next(item for item in result if item['source'].startswith('/other/'))
            digest = hashlib.sha256(b'frequency: 30\n').hexdigest()
            self.assertEqual(different['file'], f'nav2_params_{digest[:12]}.yaml')
            for item in result:
                path = output / item['file']
                self.assertEqual(path.parent, output)
                self.assertEqual(hashlib.sha256(path.read_bytes()).hexdigest(),
                                 item['sanitized_sha256'])
                self.assertNotIn('content', item)
            self.assertEqual(
                fingerprint.save_fingerprint_configurations(output, list(reversed(snapshots))),
                result)

    def test_configuration_redaction_and_snapshot_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'compose.yaml'
            path.write_text(yaml.safe_dump({
                'password': 'private-password',
                'camera': {'api_key': 'private-key', 'model': 'camera-v1'},
                'connection': 'https://user:private-password@host/path?token=private-token',
                'services': {'app': {
                    'environment': ['PASSWORD=private-password', 'ROS_DISTRO=jazzy'],
                    'command': ['login', '--password', 'private-password'],
                }},
            }))
            errors = []
            snapshot = fingerprint.snapshot_file(path, errors)
            self.assertEqual(errors, [])
            for secret in ('private-password', 'private-key', 'private-token'):
                self.assertNotIn(secret, snapshot['content'])
            self.assertIn('camera-v1', snapshot['content'])
            self.assertIn('jazzy', snapshot['content'])
            self.assertEqual(
                snapshot['sanitized_sha256'],
                hashlib.sha256(snapshot['content'].encode()).hexdigest())

    def test_compose_commands_preserve_content_without_credential_evidence(self):
        commands = [
            'ros2 launch onav_autonomy navigation.launch.py use_sim_time:=false',
            ['ros2', 'launch', 'onav_autonomy', 'navigation.launch.py'],
            ['login', 'unlabeled-positional-value'],
            ['sh', '-c', 'echo password support; ros2 launch onav bringup.launch.py'],
            None,
        ]
        for command in commands:
            with self.subTest(command=command), tempfile.TemporaryDirectory() as directory:
                path = Path(directory) / 'compose.yaml'
                path.write_text(yaml.safe_dump({'services': {'app': {
                    'command': command, 'entrypoint': command,
                }}}))
                errors = []
                snapshot = fingerprint.snapshot_file(path, errors)
                self.assertEqual(errors, [])
                app = yaml.safe_load(snapshot['content'])['services']['app']
                self.assertEqual(app['command'], command)
                self.assertEqual(app['entrypoint'], command)

    def test_command_redacts_only_recognizable_credential_values(self):
        cases = [
            (['login', '--password', 'sensitive value', '--mode', 'normal'],
             ['login', '--password', '[REDACTED]', '--mode', 'normal']),
            ('login --password="sensitive value" --mode normal',
             'login --password=[REDACTED] --mode normal'),
            ("login --api-key 'sensitive value' --mode normal",
             'login --api-key [REDACTED] --mode normal'),
            ('TOKEN=sensitive login --mode normal',
             'TOKEN=[REDACTED] login --mode normal'),
            ('login --verbose --password sensitive --mode normal',
             'login --verbose --password [REDACTED] --mode normal'),
            ('ros2 run app node --ros-args -p password:=sensitive',
             'ros2 run app node --ros-args -p password:=[REDACTED]'),
            (['sh', '-c', 'login --password sensitive; echo ready'],
             ['sh', '-c', 'login --password [REDACTED]; echo ready']),
            ('client https://user:sensitive@example.invalid/path?token=sensitive',
             'client https://[REDACTED]@example.invalid/path?token=[REDACTED]'),
        ]
        for command, expected in cases:
            with self.subTest(command=command):
                self.assertEqual(fingerprint.sanitize_command(command), expected)

    def test_robot_probe_collects_versions_digests_and_bind_mounted_configs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            robot = root / 'etc/clearpath/robot.yaml'
            robot.parent.mkdir(parents=True)
            robot.write_text(
                'serial_number: a300-12345\nsystem:\n  ros2:\n'
                '    middleware:\n      implementation: rmw_zenoh_cpp\n'
                'platform:\n  password: must-not-leak\n')
            config = root / 'deployment/config'
            config.mkdir(parents=True)
            (config / 'outdoornav.yaml').write_text(
                'version: 2.0.1\nnavigation:\n  param_file: /opt/onav/app/custom.yaml\n'
                'localization:\n  param_file: /opt/onav/config/localization.yaml\n')
            (config / 'localization.yaml').write_text('filter_frequency: 30\n')
            default_config = root / 'opt/onav/config'
            default_config.mkdir(parents=True)
            (default_config / 'outdoornav.yaml').write_text('version: rolling\n')
            app = root / 'deployment/app'
            app.mkdir()
            (app / 'custom.yaml').write_text('controller_frequency: 20\n')
            (app / 'docker-compose.yml').write_text('services: {}\n')
            launch = app / 'docker/launch/test.py'
            launch.parent.mkdir(parents=True)
            launch.write_text('print("launch")\n')
            cid = 'a' * 64
            image_id = 'sha256:' + 'b' * 64
            image_digest = 'repo@sha256:' + 'c' * 64

            def robot_path(path):
                path = Path(path)
                if str(path) in ('/etc/clearpath/robot.yaml', '/opt/onav/config',
                                 '/opt/onav/config/outdoornav.yaml', '/opt/onav/app'):
                    return root / str(path).lstrip('/')
                return path

            def command(args, errors, **kwargs):
                if args[:3] == ['docker', 'ps', '--all']:
                    return cid
                if args[:2] == ['docker', 'inspect']:
                    return json.dumps([{
                        'Name': '/onav-autonomy', 'Image': image_id,
                        'State': {'Running': True},
                        'Config': {
                            'Image': 'repo:rolling',
                            'Env': ['COMMIT_CPR_ONAV_AUTONOMY=abc123',
                                    'ROS_DISTRO=jazzy', 'PASSWORD=must-not-leak'],
                            'Labels': {'com.docker.compose.service': 'onav-autonomy'},
                        },
                        'Mounts': [
                            {'Source': str(app), 'Destination': '/opt/onav/app'},
                            {'Source': str(config), 'Destination': '/opt/onav/config'},
                        ],
                    }])
                if args[:3] == ['docker', 'exec', '-i']:
                    self.assertIn('def runtime_software', kwargs['input_text'])
                    return json.dumps({
                        'installed_package_versions': {'rclpy': '7.1.0'},
                        'source_revisions': {'/opt/base_ws/src/nav': {'commit': 'def456'}},
                        'errors': [],
                    })
                if args[:3] == ['docker', 'image', 'inspect']:
                    return json.dumps([{'Id': image_id, 'RepoDigests': [image_digest]}])
                self.fail(f'Unexpected command: {args}')

            with patch.object(fingerprint, 'Path', side_effect=robot_path), \
                    patch.object(fingerprint, 'run_command', side_effect=command), \
                    patch.object(fingerprint, 'git_revision', return_value={'commit': '123abc'}):
                result = fingerprint.probe_robot('def runtime_software(): pass')
            self.assertEqual(result['status'], 'ok', result['errors'])
            self.assertEqual(result['robot_identity'][0]['serial_number'], 'a300-12345')
            self.assertIn('2.0.1', [item['version'] for item in result['outdoornav_versions']])
            self.assertEqual(result['images'][0]['repo_digests'], [image_digest])
            self.assertEqual(result['containers'][0]['image_id'], image_id)
            self.assertEqual(result['containers'][0]['runtime']['installed_package_versions'],
                             {'rclpy': '7.1.0'})
            self.assertIn(str(app / 'custom.yaml'),
                          [item['source'] for item in result['configurations']])
            self.assertIn(str(config / 'localization.yaml'),
                          [item['source'] for item in result['configurations']])
            self.assertEqual(result['deployments'][0]['launch_file_sha256'][str(launch)],
                             hashlib.sha256(launch.read_bytes()).hexdigest())
            self.assertNotIn('must-not-leak', json.dumps(result))

    def test_invalid_remote_payload_is_reported_without_aborting_capture(self):
        for payload in ([], {}, {'status': 'ok', 'errors': [], 'configurations': [{}]}):
            with self.subTest(payload=payload), tempfile.TemporaryDirectory() as directory, \
                    patch.object(fingerprint, 'run_command', return_value=json.dumps(payload)), \
                    patch.object(fingerprint, 'fingerprint_probe_source', return_value='probe'), \
                    patch.object(fingerprint, 'git_revision', return_value={'commit': 'abc123'}), \
                    patch('sys.stderr', new_callable=io.StringIO) as stderr:
                result = fingerprint.collect_fingerprint(
                    Path(directory), 'cpr-rooftop', 'robot', remote=True, port=1122)
                self.assertEqual(result['status'], 'error')
                self.assertIn('invalid payload structure', result['errors'][0])
                self.assertIn('WARNING', stderr.getvalue())
                self.assertTrue((Path(directory) / 'fingerprint/fingerprint.json').is_file())

    def test_remote_collection_uses_port_and_does_not_fall_back_to_pc(self):
        probe_source = fingerprint.fingerprint_probe_source()
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            robot = {'status': 'ok', 'errors': [], 'robot_host': {'hostname': 'robot'},
                     'configurations': [
                         {'source': '/opt/onav/config/outdoornav.yaml',
                          'content': 'version: 2.0.1\n'},
                         {'source': '/opt/onav/ros2/config/outdoornav.yaml',
                          'content': 'version: 2.0.1\n'},
                     ]}
            with patch.object(fingerprint, 'run_command', return_value=json.dumps(robot)) as run, \
                    patch.object(fingerprint, 'probe_robot') as local, \
                    patch.object(fingerprint, 'git_revision', return_value={'commit': 'abc123'}), \
                    patch.object(fingerprint, 'fingerprint_probe_source', return_value=probe_source):
                result = fingerprint.collect_fingerprint(
                    output, 'cpr-rooftop', 'robot', remote=True, port=1122)
            self.assertEqual(result['status'], 'ok')
            self.assertEqual(run.call_args.args[0][:4], ['ssh', '-T', '-p', '1122'])
            source = run.call_args.kwargs['input_text']
            compile(source, '<remote fingerprint>', 'exec')
            local.assert_not_called()
            data = json.loads((output / 'fingerprint/fingerprint.json').read_text())
            self.assertEqual(data['target']['transport'], 'SSH')
            self.assertEqual(data['robot_host']['hostname'], 'robot')
            self.assertIn('nav_diagnostics_capture.py', data['collector']['sha256'])
            self.assertIn('version', data['collector'])
            self.assertEqual(data['schema_version'], 2)
            self.assertEqual(len(data['configurations']), 1)
            self.assertEqual(data['configurations'][0]['file'], 'outdoornav.yaml')
            self.assertEqual(len(data['configurations'][0]['sources']), 2)
            self.assertEqual(
                (output / 'fingerprint/outdoornav.yaml').read_text(), 'version: 2.0.1\n')

    def test_ssh_failure_is_persisted_and_warned(self):
        probe_source = fingerprint.fingerprint_probe_source()
        def failed(command, errors, **kwargs):
            errors.append('ssh exited 255: Permission denied')
            return None

        with tempfile.TemporaryDirectory() as directory, \
                patch.object(fingerprint, 'run_command', side_effect=failed), \
                patch.object(fingerprint, 'probe_robot') as local, \
                patch.object(fingerprint, 'fingerprint_probe_source', return_value=probe_source), \
                patch('sys.stderr') as stderr:
            output = Path(directory)
            result = fingerprint.collect_fingerprint(output, 'robot', 'robot', remote=True)
            self.assertEqual(result['status'], 'error')
            self.assertIn('Permission denied', result['errors'][0])
            self.assertTrue((output / 'fingerprint/fingerprint.json').exists())
            stderr.write.assert_called()
            local.assert_not_called()

    def test_embedded_probe_executes_without_ros_or_collector_imports(self):
        source = fingerprint.fingerprint_probe_source()
        self.assertNotIn('import rclpy', source)
        result = subprocess.run(
            [sys.executable, '-', '--runtime'], input=source,
            capture_output=True, text=True, timeout=60)
        self.assertEqual(result.returncode, 0, result.stderr)
        data = json.loads(result.stdout)
        self.assertIn('installed_package_versions', data)
        self.assertIn('errors', data)

    def run_capture(self, output, *extra_args):
        """Run main() against a mocked robot graph; returns the collect_fingerprint mock."""
        node = MagicMock()
        for name in ('rates', 'rate_errors', 'trajectory_errors', 'goals', 'goal_feedback',
                     'monitored_actions', 'velocity_samples', 'velocity_topics',
                     'lock_events', 'tf_edges', 'diagnostics'):
            setattr(node, name, {})
        for name in ('trajectory', 'localization_poses', 'map_odom', 'initial_poses',
                     'goal_poses', 'collision_events', 'subscriptions', 'logs'):
            setattr(node, name, [])
        node.global_path = None
        node.global_path_frame = None
        node.snapshot_graph.return_value = {
            'nodes': [], 'duplicate_nodes': [], 'topics': {}, 'services': {}, 'actions': {},
        }
        node.lifecycle_states.return_value = {}
        node.tf_buffer.all_frames_as_yaml.return_value = '{}\n'

        def collect(directory, *args, **kwargs):
            target = directory / 'fingerprint'
            target.mkdir()
            (target / 'fingerprint.json').write_text('{}')
            return {'status': 'ok', 'file': 'environment/fingerprint/fingerprint.json'}

        with patch.object(fingerprint, 'Capture', return_value=node), \
                patch.object(fingerprint.rclpy, 'init'), \
                patch.object(fingerprint.rclpy, 'try_shutdown'), \
                patch.object(fingerprint, 'TF_LOOKUPS', []), \
                patch.object(fingerprint, 'wait_for_graph', return_value=1), \
                patch.object(fingerprint, 'spin_for'), \
                patch.object(fingerprint, 'run_ping', return_value=None), \
                patch.object(fingerprint, 'copy_middleware_config'), \
                patch.object(fingerprint, 'collect_fingerprint', side_effect=collect) as collector, \
                patch.object(sys, 'argv', [
                    'nav_diagnostics_capture.py', '--namespace', 'robot',
                    '--output', str(output), '--remote-fingerprint', '--ssh-port', '1122',
                    '--ssh-host', 'rooftop', '--skip-docker-logs', '--skip-service-logs',
                    *extra_args,
                ]), redirect_stdout(io.StringIO()):
            fingerprint.main()
        return collector

    def test_capture_includes_fingerprint_in_archive_and_summary(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / 'capture'
            collector = self.run_capture(output)
            self.assertTrue(collector.call_args.kwargs['remote'])
            self.assertEqual(collector.call_args.kwargs['port'], 1122)
            # Only the archive is kept, so read the summary from it
            self.assertFalse(output.exists())
            with tarfile.open(output.with_name('capture.tar.gz')) as archive:
                summary = archive.extractfile('capture/summary.md').read().decode()
                self.assertIn('environment/fingerprint/fingerprint.json', summary)
                self.assertIn('capture/environment/fingerprint/fingerprint.json',
                              archive.getnames())
                for name in ('timeline.json', 'timeline.csv', 'timeline_plot.png'):
                    self.assertIn(f'capture/timeline/{name}', archive.getnames())
            self.assertFalse(output.with_name('capture.tar.gz.partial').exists())

    def test_capture_directory_kept_when_requested_or_not_archived(self):
        with tempfile.TemporaryDirectory() as directory:
            kept = Path(directory) / 'kept'
            self.run_capture(kept, '--keep-directory')
            self.assertTrue((kept / 'summary.md').is_file())
            self.assertTrue(kept.with_name('kept.tar.gz').is_file())

            unarchived = Path(directory) / 'unarchived'
            self.run_capture(unarchived, '--no-archive')
            self.assertTrue((unarchived / 'summary.md').is_file())
            self.assertFalse(unarchived.with_name('unarchived.tar.gz').exists())

    def test_existing_output_directory_is_never_removed(self):
        with tempfile.TemporaryDirectory() as directory:
            existing = Path(directory) / 'existing'
            existing.mkdir()
            (existing / 'unrelated.txt').write_text('keep me')
            self.run_capture(existing)
            self.assertEqual((existing / 'unrelated.txt').read_text(), 'keep me')
            self.assertTrue(existing.with_name('existing.tar.gz').is_file())


class TimelineTest(unittest.TestCase):

    def test_goal_and_initial_pose_messages_are_observations(self):
        pose = {'time': 101.0, 'x': 1.0, 'y': 2.0, 'yaw_deg': 90.0, 'frame': 'map'}
        events = fingerprint.timeline_events(
            100.0, 102.0, {'goals': [], 'goal_pose_messages': [pose]},
            {'map_odom_jumps': [], 'initial_pose_events': [pose]}, [], {}, [], {}, [])
        self.assertEqual(len(events), 2)
        self.assertEqual({event['category'] for event in events}, {'Navigation', 'Localization'})
        self.assertTrue(all(event['classification'] == 'observation' for event in events))

    def test_events_use_receipt_time_and_preserve_evidence_and_thresholds(self):
        goals = {'goals': [
            {'goal_id': 'current', 'action': '/robot/navigate_to_pose',
             'started_before_capture': False, 'transitions': [
                 {'time': 103.0, 'status': 'ACCEPTED'},
                 {'time': 110.0, 'status': 'ABORTED'},
             ], 'result_received_time': 111.0, 'error_code': 42, 'error_msg': 'No path',
             'result_status': 'ABORTED'},
            {'goal_id': 'old', 'action': '/robot/navigate_to_pose',
             'started_before_capture': True, 'finished_before_capture': True,
             'result_received_time': 102.5, 'error_code': 42, 'result_status': 'ABORTED',
             'transitions': [{'time': 102.0, 'status': 'ABORTED'}]},
        ]}
        localization = {'map_odom_jumps': [
            {'time': 109.5, 'translation_m': 1.0, 'rotation_deg': 0.0},
        ]}
        poses = [
            {'time': stamp, 'std_xy_m': uncertainty, 'std_yaw_deg': 0.0}
            for stamp, uncertainty in ((100.0, 0.1), (104.0, 0.3), (105.0, 0.4), (106.5, 0.25))
        ]
        locks = {'emergency_stop': [(100.0, False), (105.0, True),
                                    (105.5, True), (106.0, False)]}
        collision = [
            {'time': stamp, 'action': action, 'polygon': 'stop_zone'}
            for stamp, action in ((107.0, 'STOP'), (108.0, 'STOP'), (109.0, 'DO_NOTHING'))
        ]
        diagnostics = {'motor': {'transitions': [
            {'time': 108.0, 'level': 'ERROR', 'message': 'Fault', 'source_time': 500.0},
            {'time': 109.0, 'level': 'OK', 'message': 'Recovered', 'source_time': 501.0},
        ]}}
        logs = [
            {'time': 1.0, 'received_time': 112.0, 'level': 'ERROR',
             'logger': 'planner', 'msg': 'No path'},
            {'time': 2.0, 'received_time': 114.0, 'level': 'ERROR',
             'logger': 'planner', 'msg': 'Outside capture'},
        ]
        events = fingerprint.timeline_events(
            100.0, 113.0, goals, localization, poses, locks, collision, diagnostics, logs)
        self.assertEqual(len(events), 16)
        self.assertEqual([event['time'] for event in events],
                         sorted(event['time'] for event in events))
        self.assertEqual({event['category'] for event in events}, {
            'Navigation', 'Localization', 'Stop locks', 'Collision monitor',
            'Diagnostics', 'ROS logs',
        })
        thresholds = [event for event in events if event['classification'] == 'threshold']
        self.assertEqual(len(thresholds), 3)
        self.assertTrue(any('within threshold' in event['message'] for event in thresholds))
        historical = next(event for event in events if 'Goal old:' in event['message'])
        self.assertEqual(historical['severity'], 'INFO')
        historical_result = next(event for event in events if 'Goal old result' in event['message'])
        self.assertEqual(historical_result['severity'], 'INFO')
        self.assertIn('finished before capture', historical_result['message'])
        log = next(event for event in events if event['category'] == 'ROS logs')
        self.assertEqual(log['time'], 112.0)
        self.assertEqual(log['source_time'], 1.0)
        self.assertEqual(log['elapsed_s'], 12.0)
        self.assertTrue(all(event['evidence'] for event in events))
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            data = fingerprint.save_timeline_outputs(
                output, 100.0, 113.0, goals, localization, poses, locks,
                collision, diagnostics, logs)
            self.assertEqual(data['events'], events)
            with (output / 'timeline.csv').open() as stream:
                self.assertEqual(len(list(csv.DictReader(stream))), 16)
            self.assertEqual(json.loads((output / 'timeline.json').read_text())['event_count'], 16)
            self.assertEqual((output / 'timeline_plot.png').read_bytes()[:8], b'\x89PNG\r\n\x1a\n')

    def test_empty_capture_still_writes_timeline_plot(self):
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            data = fingerprint.save_timeline_outputs(
                output, 100.0, 101.0, {'goals': []}, {'map_odom_jumps': []},
                [], {}, [], {}, [])
            self.assertEqual(data['event_count'], 0)
            self.assertTrue((output / 'timeline_plot.png').is_file())

    def test_diagnostic_callback_records_only_level_or_message_transitions(self):
        capture = SimpleNamespace(diagnostics={})
        status = SimpleNamespace(level=0, name='motor', hardware_id='m1',
                                 message='Ready', values=[])
        message = SimpleNamespace(status=[status],
                                  header=SimpleNamespace(stamp=SimpleNamespace(sec=200, nanosec=0)))
        with patch.object(fingerprint.time, 'time', return_value=100.0):
            fingerprint.Capture.on_diagnostics(capture, '/robot/diagnostics', message)
            fingerprint.Capture.on_diagnostics(capture, '/robot/diagnostics', message)
            status.level = 2
            status.message = 'Fault'
            fingerprint.Capture.on_diagnostics(capture, '/robot/diagnostics', message)
        record = capture.diagnostics['/robot/diagnostics | motor']
        self.assertEqual(len(record['transitions']), 2)
        self.assertEqual(record['transitions'][1]['level'], 'ERROR')
        self.assertEqual(record['transitions'][1]['time'], 100.0)
        self.assertEqual(record['transitions'][1]['source_time'], 200.0)


if __name__ == '__main__':
    unittest.main()
