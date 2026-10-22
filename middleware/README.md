# Connecting to an OutdoorNav robot

`connect_robot.sh` configures the current shell to talk to a robot over either Zenoh (`rmw_zenoh_cpp`) or Cyclone DDS (`rmw_cyclonedds_cpp`). The robot and the remote PC must use the same middleware and the same `ROS_DOMAIN_ID` (default `0`).

The commands below assume the workspace is `~/onav_ws` and this repository is at
`~/onav_ws/src/onav-app/api`. The source and launch commands use full paths so they
can be run from any working directory.

## Robot

The middleware is selected in the robot's `robot.yaml` (see `../clearpath_outdoornav_setup/onav/*/*/robot.yaml`) and the robot service will restart on saving the file with the following updates:

```yaml
system:
  ros2:
    domain_id: 0
    middleware:
      implementation: rmw_zenoh_cpp    # or rmw_cyclonedds_cpp
```

### Zenoh

- The robot must run a Zenoh router listening on TCP port 7447 (`ros2 run rmw_zenoh_cpp rmw_zenohd`, normally started by the robot services).
- Check it with `ss -ltn | grep 7447`, and allow the port through any firewall.

### Cyclone DDS

- No router is needed. Allow UDP between the PC and robot (DDS discovery and data ports); multicast is not required.
- Over a VPN, datagrams are limited to 1400 bytes by `cyclonedds.xml.in`.

## Remote PC

Requirements: ROS 2 Jazzy, `gettext-base` (`envsubst`), and the RMW package for the middleware in use:

```bash
sudo apt install gettext-base ros-jazzy-rmw-zenoh-cpp ros-jazzy-rmw-cyclonedds-cpp
```

### Direct connection

Use these commands when the PC can directly reach the robot's network address. Source the script because it sets environment variables in the current shell:

```bash
export ROS_DOMAIN_ID=0  # Replace 0 with the robot's system.ros2.domain_id.

# Zenoh
source ~/onav_ws/src/onav-app/api/middleware/connect_robot.sh zenoh <robot-ip> [robot-hostname]

# Cyclone DDS
source ~/onav_ws/src/onav-app/api/middleware/connect_robot.sh cyclone_dds <robot-ip>

# Interactive (prompts for middleware and robot address)
source ~/onav_ws/src/onav-app/api/middleware/connect_robot.sh
```

### SSH-tunneled Zenoh connection

Use this option when the PC can SSH to a host that can reach the robot's Zenoh router, but the PC cannot directly reach the robot network. The defaults match the rooftop robot: SSH port `1122`, and router `192.168.131.1:7447`.

```bash
export ROS_DOMAIN_ID=0  # Replace 0 with the robot's system.ros2.domain_id.
source ~/onav_ws/src/onav-app/api/middleware/connect_robot.sh zenoh --ssh-tunnel robot@cpr-rooftop
```

The script opens a local forward at `127.0.0.1:7447` and configures Zenoh to use it. The SSH host must be able to reach the robot's router at `192.168.131.1:7447`. SSH key authentication is recommended; with password authentication, SSH prompts while establishing the tunnel.

Override defaults if needed; use another local port if port `7447` is already occupied:

```bash
export ROS_DOMAIN_ID=0  # Replace 0 with the robot's system.ros2.domain_id.
source ~/onav_ws/src/onav-app/api/middleware/connect_robot.sh zenoh --ssh-tunnel robot@cpr-rooftop \
  --ssh-port 1122 --router-ip 192.168.131.1 --local-port 17447
```

Close the managed tunnel in the same shell after finishing:

```bash
cpr_zenoh_tunnel_stop
```

### After connecting

Both connection modes configure the current shell. The script:

- resolves the robot address and adds `<ip> <hostname>` to `/etc/hosts` if missing (needs sudo; skip with `ROBOT_PIN_HOST=0`),
- generates the middleware config into `~/.cache/clearpath_remote/` from `zenoh_session.json5.in` or `cyclonedds.xml.in`,
- exports `RMW_IMPLEMENTATION` plus `ZENOH_SESSION_CONFIG_URI` or `CYCLONEDDS_URI`,
- for SSH-tunneled Zenoh, runs an SSH ControlMaster in the background,
- detects the robot namespace from the topic list and exports `ROBOT_NAMESPACE`.

## Visualization

Visualize OutdoorNav data from a remote PC with RViz2 or Foxglove. RViz2 uses the
ROS middleware connection configured above; Foxglove connects to a Foxglove
WebSocket bridge.

### RViz2

Install RViz2 on a PC with ROS 2 Jazzy:

```bash
sudo apt install ros-jazzy-rviz2
```

Use either the [direct connection](#direct-connection) or
[SSH-tunneled Zenoh connection](#ssh-tunneled-zenoh-connection) above. The robot
and PC must use the same `ROS_DOMAIN_ID`.

After sourcing the connector, launch RViz2 in the same shell:

```bash
ros2 launch ~/onav_ws/src/onav-app/api/rviz/rviz_navigation.launch.py namespace:=$ROBOT_NAMESPACE
```

The launch file loads [outdoornav.rviz](../rviz/outdoornav.rviz) by default and
applies the detected robot namespace. No ROS package build is needed to launch
this file by path. Override the config with `rviz_config:=/path/to/config.rviz`.

### Foxglove

Install [Foxglove Desktop](https://foxglove.dev/download). Foxglove connects to
the robot's Foxglove WebSocket bridge, not directly to Zenoh.

OutdoorNav starts its Foxglove bridge on port `8780` in
`docker/launch/onav_ui.launch.py`. Confirm it is running and reachable from the
SSH host:

```bash
ssh -p 1122 robot@cpr-rooftop "ss -ltn | grep ':8780'"
```

For a direct connection, choose **Open connection** -> **Foxglove WebSocket** and
connect to `ws://<robot-ip>:8780`.

#### Tunnel the robot's Foxglove bridge

If the PC cannot reach the robot's bridge directly, open an SSH tunnel on the
remote PC and keep the terminal running:

```bash
ssh -p 1122 -N -o ExitOnForwardFailure=yes \
  -L 127.0.0.1:8780:192.168.131.1:8780 robot@cpr-rooftop
```

In Foxglove Desktop, choose **Open connection** -> **Foxglove WebSocket**, enter `ws://127.0.0.1:8780`, and connect. Close the tunnel with `Ctrl+C` in its terminal when finished.

This option does not require ROS on the PC or a separate Zenoh tunnel. The SSH host must be able to reach `192.168.131.1:8780`; a bridge listening only on robot loopback would instead need a forwarding destination of `127.0.0.1:8780`. No additional externally forwarded port is needed if SSH TCP forwarding is permitted.

### Troubleshooting

- `could not determine namespace`: wrong middleware or domain ID, or the robot is unreachable (Zenoh: port 7447; Cyclone DDS: UDP).
- `already maps to ... in /etc/hosts`: fix the conflicting entry in `/etc/hosts`.
- Use a new shell per robot, since the script exports environment variables.
