<img src="../images/clearpath_robotics_by_ra.png" alt="Clearpath Logo" width="50%" />&nbsp;

## Install

```
cd ~
mkdir -p onav_ws/src
cd ~/onav_ws/src/
git clone -b jazzy https://github.com/cpr-application/clearpath_outdoornav.git
rosdep install --from-paths src --ignore-src -r -y
cd ~/onav_ws/ && colcon build --packages-up-to clearpath_outdoornav_msgs
```

## Build python package

```
cd ~/onav_ws/ && colcon build --packages-up-to clearpath_outdoornav_python_api
```

## Run an API Endpoint Example

```
source ~/onav_ws/install/setup.bash
ros2 launch clearpath_outdoornav_python_api <endpoint_example>
```

where <example_endpoint> is the name of one of the endpoint examples (See [setup.py](setup.py) for the list of all available endpoint and application examples)

## Run an API Application Example

```
source ~/onav_ws/install/setup.bash
ros2 launch clearpath_outdoornav_python_api <application_example>
```

where <application_example> is the name of one of the application examples (See [setup.py](setup.py) for the list of all available endpoint and application examples)
