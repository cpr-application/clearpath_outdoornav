<img src="../images/clearpath_robotics_by_ra.png" alt="Clearpath Logo" width="50%" />&nbsp;

# Build/Install/Run on Robot PC

## Install

```
cd ~
mkdir -p onav_ws/src
cd ~/onav_ws/src/
git clone https://github.com/cpr-application/clearpath_outdoornav.git
cd ~/onav_ws/
rosdep install --from-paths src --ignore-src -r -y
cd ~/onav_ws/ && colcon build --packages-up-to clearpath_outdoornav_msgs
```

## Build

```
cd ~/onav_ws/ && colcon build --packages-up-to clearpath_outdoornav_cpp_api
```

### Run API Endpoint Example

```
source ~/onav_ws/install/setup.bash
ros2 run clearpath_outdoornav_cpp_api <endpoint_example>
```

where <example_endpoint> is the name of one of the endpoint examples (See [setup.py](setup.py) for the list of all available endpoint and application examples)

### Run API Application Example

```
source ~/onav_ws/install/setup.bash
ros2 run clearpath_outdoornav_cpp_api <application_example>
```

where <application_example> is the name of one of the application examples (See [setup.py](setup.py) for the list of all available endpoint and application examples)


# Build/Install/Run on Remote PC
