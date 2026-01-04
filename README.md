# ERC, software part

## Getting started

After installing [WSL2 Ubuntu 24.04]() and [ROS2](),

1. Install WSL2 with Ubuntu 24.04: `wsl --install -d Ubuntu-24.04`
2. Install ROS2 in WSL:

```bash
# add ROS2 packages source to APT
sudo apt update && sudo apt install curl -y
export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb

# install ROS2
sudo apt update
sudo apt install ros-kilted-desktop

# install colcon mixins
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

3. Build this project:

```bash
# activate ROS2
source /opt/ros/kilted/setup.bash

# install dependencies
rosdep update
rosdep install --rosdistro kilted --from-paths --ignore-src --default-yes src

# actually build, if your computer is low on RAM then use --executor sequential
colcon build

# activate the project
source install/local_setup.bash
```

4. Launch the simulation:

```bash
# activate ROS2
source install/setup.bash

# Enable GPU in WSL2
export GALLIUM_DRIVER=d3d12

# You might or might not need to set this variable,
# it switches between integrated (Intel) and discrete (NVIDIA) GPU.
# Pick whatever makes it run faster.
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA

# launch the simulation
ros2 launch simulation simulation.launch.py
```
