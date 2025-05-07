# Sub-Horizon Stonefish Simulator

## Installation
#### Prerequisites

- ROS2 Jazzy
- Tested on Ubuntu 24.04

### Clone Repository

The cloned GIT is the ROS2 workspace, you dont need to make you own workspace.

SSH:
```bash
  git clone --recurse-submodules git@github.com:Joranikus/stonefish_simulator.git
```
HTTPS:
```bash
  git clone --recurse-submodules https://github.com/Joranikus/stonefish_simulator.git
```

### Updating

```bash
  sudo apt update && rosdep update
```
### Installing Dependencies

```bash
  sudo apt install -y tmux libglm-dev python3-pynput
```

### Building Stonefish

```bash
  cd stonefish_simulator/include/stonefish
```
```bash
  mkdir build && cd build
```
```bash
  cmake ..
```
```bash
  make -j$(nproc)
```
```bash
  sudo make install
```
Got to git root folder:
```bash
  cd ../../..
```

### Building The Simualtor

Needs to be run in git root:
```bash
  colcon build --symlink-install
```
```bash
  source install/setup.bash
```

### Running DEMO Script

Needs to be run in git root:
```bash
  bash src/stonefish_sim/scripts/gbr_keyboard_demo.sh
```