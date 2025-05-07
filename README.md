
## Installation
#### Prerequisites

- ROS2 Jazzy
- Tested on Ubuntu 24.04

### Updating

```bash
  sudo apt update
```
```bash
  rosdep update
```
### Installing Dependencies

```bash
  sudo apt install tmux
```
```bash
  sudo apt install libglm-dev
```
```bash
  pip install pynput --break-system-packages
```

### Clone Repository

SSH:
```bash
  git clone --recurse-submodules git@github.com:Joranikus/stonefish_simulator.git
```
HTTPS:
```bash
  git clone --recurse-submodules https://github.com/Joranikus/stonefish_simulator.git
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
```bash
  cd ../../..
```

### Building The Simualtor

```bash
  colcon build --symlink-install
```
```bash
  source install/setup.bash
```

### Running DEMO Script

```bash
  cd src/stonefish_sim/scripts
```
```bash
  bash gbr_keyboard_demo
```