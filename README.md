
## Installation
#### Prerequisites

- ROS2 Jazzy
- Tested on Ubuntu 24.04

### Updating

```bash
  sudo apt update
  rosdep update
  rosdep install --from-paths src --ignore-src -r -y
```

### Installing Dependencies

```bash
  sudo apt insall tmux
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
  mkdir build && cd build
  cmake ..
  make
  sudo make install
  cd ../..
```

### Building The Simualtor

```bash
  colcon build --symlink-install
  source install/setup.bash
```

### Running DEMO Script

```bash
  cd src/stonefish_sim/scripts
  bash gbr_keyboard_demo
```