# arm_ws — Panda MoveIt (ROS 2) Workspace

This repository contains a minimal ROS 2 workspace for the Franka Emika **Panda** robot using **MoveIt 2**. It provides the robot description and a ready-to-run planning setup in RViz.

> Note: This is a *motion planning* demo (no physics simulator by default). You can add Gazebo/Ignition later if you need dynamics.

---

## Project structure (what each folder/package does)

```
arm_ws/
├── src/
│   ├── panda_description/                 # URDF/Xacro for the Panda + meshes/textures
│   └── panda_moveit_config/               # MoveIt2 configuration for Panda
│       └── config/
│           ├── ompl_planning.yaml         # Global planning algorithms/params (OMPL)
│           ├── moveit_controllers.yaml    # MoveIt controller interface (fake/real)
│           ├── ros2_controllers.yaml      # ros2_control controllers (fake by default)
│           ├── chomp_planning.yaml        # (Optional) CHOMP planner params
│           ├── pilz_cartesian_limits.yaml # (Optional) Pilz Cartesian limits
│           └── ...                        # Other kinematics/limits/config YAMLs
├── .gitignore
└── README.md
```

Key ideas
- **panda_description** defines links/joints and collision/visual geometry.
- **panda_moveit_config** defines kinematics, planning pipeline, controller stubs, and RViz defaults for the MoveIt MotionPlanning plugin.
- The `ompl_planning.yaml` selects the default planner and its parameters; other planners (CHOMP/Pilz) are present but *disabled by default* unless you switch the planning pipeline.

---

## How to run the demo

### Prerequisites
- Ubuntu 22.04
- ROS 2 **Humble** (or newer) installed and sourced
- MoveIt 2 installed (Debian packages are fine)

Quick install (Humble):
```bash
sudo apt update
sudo apt install -y ros-humble-moveit   ros-humble-moveit-resources-panda-description   ros-humble-moveit-resources-panda-moveit-config
```

### Build this workspace
```bash
# 1) Get the workspace
mkdir -p ~/arm_ws/src && cd ~/arm_ws/src
git clone https://github.com/Feic2/arm_ws.git .
cd ~/arm_ws

# 2) Resolve dependencies and build
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### Launch MoveIt (RViz MotionPlanning demo)
via the MoveIt tutorials demo launch (uses this Panda config):
```bash
ros2 launch moveit2_tutorials demo.launch.py
```

## Brief algorithm descriptions

### Path planner used
- **Planning pipeline:** `ompl` (Open Motion Planning Library).
- **Default algorithm:** **RRTConnect** (geometric variant) configured in `config/ompl_planning.yaml`.
- Why this choice: RRTConnect usually gives fast, feasible paths for high-DOF arms and is the MoveIt default for Panda. Other OMPL planners (e.g., PRM, RRT*, KPIECE) are available via the same file if you want to experiment.
- Typical flow: OMPL plans a collision-free path in joint space using the Panda’s kinematic model and limits defined in the MoveIt config.

### Obstacle avoidance mechanism
- **Collision checking:** MoveIt uses FCL-based collision detection against the **Planning Scene**, which includes:
  - the Panda’s collision geometry (from URDF/SRDF), and
  - any environment objects added to the scene.
- **Safety margin:** The SRDF’s Allowed Collision Matrix and link padding control which contacts are permitted and how conservative avoidance is.


