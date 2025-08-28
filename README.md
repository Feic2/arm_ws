# arm_ws — Panda MoveIt (ROS 2) Workspace

This repository contains a minimal ROS 2 workspace for the Franka Emika **Panda** robot using **MoveIt 2**. It provides the robot description and a ready-to-run planning setup in RViz (with fake controllers).

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
Option A — via the MoveIt tutorials demo launch (uses this Panda config):
```bash
ros2 launch moveit2_tutorials demo.launch.py
```
Option B — bring up MoveIt with fake controllers using this repo’s config (if you have a local launch file):
```bash
# Example if you add a launch/ folder later:
ros2 launch panda_moveit_config demo.launch.py
```
Then in RViz:
- Add the **MotionPlanning** display if it isn’t present.
- Choose a Planning Group (e.g., `panda_arm` or `panda_manipulator`).
- Set a Goal State/Position and click **Plan** → **Execute**. The robot will move in RViz using *fake* controllers.

Troubleshooting
- If you don’t see meshes, ensure the Panda description package is found and the RViz Fixed Frame is `world` or `panda_link0`.
- If planning fails immediately, verify the Planning Scene has no red (in-collision) links and that `ompl` is the active planning pipeline.
- To reload the environment, restart RViz and the launch above.

---

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
- **Dynamic obstacles:** You can insert/remove objects at runtime (e.g., via the RViz *PlanningScene* panel or a node using `moveit_msgs/CollisionObject`). The planner replans to avoid new obstacles.
- **Execution:** With fake controllers, the plan is visualized/executed in RViz; with hardware/ros2_control, the same plan would produce `FollowJointTrajectory` commands.

---

## Notes & next steps
- To turn on **Pilz** (industrial Cartesian motions like LIN/CIRC), switch the planning pipeline to `pilz_industrial_motion_planner` and provide limits via the included YAMLs.
- To try **CHOMP**, enable the `chomp` pipeline and provide a good collision cost map; it optimizes a seed path for smoother, obstacle-cost-aware motion.
- For a physics simulator, add a `gazebo_ros2_control` setup and real controllers (replace fake ones in `ros2_controllers.yaml`).

---

## One-time Git/GitHub setup (so you can push this README)
```bash
git config --global user.name "Fei Chen"
git config --global user.email "Feic2@users.noreply.github.com"   # or your real GitHub email
git add README.md
git commit -m "Add detailed README for Panda MoveIt workspace"
git push -u origin main
```
