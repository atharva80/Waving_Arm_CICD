Waving Arm: A ROS 2, Docker, and CI/CD Demonstration
This is a small, self-contained ROS 2 (Humble) project that demonstrates a professional DevOps
workflow. The project consists of a simple 2-link robotic arm (URDF/XACRO) that waves in RViz,
controlled by a custom Python node.
The primary goal of this repository is to serve as a portfolio piece demonstrating proficiency in:
ROS 2: A complete, working ROS 2 package with a node, launch file, and URDF/XACRO
description.
Containerization (Docker): A Dockerfile that packages the entire application, its
dependencies, and the ROS 2 environment into a single, reproducible image.
Continuous Integration (CI/CD): A GitHub Actions workflow that automatically builds and tests
the ROS 2 application and the Docker image on every push, ensuring code quality and deployment
readiness.
System Architecture
The system is composed of four main components, all launched by arm.launch.py :
1. robot_state_publisher : Reads the urdf/arm.xacro file, processes it, and publishes the
robot's structure (TF transforms) for all other nodes to use.
2. arm_waving_node (Custom Node): A simple Python node that calculates a sinusoidal wave angle
and continuously publishes it to the /joint_states topic.
3. rviz2 : Subscribes to the /tf and /joint_states topics to visualize the final, moving 3D
model of the robotic arm.
The data flow is as follows: arm_waving_node → /joint_states → robot_state_publisher →
/tf → rviz2
How to Run Locally
This project can be built and run on any machine with ROS 2 Humble installed.
1. Dependencies
Ensure you have the core ROS 2 Humble libraries ( desktop preset is recommended).
2. Build the Package
# Clone this repository into your ROS 2 workspace's src directory
cd ~/ros2_ws/src
git clone https://github.com/atharva80/Waving_Arm_CICD
# Navigate to the workspace root
cd ~/ros2_ws
# Install dependencies (optional, but good practice)
rosdep install -i --from-path src -y --rosdistro humble
# Build the package
colcon build --packages-select waving_arm
3. Run the Demonstration
# Source your workspace
source ~/ros2_ws/install/setup.bash
# Run the launch file
ros2 launch waving_arm arm.launch.py
This will open RViz. If the arm isn't visible, you may need to:
1. Set the Fixed Frame in the "Global Options" (top-left) to base_link .
2. Click "Add" (bottom-left) and add a RobotModel display.
3. Go to File > Save Config As... and save it as view_arm.rviz in the src/waving_arm/config/
folder, so it loads automatically next time.
You should now see the blue arm waving.
DevOps Pipeline: Docker & CI/CD
This project is configured for automated building and testing.
Containerization (Docker)
The included Dockerfile builds a self-contained image with all ROS 2 dependencies, project code,
and configurations. It is designed to be built by the CI pipeline.
You can also build it locally:
# From the repository root (e.g., ~/ros2_ws/src/waving_arm)
docker build -t waving_arm_demo .
Continuous Integration (GitHub Actions)
The .github/workflows/ci.yml file configures an automated pipeline that runs on every git push :
1. build_and_test Job:
Spins up an Ubuntu runner.
Installs ROS 2 Humble using ros-tooling/setup-ros .
Runs ros-tooling/action-ros-ci which automatically installs dependencies ( rosdep ),
builds the code ( colcon build ), and runs any tests ( colcon test ). This validates all ROS 2
code.
2. build_docker Job:
Runs after the build_and_test job succeeds.
Builds the Dockerfile using docker/build-push-action .
This validates that the project is not only buildable but also fully deployable as a container.
