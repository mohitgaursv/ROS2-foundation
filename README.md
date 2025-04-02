# ROS 2 Setup and Basics

## Section 1: Install ROS 2 and Setup Your Environment

Follow the official ROS 2 installation guide for your operating system.

## Section 2: ROS Basics

### 1. Creating a ROS 2 Workspace

#### a) Navigate to the Home Directory

```bash
cd
```
#### b) Make a file named as ros2_ws
```bash
mkdir ros2_ws
```
#### c) Create a source file within ros2 workspace
```bash
mkdir src
```
#### d) Build your workspace in the ros2_ws
```bash
colcon build
```
#### e) Source your file after you have build your workspace in the bashrc
```bash
gedit .bashrc
source ~/ros2_ws/src/install/setup.bash
```
### 2. Create a  Package using Python and C++
Using Python
```bash
cd ros2_ws/src/
ros2 pkg create my_py_pkg --build-type ament_python --dependencies rclpy
```
#### a) Build the python package using colcon build or build particular package
```bash
cd ros2_ws/
colcon build
colcon build --package-select my_py_pkg
```
Using C++
```bash
cd ros2_ws/src/
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclpy

```


#### b) Build the C++ package using colcon build or build particular package
```bash
cd ros2_ws/
colcon build
colcon build --package-select my_cpp_pkg
```
### 3. ROS Nodes
## ROS 2 Nodes: Modular Components

In ROS 2, a **Node** is a fundamental building block. Think of it as a self-contained subprogram within your application, designed to perform a specific task. A node reside within a package.

**Key Characteristics:**

* **Single Responsibility:** Each node should focus on a single, well-defined function. This promotes modularity and maintainability.
    * **Analogy:** Similar to how a class in object-oriented programming encapsulates a specific set of functionalities, a ROS 2 node encapsulates a single purpose. If additional functionality is needed, create another node.
* **Communication:** Nodes interact with each other through various mechanisms:
    * **Topics:** For asynchronous, one-to-many message passing.
    * **Services:** For synchronous, request-response communication.
    * **Parameters:** For configuring node behavior at runtime.

**Example:**

Imagine a robot application. You might have separate nodes for:

* `camera_node`: Responsible for capturing and processing images from a camera.
* `navigation_node`: Handles path planning and robot movement.
* `lidar_node`: processes lidar data.

These nodes would communicate with each other using topics, services, and parameters to achieve the robot's overall functionality.

**Benefits of using Nodes:**

* **Modularity:** Easier to develop, test, and maintain individual components.
* **Reusability:** Nodes can be reused in different applications.
* **Scalability:** Allows for distributed processing across multiple machines.
* **Flexibility:** One node can be in Python and other in C++ and they both can communicate with each other easily.

#### 1) Write a Python node- Minimal Code

  


