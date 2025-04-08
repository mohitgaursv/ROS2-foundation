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

Running the Minimal Python Node

Navigate to your ROS 2 package directory and create your first Python node:

```bash
cd ros2_ws/src/my_py_pkg/my_py_pkg

# List current contents (optional)
ls

# Create a new Python file for your node
touch my_first_node.py

# Go back to the workspace source directory
cd ../..

# Open the workspace in VS Code
code .
```
#### 🐍 Minimal ROS 2 Python Node

Below is a simple ROS 2 Python node that logs "Hello world" when run:

```python
#!/usr/bin/env python3  # Add the interpreter line to run the script directly

# 📦 Import necessary ROS 2 libraries
import rclpy
from rclpy.node import Node  # Import the Node class to create your custom node

def main(args=None):
    # 🔧 Initialize ROS 2 communication
    rclpy.init(args=args)

    # 🧠 Create a node with the name "py_test"
    node = Node("py_test")

    # 🗣️ Log a message to the terminal
    node.get_logger().info("Hello world")

   # 🗣️ Keep the node hanging by spinning it. Node is created and passed to spin functionality
    rclpy.spin(node)

    # 🛑 Shutdown the ROS 2 communication cleanly
    rclpy.shutdown()

# ▶️ Run the main function when the script is executed directly
if __name__ == "__main__":
    main()

```
### 🚀 Running the Python Node

Navigate back to your file location and execute your Python node:

```bash
# 🔙 Go to the package's Python source folder
cd ros2_ws/src/my_py_pkg/my_py_pkg

# 📂 List the contents (optional)
ls

# ✅ Make the Python file executable
chmod +x my_first_node.py

# ▶️ Run the Python node
./my_first_node.py


```
### ⚙️ Creating a ROS 2 Executable (Console Script)

We are creating an **executable named `py_node`**.

📁 Go to your `setup.py` file and scroll to the `entry_points` section.  
🔧 Under the `console_scripts` list, add the following line:

```python
'py_node = my_py_pkg.my_first_node:main',
```


  


