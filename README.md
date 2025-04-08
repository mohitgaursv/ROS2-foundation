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
#### 🚀 Running the Python Node

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
#### ⚙️ Creating a ROS 2 Executable (Console Script)

We are creating an **executable named `py_node`**.

📁 Go to your `setup.py` file and scroll to the `entry_points` section.  
🔧 Under the `console_scripts` list, add the following line:

```python
'py_node = my_py_pkg.my_first_node:main',
```

#### 🏗️ Build the Package

Since the executable has been created, we now need to **build the package**:

```bash
# 🔨 Build only the selected package
colcon build --packages-select my_py_pkg

# 🔙 Go to home directory (optional)
cd
```
#### 🔄 Source the Workspace


Since we built something new, we need to **source the workspace** so the environment is aware of the changes:

```
source install/setup.bash
```

#### 🔄 Run the executable directly from the terminal
🧠 Terminology

    Node Name:
    This is the name you define inside your Python or C++ ROS 2 file using rclpy.create_node() (Python) or Node("name") (C++).
    Example:
    ``` 
    node = Node("my_node_name")  # <- This is the Node Name
    ```
    Executable Name:
This is the name you assign in your setup.py or CMakeLists.txt and use with ROS 2 CLI commands like ros2 run.
Example:
```
ros2 run my_py_pkg my_executable_name
```
🧑‍💻 Example: Running the Node

To run the node from the terminal, use the following command:


```bash
ros2 run my_py_pkg py_node
```
#### 🐍 Minimal ROS 2 Python Node with OOPs
```
#!/usr/bin/env python3  # Add the interpreter line to run the script directly

# 📦 Import necessary ROS 2 libraries
import rclpy
from rclpy.node import Node  # Import the Node class to create your custom node

# 🧠 Define the custom node class
class Mynode(Node):
    def __init__(self):
        super().__init__("py_test")  # Initialize the parent Node class with name "py_test"
        self.get_logger().info("Hello World")  # 🗣️ Log a message to the terminal

def main(args=None):
    # 🔧 Initialize ROS 2 communication
    rclpy.init(args=args)

    # 🧠 Create an instance of Mynode (our custom node class)
    node = Mynode()

    # 🗣️ Keep the node running by spinning it
    rclpy.spin(node)

    # 🛑 Shutdown the ROS 2 communication cleanly
    rclpy.shutdown()

# ▶️ Run the main function when the script is executed directly
if __name__ == "__main__":
    main()
```

#### 🏗️ Explanation of python code with OOPs
🧠 Object-Oriented Programming (OOP) Concepts in Mynode Class

    Class: Mynode
    The Mynode class is a custom class that defines the behavior of a ROS 2 node. It inherits functionality from the base Node class and extends it by adding custom behavior, such as logging a message.

    Inheritance: Mynode inherits from Node
    The Mynode class inherits from the Node class, enabling it to leverage built-in ROS 2 node functionality (e.g., communication, logging). This allows the custom node to maintain ROS 2's standard features while adding specific behavior like logging a "Hello World" message.

    Constructor (__init__)
    The __init__ method is the constructor for the Mynode class. It is automatically invoked when an instance of Mynode is created. The constructor is used to initialize the node, configure settings (e.g., setting the node's name), and perform setup tasks such as logging a message.

    Object Creation
    In the main function, an instance of the Mynode class is created. This instance represents the actual node running in the ROS 2 system. The node will perform the behavior defined in the Mynode class, such as logging messages and spinning to keep the node alive.

    Encapsulation
    All node-specific logic (such as logging and spinning) is encapsulated within the Mynode class. This ensures that the code is modular, making it easier to maintain, extend, and reuse. The class handles all aspects of node functionality, isolating the details from other parts of the system.

