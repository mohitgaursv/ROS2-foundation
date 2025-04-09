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
    
#### 🐍  ROS 2 Python Node with additional functionalities- Execute my function for every x amount of time(Timer Node)
This is a simple ROS 2 Python node that prints "Hello" followed by a counter every second using a timer callback. It's a great starting point to learn how to create nodes in ROS 2 using Python.
```
#!/usr/bin/env python3

# 🌟 Importing ROS 2 core client libraries
import rclpy
from rclpy.node import Node

# 🚀 Define a custom node class
class MyNode(Node):
    def __init__(self):
        # 🧠 Initialize the node with the name 'py_test'
        super().__init__("py_test")

        # ✅ Print a welcome message once when node starts
        self.get_logger().info("✨ Hello World from MyNode ✨")

        # 🔢 Initialize the counter
        self.counter_ = 0

        # ⏱️ Set up a timer to call 'timer_callback' every 1 second
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        # 💬 Log "Hello <counter>" every second
        self.get_logger().info(f"👋 Hello {self.counter_}")
        self.counter_ += 1

# 🎯 Entry point of the program
def main(args=None):
    # 🛠️ Initialize the rclpy library
    rclpy.init(args=args)

    # 🧩 Create an instance of MyNode
    node = MyNode()

    # 🔄 Keep the node alive and responsive
    rclpy.spin(node)

    # 📴 Shutdown the node gracefully
    rclpy.shutdown()

# 🚪 Start execution if run as a script
if __name__ == "__main__":
    main()

```
## Section 3: Introduction to ROS2 tools
1~✨ Let us introduce few basic ROS2 terminal commands 
```bashrc
# 📄 Show all sourced files in your shell
cat ~/.bashrc
# (Look for lines like: `source /opt/ros/humble/setup.bash` or `source ~/ros2_ws/install/setup.bash`)

# 🚀 Run a ROS 2 package (e.g., demo_nodes_cpp) and explore available executables
ros2 run demo_nodes_cpp <TAB>
# (Press Tab after typing the package to see all runnable demos)

# 📚 Get help for ros2 run
ros2 run -h

# 📡 List all currently active nodes
ros2 node list

# 🔍 Get detailed information about a specific node
ros2 node info /<node_name>
# Example:
ros2 node info /turtlesim

```

2~✨ How to Change the Name of a Running Node in ROS 2

To change the name of a running node, you can use the --ros-args option with the -r remapping rule. Here’s how:
```bash
# Change the node name to 'heystalker'
ros2 run my_py_pkg py_node --ros-args -r __node:=heystalker
```
🧠 Explanation:

ros2 run runs the specified ROS 2 node, where my_py_pkg is the package name, and py_node is the executable within the package. The --ros-args flag is used to pass ROS-specific arguments, and -r __node:=heystalker remaps the node name to heystalker.


3~✨ Turtlesim - Simulate a Robot in ROS 2

The Turtlesim package is a great way to start experimenting with ROS 2 concepts and visualizing robot motion.
```bash
# Install the Turtlesim package for ROS 2 Humble
sudo apt install ros-humble-turtlesim
# Launch the Turtlesim simulation (a window will appear with a turtle)
ros2 run turtlesim turtlesim_node
# Launch the teleoperation node to control the turtle with the keyboard
ros2 run turtlesim turtle_teleop_key
```
🧠Explanation: This allows you to control the turtle using your keyboard (with arrow keys or W, A, S, D).

4~✨ RQT graph - use rqt_graph to view all active nodes/debug nodes.


Exercise Goal:

The core task is to use rqt_graph to visualize a specific arrangement of ROS 2 nodes. This will likely involve creating custom nodes and potentially organizing them within ROS 2 packages to match a provided (or implied) diagram.
![Screenshot from 2025-04-09 14-04-11](https://github.com/user-attachments/assets/c635e22b-620f-4536-8455-d3245ad11212)

## Section 4: ROS 2 Topics - Make Your Nodes Communicate Between Each Other. 
1~✨ What is a ROS2 Topic?

![image](https://github.com/user-attachments/assets/8a212b7b-3a59-4a05-9918-acf37701f1a1)

📡 Publisher-Subscriber Model (ROS Concept)

The image above demonstrates the Publisher-Subscriber architecture using a radio broadcast analogy.
🧠 Concept

    Topic: Acts as a named channel (e.g., 98.7 FM) through which data is transmitted. Publishers send data to a topic, and subscribers listen to it.

    Publisher: A node that sends data/messages to a topic. In the image, these are represented by the radio towers.

    Subscriber: A node that receives data/messages from a topic. These are the devices (like a mobile phone, car radio, or boombox) that are tuned in to the same frequency.

💡 Key Points

    Multiple publishers can publish to the same topic.

    Multiple subscribers can listen to the same topic.

    Publishers and subscribers are decoupled — they don’t need to know about each other.

    Topics ensure scalable and flexible communication in a distributed system like ROS.

📦 Real-world Example in ROS

    A LiDAR sensor (Publisher) publishes point cloud data to the topic /lidar_points.

    A visualization tool (Subscriber) subscribes to /lidar_points to render the environment.

   
2~✨ Write a Python Publisher?


