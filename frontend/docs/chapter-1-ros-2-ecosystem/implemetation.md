# ROS 2 Implementation Guide: From Installation to Your First Robot Program

## Prerequisites

Before installing ROS 2, make sure you have:
- Ubuntu 22.04 (Jammy) or Windows 10/11 with WSL2
- At least 4GB of RAM (8GB recommended)
- At least 10GB of free disk space
- A stable internet connection

**Note**: For beginners, we recommend using Ubuntu 22.04 as it has the best ROS 2 support and community documentation.

## Installing ROS 2 Humble Hawksbill

ROS 2 Humble Hawksbill is the latest long-term support (LTS) version, making it perfect for learning and development.

### On Ubuntu 22.04

1. **Set up locale**:
```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
```

2. **Set up sources**:
```bash
sudo apt update && sudo apt install curl gnupg lsb-release
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo gpg --dearmor -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

3. **Install ROS 2**:
```bash
sudo apt update
sudo apt install ros-humble-desktop
```

4. **Install colcon and other tools**:
```bash
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update
```

5. **Source the ROS 2 environment**:
```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### On Windows with WSL2

1. Install WSL2 with Ubuntu 22.04
2. Follow the Ubuntu installation instructions above within your WSL terminal

## Creating Your First ROS 2 Workspace

A workspace is where you'll develop your ROS 2 packages. Let's create one:

```bash
# Create the workspace directory
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Source ROS 2 environment (if not already sourced)
source /opt/ros/humble/setup.bash

# Build the workspace (even though it's empty, this sets up the structure)
colcon build
```

## Creating Your First ROS 2 Package

Let's create a simple package called `my_robot_tutorials`:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_tutorials
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

## Creating Your First Publisher Node

Let's create a simple publisher node that sends "Hello Robot World!" messages:

1. **Create the Python file**:
```bash
cd ~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials
```

Create a file called `publisher_member_function.py`:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello Robot World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

2. **Make the file executable**:
```bash
chmod +x ~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/publisher_member_function.py
```

## Creating Your First Subscriber Node

Now let's create a subscriber that listens to the messages:

Create a file called `subscriber_member_function.py` in the same directory:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Make it executable:
```bash
chmod +x ~/ros2_ws/src/my_robot_tutorials/my_robot_tutorials/subscriber_member_function.py
```

## Building and Running Your Nodes

1. **Build your workspace**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

2. **Run the publisher in one terminal**:
```bash
ros2 run my_robot_tutorials publisher_member_function
```

3. **Run the subscriber in another terminal** (open a new terminal and source the setup file):
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_tutorials subscriber_member_function
```

You should now see the publisher sending messages and the subscriber receiving them!

## Understanding the ROS 2 Command Line Tools

ROS 2 provides several useful command-line tools:

### ros2 topic
- `ros2 topic list` - Show all active topics
- `ros2 topic echo <topic_name>` - Print messages from a topic
- `ros2 topic info <topic_name>` - Show information about a topic

### ros2 node
- `ros2 node list` - Show all active nodes
- `ros2 node info <node_name>` - Show information about a node

### ros2 service
- `ros2 service list` - Show all active services
- `ros2 service call <service_name> <service_type>` - Call a service

## Creating a Simple Service

Let's create a simple service that adds two numbers:

Create `add_two_ints_server.py`:

```python
from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f'Incoming request\na: {request.a}, b: {request.b}\n')
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

Create `add_two_ints_client.py`:

```python
from example_interfaces.srv import AddTwoInts
import sys
import rclpy
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(AddTwoInts, 'add_two_ints')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = AddTwoInts.Request()

    def send_request(self, a, b):
        self.req.a = a
        self.req.b = b
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(int(sys.argv[1]), int(sys.argv[2]))
    minimal_client.get_logger().info(
        f'Result of add_two_ints: {response.sum}')

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Running the Service

1. **Build the workspace**:
```bash
cd ~/ros2_ws
colcon build --packages-select my_robot_tutorials
source install/setup.bash
```

2. **Run the service server**:
```bash
ros2 run my_robot_tutorials add_two_ints_server
```

3. **In another terminal, run the client**:
```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_robot_tutorials add_two_ints_client 5 3
```

## Practical Exercises

### Exercise 1: Modify the Publisher
Modify the publisher to send different types of messages (e.g., temperature readings, sensor data) instead of "Hello Robot World".

### Exercise 2: Create a New Topic
Create a new topic that publishes random numbers and have a subscriber that calculates and prints the running average.

### Exercise 3: Service Enhancement
Enhance the add service to also subtract, multiply, and divide numbers based on a parameter.

### Exercise 4: Node Parameters
Add parameters to your nodes to control the publishing rate or other behaviors.

## Troubleshooting Common Issues

1. **Package not found**: Make sure you've sourced your workspace (`source install/setup.bash`)

2. **Permission denied**: Make sure your Python files are executable (`chmod +x filename.py`)

3. **No messages received**: Check that topic names match exactly between publisher and subscriber

4. **Import errors**: Ensure you have the correct message types installed (`sudo apt install ros-humble-common_interfaces`)

## Next Steps

Now that you've created your first ROS 2 nodes, try:
- Creating more complex message types
- Using launch files to run multiple nodes at once
- Exploring ROS 2 tools like RViz for visualization
- Learning about ROS 2 actions for long-running tasks

This implementation guide provides you with the foundation to start building more complex robotic applications. In the next chapter, we'll explore simulation environments where you can test your ROS 2 nodes with virtual robots.