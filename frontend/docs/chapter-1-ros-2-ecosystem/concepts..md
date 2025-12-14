# ROS 2 Core Concepts: Building Blocks of Robot Communication

## Deep Dive into ROS 2 Architecture

ROS 2 is built on a distributed architecture that allows multiple processes (nodes) to communicate with each other through a publish-subscribe model, services, and actions. Understanding these concepts is crucial for building effective robotic systems.

## 1. Nodes: The Fundamental Building Blocks

Nodes are the basic execution units in ROS 2 - they are individual programs that perform specific tasks and communicate with other nodes to achieve complex robot behaviors. Think of nodes as specialized workers in a factory, each responsible for a particular function.

### Key Characteristics of Nodes:
- **Process Isolation**: Each node runs in its own process, providing fault isolation
- **Communication Hub**: Nodes serve as interfaces between different parts of the robot system
- **Resource Management**: Each node manages its own memory, CPU, and I/O resources
- **Lifecycle Management**: Nodes can be started, stopped, paused, and configured independently

### Example: A Mobile Robot Node Structure
```
┌─────────────────────────────────────────────────────────┐
│                    Mobile Robot System                  │
├─────────────────┬─────────────────┬─────────────────────┤
│   Camera Node   │  Navigation Node│   Motor Control     │
│  (Image Capture) │ (Path Planning) │    Node (Motion)    │
└─────────────────┴─────────────────┴─────────────────────┘
         │                  │                   │
         └──────────────────┼───────────────────┘
                            │
                    ┌───────▼────────┐
                    │   Map Server   │
                    │ (SLAM/Mapping) │
                    └────────────────┘
```

### Advanced Node Example with Error Handling:
```python
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_msgs.msg import String
import threading
import time

class RobustRobotNode(Node):
    def __init__(self):
        super().__init__('robust_robot_node')

        # Initialize components with error handling
        try:
            # Create publishers and subscribers
            self.publisher = self.create_publisher(String, 'robot_status', 10)

            # Create a timer with specific callback group
            self.timer_callback_group = MutuallyExclusiveCallbackGroup()
            self.timer = self.create_timer(
                1.0,  # 1 second
                self.timer_callback,
                callback_group=self.timer_callback_group
            )

            # Log successful initialization
            self.get_logger().info("Robust Robot Node initialized successfully!")

        except Exception as e:
            self.get_logger().error(f"Failed to initialize node: {str(e)}")
            raise

    def timer_callback(self):
        """Timer callback function that publishes robot status"""
        try:
            msg = String()
            msg.data = f'Robot active at {time.time()}'
            self.publisher.publish(msg)
            self.get_logger().info(f'Published: "{msg.data}"')
        except Exception as e:
            self.get_logger().error(f"Error in timer callback: {str(e)}")

    def on_shutdown(self):
        """Cleanup function called during node shutdown"""
        self.get_logger().info("Shutting down RobustRobotNode...")
        # Perform cleanup operations here

def main(args=None):
    rclpy.init(args=args)

    try:
        robot_node = RobustRobotNode()

        # Use multi-threaded executor to handle multiple callbacks
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(robot_node)

        try:
            # Spin with error handling
            executor.spin()
        except KeyboardInterrupt:
            robot_node.get_logger().info("Interrupted by user")
        finally:
            robot_node.on_shutdown()
            executor.shutdown()
            robot_node.destroy_node()

    except Exception as e:
        print(f"Error in main: {str(e)}")
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 2. Topics: The Asynchronous Communication Backbone

Topics form the backbone of ROS 2's communication system, enabling asynchronous, many-to-many communication between nodes. They follow a publish-subscribe pattern where publishers send data without knowing who will receive it, and subscribers receive data without knowing who sent it.

### Topic Communication Patterns:

#### 1. One-to-Many (Broadcast)
```
Publisher ────┐
              ├───→ Multiple Subscribers
Publisher ────┘
```

#### 2. Many-to-One (Aggregation)
```
Multiple Publishers ────→ Single Subscriber
```

#### 3. Many-to-Many (Network)
```
Multiple Publishers ↔ Multiple Subscribers
```

### Quality of Service (QoS) Settings:

QoS profiles allow you to fine-tune communication behavior based on your application's needs:

```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# Real-time critical (e.g., motor commands)
real_time_qos = QoSProfile(
    depth=1,  # Keep only the latest message
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)

# Data logging (e.g., sensor data for analysis)
logging_qos = QoSProfile(
    depth=100,  # Keep many messages
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_ALL
)

# Best-effort (e.g., camera images where some loss is acceptable)
best_effort_qos = QoSProfile(
    depth=5,  # Small buffer
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST
)
```

### Advanced Topic Example with QoS:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image, LaserScan
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class AdvancedTopicNode(Node):
    def __init__(self):
        super().__init__('advanced_topic_node')

        # Create different QoS profiles for different use cases
        self.critical_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE
        )

        self.sensor_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )

        # Publishers with different QoS
        self.motor_cmd_publisher = self.create_publisher(
            String, 'motor_commands', self.critical_qos
        )

        self.sensor_data_publisher = self.create_publisher(
            String, 'sensor_data', self.sensor_qos
        )

        # Subscriber with matching QoS
        self.status_subscriber = self.create_subscription(
            String, 'robot_status', self.status_callback, self.critical_qos
        )

        self.get_logger().info("Advanced Topic Node initialized with QoS profiles")

    def status_callback(self, msg):
        self.get_logger().info(f"Received status: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedTopicNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 3. Services: Synchronous Request-Response Communication

Services provide synchronous communication where a client sends a request and waits for a response. This is ideal for operations that must complete before proceeding.

### When to Use Services:
- Configuration changes that must succeed before continuing
- Data requests where you need a specific response
- Operations that have a clear start and end
- Synchronous coordination between nodes

### Service Architecture:
```
Client Node                    Service Server Node
     │                               │
     │────── Request (Request.msg)───▶│
     │                               │
     │◀─── Response (Response.msg)───│
     │                               │
```

### Advanced Service Example:
```python
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts, Trigger
from std_srvs.srv import Empty
import time

class AdvancedServiceNode(Node):
    def __init__(self):
        super().__init__('advanced_service_node')

        # Multiple service servers
        self.add_srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_callback
        )

        self.complex_srv = self.create_service(
            Trigger,  # Built-in service type
            'complex_operation',
            self.complex_callback
        )

        # Service with custom logic
        self.reset_srv = self.create_service(
            Empty,  # No request/response data
            'reset_system',
            self.reset_callback
        )

        self.get_logger().info("Advanced Service Node initialized")

    def add_callback(self, request, response):
        """Add two integers with validation"""
        try:
            # Input validation
            if not isinstance(request.a, (int, float)) or not isinstance(request.b, (int, float)):
                response.success = False
                response.message = "Invalid input: a and b must be numbers"
                return response

            # Perform calculation
            result = request.a + request.b
            response.sum = result

            # Log operation
            self.get_logger().info(f'Calculated {request.a} + {request.b} = {result}')

            response.success = True
            response.message = f"Successfully calculated: {request.a} + {request.b} = {result}"

            return response

        except Exception as e:
            response.success = False
            response.message = f"Error during calculation: {str(e)}"
            return response

    def complex_callback(self, request, response):
        """Simulate a complex operation with progress"""
        try:
            self.get_logger().info("Starting complex operation...")

            # Simulate processing time
            for i in range(10):
                time.sleep(0.1)  # Simulate work
                self.get_logger().debug(f"Progress: {i*10}%")

            response.success = True
            response.message = "Complex operation completed successfully"
            self.get_logger().info("Complex operation finished")

            return response

        except Exception as e:
            response.success = False
            response.message = f"Complex operation failed: {str(e)}"
            return response

    def reset_callback(self, request, response):
        """Reset system to initial state"""
        try:
            self.get_logger().info("Resetting system...")

            # Perform reset operations here
            # Reset parameters, clear buffers, reinitialize components
            self.get_logger().info("System reset completed")

            return response  # Empty response for Empty service type

        except Exception as e:
            self.get_logger().error(f"Reset failed: {str(e)}")
            # Note: Empty service type doesn't support response fields

def main(args=None):
    rclpy.init(args=args)
    node = AdvancedServiceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 4. Actions: Long-Running Tasks with Feedback

Actions are designed for long-running operations that require continuous feedback and the ability to be canceled. They combine features of both topics and services.

### Action Components:
- **Goal**: The desired outcome (sent once)
- **Feedback**: Continuous updates during execution (sent multiple times)
- **Result**: Final outcome when completed (sent once)

### Action State Machine:
```
    [Goal Sent] → [ACTIVE] ──→ [SUCCEEDED/ABORTED/CANCELED]
                     │
                     ▼
                [EXECUTING]
                     │
                     ▼
                [FEEDBACK]
```

### When to Use Actions:
- Navigation to distant locations
- Complex manipulator movements
- Data processing tasks
- Any operation that takes significant time

### Action Example:
```python
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import time
import threading
from example_interfaces.action import Fibonacci

class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__('fibonacci_action_server')

        # Create action server
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            execute_callback=self.execute_callback,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info("Fibonacci Action Server initialized")

    def goal_callback(self, goal_request):
        """Accept or reject a goal"""
        self.get_logger().info(f"Received goal request: {goal_request.order}")
        # Accept all goals for this example
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Accept or reject a cancel request"""
        self.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the goal and provide feedback"""
        self.get_logger().info('Executing goal...')

        # Get the target order from the goal
        target_order = goal_handle.request.order
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Generate Fibonacci sequence
        for i in range(1, target_order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return Fibonacci.Result()

            # Calculate next Fibonacci number
            if i < len(feedback_msg.sequence):
                continue  # Skip if already calculated

            next_fib = feedback_msg.sequence[-1] + feedback_msg.sequence[-2]
            feedback_msg.sequence.append(next_fib)

            # Publish feedback
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')

            # Simulate processing time
            time.sleep(0.5)

        # Check if goal was canceled during execution
        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Goal canceled during execution')
            return Fibonacci.Result()

        # Goal completed successfully
        goal_handle.succeed()
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Result: {result.sequence}')

        return result

def main(args=None):
    rclpy.init(args=args)
    node = FibonacciActionServer()

    try:
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 5. Parameters: Dynamic Configuration System

Parameters in ROS 2 provide a way to configure nodes at runtime without restarting them. They're stored in a distributed parameter server system.

### Parameter Types:
- **Private Parameters**: Specific to a single node
- **Global Parameters**: Shared across the system
- **Declarative Parameters**: Declared at startup with default values
- **Dynamic Parameters**: Can be changed during runtime

### Parameter Example with Validation:
```python
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters, ListParameters

class ParameterNode(Node):
    def __init__(self):
        super().__init__('parameter_node')

        # Declare parameters with descriptors for validation
        self.declare_parameter(
            'robot_name',
            'default_robot',
            ParameterDescriptor(
                description='Name of the robot',
                type=ParameterType.PARAMETER_STRING
            )
        )

        self.declare_parameter(
            'max_speed',
            1.0,
            ParameterDescriptor(
                description='Maximum robot speed in m/s',
                type=ParameterType.PARAMETER_DOUBLE,
                additional_constraints='Must be positive and less than 10.0'
            )
        )

        self.declare_parameter(
            'safety_distance',
            0.5,
            ParameterDescriptor(
                description='Minimum safe distance from obstacles',
                type=ParameterType.PARAMETER_DOUBLE
            )
        )

        # Parameter change callback
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Timer to periodically check parameters
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info("Parameter Node initialized with validation")

    def parameter_callback(self, params):
        """Validate parameter changes"""
        for param in params:
            if param.name == 'max_speed':
                if param.value <= 0 or param.value > 10.0:
                    self.get_logger().error(f"Invalid max_speed: {param.value}. Must be > 0 and <= 10.0")
                    return SetParametersResult(successful=False, reason="max_speed out of range")
            elif param.name == 'safety_distance':
                if param.value <= 0:
                    self.get_logger().error(f"Invalid safety_distance: {param.value}. Must be > 0")
                    return SetParametersResult(successful=False, reason="safety_distance must be positive")

        self.get_logger().info("All parameter changes accepted")
        return SetParametersResult(successful=True)

    def timer_callback(self):
        """Periodically display current parameter values"""
        robot_name = self.get_parameter('robot_name').value
        max_speed = self.get_parameter('max_speed').value
        safety_distance = self.get_parameter('safety_distance').value

        self.get_logger().info(
            f"Current parameters - Robot: {robot_name}, "
            f"Max Speed: {max_speed} m/s, "
            f"Safety Distance: {safety_distance} m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = ParameterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## 6. Middleware (DDS): The Communication Foundation

DDS (Data Distribution Service) is the underlying middleware that powers ROS 2's communication. It provides:

### Core DDS Features:
- **Automatic Discovery**: Nodes automatically find each other on the network
- **Data-Centric Architecture**: Focus on data rather than communication endpoints
- **Quality of Service**: Configurable reliability, latency, and bandwidth settings
- **Platform Independence**: Works across different operating systems and hardware
- **Security**: Built-in authentication and encryption capabilities

### DDS QoS Policies:
- **Reliability**: Best effort vs. reliable delivery
- **Durability**: Volatile vs. persistent data
- **History**: Keep last N samples vs. keep all samples
- **Deadline**: Maximum time between samples
- **Liveliness**: How to detect if a publisher is alive

### Working with DDS in ROS 2:
```python
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.qos_event import SubscriptionEventCallbacks, PublisherEventCallbacks
from rclpy.event_handler import PublisherEventCallbacks, SubscriptionEventCallbacks

# Example of advanced QoS configuration
def create_robust_qos_profile():
    """Create a QoS profile for critical communications"""
    return QoSProfile(
        depth=10,  # Number of messages to buffer
        reliability=ReliabilityPolicy.RELIABLE,  # Ensure delivery
        durability=DurabilityPolicy.TRANSIENT_LOCAL,  # Keep for late-joining subscribers
        history=HistoryPolicy.KEEP_LAST,  # Keep only the most recent messages
        lifespan=rclpy.duration.Duration(seconds=30),  # Message lifespan
        deadline=rclpy.duration.Duration(seconds=1),  # Maximum time between messages
    )
```

## Integration: How All Components Work Together

Here's a comprehensive example showing how all ROS 2 concepts integrate in a real robot system:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Float64
from std_srvs.srv import Trigger
from example_interfaces.action import Fibonacci
from rclpy.parameter import Parameter

class IntegratedRobotNode(Node):
    def __init__(self):
        super().__init__('integrated_robot_node')

        # 1. Parameters - Configuration system
        self.declare_parameter('robot_mode', 'idle')
        self.declare_parameter('max_velocity', 1.0)

        # 2. Topics - Continuous communication
        qos_profile = QoSProfile(depth=10)
        self.status_publisher = self.create_publisher(String, 'robot_status', qos_profile)
        self.velocity_publisher = self.create_publisher(Float64, 'current_velocity', qos_profile)
        self.command_subscriber = self.create_subscription(
            String, 'robot_command', self.command_callback, qos_profile
        )

        # 3. Services - Synchronous operations
        self.emergency_stop_service = self.create_service(
            Trigger, 'emergency_stop', self.emergency_stop_callback
        )

        # 4. Actions - Long-running tasks
        self.navigation_action_server = ActionServer(
            self, Fibonacci, 'navigate_to_goal', self.navigate_callback
        )

        # 5. Timer - Regular updates
        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info("Integrated Robot Node initialized with all ROS 2 concepts")

    def command_callback(self, msg):
        """Handle incoming commands via topic"""
        self.get_logger().info(f"Received command: {msg.data}")
        # Process command and update internal state

    def emergency_stop_callback(self, request, response):
        """Handle emergency stop via service"""
        self.get_logger().warn("EMERGENCY STOP ACTIVATED!")
        # Execute emergency stop procedures
        response.success = True
        response.message = "Emergency stop executed"
        return response

    async def navigate_callback(self, goal_handle):
        """Handle navigation via action"""
        self.get_logger().info("Starting navigation action")
        # Execute navigation with feedback
        goal_handle.succeed()
        return Fibonacci.Result()

    def control_loop(self):
        """Main control loop publishing status updates"""
        status_msg = String()
        status_msg.data = f"Mode: {self.get_parameter('robot_mode').value}"
        self.status_publisher.publish(status_msg)

def main(args=None):
    rclpy.init(args=args)
    node = IntegratedRobotNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Understanding these core concepts and how they work together is fundamental to building robust, scalable robotic systems with ROS 2. Each component serves a specific purpose and choosing the right communication pattern for your use case is crucial for system performance and reliability.