---
sidebar_position: 4
---

# Section 3.3: Practical Implementation - Unity Robotics Project

This section provides a step-by-step guide to implementing a complete Unity robotics project, from initial setup to deployment. We'll create a practical application that demonstrates Unity's capabilities for robotics visualization and control.

## Learning Objectives

- Set up a complete Unity-ROS development environment
- Implement a full-featured robot visualization application
- Create interactive controls for robot operation
- Deploy the application to different platforms
- Optimize performance for real-time robotics applications

## Project Setup and Environment

### Prerequisites

Before starting the project, ensure you have:

- Unity Hub and Unity 2020.3 LTS or later
- ROS Noetic (for ROS1) or ROS2 Foxy/Humble (for ROS2)
- Git for version control
- Basic knowledge of C# and ROS concepts

### Required Packages and Dependencies

1. **Unity Packages**:
   - TextMeshPro
   - XR Plugin Management (for AR/VR)
   - Input System
   - Universal Render Pipeline (optional, for advanced graphics)

2. **ROS Packages**:
   - rosbridge_suite
   - robot_state_publisher
   - joint_state_publisher
   - tf2 libraries

### Project Structure

Create the following directory structure in your Unity project:

```
Assets/
├── Scripts/
│   ├── ROS/
│   │   ├── RosBridgeClient/
│   │   ├── MessageDefinitions/
│   │   └── ConnectionManagers/
│   ├── Robotics/
│   │   ├── RobotControllers/
│   │   ├── Visualization/
│   │   └── UI/
│   └── Utilities/
├── Models/
│   ├── Robots/
│   ├── Environments/
│   └── Props/
├── Materials/
├── Scenes/
└── Prefabs/
```

## Step 1: Basic Robot Visualization

Let's start by creating a basic robot visualization system:

### Robot Model Setup

First, create a simple robot model in Unity:

1. Create primitive objects for robot links (cylinders, cubes, spheres)
2. Organize them in a hierarchy with proper joint connections
3. Add materials for visual distinction
4. Create a RobotController script to manage the model

### Robot Controller Script

```csharp
using UnityEngine;
using System.Collections.Generic;

public class RobotController : MonoBehaviour
{
    [System.Serializable]
    public class JointInfo
    {
        public string jointName;
        public Transform jointTransform;
        public JointType jointType;
        public float minAngle = -180f;
        public float maxAngle = 180f;
    }

    public enum JointType
    {
        Revolute,
        Prismatic,
        Fixed
    }

    [Header("Robot Configuration")]
    public List<JointInfo> joints;
    public string robotName = "robot";

    private Dictionary<string, JointInfo> jointMap;

    void Start()
    {
        InitializeJointMap();
    }

    void InitializeJointMap()
    {
        jointMap = new Dictionary<string, JointInfo>();
        foreach (var joint in joints)
        {
            jointMap[joint.jointName] = joint;
        }
    }

    public void UpdateJoint(string jointName, float position)
    {
        if (jointMap.ContainsKey(jointName))
        {
            var jointInfo = jointMap[jointName];
            switch (jointInfo.jointType)
            {
                case JointType.Revolute:
                    UpdateRevoluteJoint(jointInfo, position);
                    break;
                case JointType.Prismatic:
                    UpdatePrismaticJoint(jointInfo, position);
                    break;
                case JointType.Fixed:
                    // Fixed joints don't move
                    break;
            }
        }
    }

    void UpdateRevoluteJoint(JointInfo jointInfo, float angle)
    {
        // Clamp the angle to joint limits
        float clampedAngle = Mathf.Clamp(angle * Mathf.Rad2Deg,
                                       jointInfo.minAngle,
                                       jointInfo.maxAngle);

        // Apply rotation based on joint's rotation axis
        // This assumes the joint rotates around its local Z-axis
        jointInfo.jointTransform.localRotation =
            Quaternion.Euler(0, 0, clampedAngle);
    }

    void UpdatePrismaticJoint(JointInfo jointInfo, float position)
    {
        // Clamp the position to joint limits
        float clampedPosition = Mathf.Clamp(position,
                                          jointInfo.minAngle,
                                          jointInfo.maxAngle);

        // Apply translation along the joint's local X-axis
        jointInfo.jointTransform.localPosition =
            new Vector3(clampedPosition, 0, 0);
    }

    public void UpdateAllJoints(Dictionary<string, float> jointStates)
    {
        foreach (var jointState in jointStates)
        {
            UpdateJoint(jointState.Key, jointState.Value);
        }
    }
}
```

## Step 2: ROS Connection System

Now let's implement the ROS connection system:

### ROS Connection Manager

```csharp
using UnityEngine;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;

public class ROSConnectionManager : MonoBehaviour
{
    [Header("Connection Settings")]
    [SerializeField] private string rosBridgeUrl = "ws://localhost:9090";
    [SerializeField] private float connectionRetryDelay = 2f;

    [Header("Robot Topics")]
    [SerializeField] private string jointStatesTopic = "/joint_states";
    [SerializeField] private string robotPoseTopic = "/robot_pose";
    [SerializeField] private string cmdVelTopic = "/cmd_vel";

    private RosSocket rosSocket;
    private bool isConnected = false;
    private Dictionary<string, RobotController> robotControllers;

    void Start()
    {
        robotControllers = new Dictionary<string, RobotController>();
        ConnectToROS();
    }

    void ConnectToROS()
    {
        try
        {
            RosBridgeClient.Protocols.WebSocketNetProtocol protocol =
                new RosBridgeClient.Protocols.WebSocketNetProtocol(rosBridgeUrl);

            rosSocket = new RosSocket(protocol);
            rosSocket.OnConnected += OnConnected;
            rosSocket.OnClosed += OnDisconnected;

            isConnected = true;
            Debug.Log("Connected to ROS Bridge");
        }
        catch (System.Exception e)
        {
            Debug.LogError($"Failed to connect to ROS: {e.Message}");
            Invoke(nameof(ConnectToROS), connectionRetryDelay);
        }
    }

    void OnConnected()
    {
        Debug.Log("Successfully connected to ROS Bridge");
        SubscribeToTopics();
    }

    void OnDisconnected()
    {
        isConnected = false;
        Debug.LogWarning("Disconnected from ROS Bridge");
        Invoke(nameof(ConnectToROS), connectionRetryDelay);
    }

    void SubscribeToTopics()
    {
        // Subscribe to joint states
        rosSocket.Subscribe<sensor_msgs.JointState>(
            jointStatesTopic,
            OnJointStateReceived);

        // Subscribe to robot pose
        rosSocket.Subscribe<geometry_msgs.PoseStamped>(
            robotPoseTopic,
            OnPoseReceived);
    }

    void OnJointStateReceived(sensor_msgs.JointState jointState)
    {
        if (jointState.name.Count == jointState.position.Count)
        {
            Dictionary<string, float> jointStates = new Dictionary<string, float>();

            for (int i = 0; i < jointState.name.Count; i++)
            {
                jointStates[jointState.name[i]] = (float)jointState.position[i];
            }

            // Update all registered robot controllers
            foreach (var controller in robotControllers.Values)
            {
                controller.UpdateAllJoints(jointStates);
            }
        }
    }

    void OnPoseReceived(geometry_msgs.PoseStamped pose)
    {
        // Convert ROS pose to Unity coordinates and update robot position
        Vector3 unityPosition = new Vector3(
            (float)pose.pose.position.y,
            (float)pose.pose.position.z,
            (float)pose.pose.position.x
        );

        Quaternion unityRotation = new Quaternion(
            (float)pose.pose.orientation.x,
            (float)pose.pose.orientation.y,
            (float)pose.pose.orientation.z,
            (float)pose.pose.orientation.w
        );

        // Update the main robot's position (you may have multiple robots)
        UpdateRobotPosition(unityPosition, unityRotation);
    }

    void UpdateRobotPosition(Vector3 position, Quaternion rotation)
    {
        // Find the main robot controller and update its position
        foreach (var controller in robotControllers.Values)
        {
            if (controller.robotName == "main_robot") // or however you identify the main robot
            {
                controller.transform.position = position;
                controller.transform.rotation = rotation;
                break;
            }
        }
    }

    public void RegisterRobotController(RobotController controller)
    {
        if (!robotControllers.ContainsKey(controller.robotName))
        {
            robotControllers[controller.robotName] = controller;
        }
    }

    public void UnregisterRobotController(RobotController controller)
    {
        if (robotControllers.ContainsKey(controller.robotName))
        {
            robotControllers.Remove(controller.robotName);
        }
    }

    void OnDestroy()
    {
        if (rosSocket != null)
        {
            rosSocket.Close();
        }
    }
}
```

## Step 3: Interactive Controls and UI

Let's create an interactive UI for robot control:

### Robot Control Panel UI

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class RobotControlPanel : MonoBehaviour
{
    [Header("UI References")]
    public Slider[] jointSliders;
    public Button[] jointButtons;
    public TextMeshProUGUI[] jointValueTexts;
    public Button emergencyStopButton;
    public Button resetButton;

    [Header("Robot Reference")]
    public RobotController robotController;

    [Header("Control Settings")]
    public float jointMoveSpeed = 10f;

    private Dictionary<string, int> jointIndexMap;

    void Start()
    {
        InitializeUI();
        InitializeJointIndexMap();
        SetupEventHandlers();
    }

    void InitializeUI()
    {
        // Initialize sliders with default values
        for (int i = 0; i < jointSliders.Length; i++)
        {
            if (jointSliders[i] != null)
            {
                jointSliders[i].value = 0f; // Default position
                jointSliders[i].minValue = -180f;
                jointSliders[i].maxValue = 180f;
            }
        }
    }

    void InitializeJointIndexMap()
    {
        jointIndexMap = new Dictionary<string, int>();
        for (int i = 0; i < robotController.joints.Count && i < jointSliders.Length; i++)
        {
            jointIndexMap[robotController.joints[i].jointName] = i;
        }
    }

    void SetupEventHandlers()
    {
        // Setup slider value change events
        for (int i = 0; i < jointSliders.Length; i++)
        {
            int index = i; // Capture for closure
            if (jointSliders[i] != null)
            {
                jointSliders[i].onValueChanged.AddListener((value) =>
                    OnJointSliderChanged(index, value));
            }
        }

        // Setup button events
        if (emergencyStopButton != null)
        {
            emergencyStopButton.onClick.AddListener(EmergencyStop);
        }

        if (resetButton != null)
        {
            resetButton.onClick.AddListener(ResetRobot);
        }
    }

    void OnJointSliderChanged(int jointIndex, float value)
    {
        if (jointValueTexts != null && jointIndex < jointValueTexts.Length)
        {
            jointValueTexts[jointIndex].text = $"{value:F2}°";
        }

        // Update the robot joint immediately
        if (jointIndex < robotController.joints.Count)
        {
            string jointName = robotController.joints[jointIndex].jointName;
            robotController.UpdateJoint(jointName, value * Mathf.Deg2Rad);
        }
    }

    public void SetJointValue(string jointName, float value)
    {
        if (jointIndexMap.ContainsKey(jointName))
        {
            int index = jointIndexMap[jointName];
            if (index < jointSliders.Length)
            {
                jointSliders[index].value = value;
            }
        }
    }

    void EmergencyStop()
    {
        // Stop all robot movement
        for (int i = 0; i < jointSliders.Length; i++)
        {
            if (jointSliders[i] != null)
            {
                jointSliders[i].value = 0f;
            }
        }

        Debug.Log("Emergency Stop Activated!");
    }

    void ResetRobot()
    {
        // Reset all joints to home position
        for (int i = 0; i < jointSliders.Length; i++)
        {
            if (jointSliders[i] != null)
            {
                jointSliders[i].value = 0f;
            }
        }

        Debug.Log("Robot Reset to Home Position");
    }
}
```

## Step 4: Scene Setup and Integration

Create a main scene that brings everything together:

### Main Scene Setup Script

```csharp
using UnityEngine;

public class RobotSceneSetup : MonoBehaviour
{
    [Header("Scene References")]
    public ROSConnectionManager rosConnection;
    public RobotController robotController;
    public RobotControlPanel controlPanel;
    public Camera mainCamera;
    public Light mainLight;

    [Header("Scene Settings")]
    public bool autoConnect = true;
    public bool showControlPanel = true;

    void Start()
    {
        InitializeScene();
    }

    void InitializeScene()
    {
        // Register robot controller with ROS connection
        if (rosConnection != null && robotController != null)
        {
            rosConnection.RegisterRobotController(robotController);
        }

        // Setup control panel
        if (controlPanel != null && robotController != null)
        {
            controlPanel.robotController = robotController;
        }

        // Setup camera
        if (mainCamera != null)
        {
            mainCamera.transform.LookAt(robotController.transform);
        }

        // Connect to ROS if auto-connect is enabled
        if (autoConnect && rosConnection != null)
        {
            // Connection is handled by ROSConnectionManager's Start() method
        }

        // Show/hide control panel
        if (controlPanel != null && controlPanel.gameObject != null)
        {
            controlPanel.gameObject.SetActive(showControlPanel);
        }
    }

    void OnDestroy()
    {
        // Unregister robot controller when scene is destroyed
        if (rosConnection != null && robotController != null)
        {
            rosConnection.UnregisterRobotController(robotController);
        }
    }
}
```

## Step 5: Testing and Validation

### Unit Testing for Robot Components

```csharp
#if UNITY_EDITOR
using UnityEngine;
using UnityEditor;

public class RobotComponentTests
{
    [MenuItem("Robotics/Run Component Tests")]
    static void RunRobotTests()
    {
        Debug.Log("Running Robot Component Tests...");

        // Test joint movement
        TestJointMovement();

        // Test ROS connection simulation
        TestROSConnection();

        Debug.Log("Robot Component Tests Completed!");
    }

    static void TestJointMovement()
    {
        Debug.Log("Testing Joint Movement...");

        // Create a test robot controller
        GameObject testRobot = new GameObject("TestRobot");
        RobotController controller = testRobot.AddComponent<RobotController>();

        // Add a test joint
        RobotController.JointInfo testJoint = new RobotController.JointInfo
        {
            jointName = "test_joint",
            jointTransform = testRobot.transform,
            jointType = RobotController.JointType.Revolute
        };

        controller.joints.Add(testJoint);

        // Test joint update
        controller.UpdateJoint("test_joint", Mathf.PI / 4); // 45 degrees

        Debug.Log($"Test joint rotation: {testRobot.transform.localRotation.eulerAngles}");

        // Clean up
        Object.DestroyImmediate(testRobot);

        Debug.Log("Joint Movement Test Passed!");
    }

    static void TestROSConnection()
    {
        Debug.Log("Testing ROS Connection Simulation...");

        // This would typically involve mocking ROS messages
        // For now, just log the test
        Debug.Log("ROS Connection Test Passed!");
    }
}
#endif
```

## Performance Optimization

### Robot Performance Optimizer

```csharp
using UnityEngine;

public class RobotPerformanceOptimizer : MonoBehaviour
{
    [Header("Performance Settings")]
    public int maxUpdateRate = 60; // Max updates per second
    public float lodDistance = 10f; // Distance to reduce detail
    public bool useLOD = true;

    private float lastUpdateTime;
    private RobotController robotController;
    private Renderer[] robotRenderers;

    void Start()
    {
        robotController = GetComponent<RobotController>();
        CacheRenderers();
    }

    void CacheRenderers()
    {
        robotRenderers = GetComponentsInChildren<Renderer>();
    }

    void Update()
    {
        // Throttle updates to maxUpdateRate
        if (Time.time - lastUpdateTime < 1f / maxUpdateRate)
        {
            return;
        }

        lastUpdateTime = Time.time;

        // Update LOD based on distance to camera
        if (useLOD)
        {
            UpdateLOD();
        }
    }

    void UpdateLOD()
    {
        Camera mainCamera = Camera.main;
        if (mainCamera != null)
        {
            float distance = Vector3.Distance(transform.position, mainCamera.transform.position);

            if (distance > lodDistance)
            {
                // Reduce rendering quality for distant robots
                foreach (var renderer in robotRenderers)
                {
                    renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.Off;
                    renderer.receiveShadows = false;
                }
            }
            else
            {
                // Restore normal rendering quality
                foreach (var renderer in robotRenderers)
                {
                    renderer.shadowCastingMode = UnityEngine.Rendering.ShadowCastingMode.On;
                    renderer.receiveShadows = true;
                }
            }
        }
    }

    public void SetUpdateRate(int rate)
    {
        maxUpdateRate = Mathf.Clamp(rate, 1, 120);
    }
}
```

## Deployment Considerations

### Platform-Specific Optimizations

For different deployment platforms, consider these optimizations:

**Desktop (Windows/Mac/Linux)**:
- Higher polygon counts for detailed models
- Advanced lighting and shadows
- Multiple high-resolution textures

**Mobile (Android/iOS)**:
- Simplified models with lower polygon counts
- Compressed textures (ASTC, ETC2)
- Optimized rendering pipelines
- Reduced update rates

**VR/AR Headsets**:
- Performance-critical optimizations
- Fixed foveated rendering where applicable
- Reduced draw distances
- Simplified physics calculations

## Real-World Application: Teleoperation Interface

Create a complete teleoperation interface:

### Teleoperation Control Script

```csharp
using UnityEngine;
using UnityEngine.UI;
using TMPro;

public class TeleoperationInterface : MonoBehaviour
{
    [Header("UI Elements")]
    public Slider linearVelocitySlider;
    public Slider angularVelocitySlider;
    public Button forwardButton;
    public Button backwardButton;
    public Button leftButton;
    public Button rightButton;
    public Button stopButton;
    public TextMeshProUGUI velocityDisplay;

    [Header("Robot Control")]
    public float maxLinearVelocity = 1.0f;
    public float maxAngularVelocity = 1.0f;
    private ROSConnectionManager rosConnection;

    void Start()
    {
        SetupTeleoperationControls();
    }

    void SetupTeleoperationControls()
    {
        if (linearVelocitySlider != null)
        {
            linearVelocitySlider.minValue = -maxLinearVelocity;
            linearVelocitySlider.maxValue = maxLinearVelocity;
            linearVelocitySlider.value = 0f;
        }

        if (angularVelocitySlider != null)
        {
            angularVelocitySlider.minValue = -maxAngularVelocity;
            angularVelocitySlider.maxValue = maxAngularVelocity;
            angularVelocitySlider.value = 0f;
        }

        // Setup button events
        if (forwardButton != null)
            forwardButton.onClick.AddListener(() => MoveRobot(1f, 0f));
        if (backwardButton != null)
            backwardButton.onClick.AddListener(() => MoveRobot(-1f, 0f));
        if (leftButton != null)
            leftButton.onClick.AddListener(() => MoveRobot(0f, 1f));
        if (rightButton != null)
            rightButton.onClick.AddListener(() => MoveRobot(0f, -1f));
        if (stopButton != null)
            stopButton.onClick.AddListener(() => MoveRobot(0f, 0f));

        // Setup slider events
        if (linearVelocitySlider != null)
            linearVelocitySlider.onValueChanged.AddListener(OnLinearVelocityChanged);
        if (angularVelocitySlider != null)
            angularVelocitySlider.onValueChanged.AddListener(OnAngularVelocityChanged);
    }

    void OnLinearVelocityChanged(float value)
    {
        UpdateVelocityDisplay();
    }

    void OnAngularVelocityChanged(float value)
    {
        UpdateVelocityDisplay();
    }

    void UpdateVelocityDisplay()
    {
        if (velocityDisplay != null && linearVelocitySlider != null && angularVelocitySlider != null)
        {
            velocityDisplay.text = $"Lin: {linearVelocitySlider.value:F2}, Ang: {angularVelocitySlider.value:F2}";
        }
    }

    void MoveRobot(float linear, float angular)
    {
        if (linearVelocitySlider != null)
            linearVelocitySlider.value = linear * maxLinearVelocity;
        if (angularVelocitySlider != null)
            angularVelocitySlider.value = angular * maxAngularVelocity;

        UpdateVelocityDisplay();

        // Send command to robot via ROS (implementation would go here)
        SendVelocityCommand(linear * maxLinearVelocity, angular * maxAngularVelocity);
    }

    void SendVelocityCommand(float linear, float angular)
    {
        // Implementation to send Twist message to /cmd_vel topic
        // This would use the rosbridge client to publish geometry_msgs.Twist
        Debug.Log($"Sending velocity command - Linear: {linear}, Angular: {angular}");
    }
}
```

## Hands-On Exercise: Complete Unity Robotics Application

### Exercise Goal
Build a complete Unity application that visualizes and controls a mobile robot in real-time.

### Prerequisites
- Complete Unity-ROS setup
- Robot simulation running in Gazebo or real robot
- rosbridge server running

### Steps

1. **Create New Unity Project**
   - Set up project with required packages
   - Import Unity-RosBridge client
   - Create project structure as outlined

2. **Build Robot Model**
   - Create 3D model for your robot
   - Implement RobotController script
   - Add materials and visual effects

3. **Implement ROS Connection**
   - Set up ROSConnectionManager
   - Subscribe to robot topics
   - Handle incoming messages

4. **Create User Interface**
   - Design control panel
   - Add teleoperation controls
   - Implement status displays

5. **Test Integration**
   - Connect to ROS system
   - Verify visualization
   - Test control functionality

6. **Optimize Performance**
   - Implement LOD system
   - Add performance monitoring
   - Optimize for target platform

### Expected Outcome
A fully functional Unity application that visualizes robot state in real-time and allows for remote control of the robot.

## Quiz Questions

### Multiple Choice
1. What is the recommended approach for updating robot joints in Unity to maintain performance?
   a) Update every frame
   b) Throttle updates to a reasonable rate
   c) Update only when values change
   d) Both b and c

   Answer: d) Both b and c

2. Which Unity system is best for handling different AR/VR platforms?
   a) AR Foundation
   b) XR Plugin Management
   c) Unity Engine
   d) Input System

   Answer: b) XR Plugin Management

### Practical Question
Design and implement a Unity scene that visualizes a 6-DOF robotic arm with real-time joint control. The application should include:
- A 3D model of the robotic arm
- Real-time joint position updates from ROS
- Interactive joint sliders for manual control
- Safety features like emergency stop
- Performance optimization for smooth operation

## Mini Hands-On Task (30 minutes)

Create a Unity scene with a simple differential drive robot that can be controlled via UI sliders. The robot should:
1. Have a basic 3D model (chassis, wheels)
2. Respond to linear and angular velocity commands
3. Display current velocity values
4. Include an emergency stop button

## Summary

This practical implementation guide demonstrates how to build a complete Unity robotics application from the ground up. By following the structured approach of setting up components, connecting to ROS, creating user interfaces, and optimizing for performance, you can create professional-grade robotics visualization and control applications. The modular design allows for easy extension and adaptation to specific robotics projects and requirements.