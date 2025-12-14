// @ts-check

/**
 * @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'intro',
    'introduction',

    // Chapter 1
    {
      type: 'category',
      label: 'Chapter 1: ROS 2 Ecosystem',
      items: [
        'chapter-1-ros-2-ecosystem/intro',
        'chapter-1-ros-2-ecosystem/concepts.',
        'chapter-1-ros-2-ecosystem/implemetation',
      ],
    },

    // Chapter 2
    {
  type: 'category',
  label: 'Chapter 2: Gazebo Simulation',
  items: [
    'chapter-2-gazebo-simulation/intro',
    'chapter-2-gazebo-simulation/concepts',
    'chapter-2-gazebo-simulation/models',
    'chapter-2-gazebo-simulation/physics_engine',
    'chapter-2-gazebo-simulation/sensors',
    'chapter-2-gazebo-simulation/plugins',
    'chapter-2-gazebo-simulation/gazebo_ros2_integration',
    'chapter-2-gazebo-simulation/implementation',
  ],
},


    // Chapter 3
    {
      type: 'category',
      label: 'Chapter 3: Unity Integration',
      items: [
        'chapter-3-unity-integration/intro',
        'chapter-3-unity-integration/unity_ros_integration',
        'chapter-3-unity-integration/practical_implementation',
        'chapter-3-unity-integration/advanced_unity_integration',
        'chapter-3-unity-integration/summary',
      ],
    },

    // Chapter 4
    {
      type: 'category',
      label: 'Chapter 4: NVIDIA Isaac Sim',
      items: [
        'chapter-4-nvidia-isaac-sim/intro',
        'chapter-4-nvidia-isaac-sim/summary',
      ],
    },

    // Chapter 5
    {
      type: 'category',
      label: 'Chapter 5: Vision-Language-Action',
      items: [
        'chapter-5-vision-language-action/intro',
        'chapter-5-vision-language-action/summary',
      ],
    },
  ],
};

export default sidebars;

