import type { SidebarsConfig } from '@docusaurus/plugin-content-docs';

const sidebars: SidebarsConfig = {
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro/index'],
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/index',
        'module-1-ros2/chapter-01-intro-physical-ai',
        'module-1-ros2/chapter-02-foundations',
        'module-1-ros2/chapter-03-ros2-intro',
        'module-1-ros2/chapter-04-communication',
        'module-1-ros2/chapter-05-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Simulation)',
      items: [
        'module-2-gazebo/index',
        'module-2-gazebo/chapter-06-intro-simulation',
        'module-2-gazebo/chapter-07-gazebo',
        'module-2-gazebo/chapter-08-unity',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3-isaac/index',
        'module-3-isaac/chapter-09-isaac-sim',
        'module-3-isaac/chapter-10-isaac-ros',
        'module-3-isaac/chapter-11-nav2-humanoids',
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/index',
        'module-4-vla/chapter-12-conversational',
        'module-4-vla/chapter-13-cognitive-planning',
        'module-4-vla/chapter-15-humanoid-dev',
      ],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/index',
        'capstone/chapter-16-capstone',
      ],
  },
  ],
};

export default sidebars;