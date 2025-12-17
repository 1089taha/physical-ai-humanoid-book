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
      label: 'Module 1: The Robotic Nervous System',
      items: ['module-1-ros2/index'],
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin',
      items: ['module-2-gazebo/index'],
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain',
      items: ['module-3-isaac/index'],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: ['module-4-vla/index'],
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: ['capstone/index'],
    },
  ],
};

export default sidebars;
