import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  textbookSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Frontmatter',
      items: [
        'frontmatter/preface',
        'frontmatter/introduction-to-physical-ai',
        'frontmatter/how-to-use-this-book',
        'frontmatter/prerequisites',
        'frontmatter/learning-outcomes',
        'frontmatter/software-stack-overview',
        'frontmatter/hardware-requirements',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/index',
        'module-1/lesson-1-ros2-architecture',
        'module-1/lesson-2-python-agents-controllers',
        'module-1/lesson-3-humanoid-urdf',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/index',
        'module-2/lesson-1-gazebo-physics-sensors',
        'module-2/lesson-2-unity-robotics-hub',
        'module-2/lesson-3-simulated-cameras-sensor-fusion',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac)',
      items: [
        'module-3/index',
        'module-3/lesson-1-isaac-sim-basics',
        'module-3/lesson-2-isaac-ros-perception-vslam',
        'module-3/lesson-3-nav2-path-planning-humanoids',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/index',
        'module-4/lesson-1-whisper-speech-to-command',
        'module-4/lesson-2-llm-cognitive-planning',
        'module-4/lesson-3-capstone-full-digital-humanoid-agent',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Endmatter',
      items: [
        'endmatter/glossary',
        'endmatter/references',
        'endmatter/appendix',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
