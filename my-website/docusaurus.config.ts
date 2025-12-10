import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'An advanced textbook on ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems',
  favicon: 'img/favicon.svg',

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://MuhammadMuneeb-Arif.github.io',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/hackathon-robotics-book/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'MuhammadMuneeb-Arif', // Usually your GitHub org/user name.
  projectName: 'hackathon-robotics-book', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/MuhammadMuneeb-Arif/hackathon-robotics-book/edit/main/my-website/',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
   navbar: {
  title: 'Physical AI & Humanoid Robotics',
  logo: {
    alt: 'Robotics Textbook Logo',
    src: 'img/logo.svg',
  },
  items: [
    {
      type: 'docSidebar',
      sidebarId: 'textbookSidebar', // <-- FIXED
      position: 'left',
      label: 'Textbook',
    },
    {
      href: 'https://github.com/MuhammadMuneeb-Arif/hackathon-robotics-book',
      label: 'GitHub',
      position: 'right',
    },
  ],
},

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Textbook',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
          ],
        },
        {
          title: 'Modules',
          items: [
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1',
            },
            {
              label: 'Module 2: Digital Twin',
              to: '/docs/module-2',
            },
            {
              label: 'Module 3: AI-Robot Brain',
              to: '/docs/module-3',
            },
            {
              label: 'Module 4: VLA Systems',
              to: '/docs/module-4',
            },
          ],
        },
        {
          title: 'Resources',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/MuhammadMuneeb-Arif/hackathon-robotics-book',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built By Muneeb Ahmed.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
