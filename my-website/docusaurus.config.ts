import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics Textbook',
  tagline: 'An advanced textbook on ROS 2, Gazebo, NVIDIA Isaac, and Vision-Language-Action systems',
  favicon: 'img/Robotics-logo.png',

  future: {
    v4: true,
  },

  url: 'https://MuhammadMuneeb-Arif.github.io',
  baseUrl: '/hackathon-robotics-book/',

  organizationName: 'MuhammadMuneeb-Arif',
  projectName: 'hackathon-robotics-book',

  onBrokenLinks: 'throw',

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
    image: 'img/docusaurus-social-card.jpg',

    colorMode: {
      respectPrefersColorScheme: true,
    },

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Robotics Textbook Logo',
        src: 'img/Robotics-logo.png',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'textbookSidebar',
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
            { label: 'Module 1: ROS 2', to: '/docs/module-1' },
            { label: 'Module 2: Digital Twin', to: '/docs/module-2' },
            { label: 'Module 3: AI-Robot Brain', to: '/docs/module-3' },
            { label: 'Module 4: VLA Systems', to: '/docs/module-4' },
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
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Textbook. Built by Muhammad Muneeb Ahmed.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
