import { themes as prismThemes } from 'prism-react-renderer';
import type { Config } from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'An AI-native textbook on Physical AI and Humanoid Robotics',
  favicon: 'img/favicon.ico',

  url: 'http://localhost:3000',
  baseUrl: '/',

  organizationName: 'your-github-username',
  projectName: 'physical-ai-humanoid-robots',

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'throw',

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          routeBasePath: 'docs',
          sidebarPath: require.resolve('./sidebars'),
          editUrl: undefined,
        },
        blog: false,
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    image: 'img/social-card.png',

    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'doc',
          docId: 'intro/index',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/your-github-username/physical-ai-humanoid-robots',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },

    footer: {
      style: 'dark',
      links: [
        {
          title: 'Book',
          items: [
            {
              label: 'Introduction',
              to: '/docs/intro',
            },
            {
              label: 'Module 1 — ROS 2',
              to: '/docs/module-1-ros2',
            },
            {
              label: 'Module 2 — Digital Twin',
              to: '/docs/module-2-gazebo',
            },
            {
              label: 'Module 3 — AI Robot Brain',
              to: '/docs/module-3-isaac',
            },
            {
              label: 'Module 4 — VLA',
              to: '/docs/module-4-vla',
            },
            {
              label: 'Capstone',
              to: '/docs/capstone',
            },
          ],
        },
      ],
      copyright: `© ${new Date().getFullYear()} Muhammad Taha Khan. Built with Docusaurus.`,
    },

    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
