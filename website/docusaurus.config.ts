// SPDX-FileCopyrightText: 2026 Polymath Robotics
// SPDX-License-Identifier: Apache-2.0

import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';
import remarkRepoLinks from './plugins/remark-repo-links';

const config: Config = {
  title: 'Protective Stop',
  tagline: 'Wireless protective-stop system documentation',

  future: {
    v4: true,
  },

  url: 'https://polymathrobotics.github.io',
  baseUrl: '/protective-stop/',
  organizationName: 'polymathrobotics',
  projectName: 'protective-stop',
  trailingSlash: false,
  onBrokenLinks: 'throw',

  headTags: [
    {
      tagName: 'link',
      attributes: {rel: 'icon', type: 'image/png', href: '/protective-stop/img/favicon.png'},
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'icon',
        type: 'image/png',
        href: '/protective-stop/img/favicon-dark.png',
        media: '(prefers-color-scheme: dark)',
      },
    },
  ],

  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  markdown: {
    mermaid: true,
    // Plain .md files are CommonMark; only .mdx is parsed as MDX.
    format: 'detect',
    hooks: {
      onBrokenMarkdownLinks: 'throw',
    },
  },

  presets: [
    [
      'classic',
      {
        docs: {
          path: '../docs',
          routeBasePath: '/',
          sidebarPath: './sidebars.ts',
          exclude: ['**/*.json'],
          beforeDefaultRemarkPlugins: [remarkRepoLinks],
          editUrl: 'https://github.com/polymathrobotics/protective-stop/tree/main/docs/',
        },
        blog: false,
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themes: [
    '@docusaurus/theme-mermaid',
    [
      require.resolve('@easyops-cn/docusaurus-search-local'),
      {
        hashed: true,
        docsRouteBasePath: '/',
        docsDir: '../docs',
        indexBlog: false,
        removeDefaultStopWordFilter: true,
        highlightSearchTermsOnTargetPage: true,
      },
    ],
  ],

  themeConfig: {
    colorMode: {respectPrefersColorScheme: true},
    navbar: {
      title: '',
      logo: {
        alt: 'Polymath Robotics',
        src: 'img/polymath-robotics-white-horizontal-full.png',
      },
      items: [
        {type: 'docSidebar', sidebarId: 'guides', position: 'left', label: 'Guides'},
        {type: 'docSidebar', sidebarId: 'hardware', position: 'left', label: 'Hardware'},
        {type: 'docSidebar', sidebarId: 'safety', position: 'left', label: 'Safety'},
        {type: 'docSidebar', sidebarId: 'design', position: 'left', label: 'Design'},
        {type: 'docSidebar', sidebarId: 'developing', position: 'left', label: 'Developing'},
        {type: 'docSidebar', sidebarId: 'archive', position: 'left', label: 'Archive'},
        {type: 'search', position: 'right'},
        {
          href: 'https://github.com/polymathrobotics/protective-stop',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      copyright: `Copyright © ${new Date().getFullYear()} Polymath Robotics, Inc.`,
    },
    mermaid: {theme: {light: 'neutral', dark: 'dark'}},
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
      additionalLanguages: ['bash', 'cmake', 'c', 'cpp', 'python', 'yaml', 'json'],
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
