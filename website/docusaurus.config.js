const math = require('remark-math')
const katex = require('rehype-katex')

const config = {
  title: 'Rapier',
  tagline: 'Fast 2D and 3D physics engine for the Rust programming language.',
  url: 'https://rapier.rs',
  baseUrl: '/',
  onBrokenLinks: 'throw',
  favicon: 'img/favicon.png',
  organizationName: 'dimforge', // Usually your GitHub org/user name.
  projectName: 'rapier', // Usually your repo name.
  themeConfig: {
    algolia: {
      apiKey: '00969eabbf113aa543d1be16629dd663',
      appId: 'MDOZU7PTCV',
      indexName: 'rapier',
    },
    prism: {
      theme: require('prism-react-renderer').themes.github,
      additionalLanguages: ['toml', 'rust'],
    },
    // announcementBar: {
    //   id: 'supportus',
    //   content:
    //     '⭐️ If you like Rapier, support us on <a target="_blank" rel="noopener noreferrer" href="https://github.com/sponsors/dimforge">GitHub Sponsor</a>! ⭐️',
    // },
    navbar: {
      title: 'Rapier',
      logo: {
        alt: 'Rapier Logo',
        src: 'img/rapier_logo_color_small.svg',
      },
      hideOnScroll: true,
      items: [
        {
          to: 'docs/',
          activeBasePath: 'docs',
          label: 'Docs',
          position: 'left',
        },
        {
          label: 'Demos',
          position: 'left',
          items: [
            {
              href: 'https://rapier.rs/demos2d/index.html', // FIXME: should depend on the base url.
              label: '2D Demos',
            },
            {
              href: 'https://rapier.rs/demos3d/index.html', // FIXME: should depend on the base url.
              label: '3D Demos',
            }
          ],
        },
        {
          to: '/community',
          position: 'left',
          activeBaseRegex: `/community/`,
          label: 'Community',
        },
        {
          href: 'https://dimforge.com/blog',
          label: 'Blog',
          position: 'left',
        },
        {
          value: '<a class="header-button-donate" href="https://github.com/sponsors/dimforge" target="_blank" rel="noopener noreferrer">Donate ♥</a>',
          className: 'header-button-donate',
          position: 'right',
          type: 'html'
        },
        {
          href: 'https://dimforge.com',
          label: 'Dimforge',
          position: 'right',
        },
        {
          href: 'https://github.com/dimforge/rapier',
          position: 'right',
          className: 'header-github-link',
          'aria-label': 'GitHub repository',
        },
      ],
    },
    footer: {
      style: 'dark',
      logo: {
        alt: 'Dimforge EURL Logo',
        src: 'https://www.dimforge.com/img/logo/logo_dimforge_small_rect.svg',
        href: 'https://www.dimforge.com/'
      },
      copyright: `Built by <a href="https://www.dimforge.com">Dimforge, EURL</a>.`,
      links: [
        {
          title: 'Resources',
          items: [
            {
              label: 'Documentation',
              to: 'docs/',
            },
            {
              label: 'Demos 2D',
              href: 'https://rapier.rs/demos2d/index.html',
            },
            {
              label: 'Demos 3D',
              href: 'https://rapier.rs/demos3d/index.html',
            },
          ],
        },
        {
          title: 'Community',
          items: [
            {
              label: 'Stack Overflow',
              href: 'https://stackoverflow.com/questions/tagged/rapier',
            },
            {
              label: 'Discord',
              href: 'https://discord.gg/vt9DJSW',
            },
            // {
            //   label: 'Twitter',
            //   href: 'https://twitter.com/dimforge',
            // },
          ],
        },
        {
          title: 'More',
          items: [
            // {
            //   label: 'Blog',
            //   to: 'blog',
            // },
            {
              label: 'GitHub',
              href: 'https://github.com/dimforge/rapier',
            },
          ],
        },
      ],
      // copyright: `Copyright © ${new Date().getFullYear()} Dimforge EURL. Website built with Docusaurus.`,
    },
  },
  plugins: [
    [
      '@docusaurus/plugin-content-docs',
      {
        id: 'community',
        path: 'community',
        routeBasePath: 'community',
        sidebarPath: require.resolve('./sidebar_community.js'),
        showLastUpdateTime: false,
      }
    ],
  ],
  presets: [
    [
      '@docusaurus/preset-classic',
      {
        docs: {
          sidebarPath: require.resolve('./sidebar_docs.js'),
          showLastUpdateTime: false,
          remarkPlugins: [math],
          rehypePlugins: [katex],
        },
        blog: {
          showReadingTime: true,
          // Please change this to your repo.
          editUrl:
            'https://github.com/dimforge/rapier.rs/edit/master/website/blog/',
        },
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      },
    ],
  ],
  stylesheets: [
    'https://cdn.jsdelivr.net/npm/katex@0.11.0/dist/katex.min.css'
  ]
};

export default config;