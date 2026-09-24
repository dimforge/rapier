import {variants, variantDocs, allVariantLabel, docId} from './src/userGuideVariants';

function variantLabel(variant) {
  return variant.version ? `${variant.label} (${variant.version})` : variant.label;
}

const apiDocumentation = {
  'API Documentation': [
    'api/javascript/JavaScript2D',
    'api/javascript/JavaScript3D',
    'api/c/C',
    'api/python/Python',
    {
      type: 'link',
      label: 'bevy_rapier2d',
      href: 'https://docs.rs/bevy_rapier2d'
    },
    {
      type: 'link',
      label: 'bevy_rapier3d',
      href: 'https://docs.rs/bevy_rapier3d'
    },
    {
      type: 'link',
      label: 'rapier2d',
      href: 'https://docs.rs/rapier2d'
    },
    {
      type: 'link',
      label: 'rapier3d',
      href: 'https://docs.rs/rapier3d'
    },
    {
      type: 'link',
      label: 'rapier2d-f64',
      href: 'https://docs.rs/rapier2d-f64'
    },
    {
      type: 'link',
      label: 'rapier3d-f64',
      href: 'https://docs.rs/rapier3d-f64'
    },
  ]
};

// The API references are docs of the main sidebar: the variant sidebars link to them.
const apiLabels = {
  'api/javascript/JavaScript2D': 'JavaScript 2D',
  'api/javascript/JavaScript3D': 'JavaScript 3D',
  'api/c/C': 'C bindings',
  'api/python/Python': 'Python bindings',
};

function apiLinks() {
  return {
    type: 'category',
    label: 'API Documentation',
    items: apiDocumentation['API Documentation'].map((item) =>
      typeof item === 'string'
        ? {type: 'link', label: apiLabels[item], href: `/docs/${item}`}
        : item,
    ),
  };
}

const config = {
  // The main sidebar: the about page, one entry point per variant of the user-guide, and the API references.
  docs: [
    'about_rapier',
    {
      type: 'category',
      label: 'User Guide',
      collapsed: false,
      items: variants.map((variant) => ({
        type: 'link',
        label: variantLabel(variant),
        href: `/docs/${docId(variant, variantDocs(variant.id)[0])}`,
      })),
    },
    apiDocumentation,
  ],
};

// One sidebar per variant of the user-guide, shown on its pages; the variant selector on top of each page switches
// between them.
for (const variant of variants) {
  config[`user_guide_${variant.id}`] = [
    {type: 'link', label: 'About Rapier', href: '/docs/'},
    {
      type: 'category',
      label: `User Guide: ${variantLabel(variant)}`,
      collapsible: false,
      items: variantDocs(variant.id).map((name) => {
        const label = variant.id === 'all' ? allVariantLabel(name) : undefined;
        return label ? {type: 'doc', id: docId(variant, name), label} : docId(variant, name);
      }),
    },
    apiLinks(),
  ];
}

export default config;
