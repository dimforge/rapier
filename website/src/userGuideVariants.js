// The variants of the user-guide and their pages, shared by the sidebars, the variant selector, and the generator
// of the variants (hence CommonJS).

// Each variant is a folder of `docs/user_guides`, generated from the templates by `generate_user_guides.sh`
// (except "All", which is the templates folder itself, showing every flavour with its color code).
const variants = [
  {id: 'rust', dir: 'rust', label: 'Rust', version: '0.35'},
  {id: 'c', dir: 'c', label: 'C', version: '0.35'},
  {id: 'js', dir: 'javascript', label: 'JavaScript', version: '0.17'},
  {id: 'python', dir: 'python', label: 'Python', version: '0.35'},
  {id: 'bevy', dir: 'bevy_plugin', label: 'Bevy plugin', version: '0.37'},
  {id: 'all', dir: 'templates', label: 'All'},
];

// The pages of the user-guide, in sidebar order. `docs` gives the name of the page in each variant that has it (the
// "All" variant has every page of every other variant, labelled with `title` and its variants in the sidebar).
const pages = [
  {
    key: 'getting_started',
    title: 'Getting started',
    docs: {
      rust: 'getting_started',
      bevy: 'getting_started_bevy',
      js: 'getting_started_js',
      c: 'getting_started_c',
      python: 'getting_started_py',
    },
  },
  {key: 'simulation_structures', only: ['rust', 'bevy', 'c', 'python']},
  {key: 'rigid_bodies'},
  {key: 'colliders'},
  {key: 'joints'},
  {key: 'joint_constraints'},
  {key: 'soft_bodies'},
  {key: 'character_controller'},
  {key: 'scene_queries'},
  {
    key: 'advanced_collision_detection',
    title: 'Advanced collision-detection',
    docs: {
      rust: 'advanced_collision_detection',
      bevy: 'advanced_collision_detection',
      js: 'advanced_collision_detection_js',
      c: 'advanced_collision_detection',
      python: 'advanced_collision_detection',
    },
  },
  {key: 'integration_parameters', only: ['rust', 'bevy', 'c', 'python']},
  {key: 'debug_render'},
  {key: 'scene_loaders', only: ['rust', 'bevy', 'c', 'python']},
  {key: 'serialization'},
  {key: 'determinism'},
  {key: 'common_mistakes'},
  {key: 'the_rapier_testbed', only: ['rust', 'c', 'python']},
  {key: 'common_recipes', only: ['rust', 'bevy', 'c', 'python']},
  {key: 'multiple_contexts', only: ['bevy']},
];

const variantById = Object.fromEntries(variants.map((v) => [v.id, v]));

// Name of the page `page` in the variant `variantId`, or `undefined` if that variant doesn't have it.
function pageDoc(page, variantId) {
  if (page.docs) {
    return page.docs[variantId];
  }
  if (page.only && !page.only.includes(variantId)) {
    return undefined;
  }
  return page.key;
}

// The page names of a variant, in sidebar order.
function variantDocs(variantId) {
  if (variantId === 'all') {
    const names = [];
    for (const page of pages) {
      for (const v of variants) {
        const name = v.id === 'all' ? undefined : pageDoc(page, v.id);
        if (name && !names.includes(name)) {
          names.push(name);
        }
      }
    }
    return names;
  }
  return pages.map((page) => pageDoc(page, variantId)).filter((name) => name !== undefined);
}

// Sidebar label of a page of the "All" variant, when its title alone is ambiguous (several pages have the same title).
function allVariantLabel(name) {
  const page = pages.find((p) => p.docs && Object.values(p.docs).includes(name));
  if (!page) {
    return undefined;
  }
  const owners = variants.filter((v) => page.docs[v.id] === name);
  // A page shared by most variants is the default one: only the exceptions are labelled.
  return owners.length === 1 ? `${page.title} (${owners[0].label})` : page.title;
}

// Doc id (relative to the docs folder) of a page of a variant.
function docId(variant, name) {
  return `user_guides/${variant.dir}/${name}`;
}

// Parses a doc id into its variant and page name, or returns `undefined` if it isn't a user-guide page.
function parseDocId(id) {
  const match = /^user_guides\/([^/]+)\/([^/]+)$/.exec(id);
  if (!match) {
    return undefined;
  }
  const variant = variants.find((v) => v.dir === match[1]);
  return variant ? {variant, name: match[2]} : undefined;
}

// The page of `targetId` corresponding to the page `name` of the variant `fromId`, or `undefined` if the target
// variant has no such page.
function correspondingDoc(fromId, name, targetId) {
  if (targetId === 'all') {
    return name;
  }
  const page = pages.find((p) =>
    fromId === 'all'
      ? variants.some((v) => v.id !== 'all' && pageDoc(p, v.id) === name)
      : pageDoc(p, fromId) === name,
  );
  return page ? pageDoc(page, targetId) : undefined;
}

module.exports = {variants, pages, variantById, variantDocs, allVariantLabel, docId, parseDocId, correspondingDoc};
