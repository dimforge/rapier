// Generation of the variants of the user-guide from its templates: every variant keeps the content of some flavour
// tags and removes the content of the others.

// The tags removed and the tags kept by each variant, keyed by the folder of the variant in `docs/user_guides`.
// The flavour tags are <rapier> (Rust), <bevy>, <js>, <c>, <py>, and the <notjs>, <notc>, <notpy> exclusions.
const VARIANT_TAGS = {
  rust: {removed: ['c', 'py', 'bevy', 'js'], kept: ['rapier', 'notjs', 'notc', 'notpy']},
  bevy_plugin: {removed: ['c', 'py', 'rapier', 'js'], kept: ['bevy', 'notjs', 'notc', 'notpy']},
  javascript: {removed: ['c', 'py', 'rapier', 'bevy', 'notjs'], kept: ['js', 'notc', 'notpy']},
  c: {removed: ['py', 'rapier', 'bevy', 'js', 'notjs', 'notc'], kept: ['c', 'notpy']},
  python: {removed: ['c', 'rapier', 'bevy', 'js', 'notjs', 'notpy'], kept: ['py', 'notc']},
};

// Deletes the content of a tag: first its occurrences without any nested tag (typically inline), then the whole
// lines from an opening tag to the next line holding a closing tag (typically blocks).
function removeTag(text, tag) {
  const open = `<${tag}>`;
  const close = `</${tag}>`;
  text = text.replace(new RegExp(`${open}[^<]*${close}`, 'g'), '');

  const lines = text.split('\n');
  const result = [];
  let inBlock = false;
  for (const line of lines) {
    if (inBlock) {
      inBlock = !line.includes(close);
    } else if (line.includes(open)) {
      // Like `sed '/open/,/close/d'`, the closing tag is only searched from the next line.
      inBlock = true;
    } else {
      result.push(line);
    }
  }
  return result.join('\n');
}

// Keeps the content of a tag, removing the tag itself.
function unwrapTag(text, tag) {
  return text.split(`<${tag}>`).join('').split(`</${tag}>`).join('');
}

// The content of the variant (identified by its folder) of a template whose code was already injected.
function variantContent(injectedTemplate, variantDir) {
  const {removed, kept} = VARIANT_TAGS[variantDir];
  let text = injectedTemplate;
  for (const tag of removed) {
    text = removeTag(text, tag);
  }
  for (const tag of kept) {
    text = unwrapTag(text, tag);
  }
  return text;
}

module.exports = {VARIANT_TAGS, variantContent};
