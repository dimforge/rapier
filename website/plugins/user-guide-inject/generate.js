// Generates the variants of the user-guide (`docs/user_guides/<variant>`) from its templates, injecting the code
// of the examples they load. Run by the `user-guide-inject` plugin at startup, or directly with `node`.
const fs = require('fs');
const path = require('path');
const {execFileSync, spawnSync} = require('child_process');
const {VARIANT_TAGS, variantContent} = require('./flavours');
const {variants, variantDocs} = require('../../src/userGuideVariants');

// Builds `docs-examples/inject_file` and returns the path of its executable.
function buildInjectTool(workspaceDir) {
  const output = execFileSync(
    'cargo',
    ['build', '--release', '--quiet', '-p', 'inject_file', '--message-format=json-render-diagnostics'],
    {cwd: workspaceDir, stdio: ['ignore', 'pipe', 'inherit'], encoding: 'utf8', maxBuffer: 64 << 20},
  );
  for (const line of output.split('\n')) {
    if (!line.startsWith('{')) continue;
    const msg = JSON.parse(line);
    if (msg.reason === 'compiler-artifact' && msg.target.name === 'inject_file' && msg.executable) {
      return msg.executable;
    }
  }
  throw new Error('user-guide-inject: could not locate the `inject_file` executable.');
}

// The pages that belong to some variant but not to this one: they aren't generated for it.
function excludedPages(variantDir) {
  const variant = variants.find((v) => v.dir === variantDir);
  const all = new Set(variantDocs('all'));
  for (const name of variantDocs(variant.id)) {
    all.delete(name);
  }
  return all;
}

// Writes `content` to `file` unless it already holds it (so that watchers aren't triggered for nothing).
function writeIfChanged(file, content) {
  if (!fs.existsSync(file) || fs.readFileSync(file, 'utf8') !== content) {
    fs.writeFileSync(file, content);
  }
}

// Names of the pages linked or imported by `content`.
function referencedPages(content) {
  return [...content.matchAll(/\.\/([A-Za-z0-9_]+)\.mdx/g)].map((match) => match[1]);
}

// Generates every variant folder. Returns the error messages of the `<load>` tags that couldn't be resolved.
function generateUserGuides(siteDir, binary) {
  const templatesDir = path.join(siteDir, 'docs/user_guides/templates');
  const injectCwd = path.join(siteDir, 'docs-examples/inject_file');
  const errors = [];
  const templates = fs.readdirSync(templatesDir).filter((file) => /\.mdx?$/.test(file));

  const injected = {};
  for (const file of templates) {
    const run = spawnSync(binary, [path.join(templatesDir, file)], {
      cwd: injectCwd,
      encoding: 'utf8',
      maxBuffer: 64 << 20,
    });
    if (run.status !== 0) {
      errors.push(`${file}: ${(run.stderr || run.error || '').toString().trim()}`);
    }
    injected[file] = run.stdout;
  }

  for (const variantDir of Object.keys(VARIANT_TAGS)) {
    const dir = path.join(siteDir, 'docs/user_guides', variantDir);
    fs.mkdirSync(dir, {recursive: true});
    const contents = {};
    for (const file of templates) {
      contents[path.parse(file).name] = {file, content: variantContent(injected[file], variantDir)};
    }
    // Skip the pages of other variants, unless a generated page of this variant links to them.
    const excluded = excludedPages(variantDir);
    let changed = true;
    while (changed) {
      changed = false;
      for (const [name, {content}] of Object.entries(contents)) {
        if (excluded.has(name)) continue;
        for (const referenced of referencedPages(content)) {
          changed = excluded.delete(referenced) || changed;
        }
      }
    }
    for (const file of fs.readdirSync(dir)) {
      if (!templates.includes(file) || excluded.has(path.parse(file).name)) {
        fs.rmSync(path.join(dir, file), {recursive: true});
      }
    }
    for (const [name, {file, content}] of Object.entries(contents)) {
      if (!excluded.has(name)) {
        writeIfChanged(path.join(dir, file), content);
      }
    }
  }
  return errors;
}

module.exports = {buildInjectTool, generateUserGuides};

if (require.main === module) {
  const siteDir = path.join(__dirname, '../..');
  const binary = buildInjectTool(path.join(siteDir, 'docs-examples'));
  const errors = generateUserGuides(siteDir, binary);
  for (const error of errors) {
    console.error(`❌ ${error}`);
  }
  process.exit(errors.length === 0 ? 0 : 1);
}
