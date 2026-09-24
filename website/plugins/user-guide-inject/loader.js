// Webpack pre-loader replacing the `<load path='…' marker='…' />` tags of a template by the
// referenced code; the loaded files are registered as dependencies so editing them hot-reloads.
// For a page of a generated variant, the page is rebuilt from its template (also registered as a
// dependency), keeping only the content of the flavour tags of that variant.
const fs = require('fs');
const path = require('path');
const { execFile } = require('child_process');
const { variantContent } = require('./flavours');

// Same pattern as `docs-examples/inject_file`.
const LOAD_TAG = /<load path='(.*)'.*marker='(.*)'.*>/g;

module.exports = function injectUserGuideCode(source) {
  const callback = this.async();
  const { binary, cwd, templatesDir, variantDir } = this.getOptions();

  let templatePath = this.resourcePath;
  if (variantDir) {
    templatePath = path.join(templatesDir, path.basename(this.resourcePath));
    this.addDependency(templatePath);
    source = fs.readFileSync(templatePath, 'utf8');
  }

  for (const [, file] of source.matchAll(LOAD_TAG)) {
    // `inject_file` resolves paths as `..<path>` from its own directory.
    this.addDependency(path.join(cwd, '..', file));
  }

  execFile(binary, [templatePath], { cwd, maxBuffer: 64 << 20 }, (err, stdout, stderr) => {
    // Exit code 1 means some tags could not be resolved: keep the partial result and warn.
    if (err && err.code !== 1) {
      callback(err);
      return;
    }
    if (err) {
      this.emitWarning(new Error(stderr.trim()));
    }
    callback(null, variantDir ? variantContent(stdout, variantDir) : stdout);
  });
};
