// Generates the variants of the user-guide from its templates at startup, and runs
// `docs-examples/inject_file` on the templates and the variant pages at compile time, so `yarn start`
// hot-reloads every variant on edits to the templates or to the example sources they load.
const fs = require('fs');
const path = require('path');
const { VARIANT_TAGS } = require('./flavours');
const { buildInjectTool, generateUserGuides } = require('./generate');

module.exports = function userGuideInjectPlugin(context) {
  const templatesDir = fs.realpathSync(path.join(context.siteDir, 'docs/user_guides/templates'));
  const workspaceDir = path.join(context.siteDir, 'docs-examples');
  const cwd = path.join(workspaceDir, 'inject_file');
  const binary = buildInjectTool(workspaceDir);

  // The docs plugin reads the generated pages (and their front matter) after this initialization.
  for (const error of generateUserGuides(context.siteDir, binary)) {
    console.warn(`[user-guide-inject] ${error}`);
  }

  const loaderRule = (include, variantDir) => ({
    test: /\.mdx?$/,
    include,
    enforce: 'pre',
    use: [
      {
        loader: require.resolve('./loader.js'),
        options: { binary, cwd, templatesDir, variantDir },
      },
    ],
  });

  return {
    name: 'user-guide-inject',
    configureWebpack() {
      return {
        module: {
          rules: [
            loaderRule(templatesDir),
            ...Object.keys(VARIANT_TAGS).map((variantDir) =>
              loaderRule(fs.realpathSync(path.join(context.siteDir, 'docs/user_guides', variantDir)), variantDir),
            ),
          ],
        },
      };
    },
  };
};
