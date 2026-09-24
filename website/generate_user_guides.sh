#!/bin/bash
# Generates the variants of the user-guide (`docs/user_guides/<variant>`) from its templates. `yarn start` and
# `yarn build` already do it (see `plugins/user-guide-inject`): this is only needed to inspect the generated pages.
set -e
cd "$(dirname "${BASH_SOURCE[0]}")"
node plugins/user-guide-inject/generate.js
