#!/bin/bash

# Publishes the release versions of every package, using the version already
# baked into each `pkg/package.json` (which comes from the `version` line of
# `builds/prepare_builds/templates/Cargo.toml.tera`).

# A pre-release such as `0.20.0-beta.0` must not become the version that a plain
# `npm install @dimforge/rapier2d` resolves to; park those under `next` instead.
dist_tag() {
    case "$1" in
    *-*) echo next ;;
    *) echo latest ;;
    esac
}

# npm has no atomic multi-package publish, so a failure partway leaves the
# release half-applied. Keep going to publish as much as possible, but make sure
# the script still exits non-zero so CI does not report a broken release green.
status=0

for entry in builds/*/pkg rapier-compat/builds/*/pkg
do
    (
        echo "Publishing $entry"
        cd $entry;
        version=$(node -p "require('./package.json').version");
        npm publish --access public --tag $(dist_tag "$version");
    ) || status=1
done;

exit $status
