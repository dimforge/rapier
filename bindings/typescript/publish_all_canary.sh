#!/bin/bash

# Publishes a throwaway `0.0.0-<short sha>-<date>` version of every package under
# the `canary` dist-tag, so `master` is always installable without touching the
# `latest` version that `publish_all_prod.sh` releases.

# As in `publish_all_prod.sh`: publish everything we can, but still exit
# non-zero if any package failed, otherwise the job reports green with packages
# missing.
status=0

for entry in builds/*/pkg rapier-compat/builds/*/pkg
do
    (
        echo "Publishing $entry"
        cd $entry; npm version 0.0.0-$(git rev-parse --short HEAD)-$(date '+%Y%m%d') --git-tag-version false;
        npm publish --tag canary --access public;
    ) || status=1
done;

exit $status
