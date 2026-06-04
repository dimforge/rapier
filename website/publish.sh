#!/bin/bash

./generate_user_guides.sh
PUBLISH_MODE=1 yarn build
cp .htaccess build/.
rsync -av --delete-after build/ crozet@ssh.cluster003.hosting.ovh.net:/home/crozet/rapier/
rm docs/user_guides/rust/*.mdx
rm docs/user_guides/bevy_plugin/*.mdx
rm docs/user_guides/javascript/*.mdx
