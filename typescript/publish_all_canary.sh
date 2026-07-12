#!/bin/bash

for entry in builds/*/pkg
do
    (
        echo "Publishing $entry"
        cd $entry; npm version 0.0.0-$(git rev-parse --short HEAD)-$(date '+%Y%m%d') --git-tag-version false; 
        npm publish --tag canary --access public;
    )
done;

for entry in rapier-compat/builds/*/pkg
do
    (
        echo "Publishing $entry"
        cd $entry; npm version 0.0.0-$(git rev-parse --short HEAD)-$(date '+%Y%m%d') --git-tag-version false; 
        npm publish --tag canary --access public;
    )
done;