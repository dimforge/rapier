#!/bin/bash

for entry in builds/*/pkg
do
    (
        echo "Publishing $entry"
        cd $entry;
        npm publish --access public;
    )
done;

for entry in rapier-compat/builds/*/pkg
do
    (
        echo "Publishing $entry"
        cd $entry;
        npm publish --access public;
    )
done;