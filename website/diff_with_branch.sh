#!/bin/bash

echo '$1: '$1
echo '$2: '$2

if [[ $1 = "--help" ]]; then
    echo "USAGE: $0 [branch1 [branch2]]
    branch1 default: master
    branch2 default: . (current branch)
    "
    exit;
fi

if [[ -n $(git status -s) ]]; then
    echo "you have uncommitted changes, chances are they will be erased while switching branches.
    Please commit or stash your changes first."
    git status -s
    exit;
fi

set -v

origin_head=$(git symbolic-ref --short HEAD)
echo $origin_head

BRANCH_1="${1:-master}"
# "." in git means' current branch
BRANCH_2="${2:-.}"

echo $BRANCH_1
echo $BRANCH_2

git checkout $BRANCH_2

FILENAMES=`find docs/user_guides/templates/ -type f`
OUTPUT_FOLDER=tmp_diff_branches/branch2_templates_injected ./inject_code_in_user_guides.sh $FILENAMES
git checkout $BRANCH_1
find docs/user_guides/templates/ -type f -print0 | OUTPUT_FOLDER=tmp_diff_branches/branch1_templates_injected/ xargs -0 ./inject_code_in_user_guides.sh
git diff --no-index -w tmp_diff_branches/branch1_templates_injected/ tmp_diff_branches/branch2_templates_injected/ > tmp_total_diff.diff

git checkout $origin_head