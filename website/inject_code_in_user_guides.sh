#!/bin/bash
set -v

KEEP_TMP="${KEEP_TMP:-0}"
PRINT_DIFF="${PRINT_DIFF:-1}"

echo $OUTPUT_FOLDER
echo $KEEP_TMP
echo $PRINT_DIFF


if [ -z ${OUTPUT_FOLDER:x} ]; then
    echo "ERROR: MISSING: OUTPUT_FOLDER env variable"
    exit;
fi

mkdir -p "$OUTPUT_FOLDER"
if [ ! -d $OUTPUT_FOLDER ]; then
    echo "ERROR: Not a directory: OUTPUT_FOLDER: $OUTPUT_FOLDER"
    exit;
fi

cd docs-examples/inject_file
OUTPUT_FOLDER="../../$OUTPUT_FOLDER"


mkdir -p "tmp_diff"

error=0
for path in "$@"
do
    file_to_inject="../../$path"
    diff_file="tmp_diff/$path"

    filename=$(basename -- "$file_to_inject")
    file_injected="$OUTPUT_FOLDER/$filename"
    echo $file_to_inject
    
    cargo run --quiet --release "$file_to_inject" > "$file_injected"
    if [ $? -eq 1 ]; then
        error=1
    fi
    if [ $PRINT_DIFF -eq 1 ]; then
        if cmp -s "$file_to_inject" "$file_injected"; then
            echo "No substitution to $file_to_inject"
        else
            echo "Substitution to $file_injected"
            mkdir -p "$(dirname $diff_file)"
            diff $file_to_inject $file_injected > "$diff_file".diff
        fi
    fi
done

if [ ! $KEEP_TMP ]; then
    rm -rf tmp_diff
fi

exit $error