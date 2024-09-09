#! /bin/bash

if [[ "$PUBLISH_MODE" == 1 ]]
then
    ./scripts/publish-rapier.sh &&
    ./scripts/publish-testbeds.sh &&
    ./scripts/publish-extra-formats.sh
else
    echo "Running in dry mode, re-run with \`PUBLISH_MODE=1 publish-all.sh\` to actually publish."

    DRY_RUN="--dry-run" ./scripts/publish-rapier.sh &&
    DRY_RUN="--dry-run" ./scripts/publish-testbeds.sh &&
    DRY_RUN="--dry-run" ./scripts/publish-extra-formats.sh
fi