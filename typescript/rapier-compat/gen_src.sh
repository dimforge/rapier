# Copy source and remove #if sections - similar to script in ../rapierXd
set -e

gen_js() {
  DIM=$1
  GENOUT="./gen${DIM}"

  # Make output directories
  mkdir -p ${GENOUT}

  # Copy common sources
  cp -r ../src.ts/* $GENOUT

  # Copy compat mode override sources
  rm -f "${GENOUT}/raw.ts" "${GENOUT}/init.ts"
  cp -r ./src${DIM}/* $GENOUT
}

gen_js "2d"
gen_js "3d"

# See https://serverfault.com/a/137848
find gen2d/ -type f -print0 | LC_ALL=C xargs -0 sed -i.bak '\:#if DIM3:,\:#endif:d'
find gen3d/ -type f -print0 | LC_ALL=C xargs -0 sed -i.bak '\:#if DIM2:,\:#endif:d'

# Clean up backup files.
find gen2d/ -type f -name '*.bak' | xargs rm
find gen3d/ -type f -name '*.bak' | xargs rm

for features_set in \
"2" "2 deterministic" "2 simd" \
"3" "3 deterministic" "3 simd"
do

  set -- $features_set # Convert the "tuple" into the param args $1 $2...
  dimension=$1
  if [ -z "$2" ]; then
    feature="${1}d";
  else
    feature="${1}d-${2}";
  fi

  mkdir -p ./builds/${feature}/pkg/

  cp ./builds/${feature}/wasm-build/rapier_wasm* ./builds/${feature}/pkg/
  cp -r ./gen${dimension}d ./builds/${feature}/

  # copy tsconfig, as they contain paths
  cp ./tsconfig.common.json ./tsconfig.json ./builds/${feature}/
  cp ./tsconfig.pkg${dimension}d.json ./builds/${feature}/tsconfig.pkg.json

  # "import.meta" causes Babel to choke, but the code path is never taken so just remove it.
  sed -i.bak 's/import.meta.url/"<deleted>"/g' ./builds/${feature}/pkg/rapier_wasm${dimension}d.js

  # Clean up backup files.
  find ./builds/${feature}/pkg/ -type f -name '*.bak' | xargs rm

done
