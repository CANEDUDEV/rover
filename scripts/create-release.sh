#!/usr/bin/env bash

set -eo pipefail

find . -type f -name "*.bin" | zip -q -j rover-binaries.zip -@

echo "Finished creating release."
