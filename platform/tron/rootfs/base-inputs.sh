#!/bin/bash
# What a base tar of build-base.sh is built from: the checksums of packages.txt and build-base.sh and the paths under
# files/. build-base.sh stores it in the base, make-image.sh compares it with the repo.
set -euo pipefail
export LC_ALL=C
cd "$(dirname "$0")"
sha256sum packages.txt build-base.sh
(cd files && find . -name __pycache__ -prune -o ! -type d -print | sort | sed 's/^/file /')
