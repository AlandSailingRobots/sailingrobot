#!/bin/sh
set -e

# check $0 in pwd
case "$1" in
    init)
        ;;
    update)
        git pull
        git submodule update
        ;;
    *)
        printf 'Usage: %s update\n' "$0"
        ;;
esac
