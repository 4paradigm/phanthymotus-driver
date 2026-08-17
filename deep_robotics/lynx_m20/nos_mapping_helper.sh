#!/bin/bash
# Install on the M20 Pro NOS as /usr/local/sbin/phanthy-m20-mapping.
set -euo pipefail

usage() {
    echo "usage: phanthy-m20-mapping start <map_name> <true|false> | stop" >&2
    exit 64
}

action="${1:-}"
case "${action}" in
    start)
        [ "$#" -eq 3 ] || usage
        map_name="$2"
        activate="$3"
        [[ "${map_name}" =~ ^[A-Za-z0-9][A-Za-z0-9_-]{0,63}$ ]] || {
            echo "invalid map name" >&2
            exit 65
        }
        case "${activate}" in
            true)
                exec /usr/local/bin/drmap mapping -s -n "${map_name}"
                ;;
            false)
                exec /usr/local/bin/drmap mapping -s -n "${map_name}" -b
                ;;
            *) usage ;;
        esac
        ;;
    stop)
        [ "$#" -eq 1 ] || usage
        exec /usr/local/bin/drmap stop_mapping
        ;;
    *) usage ;;
esac
