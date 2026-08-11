#!/usr/bin/env bash
# M300 container entry point. The UART transceiver is a carrier-board resource
# and must be enabled in host sysfs before PSDK opens /dev/ttyTHS0.
set -euo pipefail

HOST_SYS="${M300_HOST_SYS:-/host-sys}"
GPIO_NUMBER="${M300_UART_ENABLE_GPIO:-472}"

find_gpio_dir() {
    local candidate
    for candidate in "${HOST_SYS}/class/gpio/PY.02" \
                     "${HOST_SYS}/class/gpio/gpio${GPIO_NUMBER}"; do
        if [[ -d "${candidate}" ]]; then
            printf '%s\n' "${candidate}"
            return 0
        fi
    done
    return 1
}

enable_uart_transceiver() {
    local gpio_dir attempt

    [[ -d "${HOST_SYS}/class/gpio" ]] || {
        echo "[m300-start] ERROR: host GPIO sysfs is unavailable at ${HOST_SYS}" >&2
        return 1
    }

    if ! gpio_dir="$(find_gpio_dir)"; then
        printf '%s' "${GPIO_NUMBER}" > "${HOST_SYS}/class/gpio/export" 2>/dev/null || true
        for attempt in {1..20}; do
            if gpio_dir="$(find_gpio_dir)"; then
                break
            fi
            sleep 0.05
        done
    fi

    [[ -n "${gpio_dir:-}" && -d "${gpio_dir}" ]] || {
        echo "[m300-start] ERROR: GPIO ${GPIO_NUMBER} was not exported; cannot enable M300 UART" >&2
        return 1
    }

    printf 'out' > "${gpio_dir}/direction"
    printf '1' > "${gpio_dir}/value"
    [[ "$(cat "${gpio_dir}/value")" == "1" ]] || {
        echo "[m300-start] ERROR: GPIO ${GPIO_NUMBER} did not remain high" >&2
        return 1
    }
    echo "[m300-start] M300 UART transceiver enabled via ${gpio_dir}"
}

enable_uart_transceiver
[[ -c /dev/ttyTHS0 && -c /dev/ttyACM0 ]] || {
    echo "[m300-start] ERROR: expected /dev/ttyTHS0 and /dev/ttyACM0" >&2
    exit 1
}

set +u
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash
set -u
exec python3 /work/main.py
