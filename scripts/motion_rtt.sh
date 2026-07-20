#!/usr/bin/env bash

# MOTION_DEBUG RTT helper.
#
#   ./scripts/motion_rtt.sh rttd   Start a TCP RTT bridge on RTT_PORT.
#   ./scripts/motion_rtt.sh nc     Connect to the bridge with nc.
#   ./scripts/motion_rtt.sh view   Connect directly with pyOCD RTT.
#   ./scripts/motion_rtt.sh all    Start rttd and connect with nc.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ELF_PATH="${ELF_PATH:-${ROOT_DIR}/build/MOTION_DEBUG/UserApp/AUV_zit6.elf}"
PYOCD_BIN="${PYOCD_BIN:-${HOME}/.local/bin/pyocd}"
RTT_PORT="${RTT_PORT:-8023}"
RTT_HOST="${RTT_HOST:-127.0.0.1}"
RTT_CHANNEL="${RTT_CHANNEL:-0}"

die() {
    echo "ERROR: $*" >&2
    exit 1
}

require_command() {
    command -v "$1" >/dev/null 2>&1 || die "command not found: $1"
}

ensure_elf() {
    if [[ ! -f "${ELF_PATH}" ]]; then
        require_command cmake
        echo "ELF not found; building MOTION_DEBUG..."
        cmake --preset MOTION_DEBUG
        cmake --build --preset MOTION_DEBUG
    fi
    [[ -f "${ELF_PATH}" ]] || die "ELF not found: ${ELF_PATH}"
}

get_rtt_address() {
    require_command arm-none-eabi-nm
    local address
    address="$(arm-none-eabi-nm "${ELF_PATH}" \
        | awk '$3 == "_SEGGER_RTT" {print "0x" $1; exit}')"
    [[ -n "${address}" ]] || die "_SEGGER_RTT not found in ${ELF_PATH}"
    printf '%s\n' "${address}"
}

pyocd_rtt() {
    local address="$1"
    exec "${PYOCD_BIN}" rtt \
        --target stm32h743xx \
        --uid 6894719E6D7C \
        --frequency 2m \
        --connect attach \
        --address "${address}" \
        --up-channel-id "${RTT_CHANNEL}" \
        --down-channel-id "${RTT_CHANNEL}"
}

run_view() {
    require_command "${PYOCD_BIN}"
    ensure_elf
    local address
    address="$(get_rtt_address)"
    echo "Viewing MOTION_DEBUG RTT at ${address}, channel ${RTT_CHANNEL}"
    pyocd_rtt "${address}"
}

run_rttd() {
    require_command socat
    require_command "${PYOCD_BIN}"
    ensure_elf
    local address
    address="$(get_rtt_address)"

    echo "MOTION_DEBUG RTT bridge: ${RTT_HOST}:${RTT_PORT}"
    echo "RTT control block: ${address}"
    echo "Connect from another terminal with: $0 nc"

    # pyOCD RTT uses stdin/stdout as its terminal. socat exposes that stream
    # as a local TCP endpoint so nc or another host-side tool can connect.
    exec socat \
        "TCP-LISTEN:${RTT_PORT},bind=${RTT_HOST},reuseaddr,fork" \
        "EXEC:${PYOCD_BIN} rtt --target stm32h743xx --uid 6894719E6D7C --frequency 2m --connect attach --address ${address} --up-channel-id ${RTT_CHANNEL} --down-channel-id ${RTT_CHANNEL},pty,raw,echo=0,stderr"
}

run_nc() {
    local nc_bin
    if command -v nc >/dev/null 2>&1; then
        nc_bin=nc
    elif command -v netcat >/dev/null 2>&1; then
        nc_bin=netcat
    else
        die "nc/netcat is required"
    fi

    echo "Connecting to MOTION_DEBUG RTT at ${RTT_HOST}:${RTT_PORT}"
    echo "Type H, X0.2, S0, L1, or STOP; press Enter to send."
    exec "${nc_bin}" "${RTT_HOST}" "${RTT_PORT}"
}

run_all() {
    run_rttd &
    local server_pid=$!

    cleanup() {
        kill "${server_pid}" 2>/dev/null || true
        wait "${server_pid}" 2>/dev/null || true
    }
    trap cleanup EXIT INT TERM

    for _ in $(seq 1 50); do
        if nc -z "${RTT_HOST}" "${RTT_PORT}" >/dev/null 2>&1; then
            run_nc
            return
        fi
        sleep 0.1
    done

    die "RTT bridge did not open ${RTT_HOST}:${RTT_PORT}"
}

case "${1:-help}" in
    rttd|daemon|agent)
        run_rttd
        ;;
    nc|client)
        run_nc
        ;;
    view)
        run_view
        ;;
    all)
        run_all
        ;;
    help|-h|--help)
        sed -n '1,12p' "$0"
        ;;
    *)
        die "unknown command '$1'; use rttd, nc, view, or all"
        ;;
esac
