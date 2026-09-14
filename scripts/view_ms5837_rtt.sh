#!/usr/bin/env bash

set -euo pipefail

workspace_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
elf_path="${workspace_dir}/build/MS5837_CAL_DEBUG/UserApp/AUV_zit6.elf"
pyocd_path="${PYOCD_BIN:-${HOME}/.local/bin/pyocd}"

if [[ ! -f "${elf_path}" ]]; then
    echo "ERROR: ELF not found: ${elf_path}" >&2
    echo "Run the MS5837 CAL Debug build first." >&2
    exit 1
fi

if [[ ! -x "${pyocd_path}" ]]; then
    echo "ERROR: pyocd not found or not executable: ${pyocd_path}" >&2
    exit 1
fi

rtt_address="$(arm-none-eabi-nm "${elf_path}" \
    | awk '$3 == "_SEGGER_RTT" {print "0x" $1; exit}')"

if [[ -z "${rtt_address}" ]]; then
    echo "ERROR: _SEGGER_RTT not found in ${elf_path}" >&2
    exit 1
fi

echo "Starting MS5837 RTT at ${rtt_address}"
exec "${pyocd_path}" rtt \
    --target stm32h743xx \
    --uid 6894719E6D7C \
    --frequency 2m \
    --connect attach \
    --address "${rtt_address}"
