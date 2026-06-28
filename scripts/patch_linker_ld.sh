#!/bin/bash
# ============================================================================
# 修复 CubeMX 生成的链接脚本 — CMake configure 时自动调用
# CubeMX 生成的 STM32H743XX_FLASH.ld 有以下问题：
#   1) .bss 段被放在 DTCMRAM（128KB），实际需 RAM（AXI SRAM 512KB）
#   2) .dma_buffer 自定义 DMA 缓冲区段需追加到 RAM_D2
# ============================================================================
set -euo pipefail

LD_FILE="$1"
[ -f "$LD_FILE" ] || exit 0

grep -q "patched_by_cmake_dma_buffer" "$LD_FILE" 2>/dev/null && exit 0

echo "Patching linker script: $LD_FILE"

# 1) .bss 段从 DTCMRAM → RAM
sed -i 's/}>DTCMRAM$/}>RAM/' "$LD_FILE"

# 2) 插入 .dma_buffer 段
if ! grep -q "dma_buffer" "$LD_FILE"; then
  LINE=$(grep -n '_sidata = LOADADDR' "$LD_FILE" | head -1 | cut -d: -f1)
  if [ -n "$LINE" ]; then
    sed -i "${LINE}i\\\n  /* patched_by_cmake_dma_buffer */\n  .dma_buffer (NOLOAD) :\n  {\n    . = ALIGN(32);\n    *(.dma_buffer)\n    . = ALIGN(32);\n  } >RAM_D2\n" "$LD_FILE"
  fi
fi

echo "Linker script patched successfully"
