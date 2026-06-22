#!/bin/bash
# ============================================================================
# ZIT6 AUV 静态分析脚本
# 运行 clang-tidy 和 cppcheck 扫描用户代码（UserApp/ 目录）
#
# 用法：
#   ./scripts/run_static_analysis.sh              # 使用已有 build 目录
#   ./scripts/run_static_analysis.sh -b build/Debug  # 指定构建目录
#   ./scripts/run_static_analysis.sh -f           # 只运行快速模式 (cppcheck only)
# ============================================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---- 参数解析 ----
BUILD_DIR=""
FAST_MODE=false

while getopts "b:f" opt; do
    case $opt in
        b) BUILD_DIR="$OPTARG" ;;
        f) FAST_MODE=true ;;
        *) echo "Usage: $0 [-b build_dir] [-f]" >&2; exit 1 ;;
    esac
done

cd "$PROJECT_ROOT"

# 尝试自动发现 build 目录
if [ -z "$BUILD_DIR" ]; then
    for d in build build/Debug build/Release; do
        if [ -f "$d/compile_commands.json" ]; then
            BUILD_DIR="$d"
            break
        fi
    done
fi

if [ -z "$BUILD_DIR" ]; then
    echo "❌ 找不到 compile_commands.json，请先运行 cmake："
    echo "   cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Debug \\"
    echo "       -DCMAKE_TOOLCHAIN_FILE=cmake/gcc-arm-none-eabi.cmake \\"
    echo "       -DCMAKE_EXPORT_COMPILE_COMMANDS=ON"
    exit 1
fi

echo "========================================================="
echo "  ZIT6 AUV - Static Analysis"
echo "  Build dir: $BUILD_DIR"
echo "========================================================="

HAS_ERROR=false

# ============================================================================
# 1. cppcheck — 轻量级快速扫描
# ============================================================================
echo ""
echo "[1/2] Running cppcheck..."
echo ""

CPPCHECK_FLAGS=(
    --enable=warning,performance,portability,information
    --suppress=missingIncludeSystem
    --suppress=unmatchedSuppression
    --suppress=ctunullpointer:UserApp/Thirdparty/cJSON/cJSON.c
    --suppress=toomanyconfigs
    --suppress=checkersReport
    --suppress=syntaxError:UserApp/Thirdparty/Eigen/*
    --error-exitcode=1
    --inline-suppr
    --language=c++
    --std=c++17
    --platform=unix64
    --check-level=normal
    -i build
    -i micro_ros_stm32cubemx_utils
    -i Drivers
    -i Middlewares
    -i install
    -i CMakeFiles
    -I UserApp/Common
    -I UserApp/Config
    -I UserApp/Peripherals
    -I UserApp/Porting
    -I UserApp/Algorithm
    -I UserApp/Component
    -I UserApp/Application
    -I UserApp/MicroRos
    -I UserApp/Thirdparty
    -I Core/Inc
    -I Core/Src
    UserApp/
)

echo ">>> cppcheck ${CPPCHECK_FLAGS[*]}"
if cppcheck "${CPPCHECK_FLAGS[@]}"; then
    echo "✅ cppcheck: 未发现问题"
else
    echo "⚠️  cppcheck: 发现问题（见上方输出）"
    HAS_ERROR=true
fi

# ============================================================================
# 2. clang-tidy — 深度分析（跳过快速模式）
# ============================================================================
if [ "$FAST_MODE" = false ]; then
    echo ""
    echo "[2/2] Running clang-tidy..."
    echo ""

    # 查找 clang-tidy 可执行文件
    CLANG_TIDY=""
    for cmd in run-clang-tidy run-clang-tidy-18 run-clang-tidy-17; do
        if command -v "$cmd" &>/dev/null; then
            CLANG_TIDY="$cmd"
            break
        fi
    done

    if [ -z "$CLANG_TIDY" ]; then
        echo "⚠️  未找到 run-clang-tidy，尝试直接使用 clang-tidy..."
        CLANG_TIDY="clang-tidy"
    fi

    # 收集 UserApp 下的源文件（跳过 cJSON 第三方库）
    USER_SOURCES=$(find UserApp -name '*.cpp' -o -name '*.c' | grep -v Thirdparty/cJSON | sort)

    if [ "$CLANG_TIDY" = "clang-tidy" ]; then
        # 直接对每个文件运行 clang-tidy
        for src in $USER_SOURCES; do
            echo "  Analyzing: $src"
            clang-tidy --quiet "$src" -p "$BUILD_DIR" 2>/dev/null || true
        done
        echo "✅ clang-tidy: 分析完成"
    else
        echo ">>> $CLANG_TIDY -p $BUILD_DIR -header-filter='UserApp/.*' UserApp/"
        # run-clang-tidy 自动并行处理所有文件
        if $CLANG_TIDY -p "$BUILD_DIR" -header-filter='UserApp/.*' \
            -quiet UserApp/ 2>/dev/null; then
            echo "✅ clang-tidy: 未发现问题"
        else
            echo "⚠️  clang-tidy: 发现问题（见上方输出）"
            HAS_ERROR=true
        fi
    fi
else
    echo ""
    echo "[2/2] Skipped (fast mode). Add -f for full analysis."
fi

# ============================================================================
# 3. 结果汇总
# ============================================================================
echo ""
echo "========================================================="
if [ "$HAS_ERROR" = true ]; then
    echo "  静态分析完成 — ⚠️  存在警告/错误"
    exit 1
else
    echo "  静态分析完成 — ✅ 一切正常"
fi
echo "========================================================="
