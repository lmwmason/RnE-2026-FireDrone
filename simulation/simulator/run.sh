#!/usr/bin/env bash
# Manual X8 FPV is the default; legacy regression runs are explicit.
#   ./run.sh                 USB joystick FPV / angle mode
#   ./run.sh all             legacy X4 scenarios + report
#   ./run.sh view gust       3-D playback of one scenario (patched firmware)
#   ./run.sh video gust      render out/gust.mp4
#   ./run.sh timing          Cortex-M4 cycle budget (needs unicorn + capstone)
cd "$(dirname "${BASH_SOURCE[0]}")"
mkdir -p out

build() {
    echo "=== Building ==="
    if ! make all; then
        echo "Build failed."; exit 1
    fi
}

case "${1:-fpv}" in
    fpv)
        make live || exit 1
        if [ ! -x .venv/bin/python ]; then python3 -m venv --system-site-packages .venv || exit 1; fi
        if ! .venv/bin/python -c "import pygame, numpy" >/dev/null 2>&1; then
            .venv/bin/python -m pip install -r requirements-fpv.txt || exit 1
        fi
        if [ "$#" -gt 0 ]; then shift; fi
        .venv/bin/python scripts/fpv.py "$@"
        ;;
    campaign)
        make x8 || exit 1
        shift
        python3 scripts/campaign.py "$@"
        ;;
    x8-view)
        make x8 || exit 1
        ./firedrone_x8_sim --scenario "${2:-gauntlet}" --out "out/x8_${2:-gauntlet}.csv" || exit 1
        python3 scripts/visualize.py --build x8 --scenario "${2:-gauntlet}"
        ;;
    view)
        build >/dev/null
        python3 scripts/analysis.py --scenarios "${2:-nominal}" --no-detail >/dev/null
        python3 scripts/visualize.py --build "${3:-patched}" --scenario "${2:-nominal}"
        ;;
    video)
        build >/dev/null
        python3 scripts/analysis.py --scenarios "${2:-nominal}" --no-detail >/dev/null
        python3 scripts/visualize.py --build "${3:-patched}" --scenario "${2:-nominal}" \
            --save "out/${2:-nominal}.mp4"
        ;;
    timing)
        for opt in -Og -O2 -O3; do
            echo "=== Cortex-M4F cycle budget, firmware compiled with $opt ==="
            make -C timing clean >/dev/null && make -C timing OPT="$opt" >/dev/null || exit 1
            python3 timing/measure_cycles.py --iters 20 --json "out/timing_${opt#-}.json" || exit 1
            echo
        done
        ;;
    all)
        build
        if python3 -c "import unicorn, capstone" 2>/dev/null; then
            echo
            echo "=== Cortex-M4F cycle budget (emulated STM32) ==="
            ./run.sh timing
        else
            echo "(skip MCU timing: pip install unicorn capstone to enable)"
        fi
        echo
        echo "=== Scenarios (original vs current firmware) ==="
        python3 scripts/analysis.py
        echo
        echo "Plots and tables are in out/   (summary.md, comparison.png, <build>_<scenario>.png)"
        echo "3-D view:  ./run.sh view gust"
        ;;
    *)
        echo "usage: $0 [fpv [options]|all|x8-view <scenario>|campaign [--cycles N --laps N]|view <scenario>|video <scenario>|timing [-O2]]"; exit 1
        ;;
esac
