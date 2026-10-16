#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p build
sources=(tests/test_rate_core.c research/rate_core.c research/original/{adrc,mrac,ukf,matrix,quaternion,sensor_gate}.c)
gcc -std=c99 -O1 -g -Wall -Wextra -Werror -Iresearch -fsanitize=undefined -fno-sanitize-recover=all -fno-omit-frame-pointer "${sources[@]}" -lm -o build/test_rate
./build/test_rate
gcc -std=c99 -Ofast -Wall -Wextra -Werror -Iresearch "${sources[@]}" -lm -o build/test_rate_fast
./build/test_rate_fast
python3 -m unittest discover -s tests -p "test_*.py" -v
