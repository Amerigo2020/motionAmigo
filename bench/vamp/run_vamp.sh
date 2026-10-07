#!/usr/bin/env bash
# Reproduces the VAMP comparison on the MotionBenchMaker problems for the Panda.
#
# Requirements: uv, a C++17 compiler, CMake and Eigen 3 (e.g. `apt install cmake libeigen3-dev`).
# vamp-planner is built from source by pip (PyPI only ships the sdist) with -march=native.
set -euo pipefail
cd "$(dirname "$0")"

VAMP_COMMIT=27cb9b66ebd8d101ce88d330f3b028398afc1cfa

# 1. VAMP (from PyPI) and the scripts' dependencies.
uv sync

# 2. The VAMP repository, only for the MotionBenchMaker problem files and its converter.
if [ ! -d vamp-src ]; then
  git clone https://github.com/KavrakiLab/vamp.git vamp-src
fi
git -C vamp-src checkout -q "$VAMP_COMMIT"
if [ ! -f vamp-src/resources/panda/problems.pkl ]; then
  (cd vamp-src && ../.venv/bin/python resources/problem_tar_to_pkl_json.py --robot panda)
fi

# 3. Run VAMP (default settings and with dynamic domain disabled).
uv run python run_vamp_mbm.py vamp-src/resources/panda/problems.pkl --out ../results/mbm-vamp-default.json
uv run python run_vamp_mbm.py vamp-src/resources/panda/problems.pkl --dynamic_domain False \
  --out ../results/mbm-vamp-no-dd.json

# 4. Export the same problems for motionAmigo and run it.
uv run python export_mbm.py vamp-src/resources/panda/problems.pkl ../data/mbm/panda_mbm.json
cd ../..
cargo build --release -p motionamigo-bench
for checker in simd portable scalar; do
  target/release/motionamigo-bench mbm --checker "$checker"
done
cp bench/results/mbm-motionamigo-simd.json /tmp/mbm-simd.json
target/release/motionamigo-bench mbm --simplify-rounds 2 --simplify-attempts 0
mv bench/results/mbm-motionamigo-simd.json bench/results/mbm-motionamigo-simd-greedy.json
mv /tmp/mbm-simd.json bench/results/mbm-motionamigo-simd.json

# 5. Comparison table.
python3 bench/compare_mbm.py | tee bench/results/mbm-comparison.md
