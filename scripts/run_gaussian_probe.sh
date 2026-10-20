#!/usr/bin/env bash
# Preserve the user's Conda install. Disable the shadowing ~/.local CPU torch.
set -euo pipefail
gaussian_workspace="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /home/subin/anaconda3/etc/profile.d/conda.sh
conda activate teamkai-yolo
export PYTHONNOUSERSITE=1
export PYTHONPATH="$gaussian_workspace/outputs/surface_runtime/gsplat-source:$gaussian_workspace/outputs/surface_runtime/gaussian-deps:$gaussian_workspace/outputs/surface_runtime/cuda-build"
export CUDA_HOME="$gaussian_workspace/outputs/surface_runtime/cuda-toolkit/nvidia/cu13"
export PATH="$CUDA_HOME/bin:$gaussian_workspace/outputs/surface_runtime/cuda-build/bin:$PATH"
export CPATH="$CUDA_HOME/include/cccl"
export LD_LIBRARY_PATH="$CUDA_HOME/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
export TORCH_EXTENSIONS_DIR="$gaussian_workspace/outputs/surface_runtime/torch_extensions"
export CUDA_CACHE_PATH="$gaussian_workspace/outputs/surface_runtime/cuda_cache"
export TORCH_CUDA_ARCH_LIST=12.0
export MAX_JOBS=2
export BUILD_3DGS=1 BUILD_2DGS=0 BUILD_3DGUT=0 BUILD_RELOC=0 BUILD_ADAM=0 BUILD_LOSSES=0 BUILD_CAMERA_WRAPPERS=0
export NUM_CHANNELS=3,4
cd "$gaussian_workspace"
exec python -s "$@"
