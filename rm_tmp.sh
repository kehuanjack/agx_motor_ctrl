#!/usr/bin/env bash
# 清理子模块内编译产物与缓存（与 .gitignore 对齐；不删除 venv / .venv）
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

_prune() {
  find "$ROOT" \
    '(' -path '*/.git' -o -path '*/.git/*' \
       -o -path '*/.venv' -o -path '*/.venv/*' \
       -o -path '*/venv' -o -path '*/venv/*' ')' -prune -o \
    "$@" -print0 2>/dev/null || true
}

remove_paths() {
  while IFS= read -r -d '' path; do
    echo "rm -rf -- $path"
    rm -rf -- "$path"
  done
}

remove_files() {
  while IFS= read -r -d '' path; do
    echo "rm -f -- $path"
    rm -f -- "$path"
  done
}

echo "==> 清理编译产物与缓存: $ROOT"

# build/ dist/
remove_paths < <(_prune -type d '(' -name build -o -name dist ')')

# libmotor：**/motor/*.{so,dll,dylib}
remove_files < <(_prune -type f '(' \
  -path '*/motor/*.so' -o -path '*/motor/*.dll' -o -path '*/motor/*.dylib' ')')

# Python 打包
remove_files < <(_prune -type f '(' -name '*.whl' -o -name '*.egg' ')')
remove_paths < <(_prune -type d -name '*.egg-info')
remove_paths < <(_prune -type d -name .eggs)

# Python 缓存
remove_paths < <(_prune -type d -name __pycache__)
remove_files < <(_prune -type f '(' -name '*.pyc' -o -name '*.pyo' ')' )

echo "==> 完成"
