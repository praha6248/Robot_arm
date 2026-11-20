#!/usr/bin/env bash
# Pobiera zależności bez uprawnień roota (gdy brak libglfw3-dev / libglm-dev w systemie):
#  - nagłówki glm i GLFW (git, tylko nagłówki),
#  - bibliotekę libglfw.so z oficjalnego koła pip "glfw" (X11).
set -euo pipefail
cd "$(dirname "$0")/.."
mkdir -p third_party/lib

[ -d third_party/glm ]  || git clone --depth 1 --branch 1.0.1 https://github.com/g-truc/glm third_party/glm
[ -d third_party/glfw ] || git clone --depth 1 --branch 3.4   https://github.com/glfw/glfw  third_party/glfw

if [ ! -f third_party/lib/libglfw.so ]; then
    tmp=$(mktemp -d)
    python3 -m pip download --no-deps --only-binary=:all: -d "$tmp" "glfw==2.11.0"
    python3 -m zipfile -e "$tmp"/glfw-*.whl "$tmp/whl"
    cp "$tmp/whl/glfw/x11/libglfw.so" third_party/lib/libglfw.so.3
    ln -sf libglfw.so.3 third_party/lib/libglfw.so
    rm -rf "$tmp"
fi
echo "OK: zależności w third_party/"
