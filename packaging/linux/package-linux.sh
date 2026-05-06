#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
BUILD_DIR="${BUILD_DIR:-"$ROOT/build-release/linux"}"
DIST_DIR="${DIST_DIR:-"$ROOT/dist"}"
STAGE_DIR="${STAGE_DIR:-"$BUILD_DIR/package/grblhal-flexihal-sim"}"
ARCHIVE_NAME="${ARCHIVE_NAME:-grblhal-flexihal-sim-linux-x64.tar.gz}"

cmake -S "$ROOT" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_WINDOWS_LAUNCHER=OFF
cmake --build "$BUILD_DIR" --config Release -j

rm -rf "$STAGE_DIR"
cmake --install "$BUILD_DIR" --prefix "$STAGE_DIR"

mkdir -p "$STAGE_DIR/sdcard"
cat > "$STAGE_DIR/run.sh" <<'RUNSH'
#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
exec ./bin/grblHAL_flexihal_sim "$@"
RUNSH
chmod +x "$STAGE_DIR/run.sh"

mkdir -p "$DIST_DIR"
tar -C "$(dirname "$STAGE_DIR")" -czf "$DIST_DIR/$ARCHIVE_NAME" "$(basename "$STAGE_DIR")"

echo "$DIST_DIR/$ARCHIVE_NAME"
