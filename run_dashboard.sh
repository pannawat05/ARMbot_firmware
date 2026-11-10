#!/bin/sh
# Build the dashboard with Docker when its source changed, then start the server.
#
#   ./run_dashboard.sh                 # dry run: camera and web page, no servo output
#   ./run_dashboard.sh --execute       # move the real arm (asks first)
#   ./run_dashboard.sh --rebuild ...   # force a fresh Docker build first
#
# Other options go to python/armbot_server.py (--http, --port, --camera serial|usb, --no-mqtt).
# The server itself runs on the Pi, not in Docker: it needs the camera, I2C, and the
# system PyTorch/OpenCV packages that .venv-cpu uses, which would make an image of
# several GB on a nearly full SD card.
set -e
ROOT="$(cd "$(dirname "$0")" && pwd)"
DASH="$ROOT/dashboard"

rebuild=0
if [ "$1" = "--rebuild" ]; then
    rebuild=1
    shift
fi

if [ ! -f "$DASH/dist/index.html" ]; then
    rebuild=1
elif [ -n "$(find "$DASH/src" "$DASH/index.html" "$DASH/package.json" "$DASH/vite.config.js" \
                 -newer "$DASH/dist/index.html" -print -quit)" ]; then
    rebuild=1
fi

if [ "$rebuild" = 1 ]; then
    echo "Building the dashboard with Docker..."
    "$DASH/build-with-docker.sh"
else
    echo "Dashboard is up to date (dashboard/dist)."
fi

cd "$ROOT/python"
exec "$ROOT/.venv-cpu/bin/python" armbot_server.py "$@"
