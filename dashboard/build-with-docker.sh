#!/bin/sh
# Build the dashboard in Docker and write the result to dashboard/dist.
set -e
rm -rf dist
cd "$(dirname "$0")"
DOCKER_BUILDKIT=1 docker build --target export --output type=local,dest=dist .
echo "Built dashboard/dist. Start the server: cd ../python && ../.venv-cpu/bin/python armbot_server.py"
