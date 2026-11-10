# Builds the dashboard with Node in a container and exports dist/ to the host:
#   ./build-with-docker.sh
# The server (python/armbot_server.py) runs on the Pi itself: it needs the camera,
# I2C, PyTorch and YOLO from .venv-cpu, which would make an image several GB.
FROM node:20-alpine AS build
WORKDIR /app
COPY package.json package-lock.json* ./
RUN npm install --no-audit --no-fund
COPY . .
RUN npm run build

FROM scratch AS export
COPY --from=build /app/dist /
