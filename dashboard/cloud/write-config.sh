#!/bin/sh
# Writes /config.json for the page from the container's environment at every start.
set -e
: "${MQTT_WS_URL:=/mqtt}"
cat > /usr/share/nginx/html/config.json <<JSON
{"mqtt": {"ws_url": "${MQTT_WS_URL}", "username": "${MQTT_USERNAME:-}", "password": "${MQTT_PASSWORD:-}", "prefix": "${MQTT_PREFIX:-armbot}"}}
JSON
echo "armbot: dashboard uses ${MQTT_WS_URL} (prefix ${MQTT_PREFIX:-armbot})"
