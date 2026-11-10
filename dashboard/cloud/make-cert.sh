#!/bin/sh
# Makes a self-signed certificate when none is mounted, so nginx can start on 443.
# Browsers warn about it once; Cloudflare accepts it with SSL mode "Full". Replace the two
# files in certs/ with a real certificate (Let's Encrypt or Cloudflare Origin) any time.
set -e
dir=/etc/nginx/certs
[ -s "$dir/fullchain.pem" ] && [ -s "$dir/privkey.pem" ] && exit 0
mkdir -p "$dir"
name="${DOMAIN:-armbot}"
openssl req -x509 -newkey rsa:2048 -nodes -days 825 -subj "/CN=$name" \
    -addext "subjectAltName=DNS:$name" \
    -keyout "$dir/privkey.pem" -out "$dir/fullchain.pem" 2>/dev/null
echo "armbot: made a self-signed certificate for $name in $dir"
