#!/bin/bash
# Regenerates mac/AppIcon.icns from mac/icon/gms_icon.png.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TMP="$(mktemp -d)"; trap 'rm -rf "$TMP"' EXIT
SET="$TMP/AppIcon.iconset"; mkdir -p "$SET"
swift "$ROOT/scripts/make_icon.swift" "$ROOT/mac/icon/gms_icon.png" "$TMP/master.png"
for spec in 16:icon_16x16 32:icon_16x16@2x 32:icon_32x32 64:icon_32x32@2x 128:icon_128x128 256:icon_128x128@2x \
            256:icon_256x256 512:icon_256x256@2x 512:icon_512x512 1024:icon_512x512@2x; do
  sips -z "${spec%%:*}" "${spec%%:*}" "$TMP/master.png" --out "$SET/${spec#*:}.png" > /dev/null
done
iconutil -c icns "$SET" -o "$ROOT/mac/AppIcon.icns"
echo "wrote mac/AppIcon.icns"
