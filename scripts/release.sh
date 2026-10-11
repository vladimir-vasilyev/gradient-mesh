#!/bin/bash
# Builds, signs and (optionally) notarizes the release DMG.
#
#   scripts/release.sh               build + sign app + signed DMG   (nothing is uploaded)
#   scripts/release.sh --notarize    additionally notarize + staple the app and the DMG
#                                    (uploads both to Apple's notary service)
#
# Needs: a "Developer ID Application" identity in the keychain, and for
# --notarize a notarytool profile (xcrun notarytool store-credentials "$PROFILE" ...).
# Output: dist/GradientMeshStudio-<version>-arm64.dmg
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IDENTITY="${SIGN_IDENTITY:-Developer ID Application: Vladimir Vasilyev (JFUPWZSDN7)}"
PROFILE="${NOTARY_PROFILE:-gms-notary}"
NOTARIZE=0; [ "${1:-}" = "--notarize" ] && NOTARIZE=1
BUILD="$ROOT/build_release"
DIST="$ROOT/dist"
APP_NAME="GradientMeshStudio"

step() { echo; echo "==> $*"; }

step "Static dependencies"
"$ROOT/scripts/build_deps.sh"

step "Release build (arm64, macOS 12)"
cmake -S "$ROOT" -B "$BUILD" -DCMAKE_BUILD_TYPE=Release -DCMAKE_OSX_DEPLOYMENT_TARGET=12.0 \
      -DCMAKE_OSX_ARCHITECTURES=arm64 -DCMAKE_PREFIX_PATH="$ROOT/build_deps/install" > /dev/null
cmake --build "$BUILD" -j --target "$APP_NAME" 2>&1 | grep -E "error|warning|Built target $APP_NAME" || true

SRC_APP="$BUILD/$APP_NAME.app"
BIN="$SRC_APP/Contents/MacOS/$APP_NAME"
VERSION="$(/usr/libexec/PlistBuddy -c 'Print :CFBundleShortVersionString' "$SRC_APP/Contents/Info.plist")"

step "Checking the binary"
THIRD=$(otool -L "$BIN" | tail -n +2 | grep -v -E "^\s+(/System/|/usr/lib/)" || true)
[ -z "$THIRD" ] || { echo "Unexpected non-system dependencies:"; echo "$THIRD"; exit 1; }
MINOS=$(otool -l "$BIN" | awk '/LC_BUILD_VERSION/{f=1} f&&/minos/{print $2; exit}')
[ "$MINOS" = "12.0" ] || { echo "minos is $MINOS, expected 12.0"; exit 1; }
[ "$(lipo -archs "$BIN")" = "arm64" ] || { echo "not arm64-only"; exit 1; }
echo "arm64, minos $MINOS, no third-party dylibs, version $VERSION"

STAGE="$DIST/stage"
rm -rf "$DIST"; mkdir -p "$STAGE"
APP="$STAGE/$APP_NAME.app"
cp -R "$SRC_APP" "$APP"

step "Signing the app (hardened runtime)"
codesign --force --options runtime --timestamp --sign "$IDENTITY" "$APP"
codesign --verify --strict --verbose=2 "$APP"

notarize() {  # file
  local out id
  out="$(xcrun notarytool submit "$1" --keychain-profile "$PROFILE" --wait 2>&1)"; echo "$out" | tail -6
  id="$(echo "$out" | awk '/^  id:/{print $2; exit}')"
  if ! echo "$out" | grep -q "status: Accepted"; then
    echo "Notarization failed; log:"; xcrun notarytool log "$id" --keychain-profile "$PROFILE" || true; exit 1
  fi
}

if [ "$NOTARIZE" = 1 ]; then
  step "Notarizing the app"
  ditto -c -k --keepParent "$APP" "$DIST/app-for-notary.zip"
  notarize "$DIST/app-for-notary.zip"
  xcrun stapler staple "$APP"
fi

step "Building the DMG"
DMG="$DIST/$APP_NAME-$VERSION-arm64.dmg"
ln -s /Applications "$STAGE/Applications"
hdiutil create -volname "Gradient Mesh Studio" -srcfolder "$STAGE" -fs HFS+ -format UDZO -ov "$DMG" > /dev/null
codesign --force --timestamp --sign "$IDENTITY" "$DMG"
codesign --verify --verbose=2 "$DMG"

if [ "$NOTARIZE" = 1 ]; then
  step "Notarizing the DMG"
  notarize "$DMG"
  xcrun stapler staple "$DMG"
  xcrun stapler validate "$DMG"
  spctl -a -t open --context context:primary-signature -vv "$DMG"
fi

step "Result"
ls -la "$DMG"; shasum -a 256 "$DMG"
[ "$NOTARIZE" = 1 ] || echo "(signed only -- run with --notarize to notarize and staple)"
