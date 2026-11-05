#!/bin/bash
# Install or update the windbot client from a GitHub release.
#
# Usage: ./update.sh [VERSION]
#   VERSION  release to install, e.g. 0.3.0. Default is the latest release.
#
# The release replaces the code in INSTALL_DIR (default /home/pi/mobile-wind-sensor-client).
# config.ini, the virtual environment and logs are kept. A git checkout is refused, use
# git pull there instead.

set -euo pipefail

REPO="finnboeger/mobile-wind-sensor-client"
INSTALL_DIR="${INSTALL_DIR:-/home/pi/mobile-wind-sensor-client}"
VERSION="${1:-}"

if [[ -d "$INSTALL_DIR/.git" ]]; then
	echo "$INSTALL_DIR is a git checkout, update it with git pull." >&2
	exit 1
fi

if [[ -n "$VERSION" ]]; then
	URL="https://github.com/$REPO/releases/download/v$VERSION/windbot-client.tar.gz"
else
	URL="https://github.com/$REPO/releases/latest/download/windbot-client.tar.gz"
fi

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

echo "Downloading $URL"
curl -fsSL "$URL" -o "$TMP/windbot-client.tar.gz"
mkdir "$TMP/release"
tar -xzf "$TMP/windbot-client.tar.gz" -C "$TMP/release"

mkdir -p "$INSTALL_DIR"
# Replace the code so files removed in the release don't linger. Everything else in
# INSTALL_DIR (config.ini, .venv, logs) stays.
rm -rf "$INSTALL_DIR/src"
cp -a "$TMP/release/." "$INSTALL_DIR/"

if [[ ! -x "$INSTALL_DIR/.venv/bin/python" ]]; then
	python3 -m venv "$INSTALL_DIR/.venv"
fi
"$INSTALL_DIR/.venv/bin/pip" install --quiet --upgrade pip
"$INSTALL_DIR/.venv/bin/pip" install --quiet -r "$INSTALL_DIR/requirements.txt"

if [[ ! -f "$INSTALL_DIR/config.ini" ]]; then
	cp "$INSTALL_DIR/config.default.ini" "$INSTALL_DIR/config.ini"
fi

# Also for windbots set up before the hotspot existed, so none of them opens a
# hotspot that shares the 4G connection
if ! sudo "$INSTALL_DIR/scripts/setup-hotspot.sh"; then
	echo "Warning: scripts/setup-hotspot.sh failed, the windbot won't open its hotspot." >&2
fi

echo "Installed $(cat "$INSTALL_DIR/VERSION")"
if systemctl is-active --quiet windbot; then
	sudo systemctl restart windbot
	echo "Restarted the windbot service"
fi
