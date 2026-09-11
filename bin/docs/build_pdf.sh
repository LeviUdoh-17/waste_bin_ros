#!/usr/bin/env bash
# Render bin_robot_guide.html to PDF using the Chrome already installed on this machine.
# Edit the .html, run this, commit both.
set -euo pipefail

cd "$(dirname "$0")"
SRC="$PWD/bin_robot_guide.html"
OUT="$PWD/bin_robot_guide.pdf"

CHROME="$(command -v google-chrome || command -v chromium || true)"
if [ -z "$CHROME" ]; then
    echo "No Chrome/Chromium found — needed to render the PDF." >&2
    exit 1
fi

# --no-pdf-header-footer drops Chrome's default URL/date furniture.
# The page size and margins come from the @page rule in the HTML.
"$CHROME" --headless=new --disable-gpu --no-sandbox \
          --no-pdf-header-footer \
          --print-to-pdf="$OUT" \
          "file://$SRC" 2>/dev/null

echo "Wrote $OUT ($(du -h "$OUT" | cut -f1))"
