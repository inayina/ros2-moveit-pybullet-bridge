#!/usr/bin/env bash
# Install Playwright (Chromium) for HOC browser screenshots (project .venv).
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
VENV_PY="${ROOT}/.venv/bin/python3"

if [ ! -x "${VENV_PY}" ]; then
  echo "==> Create project venv"
  python3 -m venv "${ROOT}/.venv"
fi

echo "==> Install Playwright Python package (into .venv)"
"${VENV_PY}" -m pip install -q playwright

echo "==> Install Chromium browser for Playwright"
"${VENV_PY}" -m playwright install chromium

"${VENV_PY}" -c "from playwright.sync_api import sync_playwright; print('[PASS] Playwright ready')"
