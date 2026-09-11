#!/usr/bin/env bash
# =============================================================================
# Cadence Virtuoso & Spectre Environment Initialization
# =============================================================================
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT_DIR/config/env_virtuoso.sh"
echo "[ENV] Cadence Virtuoso & Spectre environment initialized from $ROOT_DIR"
