#!/bin/bash
# ==============================================================================
# Script: Run Single UVM Test using Synopsys VCS
# Usage : ./scripts/run_vcs.sh [TEST_NAME] [VERBOSITY] [SEED]
# ==============================================================================
set -e

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

TEST="${1:-newton_multivar_quadratic_test}"
VERB="${2:-UVM_MEDIUM}"
SEED="${3:-1}"

cd "${ROOT_DIR}"
make run TEST="${TEST}" VERB="${VERB}" SEED="${SEED}"
