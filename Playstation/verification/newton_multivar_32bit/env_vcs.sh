#!/bin/bash
# ==============================================================================
# Environment Setup Script for Synopsys VCS & Verdi
# Usage: source env_vcs.sh
# ==============================================================================

export VCS_HOME="/home/user17/Synopsys/VCS_Simulation/tool/vcs_install/vcs/W-2024.09"
export VERDI_HOME="/home/user17/Synopsys/Verdi_Debug/tool/verdi_install/verdi/W-2024.09-1"

# Synopsys License Configuration
export SNPSLMD_LICENSE_FILE="27020@14.139.1.126"
export LM_LICENSE_FILE="5280@14.139.1.126"

# Binary & Library Paths
export PATH="${VCS_HOME}/bin:${VERDI_HOME}/bin:${PATH}"
export LD_LIBRARY_PATH="${VERDI_HOME}/share/PLI/VCS/LINUX64:${LD_LIBRARY_PATH}"

echo "======================================================================"
echo "  Synopsys Environment Configured for Newton Multivariable BCD TB"
echo "  VCS_HOME   : ${VCS_HOME}"
echo "  VERDI_HOME : ${VERDI_HOME}"
echo "  VCS Binary : $(which vcs 2>/dev/null || echo 'Not Found')"
echo "  Verdi Bin  : $(which verdi 2>/dev/null || echo 'Not Found')"
echo "======================================================================"
