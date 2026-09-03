#!/usr/bin/env bash
# Cadence Virtuoso & Spectre Environment Setup for Custom Cells

export CDS_ROOT="/home/user17/Cadence"
export CDSHOME="/home/user17/Cadence/Virtuoso_CustomIC_Design/tool/ic618"
export SPECTRE_HOME="/home/user17/Cadence/Spectre_Simulation/tool/spectre211"
export CDS_LIC_FILE="5280@14.139.1.126"
export CDS_AUTO_64BIT="ALL"
export CDS_Netlisting_Mode="Analog"
export CDS_AHDLCMI_ENABLE="YES"

# PDK paths
export FOUNDRY="/home/user17/Cadence/PDK_Foundry/tool/foundry"
export GPDK090="$FOUNDRY/cds_ff_mpt_v_0.5/lib/gpdk090"

# PATH updates
export PATH="$CDSHOME/tools/bin:$CDSHOME/bin:$SPECTRE_HOME/bin:$PATH"
