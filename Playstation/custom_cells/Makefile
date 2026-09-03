SHELL := /bin/bash
.SHELLFLAGS := -eu -o pipefail -c
.DEFAULT_GOAL := help

VIRTUOSO_WS := /home/user17/Cadence/Virtuoso_CustomIC_Design
VIRTUOSO    := $(shell command -v virtuoso 2>/dev/null || echo "$(VIRTUOSO_WS)/tool/ic618/bin/virtuoso")
SPECTRE     := $(shell command -v spectre 2>/dev/null || echo "/home/user17/Cadence/Spectre_Simulation/tool/spectre211/bin/spectre")

WORK_DIR    := $(CURDIR)/work
LOG_DIR     := $(CURDIR)/logs
REPORT_DIR  := $(CURDIR)/reports
OUTPUT_DIR  := $(CURDIR)/outputs
CONFIG_DIR  := $(CURDIR)/config
OA_DIR      := $(CURDIR)/oa_libs
SCRIPTS_DIR := $(CURDIR)/scripts

export CDSHOME             := $(VIRTUOSO_WS)/tool/ic618
export CDS_AUTO_64BIT      := ALL
export CDS_Netlisting_Mode := Analog
export CDS_LIC_FILE        := 5280@14.139.1.126

CELL ?= compressor_4to2

# All 12 Full-Custom Cells in Library
ALL_CELLS := \
	booth_encoder \
	booth_selector \
	compressor_4to2 \
	csa_3to2_slice \
	kogge_stone_cells \
	cas_divider_slice \
	srt_radix4_stage \
	bitcell_8t_2r1w \
	sram_6t_cell \
	dynamic_overflow_detect \
	fast_comparator \
	tg_mux

.PHONY: help check preflight virtuoso gui schematic view_schematic view_layout sch layout sim sim_all wave clean

help:
	@echo "=========================================================================="
	@echo " Full-Custom IC Design & Verification Library"
	@echo " Cadence Virtuoso IC618 & Spectre Simulation Flow"
	@echo "=========================================================================="
	@echo " Simulation & Verification:"
	@echo "   make sim CELL=<name>         Run Spectre simulation & generate report"
	@echo "   make wave CELL=<name>        Open Cadence ViVA Waveform Viewer"
	@echo "   make sim_all                 Run automated simulation across all 12 cells"
	@echo ""
	@echo " Schematic & Layout (Cadence Virtuoso):"
	@echo "   make schematic CELL=<name>   Generate Virtuoso schematic from SPICE netlist"
	@echo "   make view_schematic CELL=<n> Open schematic diagram in Virtuoso GUI"
	@echo "   make view_layout CELL=<name> Open layout editor in Virtuoso GUI"
	@echo "   make virtuoso                Launch Virtuoso Library Manager"
	@echo ""
	@echo " Maintenance & Diagnostic:"
	@echo "   make check                   Verify EDA tool binaries and licenses"
	@echo "   make clean                   Clean temporary runtime files (preserves oa_libs/)"
	@echo "=========================================================================="
	@echo "Available Custom Cells:"
	@echo "  Arithmetic  : booth_encoder, booth_selector, compressor_4to2,"
	@echo "                csa_3to2_slice, kogge_stone_cells"
	@echo "  Divider     : cas_divider_slice, srt_radix4_stage"
	@echo "  Memory      : bitcell_8t_2r1w, sram_6t_cell"
	@echo "  Specialized : dynamic_overflow_detect, fast_comparator, tg_mux"
	@echo "=========================================================================="

check:
	@echo "=== Checking Cadence Virtuoso & Spectre Environment ==="
	@which virtuoso || { echo "[ERROR] Virtuoso not found in PATH. Run: source $(CONFIG_DIR)/env_virtuoso.sh"; exit 1; }
	@which spectre  || { echo "[ERROR] Spectre not found in PATH. Run: source $(CONFIG_DIR)/env_virtuoso.sh"; exit 1; }
	@"$(VIRTUOSO)" -W
	@"$(SPECTRE)" -V
	@echo "[PASS] Cadence Virtuoso & Spectre are ready."

preflight:
	@test -f $(CONFIG_DIR)/cds.lib || { echo "[ERROR] Missing $(CONFIG_DIR)/cds.lib" >&2; exit 1; }
	@mkdir -p $(WORK_DIR) $(LOG_DIR) $(REPORT_DIR) $(OUTPUT_DIR) $(OA_DIR)
	@cp -f $(CONFIG_DIR)/cds.lib $(WORK_DIR)/cds.lib

virtuoso: preflight check
	@echo "Launching Cadence Virtuoso GUI in $(WORK_DIR)..."
	@cd $(WORK_DIR) && "$(VIRTUOSO)" -log $(LOG_DIR)/virtuoso.log &

gui: virtuoso

schematic: preflight check
	@bash $(SCRIPTS_DIR)/import_schematic.sh "$(CELL)"

view_schematic: preflight check
	@bash $(SCRIPTS_DIR)/open_design.sh "$(CELL)" schematic

sch: view_schematic

view_layout: preflight check
	@bash $(SCRIPTS_DIR)/open_design.sh "$(CELL)" layout

layout: view_layout

sim: preflight check
	@bash $(SCRIPTS_DIR)/run_cell_sim.sh "$(CELL)"

sim_all: preflight check
	@echo "=========================================================================="
	@echo " Running Spectre Simulation Regression on All Custom Cells"
	@echo "=========================================================================="
	@for c in $(ALL_CELLS); do \
		echo "--------------------------------------------------------"; \
		bash $(SCRIPTS_DIR)/run_cell_sim.sh "$$c"; \
	done
	@echo "=========================================================================="
	@echo "[PASS] All cell simulations completed successfully."
	@echo "Logs     : $(LOG_DIR)/"
	@echo "Reports  : $(REPORT_DIR)/"
	@echo "Waveforms: $(OUTPUT_DIR)/"
	@echo "=========================================================================="

wave: preflight check
	@bash $(SCRIPTS_DIR)/view_wave.sh "$(CELL)"

clean:
	@echo "Cleaning temporary simulation runtime files, logs, and lockfiles..."
	@rm -rf $(WORK_DIR)/* $(LOG_DIR)/* $(OUTPUT_DIR)/* $(REPORT_DIR)/*
	@rm -f *.log *.cdslck cds.log* pvs.log* spiceIn.log
	@find . -name "*.cdslck" -delete 2>/dev/null || true
	@mkdir -p $(WORK_DIR) $(LOG_DIR) $(OUTPUT_DIR) $(REPORT_DIR)
	@echo "[PASS] Workspace cleaned (oa_libs/ database preserved)."
