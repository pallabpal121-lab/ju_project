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

export CDSHOME          := $(VIRTUOSO_WS)/tool/ic618
export CDS_AUTO_64BIT   := ALL
export CDS_Netlisting_Mode := Analog
export CDS_LIC_FILE     := 5280@14.139.1.126

CELL ?= booth_selector_cell

.PHONY: help check preflight virtuoso gui schematic view_schematic view_layout sch layout sim sim_all wave flow clean

help:
	@echo "=========================================================================="
	@echo " Full-Custom Cell Design Platform (Cadence Virtuoso & Spectre Flow)"
	@echo "=========================================================================="
	@echo "  make schematic CELL=<n>      Generate graphical Schematic from SPICE"
	@echo "  make view_schematic CELL=<n> Open exact Schematic Diagram directly in Virtuoso"
	@echo "  make view_layout CELL=<n>    Open Layout Editor directly in Virtuoso"
	@echo "  make sim CELL=<name>         Run Cadence Spectre simulation on a custom cell"
	@echo "  make wave CELL=<name>        Open Cadence ViVA Waveform Viewer for the cell"
	@echo "  make flow CELL=<name>        Run full push-button flow (Sch -> Sim -> Wave)"
	@echo "  make virtuoso                Launch Cadence Virtuoso IC618 Library Manager"
	@echo "  make sim_all                 Run Spectre simulation on all custom cells"
	@echo "  make clean                   Remove runtime logs, waveforms, reports & lockfiles"
	@echo "=========================================================================="
	@echo "Cell Design Examples for 'make sim CELL=<name>':"
	@echo "  make sim CELL=booth_selector_cell"
	@echo "  make sim CELL=compressor_4to2"
	@echo "  make sim CELL=fused_3input_csa_cell"
	@echo "  make sim CELL=cas_divider_slice"
	@echo "  make sim CELL=bitcell_8t_2r1w"
	@echo "  make sim CELL=sram_6t_prog_cell"
	@echo "=========================================================================="
	@echo "Workspace Structure (Standard Industrial EDA Layout):"
	@echo "  config/   - cds.lib, .cdsinit, environment setup scripts"
	@echo "  logs/     - Spectre simulation & Virtuoso run logs"
	@echo "  reports/  - Timing, power, delay & noise margin measurement reports"
	@echo "  outputs/  - Simulation waveforms (.raw / PSF) & exported netlists"
	@echo "  work/     - Runtime execution directory"
	@echo "=========================================================================="

check:
	@echo "=== Checking Cadence Environment ==="
	@which virtuoso || { echo "[ERROR] Virtuoso not found in PATH. Run: source $(CONFIG_DIR)/env_virtuoso.sh"; exit 1; }
	@which spectre  || { echo "[ERROR] Spectre not found in PATH. Run: source $(CONFIG_DIR)/env_virtuoso.sh"; exit 1; }
	@"$(VIRTUOSO)" -W
	@"$(SPECTRE)" -V
	@echo "[PASS] Cadence Virtuoso & Spectre are ready."

preflight:
	@echo "[INFO] Validating Virtuoso configuration..."
	@test -f $(CONFIG_DIR)/cds.lib || { echo "[ERROR] Missing $(CONFIG_DIR)/cds.lib" >&2; exit 1; }
	@mkdir -p $(WORK_DIR) $(LOG_DIR) $(REPORT_DIR) $(OUTPUT_DIR)
	@echo "[PASS] Workspace directories and cds.lib verified."

virtuoso: preflight check
	@echo "Launching Cadence Virtuoso GUI in $(WORK_DIR)..."
	@cd $(WORK_DIR) && cp -f $(CONFIG_DIR)/cds.lib . 2>/dev/null || true
	@cd $(WORK_DIR) && "$(VIRTUOSO)" -log $(LOG_DIR)/virtuoso.log &

gui: virtuoso

schematic: preflight check
	@bash $(CURDIR)/scripts/import_schematic.sh "$(CELL)"

view_schematic: schematic
	@bash $(CURDIR)/scripts/open_design.sh "$(CELL)" schematic

sch: view_schematic

view_layout: schematic
	@bash $(CURDIR)/scripts/open_design.sh "$(CELL)" layout

layout: view_layout

sim: preflight check
	@bash $(CURDIR)/scripts/run_cell_sim.sh "$(CELL)"

flow: schematic sim wave
	@echo "=========================================================================="
	@echo "[PASS] Full custom flow completed for $(CELL)."
	@echo "=========================================================================="

sim_all: preflight check
	@for c in booth_selector_cell compressor_4to2 fused_3input_csa_cell kogge_stone_prefix_cells cas_divider_slice srt_radix4_stage bitcell_8t_2r1w sram_6t_prog_cell; do \
		echo "--------------------------------------------------------"; \
		bash $(CURDIR)/scripts/run_cell_sim.sh "$$c"; \
	done
	@echo "========================================================"
	@echo "[PASS] All cell simulations completed."
	@echo "Logs saved in: $(LOG_DIR)/"
	@echo "Reports saved in: $(REPORT_DIR)/"
	@echo "Waveforms saved in: $(OUTPUT_DIR)/"

wave: check
	@if [ ! -d "$(OUTPUT_DIR)/$(CELL).raw" ]; then \
		echo "[INFO] No waveform dataset found for $(CELL). Running simulation first..."; \
		$(MAKE) sim CELL=$(CELL); \
	fi
	@echo "Opening Cadence ViVA Waveform Viewer for $(CELL)..."
	@viva -raw $(OUTPUT_DIR)/$(CELL).raw &

clean:
	@echo "Cleaning temporary runtime files, logs, and lockfiles..."
	@rm -rf $(WORK_DIR)/* $(LOG_DIR)/* $(OUTPUT_DIR)/* $(REPORT_DIR)/*
	@rm -f *.log *.cdslck cds.log*
	@find . -name "*.cdslck" -delete
	@mkdir -p $(WORK_DIR) $(LOG_DIR) $(OUTPUT_DIR) $(REPORT_DIR)
	@echo "[PASS] Workspace cleaned."
