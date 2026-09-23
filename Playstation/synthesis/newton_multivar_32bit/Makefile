# ==============================================================================
# Makefile: Production-Grade ASIC Synthesis Automation Flow
# Design   : Universal Multivariable Newton BCD Accelerator
# Top      : newton_bcd_axi_lite
# Foundry  : SCL 180nm CMOS (0.18 um)
# Tool     : Synopsys Design Compiler (dc_shell)
# ==============================================================================

# Tool Invocations
DC_SHELL        ?= dc_shell
DESIGN_VISION   ?= design_vision
PYTHON          ?= python3

# Synthesis Parameters (Overrideable from CLI)
DESIGN_NAME     ?= newton_bcd_axi_lite
CLK_PERIOD      ?= 20.0
TECH_NODE       ?= SCL180
CORNER          ?= typical

# Directory Definitions
WORK_DIR        = work
LOGS_DIR        = logs
REPORTS_DIR     = reports
OUTPUTS_DIR     = outputs
SCRIPTS_DIR     = scripts
CONSTRAINTS_DIR = constraints
FILELIST_DIR    = filelist

# Main Script Paths
RUN_DC_SCRIPT   = $(SCRIPTS_DIR)/run_dc.tcl
GEN_REPORT      = $(SCRIPTS_DIR)/gen_summary_report.py
SYNTH_LOG       = $(LOGS_DIR)/synth.log
ENV_SCRIPT      = $(SCRIPTS_DIR)/env_synth.sh

.PHONY: all synth syn check report check_design gui clean distclean help

# Default Target
all: synth report check

# ------------------------------------------------------------------------------
# 1. Primary Batch Synthesis Execution
# ------------------------------------------------------------------------------
synth syn:
	@mkdir -p $(WORK_DIR) $(WORK_DIR)/alib $(LOGS_DIR) $(REPORTS_DIR) $(OUTPUTS_DIR)
	@echo "======================================================================"
	@echo " Starting Synopsys Design Compiler ASIC Synthesis Flow                "
	@echo " Design Name    : $(DESIGN_NAME)                                      "
	@echo " Primary Clock  : s_axi_aclk @ $(CLK_PERIOD) ns (50.0 MHz)            "
	@echo " Tech Node      : $(TECH_NODE)                                        "
	@echo " PVT Corner     : $(CORNER)                                           "
	@echo " Log File       : $(SYNTH_LOG)                                        "
	@echo "======================================================================"
	@DESIGN_NAME=$(DESIGN_NAME) SYNTH_CLK_PERIOD=$(CLK_PERIOD) TECH_NODE=$(TECH_NODE) CORNER=$(CORNER) \
		$(DC_SHELL) -64bit -output_log_file $(LOGS_DIR)/command.log -f $(RUN_DC_SCRIPT) 2>&1 | tee $(SYNTH_LOG)
	@echo "======================================================================"
	@echo " Synthesis execution completed. Run 'make report' for QoR summary.    "
	@echo "======================================================================"

# ------------------------------------------------------------------------------
# 2. QoR Report & Sign-Off Summary Extractor
# ------------------------------------------------------------------------------
report:
	@if [ -f "$(GEN_REPORT)" ]; then \
		$(PYTHON) $(GEN_REPORT) $(REPORTS_DIR) $(OUTPUTS_DIR); \
	else \
		echo "[ERROR] Report generator script not found: $(GEN_REPORT)"; \
		exit 1; \
	fi

# ------------------------------------------------------------------------------
# 3. Automated Sign-Off Verification (WNS >= 0.0 ns and Zero Latches)
# ------------------------------------------------------------------------------
check:
	@echo "======================================================================"
	@echo " Performing Automated Synthesis Sign-Off Validation                  "
	@echo "======================================================================"
	@if [ ! -f "$(REPORTS_DIR)/qor.rpt" ]; then \
		echo "[FAIL] QoR report ($(REPORTS_DIR)/qor.rpt) not found. Run 'make synth' first."; \
		exit 1; \
	fi
	@echo -n "Checking Worst Negative Slack (WNS >= 0.0 ns)... "
	@WNS=$$(grep -E "Design\s+WNS:" $(REPORTS_DIR)/qor.rpt | head -n 1 | awk '{print $$3}'); \
	if [ -z "$$WNS" ]; then \
		WNS=$$(grep -E "Critical Path Slack:" $(REPORTS_DIR)/qor.rpt | head -n 1 | awk '{print $$NF}'); \
	fi; \
	VIOL=$$(grep -E "Design\s+WNS:" $(REPORTS_DIR)/qor.rpt | head -n 1 | awk '{print $$NF}'); \
	if [ -z "$$VIOL" ]; then \
		VIOL=$$(grep -E "No\. of Violating Paths:" $(REPORTS_DIR)/qor.rpt | head -n 1 | awk '{print $$NF}'); \
	fi; \
	if [ -z "$$WNS" ]; then \
		echo "[FAIL] Could not parse WNS from qor.rpt"; exit 1; \
	fi; \
	IS_VIOL=$$(awk -v w="$$WNS" 'BEGIN {if (w < 0.0) print 1; else print 0}'); \
	if [ "$$IS_VIOL" -eq 1 ] || [ "$$VIOL" != "0.00" -a "$$VIOL" != "0" ]; then \
		echo "[FAIL] Timing violated! WNS = $$WNS ns, Violating Paths = $$VIOL"; \
		exit 1; \
	else \
		echo "[PASS] WNS = $$WNS ns (Zero setup violations, Violating Paths = $$VIOL)"; \
	fi
	@echo -n "Checking for unintended combinational latches... "
	@LATCHES=$$(grep -i "latch" $(REPORTS_DIR)/check_design.rpt 2>/dev/null | wc -l); \
	if [ "$$LATCHES" -gt 0 ]; then \
		echo "[FAIL] Detected $$LATCHES latch references in check_design.rpt!"; \
		exit 1; \
	else \
		echo "[PASS] Zero latches detected."; \
	fi
	@echo -n "Checking structural deliverables existence... "
	@if [ -f "$(OUTPUTS_DIR)/$(DESIGN_NAME)_synth.v" -a -f "$(OUTPUTS_DIR)/$(DESIGN_NAME)_synth.sdc" -a -f "$(OUTPUTS_DIR)/$(DESIGN_NAME)_synth.sdf" ]; then \
		echo "[PASS] Netlist, SDC, and SDF exported."; \
	else \
		echo "[FAIL] Missing output deliverables in $(OUTPUTS_DIR)/"; \
		exit 1; \
	fi
	@echo "======================================================================"
	@echo " SIGN-OFF RESULT: PASSED ALL CHECKS (TIMING MET, ZERO LATCHES)        "
	@echo "======================================================================"

# ------------------------------------------------------------------------------
# 4. RTL Lint, Elaboration & Link Check Only (Fast Check)
# ------------------------------------------------------------------------------
check_design:
	@mkdir -p $(WORK_DIR) $(LOGS_DIR) $(REPORTS_DIR)
	@echo "Running RTL syntax analysis, elaboration, and latch checks..."
	@DESIGN_NAME=$(DESIGN_NAME) SYNTH_CLK_PERIOD=$(CLK_PERIOD) TECH_NODE=$(TECH_NODE) CORNER=$(CORNER) \
		$(DC_SHELL) -64bit -output_log_file $(LOGS_DIR)/check_design.log -x " \
			set DESIGN_NAME \"$(DESIGN_NAME)\"; \
			set RTL_DIR \"../../rtl/newton_multivar_32bit\"; \
			set search_path [list . ./scripts ./constraints ./filelist \$${RTL_DIR} /home/user17/Synopsys/Design_Compiler/lib]; \
			set target_library [list tsl18fs120_typ.db]; \
			set link_library [list * tsl18fs120_typ.db standard.sldb]; \
			set hdlin_sv_packages true; \
			set hdlin_check_no_latch true; \
			define_design_lib WORK -path ./work; \
			analyze -format sverilog -work WORK [list \
				\$${RTL_DIR}/newton_multivar_pkg.sv \
				\$${RTL_DIR}/q16_alu.sv \
				\$${RTL_DIR}/q16_divider.sv \
				\$${RTL_DIR}/dfg_multivar_engine.sv \
				\$${RTL_DIR}/newton_2var_core.sv \
				\$${RTL_DIR}/state_bram.sv \
				\$${RTL_DIR}/newton_bcd_top.sv \
				\$${RTL_DIR}/newton_bcd_axi_lite.sv \
			]; \
			elaborate \$${DESIGN_NAME} -work WORK; \
			current_design \$${DESIGN_NAME}; \
			link; \
			check_design -multiple_designs > $(REPORTS_DIR)/check_design.rpt; \
			exit; \
		" 2>&1 | tee $(LOGS_DIR)/check_design.log
	@echo "Check design finished. Report saved to $(REPORTS_DIR)/check_design.rpt"

# ------------------------------------------------------------------------------
# 5. Interactive GUI (Synopsys Design Vision)
# ------------------------------------------------------------------------------
gui:
	@mkdir -p $(WORK_DIR) $(LOGS_DIR)
	$(DESIGN_VISION) -64bit &

# ------------------------------------------------------------------------------
# 6. Cleaning Targets
# ------------------------------------------------------------------------------
clean:
	@echo ">>> Cleaning temporary synthesis work files, logs, and cache..."
	rm -rf $(WORK_DIR)/* $(LOGS_DIR)/*
	rm -rf command.log default.svf filenames.log view.log alib cksum_dir *.mr *.pvl *.syn *.pvk
	@echo ">>> Temporary scratch files cleaned."

distclean:
	@echo ">>> Removing all deliverables, reports, logs, and generated sign-off documents..."
	rm -rf $(WORK_DIR)/* $(LOGS_DIR)/* $(OUTPUTS_DIR)/* $(REPORTS_DIR)/*
	rm -rf SYNTHESIS_SIGNOFF.md command.log default.svf filenames.log view.log alib cksum_dir *.mr *.pvl *.syn *.pvk
	@echo ">>> Full distclean completed."

# ------------------------------------------------------------------------------
# 7. Help
# ------------------------------------------------------------------------------
help:
	@echo "Production-Grade ASIC Synthesis Makefile Targets:"
	@echo "  make synth        : Run full synthesis batch flow and tee output to $(SYNTH_LOG)"
	@echo "  make report       : Parse reports and print formatted markdown QoR table"
	@echo "  make check        : Verify zero timing violations (WNS >= 0.0 ns) and zero latches"
	@echo "  make check_design : Quick RTL analyze, elaborate, link, and lint check"
	@echo "  make gui          : Launch Synopsys Design Vision GUI"
	@echo "  make clean        : Remove work cache, temp files, and logs (preserves deliverables)"
	@echo "  make distclean    : Remove all outputs, reports, and generated sign-off artifacts"
	@echo ""
	@echo "Configurable Options:"
	@echo "  CLK_PERIOD=<ns>   : Clock period in ns (default: 20.0 = 50 MHz)"
	@echo "  CORNER=<name>     : PVT corner: 'typical' (1.8V, 25C), 'slow' (1.62V, 125C), 'fast' (1.98V, 0C)"
	@echo "  TECH_NODE=<name>  : Technology node (default: SCL180)"
