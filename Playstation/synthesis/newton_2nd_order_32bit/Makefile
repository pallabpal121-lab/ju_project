# ==============================================================================
# Makefile: Automated Synopsys Design Compiler Synthesis Flow
# Design   : Universal Newton 2nd-Order Accelerator (32-bit Q16.16)
# Standard : Industry Reference Methodology (RM)
# ==============================================================================

# Tool Invocations
DC_SHELL       ?= dc_shell
DESIGN_VISION  ?= design_vision
PYTHON         ?= python3

# Configurable Synthesis Parameters
CLK_PERIOD     ?= 10.0
TECH_NODE      ?= SCL180
CORNER         ?= typical

# Directory Definitions
WORK_DIR       = work
LOGS_DIR       = logs
REPORTS_DIR    = reports
OUTPUTS_DIR    = outputs
SCRIPTS_DIR    = scripts
CONSTRAINTS_DIR= constraints
FILELIST_DIR   = filelist

# Main Scripts
RUN_SYN_SCRIPT = $(SCRIPTS_DIR)/run_synthesis.tcl
GEN_REPORT     = $(SCRIPTS_DIR)/gen_report.py
SYNTH_LOG      = $(LOGS_DIR)/synthesis.log

.PHONY: all syn check_design gui reports clean distclean help

all: syn

# ------------------------------------------------------------------------------
# 1. Full Batch Synthesis Run
# ------------------------------------------------------------------------------
syn:
	@mkdir -p $(WORK_DIR) $(LOGS_DIR) $(REPORTS_DIR) $(OUTPUTS_DIR)
	@echo "======================================================================"
	@echo " Starting Synopsys Design Compiler Synthesis Flow                     "
	@echo " Design       : newton_2nd_order_top                                  "
	@echo " Clock Period : $(CLK_PERIOD) ns (Freq: $$(awk 'BEGIN {printf "%.1f", 1000/$(CLK_PERIOD)}') MHz)"
	@echo " Tech Node    : $(TECH_NODE)                                          "
	@echo " Corner       : $(CORNER)                                             "
	@echo " Log File     : $(SYNTH_LOG)                                          "
	@echo "======================================================================"
	@SYNTH_CLK_PERIOD=$(CLK_PERIOD) TECH_NODE=$(TECH_NODE) CORNER=$(CORNER) \
		$(DC_SHELL) -64bit -output_log_file $(LOGS_DIR)/command.log -f $(RUN_SYN_SCRIPT) 2>&1 | tee $(SYNTH_LOG)
	@echo "======================================================================"
	@echo " Synthesis completed. Generating Sign-Off Report..."
	@$(MAKE) --no-print-directory reports

# ------------------------------------------------------------------------------
# 2. Syntax, Analyze, Elaborate, and Link Check Only
# ------------------------------------------------------------------------------
check_design:
	@mkdir -p $(WORK_DIR) $(LOGS_DIR) $(REPORTS_DIR) $(OUTPUTS_DIR)
	@echo "Running RTL lint, elaboration and link check..."
	@SYNTH_CLK_PERIOD=$(CLK_PERIOD) TECH_NODE=$(TECH_NODE) CORNER=$(CORNER) \
		$(DC_SHELL) -64bit -output_log_file $(LOGS_DIR)/command.log -x "source $(SCRIPTS_DIR)/common_setup.tcl; source $(SCRIPTS_DIR)/dc_setup.tcl; source $(SCRIPTS_DIR)/read_design.tcl; exit" 2>&1 | tee $(LOGS_DIR)/check_design.log

# ------------------------------------------------------------------------------
# 3. Interactive Graphical User Interface (Design Vision)
# ------------------------------------------------------------------------------
gui:
	@mkdir -p $(WORK_DIR) $(LOGS_DIR)
	$(DESIGN_VISION) &

# ------------------------------------------------------------------------------
# 4. Summary Quality of Results (QoR) Extractor & Sign-Off Generation
# ------------------------------------------------------------------------------
reports:
	@if [ -f "$(GEN_REPORT)" ]; then \
		$(PYTHON) $(GEN_REPORT) $(REPORTS_DIR) $(OUTPUTS_DIR); \
	else \
		echo "Report generator script not found: $(GEN_REPORT)"; \
	fi

# ------------------------------------------------------------------------------
# 5. Cleanups
# ------------------------------------------------------------------------------
clean:
	@echo ">>> Cleaning temporary synthesis scratch, logs, and parser cache..."
	rm -rf $(WORK_DIR)/* $(LOGS_DIR)/* command.log default.svf filenames.log view.log alib cksum_dir *.mr *.pvl *.syn *.pvk
	@echo ">>> Scratch clean completed."

distclean:
	@echo ">>> Cleaning all generated outputs, reports, deliverables, and scratch..."
	rm -rf $(WORK_DIR)/* $(LOGS_DIR)/* $(OUTPUTS_DIR)/* $(REPORTS_DIR)/*
	rm -rf SYNTHESIS_SIGNOFF.md command.log default.svf filenames.log view.log alib cksum_dir *.mr *.pvl *.syn *.pvk
	@echo ">>> Distclean completed."

# ------------------------------------------------------------------------------
# 6. Help Target
# ------------------------------------------------------------------------------
help:
	@echo "Available Make Targets:"
	@echo "  make syn          : Run complete synthesis flow in batch mode"
	@echo "  make check_design : Check RTL syntax, elaboration and linking only"
	@echo "  make gui          : Launch Synopsys Design Vision GUI"
	@echo "  make reports      : Extract QoR metrics and update SYNTHESIS_SIGNOFF.md"
	@echo "  make clean        : Remove temporary scratch files (keeps reports & outputs)"
	@echo "  make distclean    : Remove all generated reports, netlists, and logs"
	@echo ""
	@echo "Options:"
	@echo "  CLK_PERIOD=<ns>   : Override target clock period (default: 10.0)"
	@echo "  TECH_NODE=<tech>  : SCL180 (default) or NANGATE45"
	@echo "  CORNER=<corner>   : typical (default), slow, fast"
