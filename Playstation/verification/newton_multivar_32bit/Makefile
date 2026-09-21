# ==============================================================================
# Universal Multivariable Newton BCD Accelerator - Production Verification Flow
# Target Design: newton_bcd_axi_lite (32-bit AXI4-Lite Slave)
# ==============================================================================

# Directory Structure
WORK_DIR    ?= work
LOGS_DIR    ?= logs
OUTPUTS_DIR ?= outputs
REPORTS_DIR ?= reports
SCRIPTS_DIR ?= scripts
FILELIST_DIR?= filelist

# Filelists
RTL_FLIST   = $(FILELIST_DIR)/rtl.f
TB_FLIST    = $(FILELIST_DIR)/tb.f

# Simulation Binary, Waveform, and Coverage DB
SIMV        = $(WORK_DIR)/simv
WAVEFORM    = $(OUTPUTS_DIR)/sim_waveform.vcd
COV_DB      = $(WORK_DIR)/simv.vdb

# Default Test Parameters
TEST        ?= newton_multivar_quadratic_test
VERB        ?= UVM_MEDIUM
SEED        ?= $(shell date +%s)
DUMP        ?= 0
DUMP_FLAGS   = $(if $(filter 1,$(DUMP)),+DUMP +FSDB,)

# Synopsys VCS Compilation Flags (with Code & Functional Coverage)
VCS_FLAGS   = \
	-full64 \
	-sverilog \
	-ntb_opts uvm \
	-timescale=1ns/1ps \
	-debug_access+all \
	-kdb \
	-lca \
	+vcs+lic+wait \
	-cm line+tgl+branch+fsm+assert \
	-cm_log $(LOGS_DIR)/cm_compile.log \
	-Mdir=$(WORK_DIR)/csrc \
	-o $(SIMV) \
	-f $(RTL_FLIST) \
	-f $(TB_FLIST)

# Runtime VCS Execution Options
SIM_FLAGS   = \
	-no_save \
	+ntb_random_seed=$(SEED) \
	-cm line+tgl+branch+fsm+assert \
	-cm_dir $(COV_DB) \
	-cm_log $(LOGS_DIR)/cm_sim.log \
	$(DUMP_FLAGS)

.PHONY: all help init compile run regression report verdi clean quad rosenbrock corner random stress tapeout reset

all: compile run report

init:
	@mkdir -p $(WORK_DIR) $(LOGS_DIR) $(OUTPUTS_DIR) $(REPORTS_DIR)

help:
	@echo "======================================================================"
	@echo "  Universal Newton Multivariable BCD Accelerator Verification Env    "
	@echo "======================================================================"
	@echo "  Directory Structure:"
	@echo "    work/     - Compilation database, binary (simv), csrc, daidir"
	@echo "    logs/     - Compilation and simulation logs"
	@echo "    outputs/  - Simulation waveforms (*.vcd, *.fsdb)"
	@echo "    reports/  - Coverage and regression summary reports"
	@echo "    filelist/ - RTL (rtl.f) and Testbench (tb.f) filelists"
	@echo "    scripts/  - Run scripts and tool configurations"
	@echo "======================================================================"
	@echo "  Available Make Targets:"
	@echo "    source env_vcs.sh              - Source Synopsys tool licenses & paths"
	@echo "    make all (or make)             - Compile, run default test, and generate report"
	@echo "    make compile                   - Compile RTL and UVM Testbench"
	@echo "    make run [TEST=...] [VERB=...] - Run specific UVM test"
	@echo "    make regression                - Run full regression suite & generate reports"
	@echo "    make report                    - Generate regression & coverage reports"
	@echo "    make quad                      - Run Multivariable Quadratic Test"
	@echo "    make rosenbrock                - Run Coupled Multivariable Test"
	@echo "    make corner                    - Run Corner Cases & Boundary Test"
	@echo "    make random                    - Run Constrained-Random Test"
	@echo "    make stress                    - Run AXI4-Lite Bus Stress Test"
	@echo "    make verdi                     - Open waveform in Synopsys Verdi"
	@echo "    make clean                     - Clean all generated artifacts"
	@echo "    make help                      - Show this help message"
	@echo "======================================================================"

compile: init
	@echo ">>> [VCS] Compiling RTL and UVM Testbench into $(WORK_DIR)/..."
	vcs $(VCS_FLAGS) -top newton_axi_tb_top -l $(LOGS_DIR)/compile.log
	@rm -f .fsm.* ucli.key cm.log
	@echo ">>> [VCS] Compilation Successful! Binary: $(SIMV)"

run: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	@echo ">>> [VCS] Running Test: $(TEST) (Verbosity: $(VERB), Seed: $(SEED))..."
	$(SIMV) +UVM_TESTNAME=$(TEST) +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name $(TEST) -l $(LOGS_DIR)/sim_$(TEST).log
	@rm -f .fsm.* ucli.key cm.log

quad: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	$(SIMV) +UVM_TESTNAME=newton_multivar_quadratic_test +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name quad -l $(LOGS_DIR)/sim_quad.log
	@rm -f .fsm.* ucli.key cm.log

rosenbrock: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	$(SIMV) +UVM_TESTNAME=newton_multivar_rosenbrock_test +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name rosenbrock -l $(LOGS_DIR)/sim_rosenbrock.log
	@rm -f .fsm.* ucli.key cm.log

corner: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	$(SIMV) +UVM_TESTNAME=newton_multivar_corner_test +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name corner -l $(LOGS_DIR)/sim_corner.log
	@rm -f .fsm.* ucli.key cm.log

random: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	$(SIMV) +UVM_TESTNAME=newton_multivar_random_test +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name random -l $(LOGS_DIR)/sim_random.log
	@rm -f .fsm.* ucli.key cm.log

stress: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	$(SIMV) +UVM_TESTNAME=newton_axi_stress_test +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name stress -l $(LOGS_DIR)/sim_stress.log
	@rm -f .fsm.* ucli.key cm.log

tapeout: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	$(SIMV) +UVM_TESTNAME=newton_multivar_tapeout_test +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name tapeout -l $(LOGS_DIR)/sim_tapeout.log
	@rm -f .fsm.* ucli.key cm.log

reset: init
	@if [ ! -f "$(SIMV)" ]; then $(MAKE) compile; fi
	$(SIMV) +UVM_TESTNAME=newton_multivar_reset_test +UVM_VERBOSITY=$(VERB) $(SIM_FLAGS) -cm_name reset -l $(LOGS_DIR)/sim_reset.log
	@rm -f .fsm.* ucli.key cm.log

regression: compile
	@echo "=================================================="
	@echo ">>> [1/7] Running Multivariable Quadratic Test..."
	@echo "=================================================="
	$(SIMV) +UVM_TESTNAME=newton_multivar_quadratic_test  +UVM_VERBOSITY=UVM_LOW -no_save +ntb_random_seed=$(SEED) -cm line+tgl+branch+fsm+assert -cm_dir $(COV_DB) -cm_log $(LOGS_DIR)/cm_sim.log +DUMP +DUMPFILE=$(WAVEFORM) -cm_name quad -l $(LOGS_DIR)/sim_quad.log
	@echo "=================================================="
	@echo ">>> [2/7] Running Coupled Multivariable Test..."
	@echo "=================================================="
	$(SIMV) +UVM_TESTNAME=newton_multivar_rosenbrock_test +UVM_VERBOSITY=UVM_LOW $(SIM_FLAGS) -cm_name rosenbrock -l $(LOGS_DIR)/sim_rosenbrock.log
	@echo "=================================================="
	@echo ">>> [3/7] Running Corner Cases & Boundary Test..."
	@echo "=================================================="
	$(SIMV) +UVM_TESTNAME=newton_multivar_corner_test     +UVM_VERBOSITY=UVM_LOW $(SIM_FLAGS) +DUMP -cm_name corner -l $(LOGS_DIR)/sim_corner.log
	@echo "=================================================="
	@echo ">>> [4/7] Running Constrained-Random Test..."
	@echo "=================================================="
	$(SIMV) +UVM_TESTNAME=newton_multivar_random_test     +UVM_VERBOSITY=UVM_LOW $(SIM_FLAGS) -cm_name random -l $(LOGS_DIR)/sim_random.log
	@echo "=================================================="
	@echo ">>> [5/7] Running AXI4-Lite Bus Stress Test..."
	@echo "=================================================="
	$(SIMV) +UVM_TESTNAME=newton_axi_stress_test          +UVM_VERBOSITY=UVM_LOW $(SIM_FLAGS) +DUMP +FSDB -cm_name stress -l $(LOGS_DIR)/sim_stress.log
	@echo "=================================================="
	@echo ">>> [6/7] Running Tapeout-Grade Comprehensive Test..."
	@echo "=================================================="
	$(SIMV) +UVM_TESTNAME=newton_multivar_tapeout_test    +UVM_VERBOSITY=UVM_LOW $(SIM_FLAGS) -cm_name tapeout -l $(LOGS_DIR)/sim_tapeout.log
	@echo "=================================================="
	@echo ">>> [7/7] Running Synchronous Reset & FSM Recovery Test..."
	@echo "=================================================="
	$(SIMV) +UVM_TESTNAME=newton_multivar_reset_test      +UVM_VERBOSITY=UVM_LOW $(SIM_FLAGS) -cm_name reset -l $(LOGS_DIR)/sim_reset.log
	@rm -f .fsm.* ucli.key cm.log
	@echo "=================================================="
	@echo ">>> Generating Verification & Coverage Reports..."
	@echo "=================================================="
	@$(MAKE) report
	@echo "=================================================="
	@echo "   FULL REGRESSION SUITE COMPLETED SUCCESSFULLY!  "
	@echo "   Reports saved to: $(REPORTS_DIR)/              "
	@echo "=================================================="

report: init
	@python3 $(SCRIPTS_DIR)/gen_report.py $(LOGS_DIR) $(REPORTS_DIR)
	@if [ -d "$(COV_DB)" ]; then \
		urg -dir $(COV_DB) -report $(REPORTS_DIR)/coverage_report -elfile $(REPORTS_DIR)/newton_cov.el > /dev/null 2>&1 || true; \
		echo ">>> [URG] Synopsys Coverage HTML report available at: $(REPORTS_DIR)/coverage_report/dashboard.html"; \
	fi
	@cat $(REPORTS_DIR)/regression_summary.rpt

verdi:
	@echo ">>> [Verdi] Launching Synopsys Verdi Debugger..."
	@if [ -f "$(OUTPUTS_DIR)/novas.fsdb" ]; then \
		verdi -dbdir $(WORK_DIR)/simv.daidir -ssf $(OUTPUTS_DIR)/novas.fsdb & \
	else \
		verdi -dbdir $(WORK_DIR)/simv.daidir -ssf $(WAVEFORM) & \
	fi

clean:
	@echo ">>> Cleaning all verification build and simulation artifacts..."
	rm -rf $(WORK_DIR)/* $(LOGS_DIR)/* $(OUTPUTS_DIR)/* $(REPORTS_DIR)/*
	rm -rf DVEfiles ucli.key vc_hdrs.h .vlog* verdi* novas* *.fsdb simv* csrc *.log *.key *.vcd .fsm.* cm.log
	@echo ">>> Clean completed."
