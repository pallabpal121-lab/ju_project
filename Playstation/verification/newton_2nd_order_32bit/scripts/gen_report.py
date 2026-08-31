#!/usr/bin/env python3
# ==============================================================================
# Script: Generate Verification & Scoreboard Reports
# Usage : python3 scripts/gen_report.py [LOGS_DIR] [REPORTS_DIR]
# ==============================================================================

import os
import sys
import re
import glob
from datetime import datetime

logs_dir = sys.argv[1] if len(sys.argv) > 1 else "logs"
reports_dir = sys.argv[2] if len(sys.argv) > 2 else "reports"

os.makedirs(reports_dir, exist_ok=True)

test_logs = sorted(glob.glob(os.path.join(logs_dir, "sim_*.log")))

tests_data = []
all_matches = []

for log_path in test_logs:
    test_file = os.path.basename(log_path)
    test_name = test_file.replace("sim_", "").replace(".log", "")
    
    with open(log_path, "r", errors="ignore") as f:
        content = f.read()

    # Parse UVM counts
    uvm_err_match = re.search(r"UVM_ERROR\s*:\s*(\d+)", content)
    uvm_fatal_match = re.search(r"UVM_FATAL\s*:\s*(\d+)", content)
    uvm_warn_match = re.search(r"UVM_WARNING\s*:\s*(\d+)", content)
    
    uvm_errors = int(uvm_err_match.group(1)) if uvm_err_match else 0
    uvm_fatals = int(uvm_fatal_match.group(1)) if uvm_fatal_match else 0
    uvm_warnings = int(uvm_warn_match.group(1)) if uvm_warn_match else 0
    
    # Parse Scoreboard
    match_re = re.search(r"Matches\s*:\s*(\d+)", content)
    mismatch_re = re.search(r"Mismatches\s*:\s*(\d+)", content)
    matches = int(match_re.group(1)) if match_re else 0
    mismatches = int(mismatch_re.group(1)) if mismatch_re else 0
    
    # Parse Coverage
    cov_match = re.search(r"Functional Coverage:\s*([\d\.]+)%", content)
    coverage = cov_match.group(1) + "%" if cov_match else "N/A"
    
    # Parse Sim Time
    time_match = re.search(r"Time:\s*(\d+)\s*ps", content)
    sim_time = (str(int(time_match.group(1)) // 1000) + " ns") if time_match else "N/A"

    # Determine status
    if uvm_errors == 0 and uvm_fatals == 0 and mismatches == 0 and matches > 0:
        status = "PASSED"
    else:
        status = "FAILED"
        
    tests_data.append({
        "test": test_name,
        "status": status,
        "matches": matches,
        "mismatches": mismatches,
        "coverage": coverage,
        "sim_time": sim_time,
        "errors": uvm_errors,
        "fatals": uvm_fatals,
        "warnings": uvm_warnings
    })

    # Extract individual scoreboard transactions
    scb_lines = re.findall(r"(PASSED \[Match #\d+\].*?)(?=\n)", content)
    for line in scb_lines:
        all_matches.append(f"[{test_name.upper()}] {line}")

# ==============================================================================
# Write Regression Summary Report
# ==============================================================================
summary_file = os.path.join(reports_dir, "regression_summary.rpt")

total_tests = len(tests_data)
passed_tests = sum(1 for t in tests_data if t["status"] == "PASSED")
failed_tests = total_tests - passed_tests
total_matches = sum(t["matches"] for t in tests_data)
total_mismatches = sum(t["mismatches"] for t in tests_data)
pass_rate = (passed_tests / total_tests * 100.0) if total_tests > 0 else 0.0

with open(summary_file, "w") as f:
    f.write("=" * 80 + "\n")
    f.write("        UNIVERSAL NEWTON 2ND-ORDER ACCELERATOR - VERIFICATION REPORT\n")
    f.write("=" * 80 + "\n")
    f.write(f"  Generated On : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    f.write(f"  Simulator    : Synopsys VCS W-2024.09 (UVM 1.1d)\n")
    f.write(f"  Design       : newton_2nd_order_top (32-bit Q16.16 Fixed-Point)\n")
    f.write("-" * 80 + "\n\n")
    
    f.write(f"{'TEST NAME':<22} | {'STATUS':<8} | {'MATCH':<5} | {'MISMATCH':<8} | {'COV %':<8} | {'SIM TIME':<10}\n")
    f.write("-" * 80 + "\n")
    for t in tests_data:
        f.write(f"{t['test']:<22} | {t['status']:<8} | {t['matches']:<5} | {t['mismatches']:<8} | {t['coverage']:<8} | {t['sim_time']:<10}\n")
    f.write("-" * 80 + "\n\n")
    
    f.write("REGRESSION SUMMARY METRICS:\n")
    f.write(f"  Total Test Cases Executed : {total_tests}\n")
    f.write(f"  Passed Tests              : {passed_tests}\n")
    f.write(f"  Failed Tests              : {failed_tests}\n")
    f.write(f"  Scoreboard Transactions   : {total_matches} Passed / {total_mismatches} Mismatches\n")
    f.write(f"  Overall Regression Status : {'100% ALL TESTS PASSED' if failed_tests == 0 else 'TESTS FAILED'}\n")
    f.write(f"  Suite Pass Rate           : {pass_rate:.1f}%\n")
    f.write("=" * 80 + "\n")

# ==============================================================================
# Write Scoreboard Detailed Audit Report
# ==============================================================================
scb_file = os.path.join(reports_dir, "scoreboard_audit.rpt")
with open(scb_file, "w") as f:
    f.write("=" * 80 + "\n")
    f.write("              SCOREBOARD DETAILED TRANSACTION AUDIT REPORT\n")
    f.write("=" * 80 + "\n")
    f.write(f"  Total Verified Transactions: {len(all_matches)}\n")
    f.write("-" * 80 + "\n\n")
    for idx, match in enumerate(all_matches, 1):
        f.write(f"#{idx:02d} {match}\n")
    f.write("\n" + "=" * 80 + "\n")

print(f">>> [Report] Regression summary report generated: {summary_file}")
print(f">>> [Report] Scoreboard audit report generated:   {scb_file}")
