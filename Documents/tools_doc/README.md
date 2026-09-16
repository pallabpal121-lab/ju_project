# Synopsys ASIC Design Flow Documentation Library

Welcome to the Synopsys ASIC Flow Documentation Library. This repository is structured into a **chronological, universally numbered reference system** aligned with the real-world digital ASIC design lifecycle—from semiconductor physics and foundry PDK rules to RTL verification, logic synthesis, DFT, physical place-and-route, signoff STA, and tapeout.

Every document in this library is formatted as a standard PDF or Markdown document, so that **every single file can be opened and viewed directly within the Antigravity IDE**.

---

## 1. Quick-Start Entry Points

### A. Senior Full-Stack Silicon Developer Field Guide
Before opening individual tool manuals, read:
👉 **[`00_START_HERE_CORE_READING/00_ASIC_FLOW_CONNECTING_THE_DOTS.md`](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/00_ASIC_FLOW_CONNECTING_THE_DOTS.md)**
*Explains the full-stack developer mental model: how data handshakes across tools, what physical silicon phenomena each tool models, and the 8-point pre-tapeout signoff checklist.*

### B. CDAC 5-Day RTL-to-GDSII Workshop Presentations
For a visual, slide-based introduction to the complete ASIC flow, open:
👉 **[`00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/`](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides)**
* [01_VLSI_Design_Implementation_Flow.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/01_VLSI_Design_Implementation_Flow.pdf): ASIC vs SoC design styles and full flow overview.
* [02_Overview_of_Physical_Design.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/02_Overview_of_Physical_Design.pdf): Physical design stages from netlist to layout.
* [03_Design_Compiler_Synthesis.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/03_Design_Compiler_Synthesis.pdf): Logic synthesis flow with Design Compiler.
* [04_ICC2_Floorplan_Creation.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/04_ICC2_Floorplan_Creation.pdf): Floorplanning in IC Compiler II.
* [05_ICC2_Power_Planning.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/05_ICC2_Power_Planning.pdf): Core power rings, straps, and cell rails in ICC2.
* [06_ICC2_Placement.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/06_ICC2_Placement.pdf): Standard cell placement, legalizing, and congestion in ICC2.
* [07_ICC2_Routing.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/RTL_to_GDSII_Workshop_Slides/07_ICC2_Routing.pdf): Global routing, track assignment, and detailed routing in ICC2.

### C. Foundational Priority Shelf
The directory **[`00_START_HERE_CORE_READING/`](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING)** contains the 11 foundational documents ordered in the exact sequence you encounter them in an ASIC project:

| Seq | Priority Document | Flow Stage & Tool | Focus & Learning Objective |
| :---: | :--- | :--- | :--- |
| **01** | [01_Foundry_PDK_Rules__DRM.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/01_Foundry_PDK_Rules__DRM.pdf) | **Foundry Technology (SCL 180nm)** | Understand physical silicon layers, design rules (DRM), metal pitches, and design constraints imposed by fabrication. |
| **02** | [02_Standard_Cell_Datasheet__FS120_datasheet.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/02_Standard_Cell_Datasheet__FS120_datasheet.pdf) | **Standard Cells (SCL FS120)** | Understand cell propagation delays, pin capacitance, truth tables, and physical footprints of primitive standard cells. |
| **03** | [03_Simulation_and_Debug_Tutorial__VerdiTut.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/03_Simulation_and_Debug_Tutorial__VerdiTut.pdf) | **Simulation & Debug (VCS + Verdi)** | Comprehensive 577-page guide to interactive debugging: hierarchy traversal, driver/load tracing in nTrace, and waveform analysis in nWave. |
| **04** | [04_Verification_Coverage_Tutorial__CoverageTut.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/04_Verification_Coverage_Tutorial__CoverageTut.pdf) | **Verification Coverage (Verdi)** | Learn code coverage (line, toggle, condition, FSM) and functional coverage reporting to ensure design completeness before synthesis. |
| **05** | [05_Timing_Constraints_SDC_Guide__SDC_Parser_UG.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/05_Timing_Constraints_SDC_Guide__SDC_Parser_UG.pdf) | **Constraints (SDC)** | Master Synopsys Design Constraints syntax: clocks (`create_clock`), I/O delays (`set_input_delay`, `set_output_delay`), false paths, and multicycle paths. |
| **06** | [06_Logic_Synthesis_DesignWare_Guide__dwbb_userguide.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/06_Logic_Synthesis_DesignWare_Guide__dwbb_userguide.pdf) | **Logic Synthesis (Design Compiler)** | Learn how Design Compiler maps RTL operators (`+`, `-`, `*`, shifters, FIFOs) into optimized gate-level architectures. |
| **07** | [07_DFT_and_ATPG_Guide__tmax_ug.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/07_DFT_and_ATPG_Guide__tmax_ug.pdf) | **DFT & ATPG (TestMAX / TetraMAX)** | Authoritative 1,652-page guide on scan flip-flop insertion, fault models (stuck-at, transition, at-speed), and test pattern generation. |
| **08** | [08_ASIC_Physical_and_IO_Flow__an_io_flow.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/08_ASIC_Physical_and_IO_Flow__an_io_flow.pdf) | **Physical Design (ICC2 / Apollo)** | Comprehensive application note covering IO pad ring architecture, core power/ground distribution, IO placement, clock pad routing, and LVS. |
| **09** | [09_PrimeTime_STA_Consistency_Rules__gcarules.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/09_PrimeTime_STA_Consistency_Rules__gcarules.pdf) | **Static Timing Analysis (PrimeTime)** | Master golden timing constraint consistency checks: unconstrained endpoints, clock-domain crossings (CDC), and timing exception validation. |
| **10** | [10_PrimeTime_Power_Analysis_Tutorial__PrimeTime_PX_Tutorials.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/10_PrimeTime_Power_Analysis_Tutorial__PrimeTime_PX_Tutorials.pdf) | **Power Signoff (PrimeTime PX)** | Hands-on tutorial on calculating static leakage and dynamic switching power from gate-level netlists and switching activity (VCD/FSDB) files. |
| **11** | [11_Chip_Finishing_and_Tapeout__chip_finishing.pdf](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING/11_Chip_Finishing_and_Tapeout__chip_finishing.pdf) | **Foundry Tapeout (SCL 180nm)** | Walkthrough of final tapeout tasks: dummy metal fill insertion, DRC cleanup, seal rings, and final GDSII delivery to the foundry. |

---

## 2. Chronological Digital ASIC Design Flow & Synopsys Tool Mapping

```
[ Stage 01: Foundry PDK & Cell Libraries ] ──► SCL 180nm DRM, FS120_datasheet
                    │
                    ▼
[ Stage 02: RTL Simulation & Verification ] ──► VCS (vcs) ──► primer, svtb_training_labs
                    │
                    ▼
[ Stage 03: Interactive Debug & Waveforms ] ──► Verdi (verdi) ──► VerdiTut, CoverageTut
                    │
                    ▼
[ Stage 04: Logic Synthesis & Constraints ] ──► Design Compiler (dc_shell) ──► SDC 2.1, dwbb_userguide
                    │
                    ▼
[ Stage 05: Design for Test (DFT) & ATPG ] ───► TestMAX ATPG (tmax) ──► tmax_ug
                    │
                    ▼
[ Stage 06: Physical Design & PnR ] ──────────► IC Compiler II (icc2_shell) ──► ICC2 Workshops, an_io_flow
                    │
                    ▼
[ Stage 07: Parasitic RC Extraction ] ────────► StarRC (starrc) ──► Generates SPEF parasitics
                    │
                    ▼
[ Stage 08: Static Timing Analysis & Power ] ─► PrimeTime & PrimeTime PX (pt_shell) ──► gcarules, SDC 2.1, PT-PX
                    │
                    ▼
[ Stage 09: Signoff Physical Verification ] ──► IC Validator (icv) ──► DRC, LVS, ERC, Antenna checks
                    │
                    ▼
[ Tapeout / Chip Finishing ] ─────────────────► Dummy Metal Fill & GDSII export ──► chip_finishing
```

---

## 3. Directory Navigation Guide (With Sequential Numbering Inside Each Folder)

Each folder contains sequentially numbered documents (`01_*`, `02_*`, etc.) to make learning systematic:

* **[`00_START_HERE_CORE_READING/`](file:///home/user17/Synopsys/documents/00_START_HERE_CORE_READING)**:
  Priority shelf holding the 11 foundational PDFs, the Full Stack Developer Field Guide, and the CDAC RTL-to-GDSII workshop slide deck.
* **[`01_Foundry_PDK_SCL180/`](file:///home/user17/Synopsys/documents/01_Foundry_PDK_SCL180)**:
  `01_DRM_Design_Rule_Manual.pdf`, `02_DRS_Design_Rule_Summary.pdf`, `03_FS120_Standard_Cell_Datasheet.pdf`, `05_an_io_flow_IO_and_Physical_Architecture.pdf`, `06_PowerStrapping_Guidelines.pdf`, and `08_chip_finishing_Tapeout_Guide.pdf`.
* **[`02_RTL_Simulation_VCS/`](file:///home/user17/Synopsys/documents/02_RTL_Simulation_VCS)**:
  `01_primer_UVM_Register_Abstraction_Layer.pdf`, `02_svtb_training_labs.pdf`, `03_VC_SpyGlass_Lint_and_CDC_Guide.pdf`, `04_WishBone_b3_Bus_Specification.pdf`, `05_vcsSwift_cosimulation_manual.pdf`, `06_AEware_Application_Note_for_FSM_Livelock.pdf`, and testbench labs in `Example_Testbench_Code/`.
* **[`03_Debug_Waveforms_Verdi/`](file:///home/user17/Synopsys/documents/03_Debug_Waveforms_Verdi)**:
  `01_VerdiTut_User_Guide_and_Tutorial.pdf`, `02_CoverageTut_Coverage_User_Guide.pdf`, `03_UVMDebugUserGuide.pdf`, and command references.
* **[`04_Logic_Synthesis_DesignCompiler/`](file:///home/user17/Synopsys/documents/04_Logic_Synthesis_DesignCompiler)**:
  `01_Design_Compiler_Synthesis_Workshop.pdf`, `02_SDC_Application_Note_v2.1.pdf`, `03_SDC_Parser_UG.pdf`, `04_dwbb_userguide_DesignWare_Building_Blocks.pdf`, and datapath components in `Component_Datasheets/`.
* **[`05_Test_ATPG_TestMAX/`](file:///home/user17/Synopsys/documents/05_Test_ATPG_TestMAX)**:
  `01_tmax_ug_ATPG_and_Diagnosis_User_Guide.pdf`, `02_tmax_cmds_Command_Reference.pdf`, `03_tmax_rules_Design_Rule_Checking.pdf`, and `04_tmax_messages_Reference.pdf`.
* **[`06_Physical_Design_ICC2/`](file:///home/user17/Synopsys/documents/06_Physical_Design_ICC2)**:
  `01_Overview_of_Physical_Design_Workshop.pdf`, `02_ICC2_Floorplan_Creation_Workshop.pdf`, `03_ICC2_Power_Planning_Workshop.pdf`, `04_ICC2_Placement_Workshop.pdf`, `05_ICC2_Routing_Workshop.pdf`, and `06_an_io_flow_Physical_and_IO_Architecture.pdf`.
* **[`07_Parasitic_Extraction_StarRC/`](file:///home/user17/Synopsys/documents/07_Parasitic_Extraction_StarRC)**:
  `01_mw_asr_Milkyway_Technology_Reference.pdf`, `02_scmComp.pdf`, and extraction guides.
* **[`08_Static_Timing_PrimeTime/`](file:///home/user17/Synopsys/documents/08_Static_Timing_PrimeTime)**:
  `01_gcarules_PrimeTime_Constraint_Consistency_Rules.pdf`, `02_SDC_Application_Note_v2.1.pdf`, `03_PrimeTime_PX_Tutorials_Power_Analysis.pdf`, and command invocation guides (`04_gca1` through `07_gcan`).
* **[`09_Physical_Verification_ICValidator/`](file:///home/user17/Synopsys/documents/09_Physical_Verification_ICValidator)**:
  `01_appnote_icv_lsf_grid.pdf`, `02_ESD_reference.pdf`, `03_ElectricalOverstressCheck.pdf`, `04_FloatingGateCheck.pdf`, and `06_Manual_of_ICGLE_template_flow_conversion.pdf`.
* **[`10_DesignWare_IP_coreTools/`](file:///home/user17/Synopsys/documents/10_DesignWare_IP_coreTools)**:
  `01_Intro.pdf`, `02_Flow.pdf`, `03_coretools_overview.pdf`, `05_corebuilder_user.pdf`, `07_coreassembler_user.pdf`, and `09_coreconsultant_user.pdf`.
* **[`11_SPICE_Simulation_HSPICE/`](file:///home/user17/Synopsys/documents/11_SPICE_Simulation_HSPICE)**:
  `01_primesim_hspice_qsg_Quick_Start_Guide.pdf`, `02_primesim_continuum_overview.pdf`, and `03_primesim_user_guide.pdf`.
* **[`12_Simulation_Engines_PrimeSim/`](file:///home/user17/Synopsys/documents/12_Simulation_Engines_PrimeSim)**:
  `01_primesim_continuum_overview.pdf`, `02_primesim_user_guide.pdf`, `03_primewave_standalone_NL_quickstart.pdf`, and `04_vcs_primesim_ams_user_guide.pdf`.
* **[`13_Custom_Analog_CustomCompiler/`](file:///home/user17/Synopsys/documents/13_Custom_Analog_CustomCompiler)**:
  `01_cc_getting_started.pdf`, `02_cc_env_ug.pdf`, `03_cc_seuser_Schematic_Editor.pdf`, `04_cc_leuser_Layout_Editor.pdf`, and `05_pycell_studio_tutorial.pdf`.
* **[`14_FPGA_Synthesis_Synplify/`](file:///home/user17/Synopsys/documents/14_FPGA_Synthesis_Synplify)**:
  `01_fpga_user_guide.pdf`, `02_fpga_attribute_reference.pdf`, `03_fpga_command_reference.pdf`, and `05_identify_debugger_ug_synplify.pdf`.
* **[`15_Library_Compiler/`](file:///home/user17/Synopsys/documents/15_Library_Compiler)**:
  `01_DPManagerUserGuide.pdf` and `02_DPManagerFAQ.pdf`.
* **[`99_Legal_FOSS_Notices/`](file:///home/user17/Synopsys/documents/99_Legal_FOSS_Notices)**:
  Consolidated archive of third-party Free and Open Source Software license disclosures.
* **[`INDEX.md`](file:///home/user17/Synopsys/documents/INDEX.md)**:
  Master index listing every file across all folders with direct clickable links.

---

## 4. Built-In Interactive Tool Help

All installed Synopsys tools feature complete command documentation accessible directly from their interactive shells:

* **In `dc_shell` (Design Compiler)**:
  ```tcl
  man compile_ultra
  man check_design
  man report_constraint
  help *clock*
  ```
* **In `icc2_shell` (IC Compiler II)**:
  ```tcl
  man create_placement
  man route_opt
  man create_clock_tree
  help *route*
  ```
* **In `pt_shell` (PrimeTime)**:
  ```tcl
  man report_timing
  man check_timing
  man set_clock_uncertainty
  help *slack*
  ```
* **In `tmax` (TestMAX ATPG)**:
  ```tcl
  man run_atpg
  man set_rules
  man report_faults
  ```
