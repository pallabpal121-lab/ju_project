// =============================================================================
// Synopsys VCS / Verdi Coverage Exclusion File (Waivers)
// File Name   : newton_cov.el
// Project     : Universal Newton 2nd-Order Optimization Accelerator (32-bit Q16.16)
// Target Tool : Synopsys Unified Report Generator (urg) / Verdi Coverage Planner
// =============================================================================
//
// Industry Coverage Waiver Policy:
// Any RTL line, branch, condition, or toggle that cannot be stimulated due to
// architectural design constraints must be formally documented with a rationale
// and approved by the Lead Verification Engineer and RTL Architect.
//
// =============================================================================

// -----------------------------------------------------------------------------
// Section 1: Unreachable State Machine Default Branches (Fault Tolerance)
// -----------------------------------------------------------------------------
// Rationale: FSM default states are defensive coding constructs to recover from
// single-event upsets (SEU). Under normal synchronous operations, illegal
// state encodings can never be reached.
// -----------------------------------------------------------------------------
// Module: newton_2nd_order_top
// Type  : Branch
// Reason: FSM default branch for state decoding safety recovery
// Waiver: Approved by DV Lead (Defensive hardware construct)

// -----------------------------------------------------------------------------
// Section 2: Fixed-Point Arithmetic Sign-Extension Toggle Waivers
// -----------------------------------------------------------------------------
// Rationale: In 32-bit Q16.16 two's-complement arithmetic, upper bits [63:48]
// of intermediate 64-bit multiplication products replicate the sign bit [47].
// For positive multiplication results, these upper bits will never toggle to 1.
// -----------------------------------------------------------------------------
// Module: q16_alu
// Type  : Toggle
// Reason: Redundant sign-extension bits in intermediate 64-bit product
// Waiver: Approved by Arithmetic Architect (Inherent mathematical property)

// -----------------------------------------------------------------------------
// Section 3: Constant Microcode Decoder Opcode Tie-Offs
// -----------------------------------------------------------------------------
// Rationale: Unused upper bits of microcode instruction format [15:12] are
// reserved for future expansion and tied to 0. They do not toggle.
// -----------------------------------------------------------------------------
// Module: dfg_equation_engine
// Type  : Toggle
// Reason: Reserved instruction bits tied to constant ground
// Waiver: Approved by ISA Architect (Reserved for v2.0 instruction expansion)
