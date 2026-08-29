// =============================================================================
// File Name   : nm_helpers.svh
// Description : Synthesizable inline functions for Nelder-Mead vector & simplex indexing
// =============================================================================

`ifndef NM_HELPERS_SVH
`define NM_HELPERS_SVH

function automatic q16_t get_vec(input vec_t v, input logic [1:0] idx);
    case (idx)
        2'd0: return v[31:0];
        2'd1: return v[63:32];
        2'd2: return v[95:64];
        2'd3: return v[127:96];
    endcase
endfunction

function automatic vec_t set_vec(input vec_t v, input logic [1:0] idx, input q16_t val);
    vec_t res;
    res = v;
    case (idx)
        2'd0: res[31:0]   = val;
        2'd1: res[63:32]  = val;
        2'd2: res[95:64]  = val;
        2'd3: res[127:96] = val;
    endcase
    return res;
endfunction

function automatic vec_t get_s_vec(input simplex_vec_t s, input logic [2:0] idx);
    case (idx)
        3'd0: return s[127:0];
        3'd1: return s[255:128];
        3'd2: return s[383:256];
        3'd3: return s[511:384];
        3'd4: return s[639:512];
        default: return s[127:0];
    endcase
endfunction

function automatic simplex_vec_t set_s_vec(input simplex_vec_t s, input logic [2:0] idx, input vec_t v);
    simplex_vec_t res;
    res = s;
    case (idx)
        3'd0: res[127:0]   = v;
        3'd1: res[255:128] = v;
        3'd2: res[383:256] = v;
        3'd3: res[511:384] = v;
        3'd4: res[639:512] = v;
    endcase
    return res;
endfunction

function automatic q16_t get_s_f(input simplex_f_t sf, input logic [2:0] idx);
    case (idx)
        3'd0: return sf[31:0];
        3'd1: return sf[63:32];
        3'd2: return sf[95:64];
        3'd3: return sf[127:96];
        3'd4: return sf[159:128];
        default: return sf[31:0];
    endcase
endfunction

function automatic simplex_f_t set_s_f(input simplex_f_t sf, input logic [2:0] idx, input q16_t val);
    simplex_f_t res;
    res = sf;
    case (idx)
        3'd0: res[31:0]   = val;
        3'd1: res[63:32]  = val;
        3'd2: res[95:64]  = val;
        3'd3: res[127:96] = val;
        3'd4: res[159:128] = val;
    endcase
    return res;
endfunction

`endif
