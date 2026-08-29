// =============================================================================
// File Name   : lasso_helpers.svh
// Description : Synthesizable inline functions for LASSO vector & dataset matrix indexing
// =============================================================================

`ifndef LASSO_HELPERS_SVH
`define LASSO_HELPERS_SVH

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

function automatic q16_t get_obs(input obs_vec_t ov, input logic [2:0] idx);
    case (idx)
        3'd0: return ov[31:0];
        3'd1: return ov[63:32];
        3'd2: return ov[95:64];
        3'd3: return ov[127:96];
        3'd4: return ov[159:128];
        3'd5: return ov[191:160];
        3'd6: return ov[223:192];
        3'd7: return ov[255:224];
    endcase
endfunction

function automatic obs_vec_t set_obs(input obs_vec_t ov, input logic [2:0] idx, input q16_t val);
    obs_vec_t res;
    res = ov;
    case (idx)
        3'd0: res[31:0]   = val;
        3'd1: res[63:32]  = val;
        3'd2: res[95:64]  = val;
        3'd3: res[127:96] = val;
        3'd4: res[159:128] = val;
        3'd5: res[191:160] = val;
        3'd6: res[223:192] = val;
        3'd7: res[255:224] = val;
    endcase
    return res;
endfunction

function automatic q16_t get_mat_elem(input dataset_mat_t m, input logic [2:0] row, input logic [1:0] col);
    int bit_offset;
    bit_offset = (int'(row) * 128) + (int'(col) * 32);
    return m[bit_offset +: 32];
endfunction

function automatic dataset_mat_t set_mat_elem(input dataset_mat_t m, input logic [2:0] row, input logic [1:0] col, input q16_t val);
    dataset_mat_t res;
    int bit_offset;
    res = m;
    bit_offset = (int'(row) * 128) + (int'(col) * 32);
    res[bit_offset +: 32] = val;
    return res;
endfunction

`endif
