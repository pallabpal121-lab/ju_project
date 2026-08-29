// =============================================================================
// File Name   : admm_helpers.svh
// Description : Synthesizable inline functions for ADMM vector, matrix, and dataset indexing
// =============================================================================

`ifndef ADMM_HELPERS_SVH
`define ADMM_HELPERS_SVH

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

function automatic q16_t get_mat(input mat_t m, input logic [1:0] r, input logic [1:0] c);
    int bit_offset;
    bit_offset = (int'(r) * 128) + (int'(c) * 32);
    return m[bit_offset +: 32];
endfunction

function automatic mat_t set_mat(input mat_t m, input logic [1:0] r, input logic [1:0] c, input q16_t val);
    mat_t res;
    int bit_offset;
    res = m;
    bit_offset = (int'(r) * 128) + (int'(c) * 32);
    res[bit_offset +: 32] = val;
    return res;
endfunction

function automatic q16_t get_data_elem(input dataset_mat_t m, input logic [2:0] row, input logic [1:0] col);
    int bit_offset;
    bit_offset = (int'(row) * 128) + (int'(col) * 32);
    return m[bit_offset +: 32];
endfunction

function automatic dataset_mat_t set_data_elem(input dataset_mat_t m, input logic [2:0] row, input logic [1:0] col, input q16_t val);
    dataset_mat_t res;
    int bit_offset;
    res = m;
    bit_offset = (int'(row) * 128) + (int'(col) * 32);
    res[bit_offset +: 32] = val;
    return res;
endfunction

function automatic q16_t q16_norm_inf(input vec_t v, input logic [2:0] n);
    q16_t max_val, cur_val;
    max_val = 32'sd0;
    for (int k = 0; k < MAX_PARAMS; k++) begin
        if (k < n) begin
            cur_val = get_vec(v, 2'(k));
            if (cur_val < 32'sd0) cur_val = -cur_val;
            if (cur_val > max_val) max_val = cur_val;
        end
    end
    return max_val;
endfunction

`endif
