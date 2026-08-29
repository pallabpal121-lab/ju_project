// =============================================================================
// File Name   : lbfgs_helpers.svh
// Description : Synthesizable inline functions for L-BFGS vector & circular history indexing
// =============================================================================

`ifndef LBFGS_HELPERS_SVH
`define LBFGS_HELPERS_SVH

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

function automatic vec_t get_hist_vec(input history_vec_t h, input logic [1:0] idx);
    case (idx)
        2'd0: return h[127:0];
        2'd1: return h[255:128];
        2'd2: return h[383:256];
        2'd3: return h[511:384];
    endcase
endfunction

function automatic history_vec_t set_hist_vec(input history_vec_t h, input logic [1:0] idx, input vec_t v);
    history_vec_t res;
    res = h;
    case (idx)
        2'd0: res[127:0]   = v;
        2'd1: res[255:128] = v;
        2'd2: res[383:256] = v;
        2'd3: res[511:384] = v;
    endcase
    return res;
endfunction

function automatic q16_t get_hist_scalar(input history_scalar_t hs, input logic [1:0] idx);
    case (idx)
        2'd0: return hs[31:0];
        2'd1: return hs[63:32];
        2'd2: return hs[95:64];
        2'd3: return hs[127:96];
    endcase
endfunction

function automatic history_scalar_t set_hist_scalar(input history_scalar_t hs, input logic [1:0] idx, input q16_t val);
    history_scalar_t res;
    res = hs;
    case (idx)
        2'd0: res[31:0]   = val;
        2'd1: res[63:32]  = val;
        2'd2: res[95:64]  = val;
        2'd3: res[127:96] = val;
    endcase
    return res;
endfunction

function automatic q16_t q16_dot(input vec_t a, input vec_t b, input logic [2:0] n);
    logic signed [63:0] sum;
    sum = 64'sd0;
    for (int k = 0; k < MAX_PARAMS; k++) begin
        if (k < n) begin
            sum = sum + (64'(get_vec(a, 2'(k))) * 64'(get_vec(b, 2'(k))));
        end
    end
    return q16_t'(sum >>> 16);
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
