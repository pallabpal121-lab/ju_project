// =============================================================================
// File Name   : sqp_helpers.svh
// Description : Fast synthesizable inline functions for SQP vector, matrix, and box clamping
// =============================================================================

`ifndef SQP_HELPERS_SVH
`define SQP_HELPERS_SVH

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

function automatic q16_t get_mat(input mat_t m, input logic [1:0] r, input logic [1:0] c);
    case ({r, c})
        4'd0:  return m[31:0];
        4'd1:  return m[63:32];
        4'd2:  return m[95:64];
        4'd3:  return m[127:96];
        4'd4:  return m[159:128];
        4'd5:  return m[191:160];
        4'd6:  return m[223:192];
        4'd7:  return m[255:224];
        4'd8:  return m[287:256];
        4'd9:  return m[319:288];
        4'd10: return m[351:320];
        4'd11: return m[383:352];
        4'd12: return m[415:384];
        4'd13: return m[447:416];
        4'd14: return m[479:448];
        4'd15: return m[511:480];
    endcase
endfunction

function automatic mat_t set_mat(input mat_t m, input logic [1:0] r, input logic [1:0] c, input q16_t val);
    mat_t res;
    res = m;
    case ({r, c})
        4'd0:  res[31:0]   = val;
        4'd1:  res[63:32]  = val;
        4'd2:  res[95:64]  = val;
        4'd3:  res[127:96] = val;
        4'd4:  res[159:128] = val;
        4'd5:  res[191:160] = val;
        4'd6:  res[223:192] = val;
        4'd7:  res[255:224] = val;
        4'd8:  res[287:256] = val;
        4'd9:  res[319:288] = val;
        4'd10: res[351:320] = val;
        4'd11: res[383:352] = val;
        4'd12: res[415:384] = val;
        4'd13: res[447:416] = val;
        4'd14: res[479:448] = val;
        4'd15: res[511:480] = val;
    endcase
    return res;
endfunction

function automatic q16_t clamp_q16(input q16_t val, input q16_t lb, input q16_t ub);
    if (val < lb) return lb;
    else if (val > ub) return ub;
    else return val;
endfunction

`endif
