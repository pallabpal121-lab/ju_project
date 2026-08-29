// =============================================================================
// File Name   : irls_helpers.svh
// Description : Fast synthesizable inline functions for IRLS vector and matrix access
// =============================================================================

`ifndef IRLS_HELPERS_SVH
`define IRLS_HELPERS_SVH

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

function automatic q16_t get_label(input label_vec_t r, input logic [2:0] idx);
    case (idx)
        3'd0: return r[31:0];
        3'd1: return r[63:32];
        3'd2: return r[95:64];
        3'd3: return r[127:96];
        3'd4: return r[159:128];
        3'd5: return r[191:160];
        3'd6: return r[223:192];
        3'd7: return r[255:224];
    endcase
endfunction

function automatic label_vec_t set_label(input label_vec_t r, input logic [2:0] idx, input q16_t val);
    label_vec_t res;
    res = r;
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

function automatic q16_t get_feat(input feat_mat_t X, input logic [2:0] m, input logic [1:0] n);
    logic [4:0] idx;
    idx = {m, n};
    case (idx)
        5'd0:  return X[31:0];
        5'd1:  return X[63:32];
        5'd2:  return X[95:64];
        5'd3:  return X[127:96];
        5'd4:  return X[159:128];
        5'd5:  return X[191:160];
        5'd6:  return X[223:192];
        5'd7:  return X[255:224];
        5'd8:  return X[287:256];
        5'd9:  return X[319:288];
        5'd10: return X[351:320];
        5'd11: return X[383:352];
        5'd12: return X[415:384];
        5'd13: return X[447:416];
        5'd14: return X[479:448];
        5'd15: return X[511:480];
        5'd16: return X[543:512];
        5'd17: return X[575:544];
        5'd18: return X[607:576];
        5'd19: return X[639:608];
        5'd20: return X[671:640];
        5'd21: return X[703:672];
        5'd22: return X[735:704];
        5'd23: return X[767:736];
        5'd24: return X[799:768];
        5'd25: return X[831:800];
        5'd26: return X[863:832];
        5'd27: return X[895:864];
        5'd28: return X[927:896];
        5'd29: return X[959:928];
        5'd30: return X[991:960];
        5'd31: return X[1023:992];
    endcase
endfunction

function automatic feat_mat_t set_feat(input feat_mat_t X, input logic [2:0] m, input logic [1:0] n, input q16_t val);
    feat_mat_t res;
    logic [4:0] idx;
    res = X;
    idx = {m, n};
    case (idx)
        5'd0:  res[31:0]   = val;
        5'd1:  res[63:32]  = val;
        5'd2:  res[95:64]  = val;
        5'd3:  res[127:96] = val;
        5'd4:  res[159:128] = val;
        5'd5:  res[191:160] = val;
        5'd6:  res[223:192] = val;
        5'd7:  res[255:224] = val;
        5'd8:  res[287:256] = val;
        5'd9:  res[319:288] = val;
        5'd10: res[351:320] = val;
        5'd11: res[383:352] = val;
        5'd12: res[415:384] = val;
        5'd13: res[447:416] = val;
        5'd14: res[479:448] = val;
        5'd15: res[511:480] = val;
        5'd16: res[543:512] = val;
        5'd17: res[575:544] = val;
        5'd18: res[607:576] = val;
        5'd19: res[639:608] = val;
        5'd20: res[671:640] = val;
        5'd21: res[703:672] = val;
        5'd22: res[735:704] = val;
        5'd23: res[767:736] = val;
        5'd24: res[799:768] = val;
        5'd25: res[831:800] = val;
        5'd26: res[863:832] = val;
        5'd27: res[895:864] = val;
        5'd28: res[927:896] = val;
        5'd29: res[959:928] = val;
        5'd30: res[991:960] = val;
        5'd31: res[1023:992] = val;
    endcase
    return res;
endfunction

`endif
