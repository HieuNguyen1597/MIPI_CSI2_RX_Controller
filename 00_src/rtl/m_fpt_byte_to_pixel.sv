// Module      : m_fpt_byte_to_pixel
// Block       : B04 - Byte-to-Pixel Unpacking
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-06
// Description : Unpacks the RAW12/RAW10 byte stream into up to four 12-bit pixels per cycle
`timescale 1ns/1ps
`default_nettype none

module m_fpt_byte_to_pixel (
  input  wire         i_sys_clk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  input  wire  [31:0] i_bp_data,
  input  wire         i_bp_valid,
  input  wire  [3:0]  i_bp_byte_en,
  input  wire         i_bp_sof,
  input  wire         i_bp_sol,
  input  wire         i_bp_eol,
  input  wire  [1:0]  i_bp_vc_tag,
  input  wire         i_bp_dt_sel,
  output logic [47:0] o_px_data,
  output logic        o_px_valid,
  output logic [2:0]  o_px_valid_count,
  output logic        o_px_sof,
  output logic        o_px_sol,
  output logic        o_px_eol,
  output logic [1:0]  o_px_vc_tag,
  output logic        o_px_dt_sel,
  output logic        o_err_wc_align
);

  // ------------------------------------------------------------------------------------------
  // Constants
  // ------------------------------------------------------------------------------------------
  // Fixed values of the block: input word, pixel bus, pixels per cycle and bytes per RAW12 /
  // RAW10 packing unit. None of them can be set at instantiation.
  localparam logic [31:0] LPARA_DATA_IN_WIDTH        = 32'd32;
  localparam logic [31:0] LPARA_DATA_OUT_WIDTH       = 32'd48;
  localparam logic [31:0] LPARA_MAX_PIXELS_PER_CYCLE = 32'd4;
  localparam logic [31:0] LPARA_K_RAW12              = 32'd3;
  localparam logic [31:0] LPARA_K_RAW10              = 32'd5;
  // The Carry Buffer holds at most K_RAW10 - 1 = 4 leftover bytes; the work buffer holds that
  // remainder plus the largest intake of one beat.
  localparam logic [31:0] LPARA_CARRY_BUF_WIDTH    = 8 * (LPARA_K_RAW10 - 1);
  localparam logic [31:0] LPARA_WORK_BUF_WIDTH     = LPARA_CARRY_BUF_WIDTH + LPARA_DATA_IN_WIDTH;
  localparam logic [31:0] LPARA_BYTE_NUM           = LPARA_DATA_IN_WIDTH / 8;
  localparam logic [31:0] LPARA_VALID_COUNT_WIDTH  = $clog2(LPARA_MAX_PIXELS_PER_CYCLE + 1);
  // Counter widths: new_byte_cnt 0..BYTE_NUM; carry_in, carry_cnt, raw_next_carry and
  // next_carry_cnt 0..K_RAW10 - 1; k is K_RAW12 or K_RAW10; work_cnt, consumed_cnt and
  // two_k_raw12 0..K_RAW10 - 1 + BYTE_NUM; units_extracted and units_out count RAW12 units
  // of two pixels, 0..MAX_PIXELS_PER_CYCLE / 2.
  localparam logic [31:0] LPARA_NEW_BYTE_CNT_WIDTH = $clog2(LPARA_BYTE_NUM + 1);
  localparam logic [31:0] LPARA_CARRY_CNT_WIDTH    = $clog2(LPARA_K_RAW10);
  localparam logic [31:0] LPARA_K_WIDTH            = $clog2(LPARA_K_RAW10 + 1);
  localparam logic [31:0] LPARA_WORK_CNT_WIDTH     = $clog2(LPARA_K_RAW10 + LPARA_BYTE_NUM);
  localparam logic [31:0] LPARA_UNITS_WIDTH        = $clog2(LPARA_MAX_PIXELS_PER_CYCLE / 2 + 1);

  // ------------------------------------------------------------------------------------------
  // Internal signals
  // ------------------------------------------------------------------------------------------
  // Byte Mask and Byte-Count Resolver
  logic [LPARA_DATA_IN_WIDTH-1:0]                        w_bp_data_m;
  logic [LPARA_NEW_BYTE_CNT_WIDTH-1:0]                   w_new_byte_cnt;
  // Controller: State Logic
  logic                                                  w_line_start;
  logic                                                  w_line_end;
  logic [LPARA_CARRY_CNT_WIDTH-1:0]                      w_carry_in;
  logic [LPARA_WORK_CNT_WIDTH-1:0]                       w_work_cnt;
  logic [LPARA_K_WIDTH-1:0]                              w_k;
  logic [LPARA_WORK_CNT_WIDTH-1:0]                       w_two_k_raw12;
  logic [LPARA_UNITS_WIDTH-1:0]                          w_units_extracted;
  logic [LPARA_CARRY_CNT_WIDTH-1:0]                      w_raw_next_carry;
  logic                                                  w_pad_unit;
  logic [LPARA_UNITS_WIDTH-1:0]                          w_units_out;
  logic                                                  w_px_emit;
  logic [LPARA_CARRY_CNT_WIDTH-1:0]                      w_next_carry_cnt;
  logic                                                  w_err_wc_align_pre;
  // Controller: Sync Pending Logic
  logic                                                  w_sof_pend_n;
  logic                                                  w_sol_pend_n;
  logic                                                  w_px_sof_pre;
  logic                                                  w_px_sol_pre;
  logic                                                  w_px_eol_pre;
  // Work Buffer Assembler and Carry Buffer Update
  logic [LPARA_CARRY_BUF_WIDTH-1:0]                      w_carry_bytes;
  logic [LPARA_WORK_BUF_WIDTH-1:0]                       w_work_buf;
  logic [LPARA_WORK_BUF_WIDTH+LPARA_CARRY_BUF_WIDTH-1:0] w_work_buf_extended;
  logic [LPARA_WORK_CNT_WIDTH-1:0]                       w_consumed_cnt;
  logic [LPARA_CARRY_BUF_WIDTH-1:0]                      w_carry_next;
  // Extractors and Format Output Mux
  logic [LPARA_DATA_OUT_WIDTH-1:0]                       w_raw12_data;
  logic [LPARA_DATA_OUT_WIDTH-1:0]                       w_raw10_data;
  logic [LPARA_DATA_OUT_WIDTH-1:0]                       w_px_data_pre;
  logic [LPARA_VALID_COUNT_WIDTH-1:0]                    w_px_valid_count_pre;
  // Flip-flops
  logic [LPARA_CARRY_BUF_WIDTH-1:0]                      reg_carry_buf;
  logic [LPARA_CARRY_CNT_WIDTH-1:0]                      reg_carry_cnt;
  logic                                                  reg_sof_pend;
  logic                                                  reg_sol_pend;
  logic [LPARA_DATA_OUT_WIDTH-1:0]                       reg_px_data;
  logic                                                  reg_px_valid;
  logic [LPARA_VALID_COUNT_WIDTH-1:0]                    reg_px_valid_count;
  logic                                                  reg_px_sof;
  logic                                                  reg_px_sol;
  logic                                                  reg_px_eol;
  logic [1:0]                                            reg_px_vc_tag;
  logic                                                  reg_px_dt_sel;
  logic                                                  reg_err_wc_align;

  // ------------------------------------------------------------------------------------------
  // Byte Mask and Byte-Count Resolver
  // ------------------------------------------------------------------------------------------
  // bp_data[7:0] is the earliest byte. A byte is kept only on bp_valid with its bp_byte_en bit
  // set, else it is 0, so the work buffer is 0 at and above work_cnt. new_byte_cnt counts the
  // enabled bytes of a valid beat.
  always_comb begin : p_com_byte_mask
    w_bp_data_m    = '0;
    w_new_byte_cnt = LPARA_NEW_BYTE_CNT_WIDTH'(0);
    for (int idx = 0; idx < LPARA_BYTE_NUM; idx++) begin
      if (i_bp_valid && i_bp_byte_en[idx]) begin
        w_bp_data_m[8*idx +: 8] = i_bp_data[8*idx +: 8];
        w_new_byte_cnt          = w_new_byte_cnt + LPARA_NEW_BYTE_CNT_WIDTH'(1);
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Controller: State Logic
  // ------------------------------------------------------------------------------------------
  // line_start clears the carry read in (carry_in) and line_end clears the carry written back,
  // so a line starts from an empty buffer and no byte crosses a line boundary. bp_dt_sel
  // selects K = 3 (RAW12) or 5 (RAW10).
  always_comb begin : p_com_state_logic
    w_line_start      = i_bp_valid && i_bp_sol;
    w_line_end        = i_bp_valid && i_bp_eol;
    w_carry_in        = w_line_start ? LPARA_CARRY_CNT_WIDTH'(0) : reg_carry_cnt;
    w_work_cnt        = LPARA_WORK_CNT_WIDTH'(w_carry_in) + LPARA_WORK_CNT_WIDTH'(w_new_byte_cnt);
    w_k               = i_bp_dt_sel ? LPARA_K_WIDTH'(LPARA_K_RAW10) : LPARA_K_WIDTH'(LPARA_K_RAW12);
    w_two_k_raw12     = LPARA_WORK_CNT_WIDTH'(2 * LPARA_K_RAW12);
    // Unit Resolver: units_extracted = floor(work_cnt / K), raw_next_carry = work_cnt mod K,
    // which is below K and fits the carry width. work_cnt is at most 8, so only RAW12 can give
    // two units. With fewer than K bytes every byte stays as remainder.
    w_units_extracted = LPARA_UNITS_WIDTH'(0);
    w_raw_next_carry  = LPARA_CARRY_CNT_WIDTH'(w_work_cnt);

    if (!i_bp_dt_sel && (w_work_cnt >= w_two_k_raw12)) begin
      w_units_extracted = LPARA_UNITS_WIDTH'(2);
      w_raw_next_carry  = LPARA_CARRY_CNT_WIDTH'(w_work_cnt - w_two_k_raw12);
    end else if (w_work_cnt >= LPARA_WORK_CNT_WIDTH'(w_k)) begin
      w_units_extracted = LPARA_UNITS_WIDTH'(1);
      w_raw_next_carry  = LPARA_CARRY_CNT_WIDTH'(w_work_cnt - LPARA_WORK_CNT_WIDTH'(w_k));
    end

    // On line_end with no whole unit, pad_unit adds one zero-padded unit so the line ends on a
    // cycle with pixels; units_extracted is then 0, so the OR never carries.
    w_pad_unit         = w_line_end && (w_units_extracted == LPARA_UNITS_WIDTH'(0));
    w_units_out        = w_units_extracted | LPARA_UNITS_WIDTH'(w_pad_unit);
    w_px_emit          = (w_units_out != LPARA_UNITS_WIDTH'(0));
    w_next_carry_cnt   = w_line_end ? LPARA_CARRY_CNT_WIDTH'(0) : w_raw_next_carry;
    // Error when the line ends mid-unit (bytes dropped) or on a beat that leaves no whole unit,
    // including a last beat without payload byte.
    w_err_wc_align_pre = w_line_end && ((w_raw_next_carry != LPARA_CARRY_CNT_WIDTH'(0)) ||
                                        (w_units_extracted == LPARA_UNITS_WIDTH'(0)));
  end

  // ------------------------------------------------------------------------------------------
  // Controller: Sync Pending Logic
  // ------------------------------------------------------------------------------------------
  // A bp_sof or line_start pulse is held pending until a cycle that produces pixels (the same
  // cycle if it does), so px_sof and px_sol never come with px_valid_count = 0. px_emit is 1 on
  // every line_end, so px_eol always comes with pixels and releases any pending flag.
  always_comb begin : p_com_sync_pending
    w_sof_pend_n = (i_bp_valid && i_bp_sof) || reg_sof_pend;
    w_sol_pend_n = w_line_start || reg_sol_pend;
    w_px_sof_pre = w_px_emit && w_sof_pend_n;
    w_px_sol_pre = w_px_emit && w_sol_pend_n;
    w_px_eol_pre = w_line_end;
  end

  // ------------------------------------------------------------------------------------------
  // Work Buffer Assembler and Carry Buffer Update
  // ------------------------------------------------------------------------------------------
  // The carried bytes come first, then the new bytes shifted up by carry_in bytes
  // ({carry_in, 3'b000} bits); the first beat of a line ignores the Carry Buffer. The bytes not
  // consumed (K x units_out) are written back, none on line_end; an idle cycle keeps the Carry
  // Buffer. w_work_buf_extended zero-extends the buffer so the byte select stays in range.
  always_comb begin : p_com_work_buffer
    w_carry_bytes       = w_line_start ? '0 : reg_carry_buf;
    w_work_buf          = {{LPARA_DATA_IN_WIDTH{1'b0}}, w_carry_bytes}
                        | ({{LPARA_CARRY_BUF_WIDTH{1'b0}}, w_bp_data_m} << {w_carry_in, 3'b000});
    w_work_buf_extended = {{LPARA_CARRY_BUF_WIDTH{1'b0}}, w_work_buf};
    w_consumed_cnt      = '0;

    case (w_units_out)
      LPARA_UNITS_WIDTH'(1): w_consumed_cnt = LPARA_WORK_CNT_WIDTH'(w_k);
      LPARA_UNITS_WIDTH'(2): w_consumed_cnt = w_two_k_raw12;
      default:               w_consumed_cnt = LPARA_WORK_CNT_WIDTH'(0);
    endcase

    w_carry_next = w_line_end ? '0 : w_work_buf_extended[{w_consumed_cnt, 3'b000} +: LPARA_CARRY_BUF_WIDTH];
  end

  // ------------------------------------------------------------------------------------------
  // RAW12 Extractor
  // ------------------------------------------------------------------------------------------
  // Unit bytes B0, B1, B2 give Pn = {B0, B2[3:0]} and Pn+1 = {B1, B2[7:4]}. Unit 0 (work bytes
  // 0..2) drives lanes 0 and 1, unit 1 (work bytes 3..5) lanes 2 and 3 when units_out is 2.
  // Lanes of units not presented are 0; a padded unit reads the zero bytes above work_cnt.
  always_comb begin : p_com_raw12_extractor
    w_raw12_data = '0;
    if (w_units_out != LPARA_UNITS_WIDTH'(0)) begin
      w_raw12_data[11:0]  = {w_work_buf[7:0],   w_work_buf[19:16]};
      w_raw12_data[23:12] = {w_work_buf[15:8],  w_work_buf[23:20]};
    end
    if (w_units_out[1]) begin
      w_raw12_data[35:24] = {w_work_buf[31:24], w_work_buf[43:40]};
      w_raw12_data[47:36] = {w_work_buf[39:32], w_work_buf[47:44]};
    end
  end

  // ------------------------------------------------------------------------------------------
  // RAW10 Extractor
  // ------------------------------------------------------------------------------------------
  // Unit bytes B0..B4 give lane j = {2'b00, B[j], B4[2j+1:2j]} for j = 0..3; lanes are 0 when
  // no unit is presented.
  always_comb begin : p_com_raw10_extractor
    w_raw10_data = '0;
    if (w_units_out != LPARA_UNITS_WIDTH'(0)) begin
      w_raw10_data[11:0]  = {2'b00, w_work_buf[7:0],   w_work_buf[33:32]};
      w_raw10_data[23:12] = {2'b00, w_work_buf[15:8],  w_work_buf[35:34]};
      w_raw10_data[35:24] = {2'b00, w_work_buf[23:16], w_work_buf[37:36]};
      w_raw10_data[47:36] = {2'b00, w_work_buf[31:24], w_work_buf[39:38]};
    end
  end

  // ------------------------------------------------------------------------------------------
  // Format Output Mux
  // ------------------------------------------------------------------------------------------
  // px_valid_count is units_out x 2 for RAW12 and units_out x 4 for RAW10 (one unit at most, so
  // only bit 0 of units_out is used), built by appending zero bits.
  always_comb begin : p_com_format_output_mux
    w_px_data_pre        = i_bp_dt_sel ? w_raw10_data : w_raw12_data;
    w_px_valid_count_pre = i_bp_dt_sel ? {w_units_out[0], 2'b00} : {w_units_out, 1'b0};
  end

  // ------------------------------------------------------------------------------------------
  // Carry Buffer, carry_cnt and sync pending flags
  // ------------------------------------------------------------------------------------------
  // resetn clears asynchronously; soft_reset clears synchronously and wins over the beat of the
  // same cycle, which is discarded.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_state
    if (!i_resetn) begin
      reg_carry_buf <= '0;
      reg_carry_cnt <= '0;
      reg_sof_pend  <= 1'b0;
      reg_sol_pend  <= 1'b0;
    end else if (i_soft_reset) begin
      reg_carry_buf <= '0;
      reg_carry_cnt <= '0;
      reg_sof_pend  <= 1'b0;
      reg_sol_pend  <= 1'b0;
    end else begin
      reg_carry_buf <= w_carry_next;
      reg_carry_cnt <= w_next_carry_cnt;
      reg_sof_pend  <= w_px_sof_pre ? 1'b0 : w_sof_pend_n;
      reg_sol_pend  <= w_px_sol_pre ? 1'b0 : w_sol_pend_n;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output Register
  // ------------------------------------------------------------------------------------------
  // One register stage: px_* and err_wc_align follow their beat by one sys_clk. It loads every
  // cycle, so px_vc_tag and px_dt_sel follow bp_vc_tag and bp_dt_sel even with no pixels. Idle
  // values are 0 after resetn and after soft_reset.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_output
    if (!i_resetn) begin
      reg_px_data        <= '0;
      reg_px_valid       <= 1'b0;
      reg_px_valid_count <= '0;
      reg_px_sof         <= 1'b0;
      reg_px_sol         <= 1'b0;
      reg_px_eol         <= 1'b0;
      reg_px_vc_tag      <= '0;
      reg_px_dt_sel      <= 1'b0;
      reg_err_wc_align   <= 1'b0;
    end else if (i_soft_reset) begin
      reg_px_data        <= '0;
      reg_px_valid       <= 1'b0;
      reg_px_valid_count <= '0;
      reg_px_sof         <= 1'b0;
      reg_px_sol         <= 1'b0;
      reg_px_eol         <= 1'b0;
      reg_px_vc_tag      <= '0;
      reg_px_dt_sel      <= 1'b0;
      reg_err_wc_align   <= 1'b0;
    end else begin
      reg_px_data        <= w_px_data_pre;
      reg_px_valid       <= w_px_emit;
      reg_px_valid_count <= w_px_valid_count_pre;
      reg_px_sof         <= w_px_sof_pre;
      reg_px_sol         <= w_px_sol_pre;
      reg_px_eol         <= w_px_eol_pre;
      reg_px_vc_tag      <= i_bp_vc_tag;
      reg_px_dt_sel      <= i_bp_dt_sel;
      reg_err_wc_align   <= w_err_wc_align_pre;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------------------------------------
  // Every output comes straight from its output register.
  assign o_px_data        = reg_px_data;
  assign o_px_valid       = reg_px_valid;
  assign o_px_valid_count = reg_px_valid_count;
  assign o_px_sof         = reg_px_sof;
  assign o_px_sol         = reg_px_sol;
  assign o_px_eol         = reg_px_eol;
  assign o_px_vc_tag      = reg_px_vc_tag;
  assign o_px_dt_sel      = reg_px_dt_sel;
  assign o_err_wc_align   = reg_err_wc_align;

endmodule

`default_nettype wire
