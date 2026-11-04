// Module      : m_fpt_tpg
// Block       : B05 - Test Pattern Generator
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-10-27
// Description : Colour-bar RAW10/RAW12 test pattern quads with sync flags and blanking
`timescale 1ns/1ps
`default_nettype none

module m_fpt_tpg (
  input  wire         i_sys_clk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  input  wire         i_core_enable,
  input  wire         i_tpg_enable,
  input  wire         i_tpg_pattern,
  input  wire  [15:0] i_tpg_width,
  input  wire  [15:0] i_tpg_height,
  input  wire         i_tpg_dual_vc_en,
  input  wire         i_px_ready,
  input  wire         i_tpg_pixel_format,
  input  wire  [15:0] i_tpg_hblank,
  input  wire  [15:0] i_tpg_vblank,
  input  wire  [1:0]  i_tpg_cfa_order,
  input  wire         i_linebuf_empty,
  output logic [47:0] o_px_data,
  output logic        o_px_valid,
  output logic        o_px_sof,
  output logic        o_px_sol,
  output logic        o_px_eol,
  output logic [1:0]  o_px_vc_tag,
  output logic        o_px_dt_sel,
  output logic        o_tpg_eof,
  output logic        o_tpg_fs,
  output logic        o_tpg_fe,
  output logic [1:0]  o_tpg_fvc,
  output logic [15:0] o_tpg_height_shadow
);

  // ------------------------------------------------------------------------------------------
  // Constants
  // ------------------------------------------------------------------------------------------
  // Block parameters, all fixed by the specification, so none of them can be overridden.
  localparam logic [31:0] LPARA_N_BANDS               = 32'd8;
  localparam logic [31:0] LPARA_PIXELS_PER_CYCLE      = 32'd4;
  localparam logic [11:0] LPARA_BAR_LEVEL             = 12'hFFF;
  localparam logic [31:0] LPARA_GUARD_DIMENSION       = 32'd8;
  localparam logic [31:0] LPARA_VBLANK_LINE_CNT_WIDTH = 32'd16;

  // log2 of the quad size: x_cnt step, width alignment and line_period = width >> 2.
  localparam logic [31:0] LPARA_QUAD_SHIFT = $clog2(LPARA_PIXELS_PER_CYCLE);

  // blank_cnt holds one line period at the largest truncated tpg_width (0xFFFC, in quads)
  // plus the largest tpg_hblank (0xFFFF): both are the maximum values of 16-bit CSR fields.
  // Plain 32-bit literals, no cast inside $clog2: some synthesis front ends size such a cast
  // wrongly.
  localparam logic [31:0] LPARA_BLANK_CNT_WIDTH =
    $clog2((32'h0000_FFFC >> LPARA_QUAD_SHIFT) + 32'h0000_FFFF + 32'd1);

  // Bits of a band index; also the shift that divides a dimension into N_BANDS bands and
  // the number of bits of k in the shift-and-add threshold calculation.
  localparam logic [31:0] LPARA_BAND_IDX_W = $clog2(LPARA_N_BANDS);

  // Pixel bus: one 12-bit lane per pixel of the quad (the lane width is the pixel format).
  localparam logic [31:0] LPARA_PX_DATA_W = LPARA_PIXELS_PER_CYCLE * 32'd12;

  // 16-bit forms of the constants that are compared with, added to or masked into the
  // 16-bit coordinates and dimensions (16 bits is the width of the CSR dimension fields).
  localparam logic [15:0] LPARA_X_STEP     = 16'(LPARA_PIXELS_PER_CYCLE);
  localparam logic [15:0] LPARA_GUARD_SIZE = 16'(LPARA_GUARD_DIMENSION);
  localparam logic [15:0] LPARA_WIDTH_MASK = 16'(~(LPARA_PIXELS_PER_CYCLE - 32'd1));

  // Control FSM states. The specification fixes no code (the state is not visible at a port).
  typedef enum logic [1:0] {
    ENUM_ST_IDLE            = 2'b00,
    ENUM_ST_CALC_THRESHOLDS = 2'b01,
    ENUM_ST_RUNNING         = 2'b10,
    ENUM_ST_STOPPING        = 2'b11
  } tpg_state_type_def;

  // ------------------------------------------------------------------------------------------
  // Internal signals
  // ------------------------------------------------------------------------------------------
  // Control FSM and Dimension Guard
  tpg_state_type_def                      reg_fsm_tpg_current_state;
  tpg_state_type_def                      w_fsm_tpg_nxt_state;
  logic                                   w_run_en;
  logic                                   w_gen_active;
  logic                                   w_latch_en;
  logic                                   w_calc_done;
  logic                                   w_blank_clr;
  logic                                   w_clr_pend;
  logic [15:0]                            w_tpg_width_truncated;
  logic                                   w_guard_ok;

  // Shadow Registers
  logic [15:0]                            reg_width_shadow;
  logic [15:0]                            reg_height_shadow;
  logic                                   reg_pattern_shadow;
  logic                                   reg_format_shadow;
  logic                                   reg_dual_vc_shadow;
  logic [15:0]                            reg_hblank_shadow;
  logic [15:0]                            reg_vblank_shadow;
  logic [1:0]                             reg_cfa_shadow;

  // Threshold Calculator: threshold_k for k = 1..N_BANDS-1 (threshold_0 is 0 by definition)
  logic [15:0]                            w_band_base;
  logic [15:0]                            w_threshold   [1:LPARA_N_BANDS-1];
  logic [15:0]                            reg_threshold [1:LPARA_N_BANDS-1];

  // Pixel Counter
  logic [15:0]                            reg_x_cnt;
  logic [15:0]                            reg_y_cnt;
  logic                                   reg_vc_toggle;
  logic                                   w_x_first;
  logic                                   w_x_last;
  logic                                   w_y_first;
  logic                                   w_y_last;
  logic                                   w_row_done;
  logic                                   w_eof;

  // Pattern Datapath, one entry per pixel lane (lane 0 = leftmost pixel of the quad)
  logic [15:0]                            w_target_coord [0:LPARA_PIXELS_PER_CYCLE-1];
  logic [LPARA_BAND_IDX_W-1:0]            w_band_idx     [0:LPARA_PIXELS_PER_CYCLE-1];
  logic                                   w_a            [0:LPARA_PIXELS_PER_CYCLE-1];
  logic                                   w_b;
  logic                                   w_on           [0:LPARA_PIXELS_PER_CYCLE-1];
  logic [11:0]                            w_pix12        [0:LPARA_PIXELS_PER_CYCLE-1];
  logic [11:0]                            w_pixel        [0:LPARA_PIXELS_PER_CYCLE-1];
  logic [LPARA_PX_DATA_W-1:0]             w_px_data;

  // Sync Logic and eof_pend
  logic                                   w_px_valid;
  logic                                   w_accept;
  logic                                   w_cnt_en;
  logic                                   w_px_sof;
  logic                                   w_px_sol;
  logic                                   w_px_eol;
  logic [1:0]                             w_px_vc_tag;
  logic                                   w_line_done;
  logic                                   w_frame_done;
  logic                                   w_tpg_fs;
  logic                                   w_tpg_fe;
  logic                                   reg_eof_pend;

  // Blanking Generator
  logic [LPARA_BLANK_CNT_WIDTH-1:0]       reg_blank_cnt;
  logic [LPARA_BLANK_CNT_WIDTH-1:0]       w_line_period;
  logic [LPARA_BLANK_CNT_WIDTH-1:0]       w_blank_nxt;
  logic                                   w_blank_active;
  logic [LPARA_VBLANK_LINE_CNT_WIDTH-1:0] reg_vblank_line_cnt;
  logic [LPARA_VBLANK_LINE_CNT_WIDTH-1:0] w_vblank_nxt;
  logic                                   w_vblank_reload;

  // ------------------------------------------------------------------------------------------
  // Control FSM
  // ------------------------------------------------------------------------------------------
  // State outputs. The datapath runs in RUNNING and STOPPING (gen_active). CALC_THRESHOLDS
  // latches the configuration (latch_en) and always lasts one cycle (calc_done). IDLE clears
  // the blanking counters (blank_clr); IDLE and CALC_THRESHOLDS clear eof_pend (clr_pend).
  always_comb begin : p_com_fsm_tpg_out
    w_gen_active = (reg_fsm_tpg_current_state == ENUM_ST_RUNNING)
                || (reg_fsm_tpg_current_state == ENUM_ST_STOPPING);
    w_latch_en   = (reg_fsm_tpg_current_state == ENUM_ST_CALC_THRESHOLDS);
    w_calc_done  = (reg_fsm_tpg_current_state == ENUM_ST_CALC_THRESHOLDS);
    w_blank_clr  = (reg_fsm_tpg_current_state == ENUM_ST_IDLE);
    w_clr_pend   = (reg_fsm_tpg_current_state == ENUM_ST_IDLE)
                || (reg_fsm_tpg_current_state == ENUM_ST_CALC_THRESHOLDS);
  end

  // run_en (tpg_enable and core_enable) drives every transition. IDLE starts a frame only with
  // Line Buffer empty and the dimension guard met. In RUNNING a drop of run_en wins over the
  // loop-back after blanking; STOPPING returns to RUNNING when run_en comes back, else goes to
  // IDLE after the last beat of the frame, skipping its blanking. Default returns to IDLE.
  always_comb begin : p_com_fsm_tpg_nxt
    w_run_en            = i_tpg_enable && i_core_enable;
    w_fsm_tpg_nxt_state = reg_fsm_tpg_current_state;
    case (reg_fsm_tpg_current_state)
      ENUM_ST_IDLE: begin
        if (w_run_en && i_linebuf_empty && w_guard_ok) begin
          w_fsm_tpg_nxt_state = ENUM_ST_CALC_THRESHOLDS;
        end
      end
      ENUM_ST_CALC_THRESHOLDS: begin
        if (w_calc_done) begin
          w_fsm_tpg_nxt_state = ENUM_ST_RUNNING;
        end
      end
      ENUM_ST_RUNNING: begin
        if (!w_run_en) begin
          w_fsm_tpg_nxt_state = ENUM_ST_STOPPING;
        end else if (reg_eof_pend && !w_blank_active && i_linebuf_empty) begin
          w_fsm_tpg_nxt_state = ENUM_ST_CALC_THRESHOLDS;
        end
      end
      ENUM_ST_STOPPING: begin
        if (w_run_en) begin
          w_fsm_tpg_nxt_state = ENUM_ST_RUNNING;
        end else if (reg_eof_pend) begin
          w_fsm_tpg_nxt_state = ENUM_ST_IDLE;
        end
      end
      default: begin
        w_fsm_tpg_nxt_state = ENUM_ST_IDLE;
      end
    endcase
  end

  // resetn is asynchronous and soft_reset synchronous; both return the FSM to IDLE.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_fsm_tpg
    if (!i_resetn) begin
      reg_fsm_tpg_current_state <= ENUM_ST_IDLE;
    end else if (i_soft_reset) begin
      reg_fsm_tpg_current_state <= ENUM_ST_IDLE;
    end else begin
      reg_fsm_tpg_current_state <= w_fsm_tpg_nxt_state;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Dimension Guard
  // ------------------------------------------------------------------------------------------
  // The width is truncated to a multiple of the quad size, the value the datapath uses.
  // The guard compares the live CSR values (not the shadows) and is checked only on the
  // way out of IDLE; the frame loop-back has no guard (software constraint).
  always_comb begin : p_com_dim_guard
    w_tpg_width_truncated = i_tpg_width & LPARA_WIDTH_MASK;
    w_guard_ok            = (w_tpg_width_truncated >= LPARA_GUARD_SIZE)
                         && (i_tpg_height >= LPARA_GUARD_SIZE);
  end

  // ------------------------------------------------------------------------------------------
  // Shadow Registers
  // ------------------------------------------------------------------------------------------
  // All eight fields are captured together in CALC_THRESHOLDS, so a CSR write takes effect from
  // the next frame boundary. Cleared by resetn only: soft_reset keeps them, since they are
  // re-latched before the next frame is generated.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_shadow
    if (!i_resetn) begin
      reg_width_shadow   <= '0;
      reg_height_shadow  <= '0;
      reg_pattern_shadow <= '0;
      reg_format_shadow  <= '0;
      reg_dual_vc_shadow <= '0;
      reg_hblank_shadow  <= '0;
      reg_vblank_shadow  <= '0;
      reg_cfa_shadow     <= '0;
    end else if (w_latch_en) begin
      reg_width_shadow   <= w_tpg_width_truncated;
      reg_height_shadow  <= i_tpg_height;
      reg_pattern_shadow <= i_tpg_pattern;
      reg_format_shadow  <= i_tpg_pixel_format;
      reg_dual_vc_shadow <= i_tpg_dual_vc_en;
      reg_hblank_shadow  <= i_tpg_hblank;
      reg_vblank_shadow  <= i_tpg_vblank;
      reg_cfa_shadow     <= i_tpg_cfa_order;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Threshold Calculator
  // ------------------------------------------------------------------------------------------
  // threshold_k = k * band_base by shifts and adds only (no multiplier); band_base is width or
  // height / N_BANDS for vertical or horizontal bars. It uses the CSR values latched into the
  // shadows in the same cycle, so the thresholds match the shadows. The guard keeps
  // band_base >= 1; the largest threshold fits 16 bits.
  always_comb begin : p_com_threshold
    w_band_base = i_tpg_pattern ? (i_tpg_height >> LPARA_BAND_IDX_W)
                                : (w_tpg_width_truncated >> LPARA_BAND_IDX_W);
    for (int k = 1; k < LPARA_N_BANDS; k++) begin
      w_threshold[k] = '0;
      for (int s = 0; s < LPARA_BAND_IDX_W; s++) begin
        if (k[s]) begin
          w_threshold[k] = w_threshold[k] + (w_band_base << s);
        end
      end
    end
  end

  // Cleared by resetn only, like the shadows.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_threshold
    if (!i_resetn) begin
      for (int k = 1; k < LPARA_N_BANDS; k++) begin
        reg_threshold[k] <= '0;
      end
    end else if (w_latch_en) begin
      for (int k = 1; k < LPARA_N_BANDS; k++) begin
        reg_threshold[k] <= w_threshold[k];
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Pixel Counter
  // ------------------------------------------------------------------------------------------
  // Position flags of the current quad: first/last quad of the line, first/last row of the
  // frame. A row is complete after the only line in single-VC mode and after the VC1 line in
  // dual-VC mode; EOF marks the line that completes the last row.
  always_comb begin : p_com_pixel_cnt
    w_x_first  = (reg_x_cnt == 16'd0);
    w_x_last   = (reg_x_cnt == (reg_width_shadow - LPARA_X_STEP));
    w_y_first  = (reg_y_cnt == 16'd0);
    w_y_last   = (reg_y_cnt == (reg_height_shadow - 16'd1));
    w_row_done = !reg_dual_vc_shadow || reg_vc_toggle;
    w_eof      = w_row_done && w_y_last;
  end

  // Only an accepted beat moves the counters, so data and flags hold while downstream stalls.
  // y_cnt advances only on the line that completes a row, so a VC0/VC1 row pair is never split;
  // vc_toggle flips per line in dual-VC mode and restarts at VC0 in CALC_THRESHOLDS.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_pixel_cnt
    if (!i_resetn) begin
      reg_x_cnt     <= '0;
      reg_y_cnt     <= '0;
      reg_vc_toggle <= '0;
    end else if (i_soft_reset) begin
      reg_x_cnt     <= '0;
      reg_y_cnt     <= '0;
      reg_vc_toggle <= '0;
    end else begin
      if (w_cnt_en) begin
        reg_x_cnt <= w_x_last ? 16'd0 : (reg_x_cnt + LPARA_X_STEP);
      end
      if (w_line_done && w_row_done) begin
        reg_y_cnt <= w_y_last ? 16'd0 : (reg_y_cnt + 16'd1);
      end
      if (w_latch_en) begin
        reg_vc_toggle <= '0;
      end else if (w_line_done) begin
        reg_vc_toggle <= reg_dual_vc_shadow ? !reg_vc_toggle : 1'b0;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Pattern Datapath
  // ------------------------------------------------------------------------------------------
  // Per lane n: coordinate x_cnt + n (vertical bars) or y_cnt (horizontal), band index = largest
  // k with coordinate >= threshold_k. Bayer site from column parity n[0] and row parity y_cnt[0]
  // relative to cfa_shadow (bit 0: odd column, bit 1: odd row); R, G, B are BAR_LEVEL while
  // band_idx bit 1, 2, 0 is 0. RAW10 keeps the 10 MSBs in the low bits of the 12-bit lane.
  always_comb begin : p_com_pattern
    w_b = reg_y_cnt[0] ^ reg_cfa_shadow[1];
    for (int n = 0; n < LPARA_PIXELS_PER_CYCLE; n++) begin
      w_target_coord[n] = reg_pattern_shadow ? reg_y_cnt : (reg_x_cnt + 16'(n));

      w_band_idx[n] = '0;
      for (int k = 1; k < LPARA_N_BANDS; k++) begin
        if (w_target_coord[n] >= reg_threshold[k]) begin
          w_band_idx[n] = LPARA_BAND_IDX_W'(k);
        end
      end

      w_a[n] = n[0] ^ reg_cfa_shadow[0];
      if (!w_a[n] && !w_b) begin
        w_on[n] = !w_band_idx[n][1];
      end else if (w_a[n] && w_b) begin
        w_on[n] = !w_band_idx[n][0];
      end else begin
        w_on[n] = !w_band_idx[n][2];
      end

      w_pix12[n] = w_on[n] ? LPARA_BAR_LEVEL : 12'h000;
      w_pixel[n] = reg_format_shadow ? {2'b00, w_pix12[n][11:2]} : w_pix12[n];
    end
  end

  // Packer: pixel_0 in px_data[11:0] up to pixel_3 in px_data[47:36]; every beat carries a
  // full quad of 12-bit lanes.
  always_comb begin : p_com_packer
    w_px_data = '0;
    for (int n = 0; n < LPARA_PIXELS_PER_CYCLE; n++) begin
      w_px_data[n*12 +: 12] = w_pixel[n];
    end
  end

  // ------------------------------------------------------------------------------------------
  // Sync Logic
  // ------------------------------------------------------------------------------------------
  // px_valid depends only on the FSM, blank_cnt and eof_pend, never on px_ready, so a beat stays
  // presented until it is accepted. line_done and frame_done mark the accepted last beat of a
  // line and of a frame. tpg_fs and tpg_fe pulse for one cycle on the accepted px_sof beat and
  // last-row px_eol beat of each VC line; tpg_fvc is the VC of that line.
  always_comb begin : p_com_sync
    w_px_valid   = w_gen_active && !w_blank_active && !reg_eof_pend;
    w_accept     = w_px_valid && i_px_ready;
    w_cnt_en     = w_accept;
    w_px_sol     = w_px_valid && w_x_first;
    w_px_sof     = w_px_sol && w_y_first;
    w_px_eol     = w_px_valid && w_x_last;
    w_px_vc_tag  = {1'b0, reg_vc_toggle};
    w_line_done  = w_accept && w_px_eol;
    w_frame_done = w_line_done && w_eof;
    w_tpg_fs     = w_accept && w_px_sof;
    w_tpg_fe     = w_accept && w_px_eol && w_y_last;
  end

  // ------------------------------------------------------------------------------------------
  // eof_pend
  // ------------------------------------------------------------------------------------------
  // Set by the accepted last beat of a frame and cleared in IDLE and CALC_THRESHOLDS: it
  // holds px_valid low from the cycle after that beat until the next configuration has
  // been latched. frame_done has priority over the clear.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_eof_pend
    if (!i_resetn) begin
      reg_eof_pend <= '0;
    end else if (i_soft_reset) begin
      reg_eof_pend <= '0;
    end else if (w_frame_done) begin
      reg_eof_pend <= 1'b1;
    end else if (w_clr_pend) begin
      reg_eof_pend <= '0;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Blanking Generator
  // ------------------------------------------------------------------------------------------
  // blank_cnt counts the idle cycles still to insert; any non-zero count forces px_valid low,
  // which also holds the counters.
  always_comb begin : p_com_blank_active
    w_blank_active = (reg_blank_cnt != '0);
  end

  // hblank_shadow idle cycles follow the accepted last beat of every line. After the last line,
  // vblank_line_cnt counts the blank line-times still to insert; a new line_period is loaded
  // when blank_cnt would reach 0, so blank line-times are exactly line_period cycles apart.
  // line_period is formed in BLANK_CNT_WIDTH bits, so the sum cannot overflow.
  always_comb begin : p_com_blank_nxt
    w_line_period = LPARA_BLANK_CNT_WIDTH'(reg_width_shadow >> LPARA_QUAD_SHIFT)
                  + LPARA_BLANK_CNT_WIDTH'(reg_hblank_shadow);
    if (w_line_done) begin
      w_blank_nxt = LPARA_BLANK_CNT_WIDTH'(reg_hblank_shadow);
    end else if (w_blank_active) begin
      w_blank_nxt = reg_blank_cnt - 1'b1;
    end else begin
      w_blank_nxt = '0;
    end
    w_vblank_nxt    = w_frame_done ? reg_vblank_shadow : reg_vblank_line_cnt;
    w_vblank_reload = (w_blank_nxt == '0) && (w_vblank_nxt != '0);
  end

  // Both counters are cleared in IDLE (blank_clr), so a stop skips the pending blanking.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_blank
    if (!i_resetn) begin
      reg_blank_cnt       <= '0;
      reg_vblank_line_cnt <= '0;
    end else if (i_soft_reset || w_blank_clr) begin
      reg_blank_cnt       <= '0;
      reg_vblank_line_cnt <= '0;
    end else begin
      reg_blank_cnt       <= w_vblank_reload ? w_line_period : w_blank_nxt;
      reg_vblank_line_cnt <= w_vblank_reload ? (w_vblank_nxt - 1'b1) : w_vblank_nxt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------------------------------------
  // Pixel stream and event pulses are combinational from the registers and px_ready; the
  // data type and the frame height are the shadows of the frame being generated.
  assign o_px_data           = w_px_data;
  assign o_px_valid          = w_px_valid;
  assign o_px_sof            = w_px_sof;
  assign o_px_sol            = w_px_sol;
  assign o_px_eol            = w_px_eol;
  assign o_px_vc_tag         = w_px_vc_tag;
  assign o_px_dt_sel         = reg_format_shadow;
  assign o_tpg_eof           = w_frame_done;
  assign o_tpg_fs            = w_tpg_fs;
  assign o_tpg_fe            = w_tpg_fe;
  assign o_tpg_fvc           = w_px_vc_tag;
  assign o_tpg_height_shadow = reg_height_shadow;

endmodule

`default_nettype wire
