// Module      : m_fpt_irq_aggregator
// Block       : B11 - Interrupt & Error Aggregator
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-15
// Description : IRQ_STATUS flags, csi2_irq, event counters, sticky overflow flags and ERROR_CAPTURE
`timescale 1ns/1ps
`default_nettype none
module m_fpt_irq_aggregator #(
  // Parameter Description: width of errsoths and errsotsynchs; equal to MAX_LANES of B02.
  // Parameter values: 1, 2 or 4; default is 4.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_LANES   = 32'd4,
  // Parameter Description: flops of each request, acknowledge and Stop State synchroniser.
  // Parameter values: 2 or 3; default is 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_SYNC_STAGES = 32'd2,
  // Parameter Description: cycles events stay blocked after soft_reset and phy_rst_in_progress.
  // Parameter values: 2 x (PARA_SYNC_STAGES + 2) + 1 to 65535; default is 256.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_RST_TAIL    = 32'd256
) (
  input  wire                      i_sys_clk,
  input  wire                      i_rxbyteclkhs,
  input  wire                      i_resetn,
  input  wire                      i_rxbyteclkhs_resetn,
  input  wire                      i_soft_reset,
  input  wire                      i_phy_rst_in_progress,
  input  wire                      i_global_irq_enable,
  input  wire  [23:0]              i_irq_enable_mask,
  input  wire  [23:0]              i_irq_clear_strobe,
  output logic [23:0]              o_irq_status,
  output logic [31:0]              o_error_capture,
  output logic [31:0]              o_ecc_err_cnt,
  output logic [31:0]              o_crc_err_cnt,
  output logic [31:0]              o_frame_drop_cnt,
  output logic [15:0]              o_lane_abort_cnt,
  output logic [31:0]              o_packet_cnt,
  output logic                     o_linebuf_ovf_sticky,
  output logic                     o_sp_fifo_ovf_sticky,
  output logic                     o_csi2_irq,
  input  wire  [PARA_MAX_LANES-1:0] i_errsoths,
  input  wire  [PARA_MAX_LANES-1:0] i_errsotsynchs,
  input  wire                      i_cl_stopstate,
  input  wire                      i_lane_abort_pulse,
  input  wire                      i_err_frame_sync,
  input  wire                      i_err_frame_level,
  input  wire                      i_err_crc,
  input  wire                      i_err_ecc_1bit,
  input  wire                      i_err_ecc_2bit,
  input  wire                      i_err_unsupported_dt,
  input  wire                      i_err_wc,
  input  wire  [1:0]               i_err_vc,
  input  wire                      i_err_lane_abort,
  input  wire                      i_packet_done_pulse,
  input  wire                      i_sp_fifo_overflow,
  input  wire                      i_err_wc_align,
  input  wire                      i_active_sel,
  input  wire                      i_linebuffer_full,
  input  wire                      i_linebuf_overflow,
  input  wire                      i_fb_overflow,
  input  wire                      i_capture_done,
  input  wire                      i_err_resolution,
  input  wire                      i_err_frame_num,
  input  wire  [13:0]              i_cur_frame_id_vc0,
  input  wire  [13:0]              i_cur_frame_id_vc1,
  input  wire  [15:0]              i_cur_line_vc0,
  input  wire  [15:0]              i_cur_line_vc1,
  input  wire                      i_meta_overflow,
  input  wire                      i_frame_drop_pulse,
  input  wire                      i_dma_buf_done_ok,
  input  wire                      i_dma_buf_done_err,
  input  wire                      i_dma_axi_write_error,
  input  wire                      i_dma_buffer_overflow,
  output logic                     o_dma_ovf_sticky
);

  // ------------------------------------------------------------------------------------------
  // Constants, parameter check and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_IRQ_WIDTH      = 32'd24;
  localparam logic [31:0] LPARA_FRAME_ID_WIDTH = 32'd14;
  localparam logic [31:0] LPARA_TAIL_CNT_WIDTH = $clog2(PARA_RST_TAIL + 32'd1);
  localparam logic [31:0] LPARA_ES_NUM         = 32'd3;
  localparam logic [31:0] LPARA_TRG_NUM        = 32'd5;
  // Trigger index 0 to 4 in IRQ_STATUS bit order; 1 when the trigger carries its channel on err_vc.
  localparam logic [4:0]  LPARA_TRG_KNOWN      = 5'b01011;
  // IRQ_STATUS bit positions used outside the event map.
  localparam logic [31:0] LPARA_BIT_FRAME_SYNC_VC0   = 32'd1;
  localparam logic [31:0] LPARA_BIT_FRAME_SYNC_VC1   = 32'd3;
  localparam logic [31:0] LPARA_BIT_CRC_ERROR        = 32'd4;
  localparam logic [31:0] LPARA_BIT_ECC_1BIT_ERROR   = 32'd5;
  localparam logic [31:0] LPARA_BIT_ECC_2BIT_ERROR   = 32'd6;
  localparam logic [31:0] LPARA_BIT_WC_CORRUPTION    = 32'd8;
  localparam logic [31:0] LPARA_BIT_FRAME_DROPPED    = 32'd14;
  localparam logic [31:0] LPARA_BIT_LANE_ALIGN_ABORT = 32'd19;

  // Illegal parameter values stop elaboration.
  generate
    if (!((PARA_MAX_LANES == 32'd1) || (PARA_MAX_LANES == 32'd2) || (PARA_MAX_LANES == 32'd4)) ||
        !((PARA_SYNC_STAGES == 32'd2) || (PARA_SYNC_STAGES == 32'd3)) ||
        (PARA_RST_TAIL < (32'd2 * (PARA_SYNC_STAGES + 32'd2) + 32'd1)) ||
        (PARA_RST_TAIL > 32'd65535))
    begin : gen_para_check
      $error("m_fpt_irq_aggregator: unsupported PARA_MAX_LANES, PARA_SYNC_STAGES or PARA_RST_TAIL");
    end
  endgenerate

  // Event synchronisers ES0 (errsoths), ES1 (errsotsynchs), ES2 (lane_abort_pulse)
  logic [LPARA_ES_NUM-1:0]     w_ev_s;
  logic [LPARA_ES_NUM-1:0]     w_idle;
  logic [LPARA_ES_NUM-1:0]     w_send;
  logic [LPARA_ES_NUM-1:0]     reg_req_tgl;
  logic [LPARA_ES_NUM-1:0]     reg_pend;
  logic [PARA_SYNC_STAGES-1:0] reg_ack_sync [0:LPARA_ES_NUM-1];
  logic [PARA_SYNC_STAGES-1:0] reg_req_sync [0:LPARA_ES_NUM-1];
  logic [LPARA_ES_NUM-1:0]     reg_req;
  logic [LPARA_ES_NUM-1:0]     w_ev_d;
  // Clock-lane Stop State
  logic [PARA_SYNC_STAGES-1:0] reg_stop_sync;
  logic                        reg_stop;
  logic                        w_ev_stop;
  // Event Conditioner
  logic                        w_x_linebuffer_full;
  logic                        reg_x_linebuffer_full;
  logic                        reg_x_linebuf_overflow;
  logic                        reg_x_sp_fifo_overflow;
  logic                        reg_x_meta_overflow;
  logic                        reg_x_dma_buffer_overflow;
  logic                        w_rise_linebuffer_full;
  logic                        w_rise_linebuf_overflow;
  logic                        w_rise_sp_fifo_overflow;
  logic                        w_rise_meta_overflow;
  logic                        w_rise_dma_buffer_overflow;
  logic [LPARA_IRQ_WIDTH-1:0]  w_evt;
  // Reset Gate
  logic                        w_gate_src;
  logic                        w_evt_block;
  logic [LPARA_TAIL_CNT_WIDTH-1:0] reg_tail_cnt;
  logic [LPARA_IRQ_WIDTH-1:0]  w_evt_g;
  logic                        w_pkt_g;
  logic                        w_lb_ovf_g;
  logic                        w_sp_ovf_g;
  logic                        w_dma_ovf_g;
  logic                        w_err_lane_abort_g;
  // IRQ Status Register and Interrupt Output
  logic [LPARA_IRQ_WIDTH-1:0]  w_flag_d;
  logic [LPARA_IRQ_WIDTH-1:0]  reg_flag;
  logic                        w_irq_d;
  logic                        reg_csi2_irq;
  // Event Counters and Sticky Overflow Flags
  logic [15:0]                 reg_ecc_1bit_count;
  logic [15:0]                 reg_ecc_2bit_count;
  logic [31:0]                 reg_crc_error_count;
  logic [31:0]                 reg_frame_drop_count;
  logic [15:0]                 reg_lane_abort_count;
  logic [31:0]                 reg_packet_count;
  logic                        reg_lb_ovf;
  logic                        reg_sp_ovf;
  logic                        reg_dma_ovf;
  // Error Capture Unit
  logic [LPARA_FRAME_ID_WIDTH-1:0] reg_fid [0:1];
  logic [1:0]                  w_chg;
  logic                        w_new_cap;
  logic [LPARA_FRAME_ID_WIDTH-1:0] w_new_id;
  logic [LPARA_FRAME_ID_WIDTH-1:0] reg_newest_fid;
  logic [LPARA_FRAME_ID_WIDTH-1:0] reg_prev_fid;
  logic [LPARA_FRAME_ID_WIDTH-1:0] w_newest_nxt;
  logic [LPARA_FRAME_ID_WIDTH-1:0] w_prev_nxt;
  logic                        reg_arm_new;
  logic                        reg_arm_prev;
  logic                        w_arm_new_nxt;
  logic                        w_arm_prev_nxt;
  logic [LPARA_TRG_NUM-1:0]    w_trg;
  logic                        w_vc_k;
  logic [LPARA_FRAME_ID_WIDTH-1:0] w_fid_k;
  logic [15:0]                 w_line_k;
  logic [LPARA_TRG_NUM-1:0]    w_hit_new;
  logic [LPARA_TRG_NUM-1:0]    w_hit_prev;
  logic [LPARA_TRG_NUM-1:0]    w_hit_sel;
  logic                        w_rec_new;
  logic                        w_rec_prev;
  logic                        w_known_sel;
  logic [1:0]                  w_fault_vc;
  logic [LPARA_FRAME_ID_WIDTH-1:0] w_fault_frame_id;
  logic [15:0]                 w_fault_line_number;
  logic [31:0]                 reg_error_capture;

  // ------------------------------------------------------------------------------------------
  // Event Synchroniser (ES0 to ES2) and Ingress Transfer Control
  // ------------------------------------------------------------------------------------------
  // Source events in rxbyteclkhs: the per-lane OR is formed before the channel. A channel is idle
  // when its request toggle equals the acknowledge seen through the synchroniser.
  always_comb begin : p_com_es_src
    w_ev_s[0] = |i_errsoths;
    w_ev_s[1] = |i_errsotsynchs;
    w_ev_s[2] = i_lane_abort_pulse;
    for (int idx = 0; idx < LPARA_ES_NUM; idx++) begin
      w_idle[idx] = (reg_req_tgl[idx] == reg_ack_sync[idx][PARA_SYNC_STAGES-1]);
      w_send[idx] = w_idle[idx] && (reg_pend[idx] || w_ev_s[idx]);
    end
  end

  // Event in sys_clk: one cycle per toggle seen at the end of the request synchroniser.
  always_comb begin : p_com_es_sys
    for (int idx = 0; idx < LPARA_ES_NUM; idx++) begin
      w_ev_d[idx] = reg_req_sync[idx][PARA_SYNC_STAGES-1] ^ reg_req[idx];
    end
  end

  generate
    for (genvar gen_idx = 0; gen_idx < LPARA_ES_NUM; gen_idx++) begin : gen_es
      // Source half, rxbyteclkhs domain, reset by i_rxbyteclkhs_resetn only: a new toggle only when
      // idle; an event while busy is held in pend. The acknowledge crosses from reg_req (sys_clk)
      // through PARA_SYNC_STAGES flops with no logic in front of the first stage.
      always_ff @(posedge i_rxbyteclkhs or negedge i_rxbyteclkhs_resetn) begin : p_ff_es_src
        if (!i_rxbyteclkhs_resetn) begin
          reg_req_tgl[gen_idx]  <= 1'b0;
          reg_pend[gen_idx]     <= 1'b0;
          reg_ack_sync[gen_idx] <= '0;
        end else begin
          reg_req_tgl[gen_idx]  <= reg_req_tgl[gen_idx] ^ w_send[gen_idx];
          reg_pend[gen_idx]     <= (reg_pend[gen_idx] || w_ev_s[gen_idx]) && !w_send[gen_idx];
          reg_ack_sync[gen_idx] <= {reg_ack_sync[gen_idx][PARA_SYNC_STAGES-2:0], reg_req[gen_idx]};
        end
      end

      // sys_clk half, reset by i_resetn only: the request toggle crosses from reg_req_tgl
      // (rxbyteclkhs) through PARA_SYNC_STAGES flops; reg_req is the previous value for edge
      // detection and the acknowledge returned to the source half.
      always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_es_sys
        if (!i_resetn) begin
          reg_req_sync[gen_idx] <= '0;
          reg_req[gen_idx]      <= 1'b0;
        end else begin
          reg_req_sync[gen_idx] <= {reg_req_sync[gen_idx][PARA_SYNC_STAGES-2:0],
                                    reg_req_tgl[gen_idx]};
          reg_req[gen_idx]      <= reg_req_sync[gen_idx][PARA_SYNC_STAGES-1];
        end
      end
    end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Clock-lane Stop State
  // ------------------------------------------------------------------------------------------
  // cl_stopstate is asynchronous and is synchronised here into sys_clk; the rising edge is taken
  // after the synchroniser. Reset by i_resetn only.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_stop
    if (!i_resetn) begin
      reg_stop_sync <= '0;
      reg_stop      <= 1'b0;
    end else begin
      reg_stop_sync <= {reg_stop_sync[PARA_SYNC_STAGES-2:0], i_cl_stopstate};
      reg_stop      <= reg_stop_sync[PARA_SYNC_STAGES-1];
    end
  end

  always_comb begin : p_com_stop
    w_ev_stop = reg_stop_sync[PARA_SYNC_STAGES-1] && !reg_stop;
  end

  // ------------------------------------------------------------------------------------------
  // Event Conditioner
  // ------------------------------------------------------------------------------------------
  // Rising edges of the level sources: the qualified line-buffer watermark and the four overflow
  // sources of IRQ_STATUS bit 13.
  always_comb begin : p_com_rise
    w_x_linebuffer_full        = i_linebuffer_full && !i_active_sel;
    w_rise_linebuffer_full     = w_x_linebuffer_full && !reg_x_linebuffer_full;
    w_rise_linebuf_overflow    = i_linebuf_overflow && !reg_x_linebuf_overflow;
    w_rise_sp_fifo_overflow    = i_sp_fifo_overflow && !reg_x_sp_fifo_overflow;
    w_rise_meta_overflow       = i_meta_overflow && !reg_x_meta_overflow;
    w_rise_dma_buffer_overflow = i_dma_buffer_overflow && !reg_x_dma_buffer_overflow;
  end

  // Previous values, updated every cycle (also while events are blocked) and reset by i_resetn
  // only, so a level held across soft_reset gives no new edge.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_rise
    if (!i_resetn) begin
      reg_x_linebuffer_full     <= 1'b0;
      reg_x_linebuf_overflow    <= 1'b0;
      reg_x_sp_fifo_overflow    <= 1'b0;
      reg_x_meta_overflow       <= 1'b0;
      reg_x_dma_buffer_overflow <= 1'b0;
    end else begin
      reg_x_linebuffer_full     <= w_x_linebuffer_full;
      reg_x_linebuf_overflow    <= i_linebuf_overflow;
      reg_x_sp_fifo_overflow    <= i_sp_fifo_overflow;
      reg_x_meta_overflow       <= i_meta_overflow;
      reg_x_dma_buffer_overflow <= i_dma_buffer_overflow;
    end
  end

  // One single-cycle event per IRQ_STATUS bit; frame errors are directed by err_vc[0] only.
  always_comb begin : p_com_evt
    w_evt     = '0;
    w_evt[0]  = i_err_frame_level && !i_err_vc[0];
    w_evt[1]  = i_err_frame_sync && !i_err_vc[0];
    w_evt[2]  = i_err_frame_level && i_err_vc[0];
    w_evt[3]  = i_err_frame_sync && i_err_vc[0];
    w_evt[4]  = i_err_crc;
    w_evt[5]  = i_err_ecc_1bit;
    w_evt[6]  = i_err_ecc_2bit;
    w_evt[7]  = i_err_unsupported_dt;
    w_evt[8]  = i_err_wc;
    w_evt[9]  = w_ev_d[0];
    w_evt[10] = w_ev_d[1];
    w_evt[11] = w_ev_stop;
    w_evt[12] = w_rise_linebuffer_full;
    w_evt[13] = w_rise_linebuf_overflow || w_rise_sp_fifo_overflow || w_rise_meta_overflow ||
                w_rise_dma_buffer_overflow;
    w_evt[14] = i_frame_drop_pulse;
    w_evt[15] = i_capture_done;
    w_evt[16] = i_dma_buf_done_ok;
    w_evt[17] = i_dma_buf_done_err;
    w_evt[18] = i_dma_axi_write_error;
    w_evt[19] = w_ev_d[2];
    w_evt[20] = i_err_resolution;
    w_evt[21] = i_err_wc_align;
    w_evt[22] = i_fb_overflow;
    w_evt[23] = i_err_frame_num;
  end

  // ------------------------------------------------------------------------------------------
  // Reset Gate
  // ------------------------------------------------------------------------------------------
  // Events are blocked while soft_reset or phy_rst_in_progress is high and for PARA_RST_TAIL cycles
  // after both are low; strobes and the capture tracking are not blocked.
  always_comb begin : p_com_gate
    w_gate_src         = i_soft_reset || i_phy_rst_in_progress;
    w_evt_block        = w_gate_src || (reg_tail_cnt != '0);
    w_evt_g            = w_evt & {LPARA_IRQ_WIDTH{!w_evt_block}};
    w_pkt_g            = i_packet_done_pulse && !w_evt_block;
    w_lb_ovf_g         = w_rise_linebuf_overflow && !w_evt_block;
    w_sp_ovf_g         = w_rise_sp_fifo_overflow && !w_evt_block;
    w_dma_ovf_g        = w_rise_dma_buffer_overflow && !w_evt_block;
    w_err_lane_abort_g = i_err_lane_abort && !w_evt_block;
  end

  // Tail counter, reloaded in every gate cycle and counting down to 0; reset by i_resetn only.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_tail
    if (!i_resetn) begin
      reg_tail_cnt <= '0;
    end else if (w_gate_src) begin
      reg_tail_cnt <= $bits(reg_tail_cnt)'(PARA_RST_TAIL);
    end else if (reg_tail_cnt != '0) begin
      reg_tail_cnt <= reg_tail_cnt - $bits(reg_tail_cnt)'(1);
    end
  end

  // ------------------------------------------------------------------------------------------
  // IRQ Status Register
  // ------------------------------------------------------------------------------------------
  // A flag is set by its event and cleared by its write-1-to-clear strobe; the set wins over a
  // clear in the same cycle.
  always_comb begin : p_com_flag
    w_flag_d = w_evt_g | (reg_flag & ~i_irq_clear_strobe);
  end

  // soft_reset clears every flag, with priority over a set in the same cycle.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_flag
    if (!i_resetn) begin
      reg_flag <= '0;
    end else if (i_soft_reset) begin
      reg_flag <= '0;
    end else begin
      reg_flag <= w_flag_d;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Interrupt Output
  // ------------------------------------------------------------------------------------------
  // csi2_irq is registered: high when an enabled flag is set and the global enable is 1; low in the
  // cycle after soft_reset.
  always_comb begin : p_com_irq
    w_irq_d = !i_soft_reset && i_global_irq_enable && (|(reg_flag & i_irq_enable_mask));
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_irq
    if (!i_resetn) begin
      reg_csi2_irq <= 1'b0;
    end else begin
      reg_csi2_irq <= w_irq_d;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Event Counters
  // ------------------------------------------------------------------------------------------
  // One increment per gated event, also when the flag is already set; the error counters saturate
  // at their maximum, packet_count wraps. soft_reset clears all six.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_counters
    if (!i_resetn) begin
      reg_ecc_1bit_count   <= '0;
      reg_ecc_2bit_count   <= '0;
      reg_crc_error_count  <= '0;
      reg_frame_drop_count <= '0;
      reg_lane_abort_count <= '0;
      reg_packet_count     <= '0;
    end else if (i_soft_reset) begin
      reg_ecc_1bit_count   <= '0;
      reg_ecc_2bit_count   <= '0;
      reg_crc_error_count  <= '0;
      reg_frame_drop_count <= '0;
      reg_lane_abort_count <= '0;
      reg_packet_count     <= '0;
    end else begin
      if (w_evt_g[LPARA_BIT_ECC_1BIT_ERROR] && (reg_ecc_1bit_count != 16'hFFFF)) begin
        reg_ecc_1bit_count <= reg_ecc_1bit_count + 16'd1;
      end
      if (w_evt_g[LPARA_BIT_ECC_2BIT_ERROR] && (reg_ecc_2bit_count != 16'hFFFF)) begin
        reg_ecc_2bit_count <= reg_ecc_2bit_count + 16'd1;
      end
      if (w_evt_g[LPARA_BIT_CRC_ERROR] && (reg_crc_error_count != 32'hFFFF_FFFF)) begin
        reg_crc_error_count <= reg_crc_error_count + 32'd1;
      end
      if (w_evt_g[LPARA_BIT_FRAME_DROPPED] && (reg_frame_drop_count != 32'hFFFF_FFFF)) begin
        reg_frame_drop_count <= reg_frame_drop_count + 32'd1;
      end
      if (w_evt_g[LPARA_BIT_LANE_ALIGN_ABORT] && (reg_lane_abort_count != 16'hFFFF)) begin
        reg_lane_abort_count <= reg_lane_abort_count + 16'd1;
      end
      if (w_pkt_g) begin
        reg_packet_count <= reg_packet_count + 32'd1;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Sticky Overflow Flags
  // ------------------------------------------------------------------------------------------
  // Set by the first gated rising edge of their source and held; no write-1-to-clear, only a reset
  // clears them.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_sticky
    if (!i_resetn) begin
      reg_lb_ovf  <= 1'b0;
      reg_sp_ovf  <= 1'b0;
      reg_dma_ovf <= 1'b0;
    end else if (i_soft_reset) begin
      reg_lb_ovf  <= 1'b0;
      reg_sp_ovf  <= 1'b0;
      reg_dma_ovf <= 1'b0;
    end else begin
      reg_lb_ovf  <= reg_lb_ovf || w_lb_ovf_g;
      reg_sp_ovf  <= reg_sp_ovf || w_sp_ovf_g;
      reg_dma_ovf <= reg_dma_ovf || w_dma_ovf_g;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Error Capture Unit: Capture Tracker
  // ------------------------------------------------------------------------------------------
  // A capture is new when a channel moves to a non-zero frame_id other than its previous value and
  // other than the newest capture; a lag channel joining the newest capture opens none. The new
  // capture is applied before the triggers of the same cycle.
  always_comb begin : p_com_tracker
    w_chg[0]       = (i_cur_frame_id_vc0 != reg_fid[0]) && (i_cur_frame_id_vc0 != '0) &&
                     (i_cur_frame_id_vc0 != reg_newest_fid);
    w_chg[1]       = (i_cur_frame_id_vc1 != reg_fid[1]) && (i_cur_frame_id_vc1 != '0) &&
                     (i_cur_frame_id_vc1 != reg_newest_fid);
    w_new_cap      = w_chg[0] || w_chg[1];
    w_new_id       = w_chg[0] ? i_cur_frame_id_vc0 : i_cur_frame_id_vc1;
    w_newest_nxt   = w_new_cap ? w_new_id : reg_newest_fid;
    w_prev_nxt     = w_new_cap ? reg_newest_fid : reg_prev_fid;
    w_arm_new_nxt  = w_new_cap ? 1'b1 : reg_arm_new;
    w_arm_prev_nxt = w_new_cap ? reg_arm_new : reg_arm_prev;
  end

  // Previous frame_id of each channel, reset by i_resetn only, so a frame_id that still names a
  // capture opened before soft_reset is not taken for a new capture.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_fid
    if (!i_resetn) begin
      reg_fid[0] <= '0;
      reg_fid[1] <= '0;
    end else begin
      reg_fid[0] <= i_cur_frame_id_vc0;
      reg_fid[1] <= i_cur_frame_id_vc1;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Error Capture Unit: Trigger Select
  // ------------------------------------------------------------------------------------------
  // Triggers in IRQ_STATUS bit order: frame synchronisation, CRC, uncorrectable header, Word
  // Count, lane abort. A known-channel trigger belongs to the capture named by cur_frame_id of
  // err_vc[0], an unknown one to the newest capture. The newest capture wins, then the lowest
  // trigger; a trigger with frame_id 0 is not recorded.
  always_comb begin : p_com_trigger
    w_trg    = {w_err_lane_abort_g, w_evt_g[LPARA_BIT_WC_CORRUPTION],
                w_evt_g[LPARA_BIT_ECC_2BIT_ERROR], w_evt_g[LPARA_BIT_CRC_ERROR],
                w_evt_g[LPARA_BIT_FRAME_SYNC_VC0] || w_evt_g[LPARA_BIT_FRAME_SYNC_VC1]};
    w_vc_k   = i_err_vc[0];
    w_fid_k  = w_vc_k ? i_cur_frame_id_vc1 : i_cur_frame_id_vc0;
    w_line_k = w_vc_k ? i_cur_line_vc1 : i_cur_line_vc0;
    w_hit_new  = '0;
    w_hit_prev = '0;
    for (int idx = 0; idx < LPARA_TRG_NUM; idx++) begin
      if (LPARA_TRG_KNOWN[idx]) begin
        w_hit_new[idx]  = w_trg[idx] && w_arm_new_nxt && (w_fid_k == w_newest_nxt) &&
                          (w_fid_k != '0);
        w_hit_prev[idx] = w_trg[idx] && w_arm_prev_nxt && (w_fid_k == w_prev_nxt) &&
                          (w_fid_k != '0);
      end else begin
        w_hit_new[idx]  = w_trg[idx] && w_arm_new_nxt && (w_newest_nxt != '0);
      end
    end
    w_rec_new   = |w_hit_new;
    w_rec_prev  = !w_rec_new && (|w_hit_prev);
    w_hit_sel   = w_rec_new ? w_hit_new : w_hit_prev;
    w_known_sel = 1'b1;
    for (int idx = LPARA_TRG_NUM - 32'd1; idx >= 0; idx--) begin
      if (w_hit_sel[idx]) begin
        w_known_sel = LPARA_TRG_KNOWN[idx];
      end
    end
    w_fault_vc          = w_known_sel ? {1'b0, w_vc_k} : 2'b11;
    w_fault_frame_id    = w_known_sel ? w_fid_k : w_newest_nxt;
    w_fault_line_number = w_known_sel ? w_line_k : 16'hFFFF;
  end

  // ------------------------------------------------------------------------------------------
  // Error Capture Unit: Capture Arm Control and error_capture
  // ------------------------------------------------------------------------------------------
  // The newest capture and the previous one are tracked, each armed while none of its errors has
  // been recorded; a record of the newest disarms both. soft_reset clears both captures and arms
  // the newest only, with priority over a record in the same cycle.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_capture
    if (!i_resetn) begin
      reg_newest_fid    <= '0;
      reg_prev_fid      <= '0;
      reg_arm_new       <= 1'b1;
      reg_arm_prev      <= 1'b0;
      reg_error_capture <= '0;
    end else if (i_soft_reset) begin
      reg_newest_fid    <= '0;
      reg_prev_fid      <= '0;
      reg_arm_new       <= 1'b1;
      reg_arm_prev      <= 1'b0;
      reg_error_capture <= '0;
    end else begin
      reg_newest_fid <= w_newest_nxt;
      reg_prev_fid   <= w_prev_nxt;
      reg_arm_new    <= w_rec_new ? 1'b0 : w_arm_new_nxt;
      reg_arm_prev   <= (w_rec_new || w_rec_prev) ? 1'b0 : w_arm_prev_nxt;
      if (w_rec_new || w_rec_prev) begin
        reg_error_capture <= {w_fault_vc, w_fault_frame_id, w_fault_line_number};
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  // Every output is taken directly from a register.
  assign o_irq_status         = reg_flag;
  assign o_csi2_irq           = reg_csi2_irq;
  assign o_ecc_err_cnt        = {reg_ecc_2bit_count, reg_ecc_1bit_count};
  assign o_crc_err_cnt        = reg_crc_error_count;
  assign o_frame_drop_cnt     = reg_frame_drop_count;
  assign o_lane_abort_cnt     = reg_lane_abort_count;
  assign o_packet_cnt         = reg_packet_count;
  assign o_linebuf_ovf_sticky = reg_lb_ovf;
  assign o_sp_fifo_ovf_sticky = reg_sp_ovf;
  assign o_dma_ovf_sticky     = reg_dma_ovf;
  assign o_error_capture      = reg_error_capture;

endmodule
`default_nettype wire
