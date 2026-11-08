// Module      : m_fpt_packet_analyzer
// Block       : B03 - Packet Analyzer
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-06
// Description : Checks CSI-2 header ECC and payload CRC, routes packets by Data Type, tracks frames.
`timescale 1ns/1ps
`default_nettype none

module m_fpt_packet_analyzer #(
  // Parameter Description: depth of the generic short packet FIFO.
  // Parameter values: power of two from 2 to 256; default is 8.
  // Parameter unit: count
  parameter logic [31:0] PARA_SP_FIFO_DEPTH = 8,
  // Parameter Description: register stages inside the header ECC engine, also the depth of the
  // delay register that keeps payload words aligned with their header; 0 removes both.
  // Parameter values: 0 or 1; default is 1.
  // Parameter unit: cycle
  parameter logic [31:0] PARA_ECC_LATENCY   = 1
) (
  input  wire         i_sys_clk,
  input  wire         i_resetn,
  input  wire         i_soft_reset,
  input  wire         i_core_enable,
  input  wire  [5:0]  i_metadata_dt,
  input  wire         i_metadata_capture_en,
  input  wire  [37:0] i_rd_data,
  input  wire         i_rd_empty,
  output logic        o_rd_en,
  input  wire  [1:0]  i_force_discard,
  output logic [31:0] o_bp_data,
  output logic [3:0]  o_bp_byte_en,
  output logic        o_bp_valid,
  output logic        o_bp_sof,
  output logic        o_bp_sol,
  output logic        o_bp_eol,
  output logic [1:0]  o_bp_vc_tag,
  output logic        o_bp_dt_sel,
  output logic [31:0] o_emb_data,
  output logic [3:0]  o_emb_byte_en,
  output logic        o_emb_valid,
  output logic        o_emb_sol,
  output logic        o_emb_eol,
  output logic [1:0]  o_emb_vc,
  output logic        o_fc_fs,
  output logic        o_fc_fe,
  output logic [15:0] o_fc_frame_num,
  output logic [1:0]  o_fc_vc,
  output logic [23:0] o_gsp_data,
  input  wire         i_gsp_pop,
  output logic        o_sp_fifo_not_empty,
  output logic        o_sp_fifo_full,
  output logic        o_sp_fifo_overflow,
  output logic        o_null_blank_seen,
  output logic        o_packet_done_pulse,
  output logic        o_err_ecc_1bit,
  output logic        o_err_ecc_2bit,
  output logic        o_err_crc,
  output logic        o_err_wc,
  output logic        o_err_unsupported_dt,
  output logic [1:0]  o_err_vc,
  output logic        o_err_lane_abort,
  output logic        o_err_frame_sync,
  output logic        o_err_frame_level,
  output logic        o_vc_unsupported_seen,
  output logic        o_dt_conflict
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and internal signals
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_WORD_WIDTH    = 32'd32;
  localparam logic [31:0] LPARA_SP_FIFO_WIDTH = 32'd24;
  localparam int          LPARA_BYTE_NUM      = 4;
  localparam int          LPARA_HDR_DATA_W    = 24;
  localparam int          LPARA_PTR_W         = $clog2(PARA_SP_FIFO_DEPTH);
  localparam int          LPARA_CNT_W         = $clog2(PARA_SP_FIFO_DEPTH + 32'd1);
  localparam logic [LPARA_CNT_W-1:0] LPARA_DEPTH_CNT = LPARA_CNT_W'(PARA_SP_FIFO_DEPTH);

  // CSI-2 Data Type codes used by the decoders.
  localparam logic [5:0]  LPARA_DT_FS         = 6'h00;
  localparam logic [5:0]  LPARA_DT_FE         = 6'h01;
  localparam logic [2:0]  LPARA_DT_GSP_HI     = 3'b001;
  localparam logic [5:0]  LPARA_DT_NULL       = 6'h10;
  localparam logic [5:0]  LPARA_DT_BLANK      = 6'h11;
  localparam logic [5:0]  LPARA_DT_RAW10      = 6'h2B;
  localparam logic [5:0]  LPARA_DT_RAW12      = 6'h2C;

  // CSI-2 payload checksum: polynomial x^16 + x^12 + x^5 + 1 in reflected form, seed all ones.
  localparam logic [15:0] LPARA_CRC_POLY      = 16'h8408;
  localparam logic [15:0] LPARA_CRC_INIT      = 16'hFFFF;

  // Packet FSM states.
  typedef enum logic [2:0] {
    ENUM_ST_IDLE    = 3'b000,
    ENUM_ST_HEADER  = 3'b001,
    ENUM_ST_PAYLOAD = 3'b010,
    ENUM_ST_DISCARD = 3'b011,
    ENUM_ST_DONE    = 3'b100
  } pkt_state_type_def;

  // Input Adapter
  logic                          w_flush;
  logic                          w_rd_req;
  logic                          w_rd_en;
  logic                          reg_rd_valid;
  logic [LPARA_WORD_WIDTH-1:0]   reg_input_data;
  logic [LPARA_BYTE_NUM-1:0]     reg_input_byte_valid;
  logic                          reg_input_sob;
  logic                          reg_input_eob;
  logic                          w_input_valid;
  logic                          w_abort_marker;

  // Header Processor
  logic [29:0]                   reg_word0_reg;
  logic                          reg_hdr_load;
  logic [5:0]                    w_ecc_parity;
  logic [5:0]                    w_syndrome;
  logic [LPARA_HDR_DATA_W-1:0]   w_ecc_hit;
  logic                          w_ecc_parity_err;
  logic [LPARA_HDR_DATA_W-1:0]   w_ecc_corrected;
  logic                          w_ecc_1bit_raw;
  logic                          w_ecc_2bit_raw;
  logic                          w_hdr_valid;
  logic [LPARA_HDR_DATA_W-1:0]   w_hdr_data;
  logic                          w_err_ecc_1bit;
  logic                          w_err_ecc_2bit;
  logic [1:0]                    w_hdr_vc;
  logic [5:0]                    w_hdr_dt;
  logic [15:0]                   w_hdr_wc;
  logic [15:0]                   w_hdr_field2;
  logic                          w_is_short;
  logic                          w_hdr_ok;
  logic                          w_vc_ok;
  logic                          w_long_ok;
  logic                          w_hdr_pyld_valid;
  logic                          w_hdr_pyld_marker;
  logic                          w_hdr_pyld_eob;
  logic [LPARA_BYTE_NUM-1:0]     w_hdr_pyld_byte_valid;
  logic [LPARA_WORD_WIDTH-1:0]   w_hdr_pyld_data;

  // DT Decoder
  logic                          w_dt_meta_hit;
  logic                          w_dt_reserved;
  logic                          w_pkt_is_embedded;
  logic                          w_pkt_is_image;
  logic                          w_dt_meta_drop;
  logic                          w_dt_null;
  logic                          w_dt_unsupported;
  logic                          w_bp_dt_sel;
  logic                          w_dt_conflict;

  // Short Packet Handler and Generic Short Packet FIFO
  logic                          w_fc_fs_raw;
  logic                          w_fc_fe_raw;
  logic                          w_fc_fs;
  logic                          w_fc_fe;
  logic                          w_gsp_wr;
  logic                          w_gsp_not_empty;
  logic                          w_gsp_full;
  logic                          w_gsp_pop_ok;
  logic                          w_gsp_push;
  logic                          w_sp_fifo_overflow;
  logic [LPARA_SP_FIFO_WIDTH-1:0] w_gsp_data;
  logic [LPARA_SP_FIFO_WIDTH-1:0] reg_gsp_mem [PARA_SP_FIFO_DEPTH];
  logic [LPARA_PTR_W-1:0]        reg_gsp_wr_ptr;
  logic [LPARA_PTR_W-1:0]        reg_gsp_rd_ptr;
  logic [LPARA_CNT_W-1:0]        reg_gsp_cnt;

  // Payload Path
  logic                          w_load;
  logic                          w_armed;
  logic [15:0]                   w_rem_cur;
  logic [1:0]                    w_footer_cnt_cur;
  logic [15:0]                   w_crc_cur;
  logic                          w_word_active;
  logic [2:0]                    w_word_bytes;
  logic [2:0]                    w_pay_n;
  logic [LPARA_BYTE_NUM-1:0]     w_payload_byte_en;
  logic                          w_payload_valid;
  logic                          w_payload_last;
  logic [15:0]                   w_rem_nxt;
  logic [2:0]                    w_crc_avail;
  logic [1:0]                    w_crc_need;
  logic [1:0]                    w_crc_in;
  logic [1:0]                    w_footer_cnt_nxt;
  logic [7:0]                    w_crc_ls;
  logic [7:0]                    w_crc_ms;
  logic                          w_payload_done;
  logic [15:0]                   w_crc_nxt;
  logic                          w_crc_mismatch;
  logic                          w_pkt_end;
  logic                          w_err_wc_pyld;
  logic                          reg_wc_armed;
  logic [15:0]                   reg_remaining_bytes;
  logic [1:0]                    reg_footer_cnt;
  logic [15:0]                   reg_crc;
  logic [7:0]                    reg_crc_lo;

  // Current packet attributes
  logic [1:0]                    w_cur_vc;
  logic                          w_cur_vc_ok;
  logic                          w_cur_image;
  logic                          w_cur_emb;
  logic                          w_cur_sof;
  logic                          w_sol_pend_cur;
  logic [1:0]                    reg_pkt_vc;
  logic                          reg_pkt_vc_ok;
  logic                          reg_pkt_is_image;
  logic                          reg_pkt_is_embedded;
  logic                          reg_pkt_sof;
  logic                          reg_sol_pend;
  logic                          reg_pkt_long;

  // Frame Supervisor
  logic                          w_hdr_fa;
  logic                          w_hdr_err_pend;
  logic                          w_err_frame_sync;
  logic                          w_err_frame_level;
  logic                          w_err_unsupported_dt;
  logic                          w_err_wc_hdr;
  logic [1:0]                    w_pkt_err_set;
  logic [1:0]                    w_frame_active_nxt;
  logic [1:0]                    w_first_image_seen_nxt;
  logic [1:0]                    w_pkt_err_in_frame_nxt;
  logic [1:0]                    reg_frame_active;
  logic [1:0]                    reg_first_image_seen;
  logic [1:0]                    reg_pkt_err_in_frame;

  // Output Routing, events and sticky status
  logic                          w_discard_gate;
  logic                          w_fwd;
  logic                          w_bp_valid;
  logic                          w_bp_sof;
  logic                          w_bp_sol;
  logic                          w_bp_eol;
  logic                          w_emb_valid;
  logic                          w_emb_sol;
  logic                          w_emb_eol;
  logic                          w_err_ecc_1bit_ev;
  logic                          w_err_ecc_2bit_ev;
  logic                          w_err_crc_det;
  logic                          w_err_wc;
  logic                          w_err_lane_abort;
  logic                          w_hdr_err_any;
  logic [1:0]                    w_err_vc;
  logic                          w_packet_done_pulse;
  logic                          w_null_set;
  logic                          w_vc_unsup_set;
  logic                          reg_err_crc_d;
  logic [1:0]                    reg_err_crc_vc_d;
  logic [LPARA_WORD_WIDTH-1:0]   reg_bp_data;
  logic [LPARA_BYTE_NUM-1:0]     reg_bp_byte_en;
  logic                          reg_bp_valid;
  logic                          reg_bp_sof;
  logic                          reg_bp_sol;
  logic                          reg_bp_eol;
  logic [1:0]                    reg_bp_vc_tag;
  logic                          reg_bp_dt_sel;
  logic [LPARA_WORD_WIDTH-1:0]   reg_emb_data;
  logic [LPARA_BYTE_NUM-1:0]     reg_emb_byte_en;
  logic                          reg_emb_valid;
  logic                          reg_emb_sol;
  logic                          reg_emb_eol;
  logic [1:0]                    reg_emb_vc;
  logic                          reg_fc_fs;
  logic                          reg_fc_fe;
  logic [15:0]                   reg_fc_frame_num;
  logic [1:0]                    reg_fc_vc;
  logic                          reg_sp_fifo_overflow;
  logic                          reg_packet_done_pulse;
  logic                          reg_err_ecc_1bit;
  logic                          reg_err_ecc_2bit;
  logic                          reg_err_crc;
  logic                          reg_err_wc;
  logic                          reg_err_unsupported_dt;
  logic [1:0]                    reg_err_vc;
  logic                          reg_err_lane_abort;
  logic                          reg_err_frame_sync;
  logic                          reg_err_frame_level;
  logic                          reg_null_blank_seen;
  logic                          reg_vc_unsupported_seen;
  logic                          reg_dt_conflict;

  // Packet FSM
  pkt_state_type_def             reg_fsm_pkt_current_state;
  pkt_state_type_def             w_fsm_pkt_nxt_state;
  logic                          reg_eob_latched;
  logic                          w_eob_latched_nxt;
  logic                          w_pyld_eob;

  // Parity code of header data bit idx (P5..P0), as defined by CSI-2 for the 24-bit header.
  function automatic logic [5:0] f_ecc_code(input logic [4:0] idx);
    case (idx)
      5'd0:    f_ecc_code = 6'h07;
      5'd1:    f_ecc_code = 6'h0B;
      5'd2:    f_ecc_code = 6'h0D;
      5'd3:    f_ecc_code = 6'h0E;
      5'd4:    f_ecc_code = 6'h13;
      5'd5:    f_ecc_code = 6'h15;
      5'd6:    f_ecc_code = 6'h16;
      5'd7:    f_ecc_code = 6'h19;
      5'd8:    f_ecc_code = 6'h1A;
      5'd9:    f_ecc_code = 6'h1C;
      5'd10:   f_ecc_code = 6'h23;
      5'd11:   f_ecc_code = 6'h25;
      5'd12:   f_ecc_code = 6'h26;
      5'd13:   f_ecc_code = 6'h29;
      5'd14:   f_ecc_code = 6'h2A;
      5'd15:   f_ecc_code = 6'h2C;
      5'd16:   f_ecc_code = 6'h31;
      5'd17:   f_ecc_code = 6'h32;
      5'd18:   f_ecc_code = 6'h34;
      5'd19:   f_ecc_code = 6'h38;
      5'd20:   f_ecc_code = 6'h1F;
      5'd21:   f_ecc_code = 6'h2F;
      5'd22:   f_ecc_code = 6'h37;
      5'd23:   f_ecc_code = 6'h3B;
      default: f_ecc_code = 6'h00;
    endcase
  endfunction

  // One byte through the checksum register, least significant bit first.
  function automatic logic [15:0] f_crc16_byte(input logic [15:0] crc_in, input logic [7:0] data_in);
    logic [15:0] w_crc_bit;
    w_crc_bit = crc_in;
    for (int idx = 0; idx < 8; idx++) begin
      if (w_crc_bit[0] ^ data_in[idx]) begin
        w_crc_bit = (w_crc_bit >> 1) ^ LPARA_CRC_POLY;
      end else begin
        w_crc_bit = w_crc_bit >> 1;
      end
    end
    return w_crc_bit;
  endfunction

  // Number of valid bytes in a word; byte_valid is contiguous from bit 0.
  function automatic logic [2:0] f_byte_cnt(input logic [3:0] byte_valid);
    f_byte_cnt = 3'd0;
    for (int idx = 0; idx < LPARA_BYTE_NUM; idx++) begin
      f_byte_cnt = f_byte_cnt + {2'd0, byte_valid[idx]};
    end
  endfunction

  // Byte-enable mask covering the lowest cnt byte lanes.
  function automatic logic [3:0] f_byte_mask(input logic [2:0] cnt);
    case (cnt)
      3'd0:    f_byte_mask = 4'b0000;
      3'd1:    f_byte_mask = 4'b0001;
      3'd2:    f_byte_mask = 4'b0011;
      3'd3:    f_byte_mask = 4'b0111;
      default: f_byte_mask = 4'b1111;
    endcase
  endfunction

  // Byte lane sel of a word; lane 0 is the earliest byte.
  function automatic logic [7:0] f_byte_sel(input logic [31:0] data_in, input logic [1:0] sel);
    case (sel)
      2'd0:    f_byte_sel = data_in[7:0];
      2'd1:    f_byte_sel = data_in[15:8];
      2'd2:    f_byte_sel = data_in[23:16];
      default: f_byte_sel = data_in[31:24];
    endcase
  endfunction

  // ------------------------------------------------------------------------------------------
  // Parameter checks
  // ------------------------------------------------------------------------------------------
  generate
  if (!((PARA_SP_FIFO_DEPTH >= 32'd2) && (PARA_SP_FIFO_DEPTH <= 32'd256) &&
        ((PARA_SP_FIFO_DEPTH & (PARA_SP_FIFO_DEPTH - 32'd1)) == 32'd0)))
  begin : gen_chk_fifo_depth
    $error("PARA_SP_FIFO_DEPTH must be a power of two from 2 to 256");
  end
  if (PARA_ECC_LATENCY > 32'd1) begin : gen_chk_ecc_latency
    $error("PARA_ECC_LATENCY must be 0 or 1");
  end
  endgenerate

  // ------------------------------------------------------------------------------------------
  // Input Adapter
  // ------------------------------------------------------------------------------------------
  // soft_reset and core_enable = 0 both stop reading and flush the packet in flight. rd_en is
  // also held low while resetn is asserted so that no entry is lost during reset.
  always_comb begin : p_com_rd_ctrl
    w_flush  = i_soft_reset | ~i_core_enable;
    w_rd_req = ~i_rd_empty & ~w_flush;
    w_rd_en  = w_rd_req & i_resetn;
  end

  // Input register: the show-ahead FIFO presents its head entry, which is captured on the same
  // edge that consumes it. Payload fields load only on a read.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_input_reg
    if (!i_resetn) begin
      reg_rd_valid         <= 1'b0;
      reg_input_data       <= '0;
      reg_input_byte_valid <= '0;
      reg_input_sob        <= 1'b0;
      reg_input_eob        <= 1'b0;
    end else begin
      reg_rd_valid <= w_rd_req;
      if (w_rd_req) begin
        reg_input_data       <= i_rd_data[31:0];
        reg_input_byte_valid <= i_rd_data[35:32];
        reg_input_sob        <= i_rd_data[36];
        reg_input_eob        <= i_rd_data[37];
      end
    end
  end

  // An entry with sob, eob and an empty byte mask marks a burst discarded upstream: it becomes
  // abort_marker and never enters the pipeline as a word.
  always_comb begin : p_com_input_adapter
    w_abort_marker = reg_rd_valid & reg_input_sob & reg_input_eob &
                     (reg_input_byte_valid == 4'b0000) & ~w_flush;
    w_input_valid  = reg_rd_valid & ~w_abort_marker & ~w_flush;
  end

  // ------------------------------------------------------------------------------------------
  // Header Processor
  // ------------------------------------------------------------------------------------------
  // Word 0 Latch: stores the 30 header bits on the first word of a burst, independent of the
  // Packet FSM; ECC bits 7:6 are not stored.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_word0
    if (!i_resetn) begin
      reg_word0_reg <= '0;
      reg_hdr_load  <= 1'b0;
    end else begin
      reg_hdr_load <= w_input_valid & reg_input_sob;
      if (w_input_valid & reg_input_sob) begin
        reg_word0_reg <= reg_input_data[29:0];
      end
    end
  end

  // ECC engine: syndrome of the six received parity bits against the parity of the received
  // data. A data-bit code corrects that bit, a single parity-bit syndrome leaves the header
  // unchanged, any other non-zero syndrome is uncorrectable.
  always_comb begin : p_com_ecc
    w_ecc_parity = 6'd0;
    w_ecc_hit    = '0;
    for (int idx = 0; idx < LPARA_HDR_DATA_W; idx++) begin
      if (reg_word0_reg[idx]) begin
        w_ecc_parity = w_ecc_parity ^ f_ecc_code(5'(idx));
      end
    end
    w_syndrome = reg_word0_reg[29:24] ^ w_ecc_parity;
    for (int idx = 0; idx < LPARA_HDR_DATA_W; idx++) begin
      w_ecc_hit[idx] = (w_syndrome == f_ecc_code(5'(idx)));
    end
    w_ecc_parity_err = (w_syndrome != 6'd0) && ((w_syndrome & (w_syndrome - 6'd1)) == 6'd0);
    w_ecc_corrected  = reg_word0_reg[23:0] ^ w_ecc_hit;
    w_ecc_1bit_raw   = (|w_ecc_hit) | w_ecc_parity_err;
    w_ecc_2bit_raw   = (w_syndrome != 6'd0) & ~w_ecc_1bit_raw;
  end

  // ECC output stage (one register stage) and Delay Register, or direct wiring when the ECC is
  // combinational. Valid bits are cleared by a flush so no stale header or word survives it.
  generate
  if (PARA_ECC_LATENCY == 32'd1) begin : gen_ecc_reg
    logic                        reg_ecc_valid;
    logic [LPARA_HDR_DATA_W-1:0] reg_ecc_data;
    logic                        reg_ecc_1bit;
    logic                        reg_ecc_2bit;
    logic                        reg_dly_valid;
    logic                        reg_dly_marker;
    logic                        reg_dly_eob;
    logic [LPARA_BYTE_NUM-1:0]   reg_dly_byte_valid;
    logic [LPARA_WORD_WIDTH-1:0] reg_dly_data;

    always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_ecc_stage
      if (!i_resetn) begin
        reg_ecc_valid      <= 1'b0;
        reg_ecc_data       <= '0;
        reg_ecc_1bit       <= 1'b0;
        reg_ecc_2bit       <= 1'b0;
        reg_dly_valid      <= 1'b0;
        reg_dly_marker     <= 1'b0;
        reg_dly_eob        <= 1'b0;
        reg_dly_byte_valid <= '0;
        reg_dly_data       <= '0;
      end else begin
        reg_ecc_valid      <= reg_hdr_load & ~w_flush;
        reg_ecc_data       <= w_ecc_corrected;
        reg_ecc_1bit       <= w_ecc_1bit_raw;
        reg_ecc_2bit       <= w_ecc_2bit_raw;
        reg_dly_valid      <= w_input_valid;
        reg_dly_marker     <= w_abort_marker;
        reg_dly_eob        <= reg_input_eob;
        reg_dly_byte_valid <= reg_input_byte_valid;
        reg_dly_data       <= reg_input_data;
      end
    end

    always_comb begin : p_com_ecc_out
      w_hdr_valid           = reg_ecc_valid;
      w_hdr_data            = reg_ecc_data;
      w_err_ecc_1bit        = reg_ecc_1bit;
      w_err_ecc_2bit        = reg_ecc_2bit;
      w_hdr_pyld_valid      = reg_dly_valid;
      w_hdr_pyld_marker     = reg_dly_marker;
      w_hdr_pyld_eob        = reg_dly_eob;
      w_hdr_pyld_byte_valid = reg_dly_byte_valid;
      w_hdr_pyld_data       = reg_dly_data;
    end
  end else begin : gen_ecc_comb
    always_comb begin : p_com_ecc_out
      w_hdr_valid           = reg_hdr_load;
      w_hdr_data            = w_ecc_corrected;
      w_err_ecc_1bit        = w_ecc_1bit_raw;
      w_err_ecc_2bit        = w_ecc_2bit_raw;
      w_hdr_pyld_valid      = w_input_valid;
      w_hdr_pyld_marker     = w_abort_marker;
      w_hdr_pyld_eob        = reg_input_eob;
      w_hdr_pyld_byte_valid = reg_input_byte_valid;
      w_hdr_pyld_data       = reg_input_data;
    end
  end
  endgenerate

  // Header fields split by wiring; hdr_ok qualifies every header-driven action (passed ECC,
  // no flush). Only Virtual Channels 0 and 1 are supported.
  always_comb begin : p_com_hdr_split
    w_hdr_vc     = w_hdr_data[7:6];
    w_hdr_dt     = w_hdr_data[5:0];
    w_hdr_field2 = w_hdr_data[23:8];
    w_hdr_wc     = w_hdr_field2;
    w_is_short   = ~(w_hdr_dt[5] | w_hdr_dt[4]);
    w_hdr_ok     = w_hdr_valid & ~w_err_ecc_2bit & ~w_flush;
    w_vc_ok      = ~w_hdr_vc[1];
    w_long_ok    = w_hdr_ok & ~w_is_short;
  end

  // ------------------------------------------------------------------------------------------
  // DT Decoder
  // ------------------------------------------------------------------------------------------
  // Long packet classification, first match wins: metadata_dt with capture enabled, metadata_dt
  // with capture disabled (dropped silently), Null/Blanking, RAW10/RAW12, anything else.
  // dt_conflict watches the programming independently of traffic.
  always_comb begin : p_com_dt_decoder
    w_dt_reserved     = (w_hdr_dt == LPARA_DT_NULL) | (w_hdr_dt == LPARA_DT_BLANK) |
                        (w_hdr_dt == LPARA_DT_RAW10) | (w_hdr_dt == LPARA_DT_RAW12);
    w_dt_meta_hit     = (w_hdr_dt == i_metadata_dt);
    w_pkt_is_embedded = w_dt_meta_hit & i_metadata_capture_en;
    w_dt_meta_drop    = w_dt_meta_hit & ~i_metadata_capture_en & ~w_dt_reserved;
    w_dt_null         = ~w_pkt_is_embedded &
                        ((w_hdr_dt == LPARA_DT_NULL) | (w_hdr_dt == LPARA_DT_BLANK));
    w_pkt_is_image    = ~w_pkt_is_embedded &
                        ((w_hdr_dt == LPARA_DT_RAW10) | (w_hdr_dt == LPARA_DT_RAW12));
    w_bp_dt_sel       = (w_hdr_dt == LPARA_DT_RAW10);
    w_dt_unsupported  = ~w_pkt_is_embedded & ~w_dt_meta_drop & ~w_dt_null & ~w_pkt_is_image;
    w_dt_conflict     = i_metadata_capture_en &
                        ((i_metadata_dt == LPARA_DT_NULL) | (i_metadata_dt == LPARA_DT_BLANK) |
                         (i_metadata_dt == LPARA_DT_RAW10) | (i_metadata_dt == LPARA_DT_RAW12));
  end

  // ------------------------------------------------------------------------------------------
  // Short Packet Handler
  // ------------------------------------------------------------------------------------------
  // Frame Detector and generic short packet selection. Line Start, Line End and the reserved
  // synchronisation codes are ignored; short packets of other Virtual Channels are dropped.
  always_comb begin : p_com_short_pkt
    w_fc_fs_raw = w_hdr_ok & w_is_short & (w_hdr_dt == LPARA_DT_FS);
    w_fc_fe_raw = w_hdr_ok & w_is_short & (w_hdr_dt == LPARA_DT_FE);
    w_fc_fs     = w_fc_fs_raw & w_vc_ok;
    w_fc_fe     = w_fc_fe_raw & w_vc_ok;
    w_gsp_wr    = w_hdr_ok & w_is_short & (w_hdr_dt[5:3] == LPARA_DT_GSP_HI) & w_vc_ok;
  end

  // Generic Short Packet FIFO control. A write on a full FIFO is accepted when gsp_pop frees an
  // entry in the same cycle, otherwise it is dropped and reported; a pop on an empty FIFO is
  // ignored. The head entry reads as 0 while the FIFO is empty.
  always_comb begin : p_com_gsp_ctrl
    w_gsp_not_empty    = (reg_gsp_cnt != '0);
    w_gsp_full         = (reg_gsp_cnt == LPARA_DEPTH_CNT);
    w_gsp_pop_ok       = i_gsp_pop & w_gsp_not_empty;
    w_gsp_push         = w_gsp_wr & (~w_gsp_full | w_gsp_pop_ok);
    w_sp_fifo_overflow = w_gsp_wr & w_gsp_full & ~w_gsp_pop_ok;
    w_gsp_data         = w_gsp_not_empty ? reg_gsp_mem[reg_gsp_rd_ptr] : '0;
  end

  // FIFO pointers and occupancy; cleared by soft_reset only, core_enable keeps the contents.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_gsp_ptr
    if (!i_resetn) begin
      reg_gsp_wr_ptr <= '0;
      reg_gsp_rd_ptr <= '0;
      reg_gsp_cnt    <= '0;
    end else if (i_soft_reset) begin
      reg_gsp_wr_ptr <= '0;
      reg_gsp_rd_ptr <= '0;
      reg_gsp_cnt    <= '0;
    end else begin
      if (w_gsp_push) begin
        reg_gsp_wr_ptr <= reg_gsp_wr_ptr + LPARA_PTR_W'(1);
      end
      if (w_gsp_pop_ok) begin
        reg_gsp_rd_ptr <= reg_gsp_rd_ptr + LPARA_PTR_W'(1);
      end
      reg_gsp_cnt <= reg_gsp_cnt + LPARA_CNT_W'(w_gsp_push) - LPARA_CNT_W'(w_gsp_pop_ok);
    end
  end

  // FIFO storage: entry = {data field, VC, Data Type}, the layout of GENERIC_SHORT_PACKET. Not
  // reset; an entry is read only after it has been written (occupancy counter).
  always_ff @(posedge i_sys_clk) begin : p_ff_gsp_mem
    if (w_gsp_push) begin
      reg_gsp_mem[reg_gsp_wr_ptr] <= w_hdr_data;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Payload Path
  // ------------------------------------------------------------------------------------------
  // The word counter arms on a long header that passed ECC, unless its burst already ended on
  // the header word. When a payload word meets its header in the same cycle, the fresh header
  // values are used directly.
  always_comb begin : p_com_payload_ctrl
    w_load           = w_long_ok & ~reg_eob_latched;
    w_armed          = w_load | reg_wc_armed;
    w_rem_cur        = w_load ? w_hdr_wc : reg_remaining_bytes;
    w_footer_cnt_cur = w_load ? 2'd0 : reg_footer_cnt;
    w_crc_cur        = w_load ? LPARA_CRC_INIT : reg_crc;
    w_word_active    = w_armed & w_hdr_pyld_valid;
    w_word_bytes     = f_byte_cnt(w_hdr_pyld_byte_valid);
  end

  // Byte classification of an armed word: the first remaining payload bytes, then up to two
  // footer bytes (CRC LS byte first, the pair may straddle two words), then ignored bytes.
  always_comb begin : p_com_payload_bytes
    w_pay_n = 3'd0;
    if (w_word_active) begin
      w_pay_n = (w_rem_cur < {13'd0, w_word_bytes}) ? w_rem_cur[2:0] : w_word_bytes;
    end
    w_payload_byte_en = f_byte_mask(w_pay_n);
    w_payload_valid   = (w_pay_n != 3'd0);
    w_payload_last    = w_payload_valid & (w_rem_cur == {13'd0, w_pay_n});
    w_rem_nxt         = w_rem_cur - {13'd0, w_pay_n};
    w_crc_avail       = w_word_bytes - w_pay_n;
    w_crc_need        = 2'd2 - w_footer_cnt_cur;
    w_crc_in          = 2'd0;
    if (w_word_active) begin
      w_crc_in = (w_crc_avail < {1'b0, w_crc_need}) ? w_crc_avail[1:0] : w_crc_need;
    end
    w_footer_cnt_nxt  = w_footer_cnt_cur + w_crc_in;
    if (w_footer_cnt_cur == 2'd0) begin
      w_crc_ls = f_byte_sel(w_hdr_pyld_data, w_pay_n[1:0]);
      w_crc_ms = f_byte_sel(w_hdr_pyld_data, w_pay_n[1:0] + 2'd1);
    end else begin
      w_crc_ls = reg_crc_lo;
      w_crc_ms = f_byte_sel(w_hdr_pyld_data, 2'd0);
    end
    w_payload_done    = w_word_active & (w_footer_cnt_nxt == 2'd2);
  end

  // CRC-16 engine: four byte stages in one cycle over the payload bytes only; the footer is
  // compared once both of its bytes are present. A burst that ends before its footer is
  // complete is a Word Count error instead.
  always_comb begin : p_com_crc
    w_crc_nxt = w_crc_cur;
    for (int idx = 0; idx < LPARA_BYTE_NUM; idx++) begin
      if (w_payload_byte_en[idx]) begin
        w_crc_nxt = f_crc16_byte(w_crc_nxt, w_hdr_pyld_data[8*idx +: 8]);
      end
    end
    w_crc_mismatch = w_payload_done & ({w_crc_ms, w_crc_ls} != w_crc_nxt);
    w_pkt_end      = w_word_active & (w_payload_done | w_hdr_pyld_eob);
    w_err_wc_pyld  = w_word_active & w_hdr_pyld_eob & ~w_payload_done;
  end

  // Payload state: disarmed at the end of the packet (footer complete or end of burst) and by
  // a flush.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_payload
    if (!i_resetn) begin
      reg_wc_armed        <= 1'b0;
      reg_remaining_bytes <= '0;
      reg_footer_cnt      <= 2'd0;
      reg_crc             <= LPARA_CRC_INIT;
      reg_crc_lo          <= '0;
    end else if (w_flush) begin
      reg_wc_armed        <= 1'b0;
      reg_remaining_bytes <= '0;
      reg_footer_cnt      <= 2'd0;
      reg_crc             <= LPARA_CRC_INIT;
      reg_crc_lo          <= '0;
    end else if (w_load | w_word_active) begin
      reg_wc_armed        <= ~w_pkt_end;
      reg_remaining_bytes <= w_word_active ? w_rem_nxt : w_rem_cur;
      reg_footer_cnt      <= w_footer_cnt_nxt;
      reg_crc             <= w_crc_nxt;
      if ((w_footer_cnt_cur == 2'd0) && (w_crc_in == 2'd1)) begin
        reg_crc_lo <= w_crc_ls;
      end
    end
  end

  // Attributes of the long packet whose payload is flowing, captured at its header. bp_sof is
  // granted to the first image packet of a frame; bp_sol is pending until the first beat.
  always_comb begin : p_com_pkt_attr
    w_cur_vc       = w_load ? w_hdr_vc : reg_pkt_vc;
    w_cur_vc_ok    = w_load ? w_vc_ok : reg_pkt_vc_ok;
    w_cur_image    = w_load ? w_pkt_is_image : reg_pkt_is_image;
    w_cur_emb      = w_load ? w_pkt_is_embedded : reg_pkt_is_embedded;
    w_cur_sof      = w_load ? (w_pkt_is_image & ~reg_first_image_seen[w_hdr_vc[0]]) : reg_pkt_sof;
    w_sol_pend_cur = w_load | reg_sol_pend;
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_pkt_attr
    if (!i_resetn) begin
      reg_pkt_vc          <= 2'd0;
      reg_pkt_vc_ok       <= 1'b0;
      reg_pkt_is_image    <= 1'b0;
      reg_pkt_is_embedded <= 1'b0;
      reg_pkt_sof         <= 1'b0;
      reg_sol_pend        <= 1'b0;
      reg_pkt_long        <= 1'b0;
    end else if (w_flush) begin
      reg_pkt_vc_ok       <= 1'b0;
      reg_pkt_is_image    <= 1'b0;
      reg_pkt_is_embedded <= 1'b0;
      reg_pkt_sof         <= 1'b0;
      reg_sol_pend        <= 1'b0;
      reg_pkt_long        <= 1'b0;
    end else begin
      if (w_load) begin
        reg_pkt_vc          <= w_cur_vc;
        reg_pkt_vc_ok       <= w_cur_vc_ok;
        reg_pkt_is_image    <= w_cur_image;
        reg_pkt_is_embedded <= w_cur_emb;
        reg_pkt_sof         <= w_cur_sof;
      end
      reg_sol_pend <= w_sol_pend_cur & ~w_payload_valid;
      if (w_hdr_valid) begin
        reg_pkt_long <= w_long_ok;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Frame Supervisor
  // ------------------------------------------------------------------------------------------
  // Frame identification at the header of a supported channel: Frame Start on an open frame,
  // Frame End on a closed frame, or an image/embedded long packet outside a frame. A Frame End
  // closing a frame that saw a CRC, Word Count or lane-abort error raises the frame-level error.
  always_comb begin : p_com_frame_events
    w_hdr_fa             = reg_frame_active[w_hdr_vc[0]];
    w_hdr_err_pend       = reg_pkt_err_in_frame[w_hdr_vc[0]];
    w_err_frame_sync     = (w_fc_fs & w_hdr_fa) | (w_fc_fe & ~w_hdr_fa) |
                           (w_long_ok & w_vc_ok & (w_pkt_is_image | w_pkt_is_embedded) & ~w_hdr_fa);
    w_err_frame_level    = w_fc_fe & w_hdr_fa & w_hdr_err_pend;
    w_err_unsupported_dt = w_long_ok & w_vc_ok & w_dt_unsupported;
    w_err_wc_hdr         = w_long_ok & w_vc_ok & reg_eob_latched;
  end

  // Per-VC context update. Frame Start reloads the context, also over an open frame; the
  // abort marker marks every open frame; a flush (soft_reset or core_enable = 0) clears all.
  always_comb begin : p_com_frame_ctx
    w_frame_active_nxt     = reg_frame_active;
    w_first_image_seen_nxt = reg_first_image_seen;
    w_pkt_err_in_frame_nxt = reg_pkt_err_in_frame;
    w_pkt_err_set          = 2'b00;
    for (int idx = 0; idx < 2; idx++) begin
      w_pkt_err_set[idx] = ((w_err_crc_det | (w_err_wc_pyld & ~w_flush)) & w_cur_vc_ok &
                            (w_cur_vc[0] == idx[0])) |
                           (w_err_wc_hdr & (w_hdr_vc[0] == idx[0])) |
                           (w_hdr_pyld_marker & reg_frame_active[idx]);
      if (w_fc_fs && (w_hdr_vc[0] == idx[0])) begin
        w_frame_active_nxt[idx]     = 1'b1;
        w_first_image_seen_nxt[idx] = 1'b0;
        w_pkt_err_in_frame_nxt[idx] = 1'b0;
      end else begin
        if (w_fc_fe && (w_hdr_vc[0] == idx[0])) begin
          w_frame_active_nxt[idx] = 1'b0;
        end
        if (w_long_ok && w_vc_ok && w_pkt_is_image && (w_hdr_vc[0] == idx[0])) begin
          w_first_image_seen_nxt[idx] = 1'b1;
        end
        if (w_pkt_err_set[idx]) begin
          w_pkt_err_in_frame_nxt[idx] = 1'b1;
        end
      end
    end
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_frame_ctx
    if (!i_resetn) begin
      reg_frame_active     <= 2'b00;
      reg_first_image_seen <= 2'b00;
      reg_pkt_err_in_frame <= 2'b00;
    end else if (w_flush) begin
      reg_frame_active     <= 2'b00;
      reg_first_image_seen <= 2'b00;
      reg_pkt_err_in_frame <= 2'b00;
    end else begin
      reg_frame_active     <= w_frame_active_nxt;
      reg_first_image_seen <= w_first_image_seen_nxt;
      reg_pkt_err_in_frame <= w_pkt_err_in_frame_nxt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output Routing
  // ------------------------------------------------------------------------------------------
  // A payload beat is forwarded only for a supported channel with an open frame and its
  // force_discard bit clear; image and embedded packets share the payload bus.
  always_comb begin : p_com_output_routing
    w_discard_gate = i_force_discard[w_cur_vc[0]];
    w_fwd          = w_payload_valid & w_cur_vc_ok & reg_frame_active[w_cur_vc[0]] &
                     ~w_discard_gate & ~w_flush;
    w_bp_valid     = w_fwd & w_cur_image;
    w_bp_sol       = w_bp_valid & w_sol_pend_cur;
    w_bp_eol       = w_bp_valid & w_payload_last;
    w_bp_sof       = w_bp_sol & w_cur_sof;
    w_emb_valid    = w_fwd & w_cur_emb;
    w_emb_sol      = w_emb_valid & w_sol_pend_cur;
    w_emb_eol      = w_emb_valid & w_payload_last;
  end

  // Error events and err_vc. CRC and Word Count errors are not reported for unsupported
  // channels. Events of one cycle always belong to one packet, so err_vc has a single source.
  always_comb begin : p_com_err_events
    w_err_ecc_1bit_ev = w_hdr_valid & w_err_ecc_1bit & ~w_flush;
    w_err_ecc_2bit_ev = w_hdr_valid & w_err_ecc_2bit & ~w_flush;
    w_err_crc_det     = w_crc_mismatch & w_cur_vc_ok & ~w_flush;
    w_err_wc          = (w_err_wc_pyld & w_cur_vc_ok & ~w_flush) | w_err_wc_hdr;
    w_err_lane_abort  = w_hdr_pyld_marker & ~w_flush;
    w_hdr_err_any     = w_err_frame_sync | w_err_frame_level | w_err_unsupported_dt |
                        w_err_ecc_1bit_ev | w_err_wc_hdr;
    if (w_hdr_err_any) begin
      w_err_vc = w_hdr_vc;
    end else if (w_err_wc) begin
      w_err_vc = w_cur_vc;
    end else if (reg_err_crc_d & ~w_flush) begin
      w_err_vc = reg_err_crc_vc_d;
    end else begin
      w_err_vc = 2'd0;
    end
    w_packet_done_pulse = (reg_fsm_pkt_current_state == ENUM_ST_DONE) & reg_pkt_long & ~w_flush;
    w_null_set          = w_long_ok & w_vc_ok & w_dt_null;
    w_vc_unsup_set      = w_hdr_ok & ~w_vc_ok;
  end

  // Output registers: every beat and event output is registered once. Payload buses read 0
  // outside a valid beat; bp_vc_tag/bp_dt_sel and emb_vc hold the values of the last image or
  // embedded header. err_crc takes one extra stage so it follows bp_eol of its packet.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_outputs
    if (!i_resetn) begin
      reg_bp_data            <= '0;
      reg_bp_byte_en         <= '0;
      reg_bp_valid           <= 1'b0;
      reg_bp_sof             <= 1'b0;
      reg_bp_sol             <= 1'b0;
      reg_bp_eol             <= 1'b0;
      reg_bp_vc_tag          <= 2'd0;
      reg_bp_dt_sel          <= 1'b0;
      reg_emb_data           <= '0;
      reg_emb_byte_en        <= '0;
      reg_emb_valid          <= 1'b0;
      reg_emb_sol            <= 1'b0;
      reg_emb_eol            <= 1'b0;
      reg_emb_vc             <= 2'd0;
      reg_fc_fs              <= 1'b0;
      reg_fc_fe              <= 1'b0;
      reg_fc_frame_num       <= '0;
      reg_fc_vc              <= 2'd0;
      reg_sp_fifo_overflow   <= 1'b0;
      reg_packet_done_pulse  <= 1'b0;
      reg_err_ecc_1bit       <= 1'b0;
      reg_err_ecc_2bit       <= 1'b0;
      reg_err_crc_d          <= 1'b0;
      reg_err_crc_vc_d       <= 2'd0;
      reg_err_crc            <= 1'b0;
      reg_err_wc             <= 1'b0;
      reg_err_unsupported_dt <= 1'b0;
      reg_err_vc             <= 2'd0;
      reg_err_lane_abort     <= 1'b0;
      reg_err_frame_sync     <= 1'b0;
      reg_err_frame_level    <= 1'b0;
    end else begin
      reg_bp_data            <= w_bp_valid ? w_hdr_pyld_data : '0;
      reg_bp_byte_en         <= w_bp_valid ? w_payload_byte_en : '0;
      reg_bp_valid           <= w_bp_valid;
      reg_bp_sof             <= w_bp_sof;
      reg_bp_sol             <= w_bp_sol;
      reg_bp_eol             <= w_bp_eol;
      if (w_long_ok & w_vc_ok & w_pkt_is_image) begin
        reg_bp_vc_tag <= w_hdr_vc;
        reg_bp_dt_sel <= w_bp_dt_sel;
      end
      reg_emb_data           <= w_emb_valid ? w_hdr_pyld_data : '0;
      reg_emb_byte_en        <= w_emb_valid ? w_payload_byte_en : '0;
      reg_emb_valid          <= w_emb_valid;
      reg_emb_sol            <= w_emb_sol;
      reg_emb_eol            <= w_emb_eol;
      if (w_long_ok & w_vc_ok & w_pkt_is_embedded) begin
        reg_emb_vc <= w_hdr_vc;
      end
      reg_fc_fs              <= w_fc_fs;
      reg_fc_fe              <= w_fc_fe;
      reg_fc_frame_num       <= (w_fc_fs | w_fc_fe) ? w_hdr_field2 : '0;
      reg_fc_vc              <= (w_fc_fs | w_fc_fe) ? w_hdr_vc : 2'd0;
      reg_sp_fifo_overflow   <= w_sp_fifo_overflow;
      reg_packet_done_pulse  <= w_packet_done_pulse;
      reg_err_ecc_1bit       <= w_err_ecc_1bit_ev;
      reg_err_ecc_2bit       <= w_err_ecc_2bit_ev;
      reg_err_crc_d          <= w_err_crc_det;
      reg_err_crc_vc_d       <= w_cur_vc;
      reg_err_crc            <= reg_err_crc_d & ~w_flush;
      reg_err_wc             <= w_err_wc;
      reg_err_unsupported_dt <= w_err_unsupported_dt;
      reg_err_vc             <= w_err_vc;
      reg_err_lane_abort     <= w_err_lane_abort;
      reg_err_frame_sync     <= w_err_frame_sync;
      reg_err_frame_level    <= w_err_frame_level;
    end
  end

  // Sticky status flags, cleared by soft_reset only. dt_conflict sets again after soft_reset
  // while the conflicting programming remains.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_sticky
    if (!i_resetn) begin
      reg_null_blank_seen     <= 1'b0;
      reg_vc_unsupported_seen <= 1'b0;
      reg_dt_conflict         <= 1'b0;
    end else if (i_soft_reset) begin
      reg_null_blank_seen     <= 1'b0;
      reg_vc_unsupported_seen <= 1'b0;
      reg_dt_conflict         <= 1'b0;
    end else begin
      reg_null_blank_seen     <= reg_null_blank_seen | w_null_set;
      reg_vc_unsupported_seen <= reg_vc_unsupported_seen | w_vc_unsup_set;
      reg_dt_conflict         <= reg_dt_conflict | w_dt_conflict;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Packet FSM
  // ------------------------------------------------------------------------------------------
  // One packet per burst. The uncorrectable header is tested before the packet type. The end
  // of the burst is latched on the header word and while the header resolves, so a burst that
  // ends during ST_HEADER is retired without waiting in ST_PAYLOAD or ST_DISCARD. A flush
  // returns to ST_IDLE.
  always_comb begin : p_com_pkt_fsm
    w_pyld_eob          = w_hdr_pyld_valid & w_hdr_pyld_eob;
    w_fsm_pkt_nxt_state = reg_fsm_pkt_current_state;
    w_eob_latched_nxt   = reg_eob_latched;
    case (reg_fsm_pkt_current_state)
      ENUM_ST_IDLE: begin
        if (w_input_valid && reg_input_sob) begin
          w_fsm_pkt_nxt_state = ENUM_ST_HEADER;
          w_eob_latched_nxt   = reg_input_eob;
        end
      end
      ENUM_ST_HEADER: begin
        w_eob_latched_nxt = reg_eob_latched | w_pyld_eob;
        if (w_hdr_valid) begin
          if (w_err_ecc_2bit) begin
            w_fsm_pkt_nxt_state = ENUM_ST_DISCARD;
          end else if (w_is_short || reg_eob_latched || w_pkt_end) begin
            w_fsm_pkt_nxt_state = ENUM_ST_DONE;
          end else begin
            w_fsm_pkt_nxt_state = ENUM_ST_PAYLOAD;
          end
        end
      end
      ENUM_ST_PAYLOAD: begin
        if (w_payload_done || w_pyld_eob) begin
          w_fsm_pkt_nxt_state = ENUM_ST_DONE;
        end
      end
      ENUM_ST_DISCARD: begin
        if (reg_eob_latched || w_pyld_eob) begin
          w_fsm_pkt_nxt_state = ENUM_ST_IDLE;
          w_eob_latched_nxt   = 1'b0;
        end
      end
      ENUM_ST_DONE: begin
        w_fsm_pkt_nxt_state = ENUM_ST_IDLE;
        w_eob_latched_nxt   = 1'b0;
      end
      default: begin
        w_fsm_pkt_nxt_state = ENUM_ST_IDLE;
        w_eob_latched_nxt   = 1'b0;
      end
    endcase
    if (w_flush) begin
      w_fsm_pkt_nxt_state = ENUM_ST_IDLE;
      w_eob_latched_nxt   = 1'b0;
    end
  end

  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_pkt_fsm
    if (!i_resetn) begin
      reg_fsm_pkt_current_state <= ENUM_ST_IDLE;
      reg_eob_latched           <= 1'b0;
    end else begin
      reg_fsm_pkt_current_state <= w_fsm_pkt_nxt_state;
      reg_eob_latched           <= w_eob_latched_nxt;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Output connections
  // ------------------------------------------------------------------------------------------
  assign o_rd_en               = w_rd_en;
  assign o_bp_data             = reg_bp_data;
  assign o_bp_byte_en          = reg_bp_byte_en;
  assign o_bp_valid            = reg_bp_valid;
  assign o_bp_sof              = reg_bp_sof;
  assign o_bp_sol              = reg_bp_sol;
  assign o_bp_eol              = reg_bp_eol;
  assign o_bp_vc_tag           = reg_bp_vc_tag;
  assign o_bp_dt_sel           = reg_bp_dt_sel;
  assign o_emb_data            = reg_emb_data;
  assign o_emb_byte_en         = reg_emb_byte_en;
  assign o_emb_valid           = reg_emb_valid;
  assign o_emb_sol             = reg_emb_sol;
  assign o_emb_eol             = reg_emb_eol;
  assign o_emb_vc              = reg_emb_vc;
  assign o_fc_fs               = reg_fc_fs;
  assign o_fc_fe               = reg_fc_fe;
  assign o_fc_frame_num        = reg_fc_frame_num;
  assign o_fc_vc               = reg_fc_vc;
  assign o_gsp_data            = w_gsp_data;
  assign o_sp_fifo_not_empty   = w_gsp_not_empty;
  assign o_sp_fifo_full        = w_gsp_full;
  assign o_sp_fifo_overflow    = reg_sp_fifo_overflow;
  assign o_null_blank_seen     = reg_null_blank_seen;
  assign o_packet_done_pulse   = reg_packet_done_pulse;
  assign o_err_ecc_1bit        = reg_err_ecc_1bit;
  assign o_err_ecc_2bit        = reg_err_ecc_2bit;
  assign o_err_crc             = reg_err_crc;
  assign o_err_wc              = reg_err_wc;
  assign o_err_unsupported_dt  = reg_err_unsupported_dt;
  assign o_err_vc              = reg_err_vc;
  assign o_err_lane_abort      = reg_err_lane_abort;
  assign o_err_frame_sync      = reg_err_frame_sync;
  assign o_err_frame_level     = reg_err_frame_level;
  assign o_vc_unsupported_seen = reg_vc_unsupported_seen;
  assign o_dt_conflict         = reg_dt_conflict;

endmodule
`default_nettype wire
