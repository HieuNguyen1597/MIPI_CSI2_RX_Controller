// Module      : m_fpt_dma
// Block       : B13 - DMA
// Spec        : FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4
// Date        : 2026-11-09
// Description : Two-buffer capture DMA writing RAW lines, records and embedded data over AXI4
`timescale 1ns/1ps
`default_nettype none
module m_fpt_dma #(
  // Parameter Description: AXI write data width; the internal FIFOs stay 64 bits wide.
  // Parameter values: 32 or 64; default 64.
  // Parameter unit: bit
  parameter logic [31:0] PARA_AXI_DATA_WIDTH = 32'd64,
  // Parameter Description: width of m_axi4_awaddr.
  // Parameter values: 32 only in this release; default 32.
  // Parameter unit: bit
  parameter logic [31:0] PARA_AXI_ADDR_WIDTH = 32'd32,
  // Parameter Description: width of m_axi4_awid and m_axi4_bid; bits 1:0 are {stream, buffer}.
  // Parameter values: 2 to 8; default 2.
  // Parameter unit: bit
  parameter logic [31:0] PARA_AXI_ID_WIDTH = 32'd2,
  // Parameter Description: largest burst issued; DMA_ATTR.max_burst_len is clamped to it.
  // Parameter values: 2 to 256 and at least PARA_META_MAX_BURST_BEATS; default 256.
  // Parameter unit: count (AXI beats)
  parameter logic [31:0] PARA_MAX_BURST_BEATS = 32'd256,
  // Parameter Description: writes in flight (all identifiers); depth of the completion FIFO.
  // Parameter values: power of two, 2 to 64; default 8.
  // Parameter unit: count
  parameter logic [31:0] PARA_MAX_OUTSTANDING = 32'd8,
  // Parameter Description: entries of the RAW data FIFO.
  // Parameter values: power of two, at least PARA_MAX_BURST_BEATS; default 512.
  // Parameter unit: count (64-bit beats)
  parameter logic [31:0] PARA_RAW_FIFO_DEPTH = 32'd512,
  // Parameter Description: entries of the metadata data FIFO.
  // Parameter values: power of two, at least 2 x PARA_META_MAX_BURST_BEATS; default 64.
  // Parameter unit: count (64-bit beats)
  parameter logic [31:0] PARA_META_DATA_FIFO_DEPTH = 32'd64,
  // Parameter Description: burst commands per stream (RAW command FIFO, metadata command FIFO).
  // Parameter values: power of two, at least 2; default 16.
  // Parameter unit: count
  parameter logic [31:0] PARA_CMD_FIFO_DEPTH = 32'd16,
  // Parameter Description: 1 = mm_aclk asynchronous to sys_clk; 0 = same clock, no synchroniser.
  // Parameter values: 0 or 1; default 1.
  // Parameter unit: none
  parameter logic [31:0] PARA_ASYNC_MM_CLK = 32'd1,
  // Parameter Description: synchroniser flops per pointer crossing.
  // Parameter values: 2 to 4; default 2.
  // Parameter unit: count
  parameter logic [31:0] PARA_SYNC_STAGES = 32'd2,
  // Parameter Description: largest metadata burst (smaller of this and max_burst_len + 1).
  // Parameter values: 1 to PARA_MAX_BURST_BEATS; default 32.
  // Parameter unit: count (AXI beats)
  parameter logic [31:0] PARA_META_MAX_BURST_BEATS = 32'd32
) (
  input  wire                             i_sys_clk,
  input  wire                             i_mm_aclk,
  input  wire                             i_resetn,
  input  wire                             i_mm_resetn,
  input  wire                             i_soft_reset,
  input  wire  [63:0]                     i_fbd_data,
  input  wire  [7:0]                      i_fbd_byte_en,
  input  wire                             i_fbd_valid,
  input  wire                             i_fbd_sof,
  input  wire                             i_fbd_sol,
  input  wire                             i_fbd_eol,
  input  wire  [1:0]                      i_fbd_vc_tag,
  input  wire                             i_fbd_err,
  output logic                            o_fbd_ready,
  input  wire  [1:0]                      i_capture_start,
  input  wire  [1:0]                      i_capture_end,
  input  wire  [1:0]                      i_capture_dropped,
  input  wire                             i_ev_fs,
  input  wire  [1:0]                      i_ev_vc,
  input  wire                             i_lead_vc,
  input  wire  [1:0]                      i_ctx_slot,
  input  wire  [1:0]                      i_ctx_orphan,
  input  wire  [13:0]                     i_frame_id_buf0,
  input  wire  [13:0]                     i_frame_id_buf1,
  input  wire  [31:0]                     i_meta_data,
  input  wire  [3:0]                      i_meta_byte_en,
  input  wire                             i_meta_valid,
  input  wire                             i_meta_last,
  input  wire                             i_meta_kind,
  input  wire  [1:0]                      i_meta_vc,
  input  wire                             i_meta_sof,
  output logic                            o_meta_ready,
  output logic                            o_dma_buf_claim,
  output logic                            o_dma_buf_index,
  input  wire                             i_active_sel,
  input  wire  [15:0]                     i_tpg_height_shadow,
  input  wire                             i_dma_abort_req,
  output logic                            o_dma_drop,
  output logic                            o_dma_buf_done_ok,
  output logic                            o_dma_buf_done_err,
  output logic                            o_dma_axi_write_error,
  output logic                            o_dma_buffer_overflow,
  input  wire                             i_dma_enable,
  input  wire                             i_dma_start,
  input  wire                             i_dma_abort,
  input  wire                             i_err_frame_policy,
  input  wire  [31:0]                     i_dma_buffer0_addr,
  input  wire  [31:0]                     i_dma_buffer1_addr,
  input  wire  [31:0]                     i_dma_buffer_size,
  input  wire  [31:0]                     i_meta_area_size,
  input  wire  [15:0]                     i_bytes_per_line,
  input  wire  [7:0]                      i_line_padding,
  input  wire  [3:0]                      i_awqos,
  input  wire  [7:0]                      i_max_burst_len,
  input  wire  [2:0]                      i_awprot,
  input  wire  [1:0]                      i_release_buf,
  output logic                            o_dma_busy,
  output logic                            o_active_buffer,
  output logic [1:0]                      o_buf0_state,
  output logic [1:0]                      o_buf1_state,
  output logic [15:0]                     o_buf0_last_valid_line,
  output logic [15:0]                     o_buf1_last_valid_line,
  output logic [7:0]                      o_axi_fault_awid,
  output logic [1:0]                      o_axi_fault_resp,
  output logic [31:0]                     o_axi_fault_awaddr,
  output logic [15:0]                     o_bresp_slverr_count,
  output logic [15:0]                     o_bresp_decerr_count,
  // AMBA AXI4 (IHI0022H), manager, write channels only. Present: AW id, addr, len, size, burst,
  // lock, cache, prot, qos, valid/ready; W data, strb, last, valid/ready; B id, resp, valid/ready.
  // Absent: AWREGION, AWUSER, WUSER, BUSER and the read channels.
  output logic [PARA_AXI_ID_WIDTH-1:0]    o_m_axi4_awid,
  output logic [PARA_AXI_ADDR_WIDTH-1:0]  o_m_axi4_awaddr,
  output logic [7:0]                      o_m_axi4_awlen,
  output logic [2:0]                      o_m_axi4_awsize,
  output logic [1:0]                      o_m_axi4_awburst,
  output logic                            o_m_axi4_awlock,
  output logic [3:0]                      o_m_axi4_awcache,
  output logic [2:0]                      o_m_axi4_awprot,
  output logic [3:0]                      o_m_axi4_awqos,
  output logic                            o_m_axi4_awvalid,
  input  wire                             i_m_axi4_awready,
  output logic [PARA_AXI_DATA_WIDTH-1:0]  o_m_axi4_wdata,
  output logic [PARA_AXI_DATA_WIDTH/8-1:0] o_m_axi4_wstrb,
  output logic                            o_m_axi4_wlast,
  output logic                            o_m_axi4_wvalid,
  input  wire                             i_m_axi4_wready,
  input  wire  [PARA_AXI_ID_WIDTH-1:0]    i_m_axi4_bid,
  input  wire  [1:0]                      i_m_axi4_bresp,
  input  wire                             i_m_axi4_bvalid,
  output logic                            o_m_axi4_bready
);

  // ------------------------------------------------------------------------------------------
  // Constants, types and parameter checks
  // ------------------------------------------------------------------------------------------
  localparam logic [31:0] LPARA_FIFO_DATA_W  = 32'd64;
  localparam logic [31:0] LPARA_FIFO_STRB_W  = 32'd8;
  localparam logic [31:0] LPARA_BEAT_SHIFT   = 32'd3;
  localparam logic [31:0] LPARA_DATA_ENTRY_W = LPARA_FIFO_DATA_W + LPARA_FIFO_STRB_W;
  // command {type, addr, len, id, awqos, awprot, vc, line, eol}; completion {resp, id, addr, vc,
  // line, eol}
  localparam logic [31:0] LPARA_CMD_W        = 32'd68;
  localparam logic [31:0] LPARA_CPL_W        = 32'd54;
  localparam logic [31:0] LPARA_TAG_W        = 32'd50;
  localparam logic [31:0] LPARA_ADDR_EXT_W   = 32'd34;
  localparam logic [31:0] LPARA_RA_WORDS     = 32'd10;
  localparam logic [31:0] LPARA_N_BUFFERS    = 32'd2;
  localparam logic [31:0] LPARA_MAX_BURST_LEN_WIDTH = 32'd8;
  localparam logic [2:0]  LPARA_AWSIZE       = (PARA_AXI_DATA_WIDTH == 32'd64) ? 3'b011 : 3'b010;
  localparam logic [31:0] LPARA_BEATS_W      = $clog2(PARA_MAX_BURST_BEATS) + 32'd1;
  localparam logic [31:0] LPARA_OUTST_W      = $clog2(PARA_MAX_OUTSTANDING) + 32'd1;
  localparam logic [31:0] LPARA_TAG_PTR_W    = $clog2(PARA_MAX_OUTSTANDING);
  localparam logic [31:0] LPARA_INFL_W       =
    $clog2(32'd2 * PARA_CMD_FIFO_DEPTH + 32'd2 * PARA_MAX_OUTSTANDING + 32'd8) + 32'd1;
  localparam logic [31:0] LPARA_WQ_DEPTH     = 32'd4;
  localparam logic [31:0] LPARA_WQ_PTR_W     = 32'd2;

  generate
    if (!((PARA_AXI_DATA_WIDTH == 32'd32) || (PARA_AXI_DATA_WIDTH == 32'd64))
        || (PARA_AXI_ADDR_WIDTH != 32'd32) ||
        (PARA_AXI_ID_WIDTH < 32'd2) || (PARA_AXI_ID_WIDTH > 32'd8)) begin : gen_para_check_axi
      $error("m_fpt_dma: unsupported PARA_AXI_DATA_WIDTH, PARA_AXI_ADDR_WIDTH, PARA_AXI_ID_WIDTH");
    end
    if ((PARA_MAX_BURST_BEATS < 32'd2) || (PARA_MAX_BURST_BEATS > 32'd256)
        || (PARA_META_MAX_BURST_BEATS < 32'd1) ||
        (PARA_META_MAX_BURST_BEATS > PARA_MAX_BURST_BEATS)) begin : gen_para_check_burst
      $error("m_fpt_dma: unsupported PARA_MAX_BURST_BEATS or PARA_META_MAX_BURST_BEATS");
    end
    if (((PARA_MAX_OUTSTANDING & (PARA_MAX_OUTSTANDING - 32'd1)) != 32'd0)
        || (PARA_MAX_OUTSTANDING < 32'd2) ||
        (PARA_MAX_OUTSTANDING > 32'd64) ||
        ((PARA_RAW_FIFO_DEPTH & (PARA_RAW_FIFO_DEPTH - 32'd1)) != 32'd0) ||
        (PARA_RAW_FIFO_DEPTH < PARA_MAX_BURST_BEATS) ||
        ((PARA_META_DATA_FIFO_DEPTH & (PARA_META_DATA_FIFO_DEPTH - 32'd1)) != 32'd0) ||
        (PARA_META_DATA_FIFO_DEPTH < 32'd2 * PARA_META_MAX_BURST_BEATS) ||
        ((PARA_CMD_FIFO_DEPTH & (PARA_CMD_FIFO_DEPTH - 32'd1)) != 32'd0) ||
        (PARA_CMD_FIFO_DEPTH < 32'd2)) begin : gen_para_check_fifo
      $error("m_fpt_dma: unsupported PARA_MAX_OUTSTANDING or PARA_*_FIFO_DEPTH");
    end
    if ((PARA_ASYNC_MM_CLK > 32'd1) || (PARA_SYNC_STAGES < 32'd2)
        || (PARA_SYNC_STAGES > 32'd4)) begin : gen_para_check_cdc
      $error("m_fpt_dma: unsupported PARA_ASYNC_MM_CLK or PARA_SYNC_STAGES");
    end
  endgenerate

  // Buffer FSM states; BUFFER_OWNERSHIP is decoded from them.
  typedef enum logic [2:0] {
    ENUM_ST_FREE     = 3'b000,
    ENUM_ST_WRITE    = 3'b001,
    ENUM_ST_DRAIN    = 3'b010,
    ENUM_ST_ABORT    = 3'b011,
    ENUM_ST_SW_VALID = 3'b100,
    ENUM_ST_SW_ERROR = 3'b101
  } buf_state_type_def;

  // ---- per-buffer state and shadow configuration
  buf_state_type_def            reg_fsm_buf_current_state [0:1];
  logic [1:0]                   w_buf_free, w_buf_writing, w_buf_leave, w_done_ok, w_done_err,
      w_abort_req;
  logic [1:0]                   reg_soft_reset_seen;
  logic [31:0]                  reg_sh_buf_addr [0:1];
  logic [31:0]                  reg_sh_raw_size [0:1];
  logic [31:0]                  reg_sh_meta_area_size [0:1];
  logic [15:0]                  reg_sh_bytes_per_line [0:1];
  logic [7:0]                   reg_sh_line_padding [0:1];
  logic [LPARA_MAX_BURST_LEN_WIDTH-1:0] reg_sh_max_burst_len [0:1];
  logic [3:0]                   reg_sh_awqos [0:1];
  logic [2:0]                   reg_sh_awprot [0:1];
  logic [1:0]                   reg_sh_policy, reg_sh_tpg;
  logic [15:0]                  reg_sh_tpg_height [0:1];
  logic [1:0]                   reg_joined [0:1];
  logic [1:0]                   reg_rec_seen [0:1];
  logic [1:0]                   reg_moved [0:1];
  logic [15:0]                  reg_rec_lines [0:1][0:1];
  logic [15:0]                  reg_lines [0:1][0:1];
  logic [1:0]                   reg_ended;
  logic [LPARA_INFL_W-1:0]      reg_inflight [0:1];
  // ---- Frame Binder
  logic [1:0]                   reg_cap_start;
  logic                         reg_ev_fs;
  logic                         reg_ev_vc;
  logic                         reg_lead_vc;
  logic [1:0]                   reg_slot_ok, reg_slot_buf;
  logic                         reg_last_buf, reg_active_buffer;
  logic [1:0]                   reg_pending_valid, reg_pending_buf;
  logic                         w_run, w_abort_all, w_clr, w_pick, w_pick_none, w_claim, w_dma_drop;
  logic [1:0]                   w_bind_end, w_bind_drop, w_load, w_fs_valid, w_fs_buf, w_pend_load;
  logic [1:0]                   w_bind_join, w_leave_hit;
  // ---- Pixel Writer
  logic [1:0]                   reg_pw_fresh, reg_pw_active_valid, reg_pw_active_buf;
  logic [15:0]                  reg_line_idx [0:1];
  logic [16:0]                  reg_line_bytes;
  logic [LPARA_BEATS_W-1:0]     reg_beats;
  logic [31:0]                  reg_burst_addr;
  logic                         reg_burst_vc;
  logic                         w_vc, w_beat, w_sof, w_sol, w_eol, w_bind_tpg, w_raw_full,
      w_fbd_ready;
  logic [1:0]                   w_sof_x, w_eol_x, w_nb_valid, w_nb_buf, w_bind_valid, w_bind_buf;
  logic [15:0]                  w_lidx [0:1];
  logic                         w_cur_valid, w_cur_buf, w_clip, w_lose, w_wbeat, w_pw_err, w_write;
  logic [16:0]                  w_stride, w_lb_prev, w_lb;
  logic [LPARA_ADDR_EXT_W-1:0]  w_line_addr, w_region_end;
  logic [31:0]                  w_beat_addr, w_bstart;
  logic [9:0]                   w_to_4k;
  logic [LPARA_BEATS_W-1:0]     w_n, w_lim;
  logic                         w_close, w_skip, w_raw_cmd_push, w_raw_data_push;
  logic [LPARA_CMD_W-1:0]       w_raw_cmd_din;
  logic [LPARA_DATA_ENTRY_W-1:0] w_raw_data_din;
  logic [1:0]                   w_wr_issue_pw, w_wr_err_pw, w_miss_evt;
  logic                         w_eol_evt, w_sof_evt, w_sof_prev_valid, w_sof_prev_buf;
  // ---- Metadata Writer
  logic [1:0]                   reg_mw_fresh, reg_mw_active_valid, reg_mw_active_buf;
  logic [31:0]                  reg_pay_ptr [0:1];
  logic [LPARA_BEATS_W-1:0]     reg_pay_beats;
  logic [31:0]                  reg_pay_addr;
  logic                         reg_pay_vc, reg_pay_buf, reg_pay_open;
  logic [63:0]                  reg_pay_data;
  logic [7:0]                   reg_pay_strb;
  logic [31:0]                  reg_ra_word [0:LPARA_RA_WORDS-1];
  logic [3:0]                   reg_ra_cnt;
  logic                         reg_ra_vc, reg_ra_full, reg_ra_busy, reg_rec_buf_valid, reg_rec_buf;
  logic [2:0]                   reg_rec_beat;
  logic [LPARA_BEATS_W-1:0]     reg_rec_bn;
  logic [31:0]                  reg_rec_start, reg_rec_addr;
  logic                         w_mv, w_mx, w_rec_wait, w_meta_ready, w_pay_bind_valid, w_pay_cut,
      w_pay_skip, w_pay_detach;
  logic                         w_ra_go, w_meta_data_push, w_meta_cmd_push, w_rec_done,
      w_rec_done_buf;
  logic                         w_rec_done_x, w_ra_we;
  logic [15:0]                  w_rec_lines;
  logic [3:0]                   w_ra_len, w_ra_widx;
  logic [LPARA_DATA_ENTRY_W-1:0] w_meta_data_din;
  logic [LPARA_CMD_W-1:0]       w_meta_cmd_din;
  logic [1:0]                   w_wr_issue_mw, w_wr_err_mw;
  logic [LPARA_BEATS_W-1:0]     w_pay_beats_nxt, w_rec_bn_nxt;
  logic [31:0]                  w_pay_addr_nxt, w_rec_start_nxt, w_rec_addr_nxt;
  logic                         w_pay_vc_nxt, w_pay_buf_nxt, w_pay_open_nxt;
  logic [63:0]                  w_pay_data_nxt;
  logic [7:0]                   w_pay_strb_nxt;
  logic [31:0]                  w_pay_ptr_nxt [0:1];
  logic [3:0]                   w_ra_cnt_nxt;
  logic                         w_ra_vc_nxt, w_ra_full_nxt, w_ra_busy_nxt, w_rec_buf_valid_nxt,
      w_rec_buf_nxt;
  logic [2:0]                   w_rec_beat_nxt;
  logic [1:0]                   w_mw_sof_x, w_mw_word_x, w_mw_act_valid_nxt, w_mw_act_buf_nxt;
  // ---- Buffer Manager status
  logic                         w_tok_valid, w_cpl_pop;
  logic [LPARA_CPL_W-1:0]       w_cpl_dout;
  logic [1:0]                   w_tok_resp, w_tok_id;
  logic [31:0]                  w_tok_addr;
  logic                         w_tok_vc, w_tok_eol;
  logic [15:0]                  w_tok_line;
  logic [1:0]                   w_axi_err, w_cpl_token;
  logic                         w_axi_write_error, w_overflow, w_cpl_empty;
  logic [15:0]                  reg_bresp_slverr_count, reg_bresp_decerr_count;
  logic                         reg_first_fault;
  logic [7:0]                   reg_axi_fault_awid;
  logic [1:0]                   reg_axi_fault_resp;
  logic [31:0]                  reg_axi_fault_awaddr;
  logic [15:0]                  reg_lvl [0:1][0:1];
  logic [1:0]                   reg_lvl_has [0:1];
  logic [1:0]                   reg_lvl_stop, reg_lvl_claimed;
  logic [15:0]                  w_buf_last_valid_line [0:1];
  logic                         w_dma_busy;
  logic [1:0]                   w_buf_state_code [0:1];
  // ---- FIFO interfaces
  logic                         w_raw_cmd_full, w_raw_cmd_empty, w_raw_cmd_pop;
  logic                         w_raw_data_full, w_raw_data_empty, w_raw_data_pop;
  logic                         w_meta_cmd_full, w_meta_cmd_empty, w_meta_cmd_pop;
  logic                         w_meta_data_full, w_meta_data_empty, w_meta_data_pop;
  logic                         w_cpl_full, w_cpl_push;
  logic [LPARA_CMD_W-1:0]       w_raw_cmd_dout, w_meta_cmd_dout;
  logic [LPARA_DATA_ENTRY_W-1:0] w_raw_data_dout, w_meta_data_dout;
  logic [LPARA_CPL_W-1:0]       w_cpl_din;
  logic [LPARA_OUTST_W-1:0]     w_cpl_used;
  // ---- AXI Engine
  logic                         reg_awvalid;
  logic [PARA_AXI_ID_WIDTH-1:0] reg_awid;
  logic [31:0]                  reg_awaddr;
  logic [7:0]                   reg_awlen;
  logic [3:0]                   reg_awqos;
  logic [2:0]                   reg_awprot;
  logic [LPARA_OUTST_W-1:0]     reg_aw_outst;
  logic [LPARA_WQ_DEPTH-1:0]    reg_wq_stream, reg_wq_skip;
  logic [LPARA_BEATS_W-1:0]     reg_wq_beats [0:LPARA_WQ_DEPTH-1];
  logic [LPARA_WQ_PTR_W:0]      reg_wq_wr_ptr, reg_wq_rd_ptr;
  logic [LPARA_BEATS_W-1:0]     reg_w_cnt;
  logic                         reg_w_half, reg_wvalid, reg_wlast;
  logic [PARA_AXI_DATA_WIDTH-1:0] reg_wdata;
  logic [PARA_AXI_DATA_WIDTH/8-1:0] reg_wstrb;
  logic [LPARA_TAG_W-1:0]       reg_tag_mem [0:3][PARA_MAX_OUTSTANDING];
  logic [LPARA_TAG_PTR_W-1:0]   reg_tag_wr_ptr [0:3];
  logic [LPARA_TAG_PTR_W-1:0]   reg_tag_rd_ptr [0:3];
  logic [LPARA_OUTST_W:0]       w_inflight_mm;
  logic                         w_issue_room, w_meta_cmd_ok, w_raw_cmd_ok, w_wq_full, w_wq_empty;
  logic                         w_take_meta, w_take_raw, w_take_write, w_wq_push, w_wq_pop;
  logic [LPARA_CMD_W-1:0]       w_sel_cmd;
  logic                         w_head_stream, w_head_skip, w_data_avail, w_w_free, w_w_load,
      w_head_last;
  logic [LPARA_BEATS_W-1:0]     w_head_beats;
  logic [LPARA_DATA_ENTRY_W-1:0] w_head_data;
  logic                         w_axi_last_beat, w_b_hs;
  logic [1:0]                   w_b_id;

  // 8-bit population count of a byte enable.
  function automatic logic [3:0] f_popcount8(input logic [7:0] be);
    logic [3:0] cnt;
    cnt = 4'd0;
    for (int idx = 0; idx < 8; idx++) cnt = cnt + 4'(be[idx]);
    return cnt;
  endfunction

  // Writer burst limit in 64-bit beats from the clamped max_burst_len (AXI beats minus one).
  function automatic logic [LPARA_BEATS_W-1:0] f_raw_lim(
    input logic [LPARA_MAX_BURST_LEN_WIDTH-1:0] mbl);
    logic [LPARA_MAX_BURST_LEN_WIDTH:0] axi_beats;
    axi_beats = {1'b0, mbl} + 1'b1;
    if (PARA_AXI_DATA_WIDTH == 32'd64) return LPARA_BEATS_W'(axi_beats);
    if (axi_beats < (LPARA_MAX_BURST_LEN_WIDTH + 32'd1)'(2)) return LPARA_BEATS_W'(1);
    return LPARA_BEATS_W'(axi_beats >> 1);
  endfunction

  // Metadata burst limit in 64-bit beats: the smaller of max_burst_len + 1 and
  // PARA_META_MAX_BURST_BEATS.
  function automatic logic [LPARA_BEATS_W-1:0] f_meta_lim(
    input logic [LPARA_MAX_BURST_LEN_WIDTH-1:0] mbl);
    logic [LPARA_MAX_BURST_LEN_WIDTH:0] axi_beats;
    axi_beats = {1'b0, mbl} + 1'b1;
    if (32'(axi_beats) > PARA_META_MAX_BURST_BEATS)
      axi_beats = (LPARA_MAX_BURST_LEN_WIDTH + 32'd1)'(PARA_META_MAX_BURST_BEATS);
    if (PARA_AXI_DATA_WIDTH == 32'd64) return LPARA_BEATS_W'(axi_beats);
    if (axi_beats < (LPARA_MAX_BURST_LEN_WIDTH + 32'd1)'(2)) return LPARA_BEATS_W'(1);
    return LPARA_BEATS_W'(axi_beats >> 1);
  endfunction

  // Base of the metadata area of channel x in a buffer.
  function automatic logic [LPARA_ADDR_EXT_W-1:0] f_meta_base(input logic [31:0] base,
      input logic [31:0] raw,
                                                             input logic [31:0] msz, input logic x);
    return LPARA_ADDR_EXT_W'(base) + (LPARA_ADDR_EXT_W'(raw) << 1) + (x ? LPARA_ADDR_EXT_W'(msz)
        : '0);
  endfunction

  // ------------------------------------------------------------------------------------------
  // Buffer status decode
  // ------------------------------------------------------------------------------------------
  // Levels of the registered Buffer FSM states read by Frame Binder and both writers.
  always_comb begin : p_com_buf_status
    w_dma_busy = 1'b0;
    for (int b = 0; b < 2; b++) begin
      w_buf_free[b]    = (reg_fsm_buf_current_state[b] == ENUM_ST_FREE);
      w_buf_writing[b] = (reg_fsm_buf_current_state[b] == ENUM_ST_WRITE);
      if ((reg_fsm_buf_current_state[b] == ENUM_ST_WRITE) || (reg_fsm_buf_current_state[b]
          == ENUM_ST_DRAIN) ||
          (reg_fsm_buf_current_state[b] == ENUM_ST_ABORT)) w_dma_busy = 1'b1;
      if (reg_fsm_buf_current_state[b] == ENUM_ST_SW_VALID) w_buf_state_code[b] = 2'b01;
      else if (reg_fsm_buf_current_state[b] == ENUM_ST_SW_ERROR) w_buf_state_code[b] = 2'b10;
      else w_buf_state_code[b] = 2'b00;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Frame Binder
  // ------------------------------------------------------------------------------------------
  // Claim one cycle after capture_start, alternating between free buffers; no claim and no dma_drop
  // while dma_enable or dma_start is clear, dma_abort is set or soft_reset is high. End and drop
  // decoder per slot.
  always_comb begin : p_com_fb_claim
    w_run       = i_dma_enable && i_dma_start && !i_dma_abort;
    w_abort_all = i_dma_abort_req || i_dma_abort;
    w_clr       = w_abort_all || i_soft_reset;
    w_pick_none = 1'b0;
    w_pick      = !reg_last_buf;
    if (w_buf_free[!reg_last_buf]) w_pick = !reg_last_buf;
    else if (w_buf_free[reg_last_buf]) w_pick = reg_last_buf;
    else w_pick_none = 1'b1;
    w_claim    = (reg_cap_start != 2'b00) && w_run && !w_pick_none && !i_soft_reset;
    w_dma_drop = (reg_cap_start != 2'b00) && w_run && w_pick_none && !i_soft_reset;
    w_bind_end  = 2'b00;
    w_bind_drop = 2'b00;
    for (int s = 0; s < 2; s++) begin
      if (i_capture_end[s] && reg_slot_ok[s]) begin
        w_bind_end[reg_slot_buf[s]] = 1'b1;
        if (i_capture_dropped[s]) w_bind_drop[reg_slot_buf[s]] = 1'b1;
      end
    end
  end

  // Pending binding of each channel, loaded at ev_fs_q: the claimed buffer for the lead channel,
  // the buffer of the slot named by ctx_slot for the lag channel unless orphan, slot not valid or
  // that buffer leaving ST_WRITE now. abort_all and soft_reset discard every pending binding
  // (priority over the load).
  always_comb begin : p_com_fb_pending
    for (int x = 0; x < 2; x++) begin
      w_load[x] = reg_ev_fs && (reg_ev_vc == 1'(x));
      if (reg_lead_vc == 1'(x)) begin
        w_fs_valid[x] = w_claim;
        w_fs_buf[x]   = w_pick;
      end else begin
        w_fs_buf[x]   = reg_slot_buf[i_ctx_slot[x]];
        w_fs_valid[x] = !i_ctx_orphan[x] && reg_slot_ok[i_ctx_slot[x]]
            && !w_buf_leave[reg_slot_buf[i_ctx_slot[x]]];
      end
      w_pend_load[x] = w_load[x] && !w_clr;
      w_leave_hit[x] = reg_pending_valid[x] && w_buf_leave[reg_pending_buf[x]];
      w_bind_join[x] = w_load[x] && (reg_lead_vc != 1'(x)) && w_fs_valid[x];
      // a pending binding replaced by a new load before the first beat of its frame was taken
      w_miss_evt[x]  = w_pend_load[x] && reg_pw_fresh[x] && reg_pending_valid[x];
    end
  end

  // Input registers (cleared by soft_reset), pending bindings, capture slots, last_buf (1 after
  // reset so the first claim takes buffer 0) and active_buffer (not changed by soft_reset).
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_frame_binder
    if (!i_resetn) begin
      reg_cap_start     <= 2'b00;
      reg_ev_fs         <= 1'b0;
      reg_ev_vc         <= 1'b0;
      reg_lead_vc       <= 1'b0;
      reg_pending_valid <= 2'b00;
      reg_pending_buf   <= 2'b00;
      reg_slot_ok       <= 2'b00;
      reg_slot_buf      <= 2'b00;
      reg_last_buf      <= 1'b1;
      reg_active_buffer <= 1'b0;
    end else begin
      reg_cap_start <= i_soft_reset ? 2'b00 : i_capture_start;
      reg_ev_fs     <= i_ev_fs && !i_soft_reset;
      if (i_soft_reset) begin
        reg_ev_vc   <= 1'b0;
        reg_lead_vc <= 1'b0;
      end else if (i_ev_fs) begin
        reg_ev_vc   <= i_ev_vc[0];
        reg_lead_vc <= i_lead_vc;
      end
      for (int x = 0; x < 2; x++) begin
        if (w_clr || w_load[x] || w_leave_hit[x]) begin
          reg_pending_valid[x] <= (w_clr || !w_load[x]) ? 1'b0 : w_fs_valid[x];
          if (!w_clr && w_load[x]) reg_pending_buf[x] <= w_fs_buf[x];
        end
      end
      for (int s = 0; s < 2; s++) begin
        if (i_soft_reset) reg_slot_ok[s] <= 1'b0;
        else if (reg_cap_start[s]) begin
          reg_slot_ok[s]  <= w_claim;
          reg_slot_buf[s] <= w_pick;
        end else if (reg_slot_ok[s] && w_buf_leave[reg_slot_buf[s]]) reg_slot_ok[s] <= 1'b0;
      end
      if (i_soft_reset) reg_last_buf <= 1'b1;
      else if (w_claim) reg_last_buf <= w_pick;
      if (w_claim) reg_active_buffer <= w_pick;
    end
  end

  // ------------------------------------------------------------------------------------------
  // Pixel Writer
  // ------------------------------------------------------------------------------------------
  // Beat decoder, binding tracker, line address and clip, write qualifier, burst and command
  // builder. fbd_ready is held at 1 except for a beat bound to a Test Pattern Generator buffer
  // while a RAW FIFO is full; on the sensor path such a beat is lost and aborts its buffer. A burst
  // closes at the burst limit, at the next 4 KB boundary or at fbd_eol, and its command follows its
  // last beat (store and forward); an open burst whose binding is gone is released by a SKIP
  // command.
  always_comb begin : p_com_pixel_writer
    logic nbp_valid, nbp_buf;
    logic [LPARA_ADDR_EXT_W-1:0] raw_ext;
    w_vc      = i_fbd_vc_tag[0];
    nbp_valid = (i_fbd_valid && i_fbd_sof) ? (reg_pw_fresh[w_vc] && reg_pending_valid[w_vc]) :
                                             reg_pw_active_valid[w_vc];
    nbp_buf   = (i_fbd_valid && i_fbd_sof) ? reg_pending_buf[w_vc] : reg_pw_active_buf[w_vc];
    w_bind_tpg  = nbp_valid && w_buf_writing[nbp_buf] && reg_sh_tpg[nbp_buf];
    w_raw_full  = w_raw_data_full || w_raw_cmd_full;
    w_fbd_ready = w_bind_tpg ? !w_raw_full : 1'b1;
    w_beat      = i_fbd_valid && w_fbd_ready;
    w_sof       = w_beat && i_fbd_sof;
    w_sol       = w_beat && i_fbd_sol;
    w_eol       = w_beat && i_fbd_eol;
    for (int x = 0; x < 2; x++) begin
      w_sof_x[x]      = w_sof && (w_vc == 1'(x));
      w_eol_x[x]      = w_eol && (w_vc == 1'(x));
      w_nb_valid[x]   = w_sof_x[x] ? (reg_pw_fresh[x] && reg_pending_valid[x])
          : reg_pw_active_valid[x];
      w_nb_buf[x]     = w_sof_x[x] ? reg_pending_buf[x] : reg_pw_active_buf[x];
      w_bind_valid[x] = w_nb_valid[x] && w_buf_writing[w_nb_buf[x]] && !i_soft_reset;
      w_bind_buf[x]   = w_nb_buf[x];
      w_lidx[x]       = w_sof_x[x] ? 16'd0 : reg_line_idx[x];
    end
    w_cur_valid  = w_bind_valid[w_vc];
    w_cur_buf    = w_bind_buf[w_vc];
    w_stride     = ((17'(reg_sh_bytes_per_line[w_cur_buf]) + 17'(reg_sh_line_padding[w_cur_buf])) >>
                    LPARA_BEAT_SHIFT) << LPARA_BEAT_SHIFT;
    raw_ext      = LPARA_ADDR_EXT_W'(reg_sh_raw_size[w_cur_buf]);
    w_line_addr  = LPARA_ADDR_EXT_W'(reg_sh_buf_addr[w_cur_buf]) + (w_vc ? raw_ext : '0) +
                   LPARA_ADDR_EXT_W'(w_lidx[w_vc]) * LPARA_ADDR_EXT_W'(w_stride);
    w_lb_prev    = w_sol ? 17'd0 : reg_line_bytes;
    w_lb         = w_lb_prev + 17'(f_popcount8(i_fbd_byte_en));
    w_beat_addr  = 32'(w_line_addr + LPARA_ADDR_EXT_W'(w_lb_prev));
    w_region_end = LPARA_ADDR_EXT_W'(reg_sh_buf_addr[w_cur_buf]) + (w_vc ? (raw_ext << 1)
        : raw_ext);
    w_clip       = (w_lb > 17'(reg_sh_bytes_per_line[w_cur_buf])) ||
                   ((w_line_addr + LPARA_ADDR_EXT_W'(w_lb)) > w_region_end);
    w_wbeat      = w_beat && w_cur_valid;
    w_lose       = !w_bind_tpg && w_raw_full;
    w_pw_err     = (w_wbeat && w_clip) || (w_wbeat && w_lose);
    w_write      = w_wbeat && !w_pw_err;
    w_bstart     = (reg_beats == '0) ? w_beat_addr : reg_burst_addr;
    w_to_4k      = 10'((13'h1000 - 13'(w_bstart[11:0])) >> LPARA_BEAT_SHIFT);
    w_n          = reg_beats + 1'b1;
    w_lim        = f_raw_lim(reg_sh_max_burst_len[w_cur_buf]);
    w_close      = w_write && (i_fbd_eol || (10'(w_n) == w_to_4k) || (w_n == w_lim));
    w_skip       = (reg_beats != '0) && !w_bind_valid[reg_burst_vc];
    w_raw_cmd_push  = w_close || (w_skip && !w_raw_cmd_full);
    w_raw_data_push = w_write;
    w_raw_data_din  = {i_fbd_data, i_fbd_byte_en};
    if (w_close)
      w_raw_cmd_din = {1'b0, w_bstart, 8'(w_n - 1'b1), 1'b0, w_cur_buf, reg_sh_awqos[w_cur_buf],
                       reg_sh_awprot[w_cur_buf], w_vc, w_lidx[w_vc], i_fbd_eol};
    else
      w_raw_cmd_din = {1'b1, 32'd0, 8'(reg_beats - 1'b1), 2'b00, 4'd0, 3'd0, 1'b0, 16'd0, 1'b0};
    for (int b = 0; b < 2; b++) begin
      w_wr_issue_pw[b] = w_close && (w_cur_buf == 1'(b));
      w_wr_err_pw[b]   = w_pw_err && (w_cur_buf == 1'(b));
    end
    w_eol_evt        = w_close && i_fbd_eol;
    w_sof_evt        = w_sof;
    w_sof_prev_valid = reg_pw_active_valid[w_vc];
    w_sof_prev_buf   = reg_pw_active_buf[w_vc];
  end

  // fresh: a load since the last Frame Start beat of the channel; active binding, line index and
  // line byte count are discarded by soft_reset; the open-burst count is not (its beats leave by
  // SKIP).
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_pixel_writer
    if (!i_resetn) begin
      reg_pw_fresh        <= 2'b00;
      reg_pw_active_valid <= 2'b00;
      reg_pw_active_buf   <= 2'b00;
      reg_line_idx[0]     <= 16'd0;
      reg_line_idx[1]     <= 16'd0;
      reg_line_bytes      <= 17'd0;
      reg_beats           <= '0;
      reg_burst_addr      <= 32'd0;
      reg_burst_vc        <= 1'b0;
    end else begin
      for (int x = 0; x < 2; x++) begin
        if (i_soft_reset) begin
          reg_pw_fresh[x]        <= 1'b0;
          reg_pw_active_valid[x] <= 1'b0;
          reg_line_idx[x]        <= 16'd0;
        end else begin
          reg_pw_fresh[x]        <= w_pend_load[x] || (reg_pw_fresh[x] && !w_sof_x[x]);
          reg_pw_active_valid[x] <= w_bind_valid[x] && !w_buf_leave[w_bind_buf[x]];
          reg_line_idx[x]        <= w_lidx[x] + 16'(w_eol_x[x]);
        end
        reg_pw_active_buf[x] <= w_bind_buf[x];
      end
      if (i_soft_reset) reg_line_bytes <= 17'd0;
      else if (w_beat) reg_line_bytes <= w_lb;
      if (w_raw_cmd_push) reg_beats <= '0;
      else if (w_write) reg_beats <= w_n;
      if (w_write) begin
        reg_burst_addr <= w_bstart;
        reg_burst_vc   <= w_vc;
      end
    end
  end

  // ------------------------------------------------------------------------------------------
  // Metadata Writer
  // ------------------------------------------------------------------------------------------
  // Payload: at meta_sof the pending binding becomes the active metadata binding (taken at most
  // once per load, valid only while its buffer is in ST_WRITE) and the payload pointer restarts at
  // offset 0x80; two words per beat by address bit 2. Record: collected word by word (up to 10
  // words) and written from offset 0x00 as one or more bursts once complete, after the open payload
  // burst is closed (pay_cut). A payload burst whose binding is gone is detached at once (the next
  // word starts a new burst); its beats already in the data FIFO are released by a SKIP command
  // and a half-filled beat not yet pushed is dropped. The command FIFO is never full then: a word
  // is taken only while it has room and the push that fills it closes the burst. Priority: the
  // detached burst, closing an open payload burst (pay_cut), the record (region check, then one
  // beat per cycle), then the accepted word. meta_ready is low while a record waits or is written.
  always_comb begin : p_com_meta_writer
    logic mbv, mnb_valid, mnb_buf, ahi, cls, last_b;
    logic [31:0] a, ba;
    logic [LPARA_ADDR_EXT_W-1:0] mbase, a_ext;
    logic [LPARA_BEATS_W-1:0] bn;
    logic [2:0] nbeats;
    logic [3:0] widx_hi;
    logic [LPARA_BEATS_W-1:0] pushed;
    pushed    = reg_pay_beats - LPARA_BEATS_W'(reg_pay_open);
    mbv       = 1'b0;
    mnb_valid = 1'b0;
    mnb_buf   = 1'b0;
    ahi       = 1'b0;
    cls       = 1'b0;
    last_b    = 1'b0;
    a         = 32'd0;
    ba        = 32'd0;
    mbase     = '0;
    a_ext     = '0;
    bn        = '0;
    nbeats    = 3'd0;
    widx_hi   = 4'd0;
    w_mx             = i_meta_vc[0];
    w_rec_wait       = reg_ra_full && !reg_ra_busy;
    w_meta_ready     = !w_meta_data_full && !w_meta_cmd_full && !w_rec_wait && !reg_ra_busy;
    w_mv             = i_meta_valid && w_meta_ready;
    w_pay_bind_valid = reg_mw_active_valid[reg_pay_vc] &&
                       w_buf_writing[reg_mw_active_buf[reg_pay_vc]] && !i_soft_reset;
    w_pay_cut        = w_rec_wait && (reg_pay_beats != '0);
    w_pay_skip       = (reg_pay_beats != '0) && !w_pay_bind_valid;
    w_ra_go          = w_rec_wait && (reg_pay_beats == '0);
    w_pay_detach     = w_pay_skip && !w_rec_wait;
    w_ra_len         = (reg_ra_cnt > 4'(LPARA_RA_WORDS)) ? 4'(LPARA_RA_WORDS) : reg_ra_cnt;
    w_meta_data_push = 1'b0;
    w_meta_data_din  = {reg_pay_data, reg_pay_strb};
    w_meta_cmd_push  = 1'b0;
    w_meta_cmd_din   = '0;
    w_wr_issue_mw    = 2'b00;
    w_wr_err_mw      = 2'b00;
    w_rec_done       = 1'b0;
    w_rec_done_buf   = reg_rec_buf;
    w_rec_done_x     = reg_ra_vc;
    w_rec_lines      = reg_ra_word[5][31:16];
    w_ra_we          = 1'b0;
    w_ra_widx        = reg_ra_cnt;
    w_pay_beats_nxt  = reg_pay_beats;
    w_pay_addr_nxt   = reg_pay_addr;
    w_pay_vc_nxt     = reg_pay_vc;
    w_pay_buf_nxt    = reg_pay_buf;
    w_pay_open_nxt   = reg_pay_open;
    w_pay_data_nxt   = reg_pay_data;
    w_pay_strb_nxt   = reg_pay_strb;
    w_pay_ptr_nxt[0] = reg_pay_ptr[0];
    w_pay_ptr_nxt[1] = reg_pay_ptr[1];
    w_ra_cnt_nxt     = reg_ra_cnt;
    w_ra_vc_nxt      = reg_ra_vc;
    w_ra_full_nxt    = reg_ra_full;
    w_ra_busy_nxt    = reg_ra_busy;
    w_rec_buf_valid_nxt = reg_rec_buf_valid;
    w_rec_buf_nxt    = reg_rec_buf;
    w_rec_beat_nxt   = reg_rec_beat;
    w_rec_bn_nxt     = reg_rec_bn;
    w_rec_start_nxt  = reg_rec_start;
    w_rec_addr_nxt   = reg_rec_addr;
    w_mw_sof_x       = 2'b00;
    w_mw_word_x      = 2'b00;
    w_mw_act_valid_nxt = reg_mw_active_valid;
    w_mw_act_buf_nxt   = reg_mw_active_buf;
    // ---- 0. payload burst without binding: detach it; SKIP its beats already in the data FIFO
    if (w_pay_detach) begin
      w_pay_beats_nxt = '0;
      w_pay_open_nxt  = 1'b0;
      if (pushed != '0) begin
        w_meta_cmd_push = 1'b1;
        w_meta_cmd_din  = {1'b1, 32'd0, 8'(pushed - 1'b1), 2'b00, 4'd0, 3'd0, 1'b0, 16'd0, 1'b0};
      end
    end
    // ---- 1. open payload burst closed by a waiting record (pay_cut)
    if (w_pay_cut && !w_meta_cmd_full && !(reg_pay_open && w_meta_data_full)) begin
      w_meta_data_push = reg_pay_open;
      w_pay_open_nxt   = 1'b0;
      w_meta_cmd_push  = 1'b1;
      if (w_pay_bind_valid) begin
        w_meta_cmd_din = {1'b0, reg_pay_addr, 8'(reg_pay_beats - 1'b1), 1'b1, reg_pay_buf,
            reg_sh_awqos[reg_pay_buf],
                          reg_sh_awprot[reg_pay_buf], reg_pay_vc, 16'd0, 1'b0};
        w_wr_issue_mw[reg_pay_buf] = 1'b1;
      end else begin
        w_meta_cmd_din = {1'b1, 32'd0, 8'(reg_pay_beats - 1'b1), 2'b00, 4'd0, 3'd0, 1'b0, 16'd0,
            1'b0};
      end
      w_pay_beats_nxt = '0;
    end else if (w_ra_go) begin
      // ---- 2a. a complete record: a record passing the end of its metadata area aborts the buffer
      if ((LPARA_ADDR_EXT_W'(w_ra_len) << 2) >
          LPARA_ADDR_EXT_W'(reg_sh_meta_area_size[reg_rec_buf])) begin
        w_wr_err_mw[reg_rec_buf] = 1'b1;
        w_ra_full_nxt = 1'b0;
        w_ra_cnt_nxt  = 4'd0;
      end else begin
        mbase = f_meta_base(reg_sh_buf_addr[reg_rec_buf], reg_sh_raw_size[reg_rec_buf],
                            reg_sh_meta_area_size[reg_rec_buf], reg_ra_vc);
        w_ra_busy_nxt   = 1'b1;
        w_rec_beat_nxt  = 3'd0;
        w_rec_bn_nxt    = '0;
        w_rec_start_nxt = mbase[31:0];
        w_rec_addr_nxt  = mbase[31:0];
      end
    end else if (reg_ra_busy && !w_meta_data_full && !w_meta_cmd_full) begin
      // ---- 2b. one record beat per cycle; a burst closes at the metadata burst limit, at 4 KB or
      // at the end
      nbeats  = 3'((w_ra_len + 4'd1) >> 1);
      widx_hi = {reg_rec_beat, 1'b1};
      ba      = reg_rec_addr;
      bn      = reg_rec_bn + 1'b1;
      last_b  = (reg_rec_beat == (nbeats - 3'd1));
      w_meta_data_push = 1'b1;
      if (widx_hi < w_ra_len)
        w_meta_data_din = {reg_ra_word[widx_hi], reg_ra_word[{reg_rec_beat, 1'b0}], 8'hFF};
      else
        w_meta_data_din = {32'd0, reg_ra_word[{reg_rec_beat, 1'b0}], 8'h0F};
      cls = last_b || (bn == f_meta_lim(reg_sh_max_burst_len[reg_rec_buf])) || ((ba[11:0] + 12'd8)
          == 12'd0);
      if (cls) begin
        w_meta_cmd_push = 1'b1;
        if (w_buf_writing[reg_rec_buf]) begin
          w_meta_cmd_din = {1'b0, reg_rec_start, 8'(bn - 1'b1), 1'b1, reg_rec_buf,
              reg_sh_awqos[reg_rec_buf],
                            reg_sh_awprot[reg_rec_buf], reg_ra_vc, 16'd0, 1'b0};
          w_wr_issue_mw[reg_rec_buf] = 1'b1;
          w_rec_done = last_b;
        end else begin
          w_meta_cmd_din = {1'b1, 32'd0, 8'(bn - 1'b1), 2'b00, 4'd0, 3'd0, 1'b0, 16'd0, 1'b0};
        end
        w_rec_start_nxt = ba + 32'd8;
        w_rec_bn_nxt    = '0;
      end else begin
        w_rec_bn_nxt = bn;
      end
      w_rec_addr_nxt = ba + 32'd8;
      w_rec_beat_nxt = reg_rec_beat + 3'd1;
      if (last_b) begin
        w_ra_busy_nxt = 1'b0;
        w_ra_full_nxt = 1'b0;
        w_ra_cnt_nxt  = 4'd0;
      end
    end
    // ---- 3a. record word: frame_id of word 1 selects the buffer in ST_WRITE; a record matching no
    // buffer is discarded at its last word
    if (w_mv && !i_meta_kind) begin
      w_ra_we     = (reg_ra_cnt < 4'(LPARA_RA_WORDS));
      w_ra_vc_nxt = w_mx;
      if (reg_ra_cnt != 4'hF) w_ra_cnt_nxt = reg_ra_cnt + 4'd1;
      if (reg_ra_cnt == 4'd0) w_rec_buf_valid_nxt = 1'b0;
      if (reg_ra_cnt == 4'd1) begin
        if ((i_meta_data[13:0] == i_frame_id_buf0) && w_buf_writing[0]) begin
          w_rec_buf_valid_nxt = 1'b1;
          w_rec_buf_nxt       = 1'b0;
        end else if ((i_meta_data[13:0] == i_frame_id_buf1) && w_buf_writing[1]) begin
          w_rec_buf_valid_nxt = 1'b1;
          w_rec_buf_nxt       = 1'b1;
        end else begin
          w_rec_buf_valid_nxt = 1'b0;
        end
      end
      if (i_meta_last) begin
        if (w_rec_buf_valid_nxt) w_ra_full_nxt = 1'b1;
        else w_ra_cnt_nxt = 4'd0;
      end
    end
    // ---- 3b. payload word
    if (w_mv && i_meta_kind) begin
      w_mw_word_x[w_mx] = 1'b1;
      if (i_meta_sof) begin
        w_mw_sof_x[w_mx] = 1'b1;
        mnb_valid = reg_mw_fresh[w_mx] && reg_pending_valid[w_mx];
        mnb_buf   = reg_pending_buf[w_mx];
      end else begin
        mnb_valid = reg_mw_active_valid[w_mx];
        mnb_buf   = reg_mw_active_buf[w_mx];
      end
      mbv   = mnb_valid && w_buf_writing[mnb_buf] && !i_soft_reset;
      w_mw_act_valid_nxt[w_mx] = mbv;
      w_mw_act_buf_nxt[w_mx]   = mnb_buf;
      mbase = f_meta_base(reg_sh_buf_addr[mnb_buf], reg_sh_raw_size[mnb_buf],
          reg_sh_meta_area_size[mnb_buf], w_mx);
      a     = i_meta_sof ? (mbase[31:0] + 32'h80) : reg_pay_ptr[w_mx];
      a_ext = LPARA_ADDR_EXT_W'(a);
      ahi   = a[2];
      if (mbv) begin
        if (a_ext >= (mbase + LPARA_ADDR_EXT_W'(reg_sh_meta_area_size[mnb_buf]))) begin
          // payload at or beyond the end of the metadata area: not written, buffer aborted
          w_wr_err_mw[mnb_buf] = 1'b1;
        end else begin
          if ((reg_pay_beats == '0) || w_pay_detach) begin
            w_pay_addr_nxt  = {a[31:3], 3'b000};
            w_pay_vc_nxt    = w_mx;
            w_pay_buf_nxt   = mnb_buf;
            w_pay_beats_nxt = LPARA_BEATS_W'(1);
            w_pay_data_nxt  = 64'd0;
            w_pay_strb_nxt  = 8'd0;
          end else if (!ahi) begin
            w_pay_beats_nxt = reg_pay_beats + 1'b1;
            w_pay_data_nxt  = 64'd0;
            w_pay_strb_nxt  = 8'd0;
          end
          if (ahi) begin
            w_pay_data_nxt[63:32] = i_meta_data;
            w_pay_strb_nxt[7:4]   = i_meta_byte_en;
          end else begin
            w_pay_data_nxt[31:0]  = i_meta_data;
            w_pay_strb_nxt[3:0]   = i_meta_byte_en;
          end
          w_pay_ptr_nxt[w_mx] = a + 32'd4;
          cls = i_meta_last || (ahi && ((w_pay_beats_nxt
              == f_meta_lim(reg_sh_max_burst_len[mnb_buf])) ||
                                        ((a[11:0] + 12'd4) == 12'd0)));
          // a beat goes to the data FIFO when its upper word is written or when its burst closes
          w_meta_data_push = ahi || cls;
          w_meta_data_din  = {w_pay_data_nxt, w_pay_strb_nxt};
          w_pay_open_nxt   = !ahi && !cls;
          if (cls) begin
            w_meta_cmd_push = 1'b1;
            w_meta_cmd_din  = {1'b0, w_pay_addr_nxt, 8'(w_pay_beats_nxt - 1'b1), 1'b1, mnb_buf,
                               reg_sh_awqos[mnb_buf], reg_sh_awprot[mnb_buf], w_mx, 16'd0, 1'b0};
            w_wr_issue_mw[mnb_buf] = 1'b1;
            w_pay_beats_nxt = '0;
          end
        end
      end
    end
  end

  // Metadata binding and payload burst state; soft_reset discards the bindings and clears the
  // Record Assembler unless a record is being pushed (an open payload burst then leaves by SKIP).
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_meta_writer
    if (!i_resetn) begin
      reg_mw_fresh        <= 2'b00;
      reg_mw_active_valid <= 2'b00;
      reg_mw_active_buf   <= 2'b00;
      reg_pay_ptr[0]      <= 32'd0;
      reg_pay_ptr[1]      <= 32'd0;
      reg_pay_beats       <= '0;
      reg_pay_addr        <= 32'd0;
      reg_pay_vc          <= 1'b0;
      reg_pay_buf         <= 1'b0;
      reg_pay_open        <= 1'b0;
      reg_pay_data        <= 64'd0;
      reg_pay_strb        <= 8'd0;
      reg_ra_cnt          <= 4'd0;
      reg_ra_vc           <= 1'b0;
      reg_ra_full         <= 1'b0;
      reg_ra_busy         <= 1'b0;
      reg_rec_buf_valid   <= 1'b0;
      reg_rec_buf         <= 1'b0;
      reg_rec_beat        <= 3'd0;
      reg_rec_bn          <= '0;
      reg_rec_start       <= 32'd0;
      reg_rec_addr        <= 32'd0;
    end else begin
      for (int x = 0; x < 2; x++) begin
        if (i_soft_reset) begin
          reg_mw_fresh[x]        <= 1'b0;
          reg_mw_active_valid[x] <= 1'b0;
        end else begin
          reg_mw_fresh[x] <= w_pend_load[x] || (reg_mw_fresh[x] && !w_mw_sof_x[x]);
          if (w_mw_word_x[x])
            reg_mw_active_valid[x] <= w_mw_act_valid_nxt[x] && !w_buf_leave[w_mw_act_buf_nxt[x]];
          else
            reg_mw_active_valid[x] <= reg_mw_active_valid[x] && !w_buf_leave[reg_mw_active_buf[x]];
        end
        reg_mw_active_buf[x] <= w_mw_act_buf_nxt[x];
        reg_pay_ptr[x]       <= w_pay_ptr_nxt[x];
      end
      reg_pay_beats <= w_pay_beats_nxt;
      reg_pay_addr  <= w_pay_addr_nxt;
      reg_pay_vc    <= w_pay_vc_nxt;
      reg_pay_buf   <= w_pay_buf_nxt;
      reg_pay_open  <= w_pay_open_nxt;
      reg_pay_data  <= w_pay_data_nxt;
      reg_pay_strb  <= w_pay_strb_nxt;
      reg_ra_busy   <= w_ra_busy_nxt;
      reg_rec_beat  <= w_rec_beat_nxt;
      reg_rec_bn    <= w_rec_bn_nxt;
      reg_rec_start <= w_rec_start_nxt;
      reg_rec_addr  <= w_rec_addr_nxt;
      reg_ra_vc     <= w_ra_vc_nxt;
      reg_rec_buf   <= w_rec_buf_nxt;
      if (i_soft_reset && !w_ra_busy_nxt) begin
        reg_ra_cnt        <= 4'd0;
        reg_ra_full       <= 1'b0;
        reg_rec_buf_valid <= 1'b0;
      end else begin
        reg_ra_cnt        <= w_ra_cnt_nxt;
        reg_ra_full       <= w_ra_full_nxt;
        reg_rec_buf_valid <= w_rec_buf_valid_nxt;
      end
    end
  end

  // Record Assembler words; not reset, read only for the words counted since the record started.
  always_ff @(posedge i_sys_clk) begin : p_ff_record_words
    if (w_ra_we) reg_ra_word[w_ra_widx] <= i_meta_data;
  end

  // ------------------------------------------------------------------------------------------
  // Buffer Manager
  // ------------------------------------------------------------------------------------------
  // Configuration sampled at the claim: addresses and sizes with bits 2:0 cleared, max_burst_len
  // clamped to PARA_MAX_BURST_BEATS - 1, attributes, policy, path and Test Pattern Generator
  // height.
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_shadow
    if (!i_resetn) begin
      for (int b = 0; b < 2; b++) begin
        reg_sh_buf_addr[b]       <= 32'd0;
        reg_sh_raw_size[b]       <= 32'd0;
        reg_sh_meta_area_size[b] <= 32'd0;
        reg_sh_bytes_per_line[b] <= 16'd0;
        reg_sh_line_padding[b]   <= 8'd0;
        reg_sh_max_burst_len[b]  <= '0;
        reg_sh_awqos[b]          <= 4'd0;
        reg_sh_awprot[b]         <= 3'd0;
        reg_sh_tpg_height[b]     <= 16'd0;
      end
      reg_sh_policy <= 2'b00;
      reg_sh_tpg    <= 2'b00;
    end else begin
      for (int b = 0; b < 2; b++) begin
        if (w_claim && (w_pick == 1'(b))) begin
          reg_sh_buf_addr[b]       <= ((b == 0) ? i_dma_buffer0_addr
              : i_dma_buffer1_addr) & 32'hFFFF_FFF8;
          reg_sh_raw_size[b]       <= i_dma_buffer_size & 32'hFFFF_FFF8;
          reg_sh_meta_area_size[b] <= i_meta_area_size & 32'hFFFF_FFF8;
          reg_sh_bytes_per_line[b] <= i_bytes_per_line;
          reg_sh_line_padding[b]   <= i_line_padding;
          if ((32'(i_max_burst_len) + 32'd1) > PARA_MAX_BURST_BEATS)
            reg_sh_max_burst_len[b] <= LPARA_MAX_BURST_LEN_WIDTH'(PARA_MAX_BURST_BEATS - 32'd1);
          else
            reg_sh_max_burst_len[b] <= i_max_burst_len;
          reg_sh_awqos[b]  <= i_awqos;
          reg_sh_awprot[b] <= i_awprot;
          reg_sh_policy[b] <= i_err_frame_policy;
          reg_sh_tpg[b]    <= i_active_sel;
          if (i_active_sel) reg_sh_tpg_height[b] <= i_tpg_height_shadow;
        end
      end
    end
  end

  // Completion token: the completion FIFO is read whenever it holds an entry, one token per cycle.
  always_comb begin : p_com_cpl_token
    w_tok_valid = !w_cpl_empty;
    w_cpl_pop   = !w_cpl_empty;
    {w_tok_resp, w_tok_id, w_tok_addr, w_tok_vc, w_tok_line, w_tok_eol} = w_cpl_dout;
    w_axi_write_error = w_tok_valid && w_tok_resp[1];
    for (int b = 0; b < 2; b++) begin
      w_cpl_token[b] = w_tok_valid && (w_tok_id[0] == 1'(b));
      w_axi_err[b]   = w_cpl_token[b] && w_tok_resp[1];
    end
    w_overflow = w_pw_err || (w_wr_err_mw != 2'b00);
  end

  generate
    for (genvar gen_idx = 0; gen_idx < LPARA_N_BUFFERS; gen_idx++) begin : gen_buf
      buf_state_type_def w_fsm_buf_nxt_state;
      logic [15:0] w_exp_lines [0:1];
      logic [1:0]  w_ch_done;
      logic        w_all_done, w_line_err, w_claim_b;

      // Completion: a channel is done when its record has been written and its line count equals
      // the expected count (record line_count, or the Test Pattern Generator height); line error
      // when the count passes it, or stays below it once the channel has moved on to its next
      // frame.
      always_comb begin : p_com_buf_done
        w_claim_b  = w_claim && (w_pick == 1'(gen_idx));
        w_line_err = 1'b0;
        for (int x = 0; x < 2; x++) begin
          w_exp_lines[x] = reg_sh_tpg[gen_idx] ? reg_sh_tpg_height[gen_idx]
              : reg_rec_lines[gen_idx][x];
          w_ch_done[x]   = reg_rec_seen[gen_idx][x] && (reg_lines[gen_idx][x] == w_exp_lines[x]);
          if (reg_rec_seen[gen_idx][x] && ((reg_lines[gen_idx][x] > w_exp_lines[x]) ||
              (reg_moved[gen_idx][x] && (reg_lines[gen_idx][x] < w_exp_lines[x]))))
            w_line_err = 1'b1;
        end
        w_all_done = reg_ended[gen_idx] && (!reg_joined[gen_idx][0] || w_ch_done[0]) &&
                     (!reg_joined[gen_idx][1] || w_ch_done[1]);
        w_abort_req[gen_idx] = w_abort_all || w_bind_drop[gen_idx] || w_wr_err_pw[gen_idx] ||
                               w_wr_err_mw[gen_idx] || w_axi_err[gen_idx] || w_line_err
                                   || !i_dma_enable;
      end

      // Buffer FSM. When two transitions hold in one state, ST_ABORT is taken.
      // soft_reset aborts a buffer in ST_WRITE or ST_DRAIN; a buffer caught by soft_reset leaves
      // ST_ABORT to ST_FREE whatever err_frame_policy (also when soft_reset comes in the exit
      // cycle); software states are kept.
      always_comb begin : p_com_buf_fsm
        w_fsm_buf_nxt_state  = reg_fsm_buf_current_state[gen_idx];
        w_done_ok[gen_idx]   = 1'b0;
        w_done_err[gen_idx]  = 1'b0;
        case (reg_fsm_buf_current_state[gen_idx])
          ENUM_ST_FREE: begin
            if (w_claim_b) w_fsm_buf_nxt_state = ENUM_ST_WRITE;
          end
          ENUM_ST_WRITE: begin
            if (w_abort_req[gen_idx] || i_soft_reset) w_fsm_buf_nxt_state = ENUM_ST_ABORT;
            else if (w_all_done) w_fsm_buf_nxt_state = ENUM_ST_DRAIN;
          end
          ENUM_ST_DRAIN: begin
            if (w_axi_err[gen_idx] || i_soft_reset) w_fsm_buf_nxt_state = ENUM_ST_ABORT;
            else if (reg_inflight[gen_idx] == '0) begin
              w_fsm_buf_nxt_state = ENUM_ST_SW_VALID;
              w_done_ok[gen_idx]  = 1'b1;
            end
          end
          ENUM_ST_ABORT: begin
            if (reg_inflight[gen_idx] == '0) begin
              if (!reg_sh_policy[gen_idx] || reg_soft_reset_seen[gen_idx] || i_soft_reset) begin
                w_fsm_buf_nxt_state = ENUM_ST_FREE;
              end else begin
                w_fsm_buf_nxt_state = ENUM_ST_SW_ERROR;
                w_done_err[gen_idx] = 1'b1;
              end
            end
          end
          ENUM_ST_SW_VALID, ENUM_ST_SW_ERROR: begin
            if (i_release_buf[gen_idx]) w_fsm_buf_nxt_state = ENUM_ST_FREE;
          end
          default: w_fsm_buf_nxt_state = ENUM_ST_FREE;
        endcase
        w_buf_leave[gen_idx] = (reg_fsm_buf_current_state[gen_idx] == ENUM_ST_WRITE) &&
                               (w_fsm_buf_nxt_state != ENUM_ST_WRITE);
      end

      // State, soft_reset_seen and writes in flight (issued commands minus completion tokens).
      always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_buf_fsm
        if (!i_resetn) begin
          reg_fsm_buf_current_state[gen_idx] <= ENUM_ST_FREE;
          reg_soft_reset_seen[gen_idx]       <= 1'b0;
          reg_inflight[gen_idx]              <= '0;
        end else begin
          reg_fsm_buf_current_state[gen_idx] <= w_fsm_buf_nxt_state;
          if ((reg_fsm_buf_current_state[gen_idx] == ENUM_ST_ABORT)
              && (w_fsm_buf_nxt_state != ENUM_ST_ABORT))
            reg_soft_reset_seen[gen_idx] <= 1'b0;
          else if (i_soft_reset && ((reg_fsm_buf_current_state[gen_idx] == ENUM_ST_WRITE) ||
                   (reg_fsm_buf_current_state[gen_idx] == ENUM_ST_DRAIN) ||
                   (reg_fsm_buf_current_state[gen_idx] == ENUM_ST_ABORT)))
            reg_soft_reset_seen[gen_idx] <= 1'b1;
          reg_inflight[gen_idx] <= reg_inflight[gen_idx] + LPARA_INFL_W'(w_wr_issue_pw[gen_idx]) +
                                   LPARA_INFL_W'(w_wr_issue_mw[gen_idx])
                                       - LPARA_INFL_W'(w_cpl_token[gen_idx]);
        end
      end

      // Completion state of the capture held by the buffer, cleared at its claim (not by
      // soft_reset).
      always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_buf_track
        if (!i_resetn) begin
          reg_joined[gen_idx]   <= 2'b00;
          reg_rec_seen[gen_idx] <= 2'b00;
          reg_moved[gen_idx]    <= 2'b00;
          reg_ended[gen_idx]    <= 1'b0;
          for (int x = 0; x < 2; x++) begin
            reg_rec_lines[gen_idx][x] <= 16'd0;
            reg_lines[gen_idx][x]     <= 16'd0;
          end
        end else if (w_claim_b) begin
          reg_joined[gen_idx]   <= reg_lead_vc ? 2'b10 : 2'b01;
          reg_rec_seen[gen_idx] <= 2'b00;
          reg_moved[gen_idx]    <= 2'b00;
          reg_ended[gen_idx]    <= 1'b0;
          for (int x = 0; x < 2; x++) begin
            reg_rec_lines[gen_idx][x] <= 16'd0;
            reg_lines[gen_idx][x]     <= 16'd0;
          end
        end else begin
          if (w_bind_end[gen_idx]) reg_ended[gen_idx] <= 1'b1;
          for (int x = 0; x < 2; x++) begin
            if (w_bind_join[x] && (w_fs_buf[x] == 1'(gen_idx))) reg_joined[gen_idx][x] <= 1'b1;
            if (w_rec_done && (w_rec_done_buf == 1'(gen_idx)) && (w_rec_done_x == 1'(x))) begin
              reg_rec_seen[gen_idx][x]  <= 1'b1;
              reg_rec_lines[gen_idx][x] <= w_rec_lines;
            end
            if (w_eol_evt && (w_cur_buf == 1'(gen_idx)) && (w_vc == 1'(x)))
              reg_lines[gen_idx][x] <= reg_lines[gen_idx][x] + 16'd1;
            if ((w_sof_evt && (w_vc == 1'(x)) && w_sof_prev_valid && (w_sof_prev_buf
                == 1'(gen_idx))) ||
                (w_miss_evt[x] && (reg_pending_buf[x] == 1'(gen_idx)))) reg_moved[gen_idx][x]
                    <= 1'b1;
          end
        end
      end
    end
  endgenerate

  // Error counters (saturating), first fault since soft_reset, and per buffer the last line up to
  // which every write returned OKAY (stops at the first response that is not OKAY; 0xFFFF from the
  // claim).
  always_ff @(posedge i_sys_clk or negedge i_resetn) begin : p_ff_status
    if (!i_resetn) begin
      reg_bresp_slverr_count <= 16'd0;
      reg_bresp_decerr_count <= 16'd0;
      reg_first_fault        <= 1'b0;
      reg_axi_fault_awid     <= 8'd0;
      reg_axi_fault_resp     <= 2'd0;
      reg_axi_fault_awaddr   <= 32'd0;
      reg_lvl_stop           <= 2'b00;
      reg_lvl_claimed        <= 2'b00;
      for (int b = 0; b < 2; b++) begin
        reg_lvl_has[b] <= 2'b00;
        for (int x = 0; x < 2; x++) reg_lvl[b][x] <= 16'd0;
      end
    end else begin
      if (i_soft_reset) begin
        reg_bresp_slverr_count <= 16'd0;
        reg_bresp_decerr_count <= 16'd0;
        reg_first_fault        <= 1'b0;
        reg_axi_fault_awid     <= 8'd0;
        reg_axi_fault_resp     <= 2'd0;
        reg_axi_fault_awaddr   <= 32'd0;
      end else if (w_axi_write_error) begin
        if ((w_tok_resp == 2'b10) && (reg_bresp_slverr_count != 16'hFFFF))
          reg_bresp_slverr_count <= reg_bresp_slverr_count + 16'd1;
        if ((w_tok_resp == 2'b11) && (reg_bresp_decerr_count != 16'hFFFF))
          reg_bresp_decerr_count <= reg_bresp_decerr_count + 16'd1;
        if (!reg_first_fault) begin
          reg_first_fault      <= 1'b1;
          reg_axi_fault_awid   <= 8'(w_tok_id);
          reg_axi_fault_resp   <= w_tok_resp;
          reg_axi_fault_awaddr <= w_tok_addr;
        end
      end
      for (int b = 0; b < 2; b++) begin
        if (w_claim && (w_pick == 1'(b))) begin
          reg_lvl_claimed[b] <= 1'b1;
          reg_lvl_stop[b]    <= 1'b0;
          reg_lvl_has[b]     <= 2'b00;
        end else if (w_cpl_token[b]) begin
          if (w_tok_resp[1]) reg_lvl_stop[b] <= 1'b1;
          else if (!reg_lvl_stop[b] && !w_tok_id[1] && w_tok_eol) begin
            reg_lvl[b][w_tok_vc]     <= w_tok_line;
            reg_lvl_has[b][w_tok_vc] <= 1'b1;
          end
        end
      end
    end
  end

  // bufN_last_valid_line: 0 before the first claim; 0xFFFF while a joined channel has no such line;
  // else the smaller value over the joined channels.
  always_comb begin : p_com_last_valid_line
    for (int b = 0; b < 2; b++) begin
      w_buf_last_valid_line[b] = 16'hFFFF;
      if (!reg_lvl_claimed[b]) w_buf_last_valid_line[b] = 16'd0;
      else if ((reg_joined[b][0] && !reg_lvl_has[b][0]) || (reg_joined[b][1] && !reg_lvl_has[b][1]))
        w_buf_last_valid_line[b] = 16'hFFFF;
      else if (reg_joined[b][0] && reg_joined[b][1])
        w_buf_last_valid_line[b] = (reg_lvl[b][0] < reg_lvl[b][1]) ? reg_lvl[b][0] : reg_lvl[b][1];
      else if (reg_joined[b][1]) w_buf_last_valid_line[b] = reg_lvl[b][1];
      else w_buf_last_valid_line[b] = reg_lvl[b][0];
    end
  end

  // ------------------------------------------------------------------------------------------
  // Asynchronous FIFOs
  // ------------------------------------------------------------------------------------------
  // RAW command and data, metadata command and data from sys_clk to mm_aclk; completion from
  // mm_aclk to sys_clk.
  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(PARA_CMD_FIFO_DEPTH), .PARA_WIDTH(LPARA_CMD_W), .PARA_ASYNC(PARA_ASYNC_MM_CLK),
    .PARA_SYNC_STAGES(PARA_SYNC_STAGES)
  ) u_raw_cmd_fifo_0 (
    .i_wr_clk(i_sys_clk), .i_wr_rst_n(i_resetn), .i_push(w_raw_cmd_push), .i_din(w_raw_cmd_din),
    .o_full(w_raw_cmd_full), .o_wr_used(),
    .i_rd_clk(i_mm_aclk), .i_rd_rst_n(i_mm_resetn), .i_pop(w_raw_cmd_pop), .o_dout(w_raw_cmd_dout),
    .o_empty(w_raw_cmd_empty)
  );

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(PARA_RAW_FIFO_DEPTH), .PARA_WIDTH(LPARA_DATA_ENTRY_W),
        .PARA_ASYNC(PARA_ASYNC_MM_CLK),
    .PARA_SYNC_STAGES(PARA_SYNC_STAGES)
  ) u_raw_data_fifo_0 (
    .i_wr_clk(i_sys_clk), .i_wr_rst_n(i_resetn), .i_push(w_raw_data_push), .i_din(w_raw_data_din),
    .o_full(w_raw_data_full), .o_wr_used(),
    .i_rd_clk(i_mm_aclk), .i_rd_rst_n(i_mm_resetn), .i_pop(w_raw_data_pop),
        .o_dout(w_raw_data_dout),
    .o_empty(w_raw_data_empty)
  );

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(PARA_CMD_FIFO_DEPTH), .PARA_WIDTH(LPARA_CMD_W), .PARA_ASYNC(PARA_ASYNC_MM_CLK),
    .PARA_SYNC_STAGES(PARA_SYNC_STAGES)
  ) u_meta_cmd_fifo_0 (
    .i_wr_clk(i_sys_clk), .i_wr_rst_n(i_resetn), .i_push(w_meta_cmd_push), .i_din(w_meta_cmd_din),
    .o_full(w_meta_cmd_full), .o_wr_used(),
    .i_rd_clk(i_mm_aclk), .i_rd_rst_n(i_mm_resetn), .i_pop(w_meta_cmd_pop),
        .o_dout(w_meta_cmd_dout),
    .o_empty(w_meta_cmd_empty)
  );

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(PARA_META_DATA_FIFO_DEPTH), .PARA_WIDTH(LPARA_DATA_ENTRY_W),
        .PARA_ASYNC(PARA_ASYNC_MM_CLK),
    .PARA_SYNC_STAGES(PARA_SYNC_STAGES)
  ) u_meta_data_fifo_0 (
    .i_wr_clk(i_sys_clk), .i_wr_rst_n(i_resetn), .i_push(w_meta_data_push), .i_din(w_meta_data_din),
    .o_full(w_meta_data_full), .o_wr_used(),
    .i_rd_clk(i_mm_aclk), .i_rd_rst_n(i_mm_resetn), .i_pop(w_meta_data_pop),
        .o_dout(w_meta_data_dout),
    .o_empty(w_meta_data_empty)
  );

  m_fpt_dma_async_fifo #(
    .PARA_DEPTH(PARA_MAX_OUTSTANDING), .PARA_WIDTH(LPARA_CPL_W), .PARA_ASYNC(PARA_ASYNC_MM_CLK),
    .PARA_SYNC_STAGES(PARA_SYNC_STAGES)
  ) u_cpl_fifo_0 (
    .i_wr_clk(i_mm_aclk), .i_wr_rst_n(i_mm_resetn), .i_push(w_cpl_push), .i_din(w_cpl_din),
    .o_full(w_cpl_full), .o_wr_used(w_cpl_used),
    .i_rd_clk(i_sys_clk), .i_rd_rst_n(i_resetn), .i_pop(w_cpl_pop), .o_dout(w_cpl_dout),
    .o_empty(w_cpl_empty)
  );

  // ------------------------------------------------------------------------------------------
  // AXI Engine
  // ------------------------------------------------------------------------------------------
  // Command selection at burst boundaries: the metadata command first when it can be accepted (a
  // SKIP, or a WRITE while fewer than PARA_MAX_OUTSTANDING writes are in flight), otherwise the RAW
  // command; nothing is taken while the AW register is occupied or the W order queue is full. The W
  // order queue is written when an address is presented and at each SKIP, so W never waits for
  // AWREADY.
  always_comb begin : p_com_axi_cmd
    w_inflight_mm = (LPARA_OUTST_W + 32'd1)'(reg_aw_outst) + (LPARA_OUTST_W + 32'd1)'(w_cpl_used);
    w_issue_room  = (32'(w_inflight_mm) < PARA_MAX_OUTSTANDING);
    w_meta_cmd_ok = !w_meta_cmd_empty && (w_meta_cmd_dout[LPARA_CMD_W-1] || w_issue_room);
    w_raw_cmd_ok  = !w_raw_cmd_empty && (w_raw_cmd_dout[LPARA_CMD_W-1] || w_issue_room);
    w_wq_full     = ((reg_wq_wr_ptr - reg_wq_rd_ptr) == (LPARA_WQ_PTR_W + 32'd1)'(LPARA_WQ_DEPTH));
    w_take_meta   = !reg_awvalid && !w_wq_full && w_meta_cmd_ok;
    w_take_raw    = !reg_awvalid && !w_wq_full && !w_meta_cmd_ok && w_raw_cmd_ok;
    w_sel_cmd     = w_take_meta ? w_meta_cmd_dout : w_raw_cmd_dout;
    w_meta_cmd_pop = w_take_meta;
    w_raw_cmd_pop  = w_take_raw;
    w_wq_push      = w_take_meta || w_take_raw;
    w_take_write   = w_wq_push && !w_sel_cmd[LPARA_CMD_W-1];
  end

  // W path: one whole burst at a time in the order of the queue; a SKIP entry pops its beats
  // without a transfer. With a 32-bit bus each 64-bit beat is sent as two transfers, bits 31:0
  // first.
  always_comb begin : p_com_axi_w
    w_wq_empty    = (reg_wq_wr_ptr == reg_wq_rd_ptr);
    w_head_stream = reg_wq_stream[reg_wq_rd_ptr[LPARA_WQ_PTR_W-1:0]];
    w_head_skip   = reg_wq_skip[reg_wq_rd_ptr[LPARA_WQ_PTR_W-1:0]];
    w_head_beats  = reg_wq_beats[reg_wq_rd_ptr[LPARA_WQ_PTR_W-1:0]];
    w_data_avail  = w_head_stream ? !w_meta_data_empty : !w_raw_data_empty;
    w_head_data   = w_head_stream ? w_meta_data_dout : w_raw_data_dout;
    w_head_last   = (reg_w_cnt == (w_head_beats - 1'b1));
    w_w_free      = !reg_wvalid || i_m_axi4_wready;
    w_w_load      = !w_wq_empty && !w_head_skip && w_w_free && w_data_avail;
    w_axi_last_beat = w_head_last && ((PARA_AXI_DATA_WIDTH == 32'd64) || reg_w_half);
    w_raw_data_pop  = 1'b0;
    w_meta_data_pop = 1'b0;
    w_wq_pop        = 1'b0;
    if (!w_wq_empty && w_head_skip && w_data_avail) begin
      w_raw_data_pop  = !w_head_stream;
      w_meta_data_pop = w_head_stream;
      w_wq_pop        = w_head_last;
    end else if (w_w_load && ((PARA_AXI_DATA_WIDTH == 32'd64) || reg_w_half)) begin
      w_raw_data_pop  = !w_head_stream;
      w_meta_data_pop = w_head_stream;
      w_wq_pop        = w_head_last;
    end
    w_b_hs   = i_m_axi4_bvalid;
    w_b_id   = i_m_axi4_bid[1:0];
    w_cpl_push = w_b_hs && !w_cpl_full;
    w_cpl_din  = {i_m_axi4_bresp, w_b_id, reg_tag_mem[w_b_id][reg_tag_rd_ptr[w_b_id]]};
  end

  // AW register, writes presented and not yet responded, W order queue, W register and burst
  // progress.
  always_ff @(posedge i_mm_aclk or negedge i_mm_resetn) begin : p_ff_axi_engine
    if (!i_mm_resetn) begin
      reg_awvalid   <= 1'b0;
      reg_awid      <= '0;
      reg_awaddr    <= 32'd0;
      reg_awlen     <= 8'd0;
      reg_awqos     <= 4'd0;
      reg_awprot    <= 3'd0;
      reg_aw_outst  <= '0;
      reg_wq_stream <= '0;
      reg_wq_skip   <= '0;
      for (int idx = 0; idx < LPARA_WQ_DEPTH; idx++) reg_wq_beats[idx] <= '0;
      reg_wq_wr_ptr <= '0;
      reg_wq_rd_ptr <= '0;
      reg_w_cnt     <= '0;
      reg_w_half    <= 1'b0;
      reg_wvalid    <= 1'b0;
      reg_wlast     <= 1'b0;
      reg_wdata     <= '0;
      reg_wstrb     <= '0;
      for (int id = 0; id < 4; id++) begin
        reg_tag_wr_ptr[id] <= '0;
        reg_tag_rd_ptr[id] <= '0;
      end
    end else begin
      if (reg_awvalid && i_m_axi4_awready) reg_awvalid <= 1'b0;
      else if (w_take_write) begin
        reg_awvalid <= 1'b1;
        reg_awid    <= PARA_AXI_ID_WIDTH'(w_sel_cmd[26:25]);
        reg_awaddr  <= w_sel_cmd[66:35];
        reg_awlen   <= (PARA_AXI_DATA_WIDTH == 32'd64) ? w_sel_cmd[34:27] : 8'({w_sel_cmd[34:27],
            1'b1});
        reg_awqos   <= w_sel_cmd[24:21];
        reg_awprot  <= w_sel_cmd[20:18];
      end
      reg_aw_outst <= reg_aw_outst + LPARA_OUTST_W'(w_take_write) - LPARA_OUTST_W'(w_b_hs);
      if (w_wq_push) begin
        reg_wq_stream[reg_wq_wr_ptr[LPARA_WQ_PTR_W-1:0]] <= w_take_meta;
        reg_wq_skip[reg_wq_wr_ptr[LPARA_WQ_PTR_W-1:0]]   <= w_sel_cmd[LPARA_CMD_W-1];
        reg_wq_beats[reg_wq_wr_ptr[LPARA_WQ_PTR_W-1:0]]  <= LPARA_BEATS_W'(w_sel_cmd[34:27]) + 1'b1;
        reg_wq_wr_ptr <= reg_wq_wr_ptr + 1'b1;
      end
      if (w_wq_pop) reg_wq_rd_ptr <= reg_wq_rd_ptr + 1'b1;
      if (!w_wq_empty && w_head_skip && w_data_avail) reg_w_cnt <= w_head_last ? '0 : (reg_w_cnt
          + 1'b1);
      if (w_w_load) begin
        reg_wvalid <= 1'b1;
        reg_wlast  <= w_axi_last_beat;
        if (PARA_AXI_DATA_WIDTH == 32'd64) begin
          reg_wdata <= PARA_AXI_DATA_WIDTH'(w_head_data[71:8]);
          reg_wstrb <= (PARA_AXI_DATA_WIDTH/8)'(w_head_data[7:0]);
          reg_w_cnt <= w_head_last ? '0 : (reg_w_cnt + 1'b1);
        end else begin
          reg_wdata  <= PARA_AXI_DATA_WIDTH'(reg_w_half ? w_head_data[71:40] : w_head_data[39:8]);
          reg_wstrb  <= (PARA_AXI_DATA_WIDTH/8)'(reg_w_half ? w_head_data[7:4] : w_head_data[3:0]);
          reg_w_half <= !reg_w_half;
          if (reg_w_half) reg_w_cnt <= w_head_last ? '0 : (reg_w_cnt + 1'b1);
        end
      end else if (w_w_free) begin
        reg_wvalid <= 1'b0;
      end
      if (w_take_write)
        reg_tag_wr_ptr[w_sel_cmd[26:25]] <= reg_tag_wr_ptr[w_sel_cmd[26:25]] + 1'b1;
      if (w_cpl_push) reg_tag_rd_ptr[w_b_id] <= reg_tag_rd_ptr[w_b_id] + 1'b1;
    end
  end

  // Tag FIFO of each identifier {address, channel, line, end of line} for its writes in flight; not
  // reset, an entry is read only after its write was presented.
  always_ff @(posedge i_mm_aclk) begin : p_ff_tag_mem
    if (w_take_write)
      reg_tag_mem[w_sel_cmd[26:25]][reg_tag_wr_ptr[w_sel_cmd[26:25]]] <=
        {w_sel_cmd[66:35], w_sel_cmd[17:0]};
  end

  // ------------------------------------------------------------------------------------------
  // Outputs
  // ------------------------------------------------------------------------------------------
  assign o_fbd_ready            = w_fbd_ready;
  assign o_meta_ready           = w_meta_ready;
  assign o_dma_buf_claim        = w_claim;
  assign o_dma_buf_index        = w_pick;
  assign o_dma_drop             = w_dma_drop;
  assign o_dma_buf_done_ok      = |w_done_ok;
  assign o_dma_buf_done_err     = |w_done_err;
  assign o_dma_axi_write_error  = w_axi_write_error;
  assign o_dma_buffer_overflow  = w_overflow;
  assign o_dma_busy             = w_dma_busy;
  assign o_active_buffer        = reg_active_buffer;
  assign o_buf0_state           = w_buf_state_code[0];
  assign o_buf1_state           = w_buf_state_code[1];
  assign o_buf0_last_valid_line = w_buf_last_valid_line[0];
  assign o_buf1_last_valid_line = w_buf_last_valid_line[1];
  assign o_axi_fault_awid       = reg_axi_fault_awid;
  assign o_axi_fault_resp       = reg_axi_fault_resp;
  assign o_axi_fault_awaddr     = reg_axi_fault_awaddr;
  assign o_bresp_slverr_count   = reg_bresp_slverr_count;
  assign o_bresp_decerr_count   = reg_bresp_decerr_count;
  assign o_m_axi4_awid          = reg_awid;
  assign o_m_axi4_awaddr        = reg_awaddr;
  assign o_m_axi4_awlen         = reg_awlen;
  assign o_m_axi4_awsize        = LPARA_AWSIZE;
  assign o_m_axi4_awburst       = 2'b01;
  assign o_m_axi4_awlock        = 1'b0;
  assign o_m_axi4_awcache       = 4'b0011;
  assign o_m_axi4_awprot        = reg_awprot;
  assign o_m_axi4_awqos         = reg_awqos;
  assign o_m_axi4_awvalid       = reg_awvalid;
  assign o_m_axi4_wdata         = reg_wdata;
  assign o_m_axi4_wstrb         = reg_wstrb;
  assign o_m_axi4_wlast         = reg_wlast;
  assign o_m_axi4_wvalid        = reg_wvalid;
  assign o_m_axi4_bready        = 1'b1;

endmodule
`default_nettype wire
