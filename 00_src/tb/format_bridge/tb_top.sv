// Block: Format Bridge. Select a test with +TEST=<name> and a seed with +SEED=<n>.
// Inputs share one clock. A valid input entry stays stable until accepted or cleared.
// RAW12 entries contain two or four pixels; RAW10 entries contain four zero-extended pixels.
// Unused pixel lanes are zero. Data Type and Virtual Channel stay constant within a line.
// Frame start accompanies line start; the alignment error accompanies line end.
// Format changes only while its path is disabled. Enables control writes, never FIFO reads.
// Both formatters consume every accepted entry. A disabled path does not wait for frame start.
// Full FIFOs reject writes even with a simultaneous read. Clear overrides all edge updates.
// Output fields are checked only when valid; padding bytes of a valid last beat are zero.
`timescale 1ns/1ps
`default_nettype none

module tb_top;
  parameter logic [31:0] PARA_STREAM_FIFO_DEPTH = 16;
  parameter logic [31:0] PARA_DMA_FIFO_DEPTH = 8;
  localparam int MAX_DEPTH = (PARA_STREAM_FIFO_DEPTH > PARA_DMA_FIFO_DEPTH) ?
                            PARA_STREAM_FIFO_DEPTH : PARA_DMA_FIFO_DEPTH;
  localparam int WAIT_LIMIT = 100000 + 100 * MAX_DEPTH;
  localparam int TIMEOUT_CYCLES = 5000000;

  logic i_sys_clk = 0;
  logic i_resetn = 1;
  logic i_soft_reset = 0;
  logic i_fb_flush = 0;
  logic [47:0] i_fb_px_data = 0;
  logic i_fb_px_valid = 0;
  logic [2:0] i_fb_px_valid_count = 0;
  logic i_fb_px_sof = 0, i_fb_px_sol = 0, i_fb_px_eol = 0;
  logic [1:0] i_fb_px_vc_tag = 0;
  logic i_fb_px_dt_sel = 0, i_fb_err_wc_align = 0;
  logic i_stream_format = 0, i_dma_format = 0;
  logic i_axis_enable = 1, i_dma_enable = 1;
  logic i_fbs_ready = 1, i_fbd_ready = 1;
  wire o_fb_ready, o_fb_overflow;
  wire o_stream_fifo_almost_full, o_dma_fifo_almost_full;
  wire [63:0] o_fbs_data, o_fbd_data;
  wire [7:0] o_fbs_byte_en, o_fbd_byte_en;
  wire o_fbs_valid, o_fbs_sof, o_fbs_eol, o_fbs_err;
  wire o_fbd_valid, o_fbd_sof, o_fbd_sol, o_fbd_eol, o_fbd_err;
  wire [1:0] o_fbs_vc_tag, o_fbd_vc_tag;

  m_fpt_format_bridge #(
    .PARA_STREAM_FIFO_DEPTH(PARA_STREAM_FIFO_DEPTH),
    .PARA_DMA_FIFO_DEPTH(PARA_DMA_FIFO_DEPTH)
  ) u_dut (
    .i_sys_clk(i_sys_clk), .i_resetn(i_resetn),
    .i_soft_reset(i_soft_reset), .i_fb_flush(i_fb_flush),
    .i_fb_px_data(i_fb_px_data), .i_fb_px_valid(i_fb_px_valid),
    .i_fb_px_valid_count(i_fb_px_valid_count),
    .i_fb_px_sof(i_fb_px_sof), .i_fb_px_sol(i_fb_px_sol), .i_fb_px_eol(i_fb_px_eol),
    .i_fb_px_vc_tag(i_fb_px_vc_tag), .i_fb_px_dt_sel(i_fb_px_dt_sel),
    .i_fb_err_wc_align(i_fb_err_wc_align), .o_fb_ready(o_fb_ready),
    .i_stream_format(i_stream_format), .i_dma_format(i_dma_format),
    .i_axis_enable(i_axis_enable), .i_dma_enable(i_dma_enable),
    .o_fbs_data(o_fbs_data), .o_fbs_valid(o_fbs_valid), .o_fbs_byte_en(o_fbs_byte_en),
    .o_fbs_sof(o_fbs_sof), .o_fbs_eol(o_fbs_eol),
    .o_fbs_vc_tag(o_fbs_vc_tag), .o_fbs_err(o_fbs_err), .i_fbs_ready(i_fbs_ready),
    .o_fbd_data(o_fbd_data), .o_fbd_valid(o_fbd_valid), .o_fbd_byte_en(o_fbd_byte_en),
    .o_fbd_sof(o_fbd_sof), .o_fbd_sol(o_fbd_sol), .o_fbd_eol(o_fbd_eol),
    .o_fbd_vc_tag(o_fbd_vc_tag), .o_fbd_err(o_fbd_err), .i_fbd_ready(i_fbd_ready),
    .o_fb_overflow(o_fb_overflow), .o_stream_fifo_almost_full(o_stream_fifo_almost_full),
    .o_dma_fifo_almost_full(o_dma_fifo_almost_full)
  );

  typedef struct packed {
    logic [47:0] data;
    logic [2:0] count;
    logic sof, sol, eol;
    logic [1:0] vc;
    logic dt, err;
  } input_entry_t;
  typedef struct packed {
    logic [63:0] data;
    logic [7:0] mask;
    logic sof, sol, eol;
    logic [1:0] vc;
    logic err;
  } beat_t;
  typedef logic [7:0] byte_t;
  typedef byte_t byte_queue_t[$];

  input_entry_t source_queue[$];
  input_entry_t current_entry, previous_entry;
  bit source_valid = 0, last_taken = 0, last_eol = 0;
  bit driver_started = 0, finished = 0, check_started = 0;
  bit checker_fault = 0;
  int gap_percent = 0;
  int ready_percent[2] = '{100, 100};
  int low_remaining[2] = '{0, 0};
  int low_max[2] = '{0, 0};
  int errors = 0, checks = 0, cycle = 0, reset_edges = 0;
  int seed = 1;
  string test_name;

  byte_queue_t native_bytes, expanded_bytes;
  beat_t stream_expected[$], dma_expected[$];
  bit pending_sof[2], pending_sol[2], pending_eol[2];
  logic [1:0] line_vc;
  bit line_dt, line_error, line_open;
  bit previous_input_stall = 0, previous_output_stall[2];
  bit previous_clear = 1;
  bit previous_enable[2], previous_format[2];
  beat_t previous_output[2];

  longint takes = 0, compared_beats[2] = '{0, 0};
  longint read_beats[2] = '{0, 0}, write_beats[2] = '{0, 0};
  longint af_cycles[2] = '{0, 0}, full_cycles[2] = '{0, 0};
  longint overflow_cycles[2] = '{0, 0};
  longint output_stalls[2] = '{0, 0};
  longint eol_waits = 0, body_waits = 0, first_waits[2] = '{0, 0};
  longint clear_events[3] = '{0, 0, 0};
  longint clear_take = 0, clear_flush = 0, clear_full_write = 0, clear_read = 0;
  bit [7:0] native_residuals = 0;
  bit [1:0] expanded_residuals = 0;
  bit [3:0] format_seen = 0, transition_seen = 0;
  int previous_count = 0;

  always #5 i_sys_clk = ~i_sys_clk;

  task automatic fail(input string message);
    errors++;
    if (errors <= 20) $error("%s cycle=%0d: %s", test_name, cycle, message);
  endtask

  task automatic require_true(input bit condition, input string message);
    checks++;
    if (!condition) fail(message);
  endtask

  task automatic compare_bit(input string name, input logic actual, input logic expected);
    checks++;
    if (actual !== expected) fail($sformatf("%s actual=%b expected=%b", name, actual, expected));
  endtask

  task automatic tick();
    @(negedge i_sys_clk);
    #2;
  endtask

  function automatic input_entry_t sampled_input();
    input_entry_t entry;
    entry.data = i_fb_px_data;
    entry.count = i_fb_px_valid_count;
    entry.sof = i_fb_px_sof;
    entry.sol = i_fb_px_sol;
    entry.eol = i_fb_px_eol;
    entry.vc = i_fb_px_vc_tag;
    entry.dt = i_fb_px_dt_sel;
    entry.err = i_fb_err_wc_align;
    return entry;
  endfunction

  function automatic beat_t sampled_output(input int path);
    beat_t value;
    value = '0;
    if (path == 0) begin
      value.data = o_fbs_data;
      value.mask = o_fbs_byte_en;
      value.sof = o_fbs_sof;
      value.eol = o_fbs_eol;
      value.vc = o_fbs_vc_tag;
      value.err = o_fbs_err;
      if (checker_fault) value.data[0] = ~value.data[0];
    end else begin
      value.data = o_fbd_data;
      value.mask = o_fbd_byte_en;
      value.sof = o_fbd_sof;
      value.sol = o_fbd_sol;
      value.eol = o_fbd_eol;
      value.vc = o_fbd_vc_tag;
      value.err = o_fbd_err;
    end
    return value;
  endfunction

  task automatic clear_model();
    native_bytes.delete();
    expanded_bytes.delete();
    stream_expected.delete();
    dma_expected.delete();
    foreach (pending_sof[f]) begin
      pending_sof[f] = 0;
      pending_sol[f] = 0;
      pending_eol[f] = 0;
      previous_output_stall[f] = 0;
    end
    line_vc = 0;
    line_dt = 0;
    line_error = 0;
    line_open = 0;
    previous_input_stall = 0;
    previous_clear = 1;
    previous_count = 0;
    last_taken = 0;
    last_eol = 0;
  endtask

  task automatic drive_idle();
    i_fb_px_valid = 0;
    i_fb_px_data = 'x;
    i_fb_px_valid_count = 'x;
    i_fb_px_sof = 'x;
    i_fb_px_sol = 'x;
    i_fb_px_eol = 'x;
    i_fb_px_vc_tag = 'x;
    i_fb_px_dt_sel = 'x;
    i_fb_err_wc_align = 'x;
  endtask

  task automatic discard_source();
    source_queue.delete();
    source_valid = 0;
    drive_idle();
  endtask

  // The producer advances only after an observed handshake, or a coordinated clear.
  always @(negedge i_sys_clk) begin : source_driver
    #1;
    if (driver_started && !finished) begin
      if (!i_resetn || i_soft_reset || i_fb_flush) discard_source();
      else begin
        if (last_taken) source_valid = 0;
        if (!source_valid && source_queue.size() != 0 && $urandom_range(99, 0) >= gap_percent)
          begin
            current_entry = source_queue.pop_front();
            source_valid = 1;
          end
        if (source_valid) begin
          i_fb_px_valid = 1;
          i_fb_px_data = current_entry.data;
          i_fb_px_valid_count = current_entry.count;
          i_fb_px_sof = current_entry.sof;
          i_fb_px_sol = current_entry.sol;
          i_fb_px_eol = current_entry.eol;
          i_fb_px_vc_tag = current_entry.vc;
          i_fb_px_dt_sel = current_entry.dt;
          i_fb_err_wc_align = current_entry.err;
        end else drive_idle();
      end
      for (int path = 0; path < 2; path++) begin
        bit ready_value;
        if (low_remaining[path] > 0) begin
          low_remaining[path]--;
          ready_value = 0;
        end else if (low_max[path] > 0 && $urandom_range(15, 0) == 0) begin
          low_remaining[path] = int'($urandom_range(low_max[path], 1));
          ready_value = 0;
        end else ready_value = $urandom_range(99, 0) < ready_percent[path];
        if (path == 0) i_fbs_ready = ready_value;
        else i_fbd_ready = ready_value;
      end
    end
  end

  // Pixel groups become ordered byte sequences, independent of implementation storage.
  task automatic append_pixels(input input_entry_t entry);
    logic [11:0] pixels[4];
    for (int p = 0; p < 4; p++) pixels[p] = entry.data[12*p +: 12];
    if (!line_dt) begin
      for (int p = 0; p < int'(entry.count); p += 2) begin
        native_bytes.push_back(pixels[p][11:4]);
        native_bytes.push_back(pixels[p+1][11:4]);
        native_bytes.push_back({pixels[p+1][3:0], pixels[p][3:0]});
      end
    end else begin
      for (int p = 0; p < 4; p++) native_bytes.push_back(pixels[p][9:2]);
      native_bytes.push_back({pixels[3][1:0], pixels[2][1:0], pixels[1][1:0], pixels[0][1:0]});
    end
    for (int p = 0; p < int'(entry.count); p++) begin
      expanded_bytes.push_back(pixels[p][7:0]);
      expanded_bytes.push_back({4'b0, pixels[p][11:8]});
    end
  endtask

  task automatic form_beat(ref byte_queue_t bytes, input int formatter,
                           output beat_t value);
    int count;
    byte_t octet;
    value = '0;
    count = (bytes.size() > 8) ? 8 : bytes.size();
    for (int b = 0; b < count; b++) begin
      octet = bytes.pop_front();
      value.data[8*b +: 8] = octet;
      value.mask[b] = 1;
    end
    value.sof = pending_sof[formatter];
    value.sol = pending_sol[formatter];
    value.eol = pending_eol[formatter] && bytes.size() == 0;
    value.vc = line_vc;
    value.err = value.eol && line_error;
    pending_sof[formatter] = 0;
    pending_sol[formatter] = 0;
    if (value.eol) pending_eol[formatter] = 0;
  endtask

  task automatic check_beat(input int path, input beat_t expected);
    beat_t actual;
    actual = sampled_output(path);
    if (path == 0) expected.sol = 0;
    compared_beats[path]++;
    checks++;
    if (actual !== expected) begin
      fail($sformatf("path=%0d beat actual=%h expected=%h", path, actual, expected));
    end
    require_true(!$isunknown(actual), $sformatf("path=%0d unknown valid beat", path));
    require_true(actual.mask != 0 && ((int'(actual.mask) + 1) & int'(actual.mask)) == 0,
                 "invalid byte enable shape");
    require_true(actual.eol || actual.mask == 8'hff, "partial non-final beat");
    require_true(!actual.err || actual.eol, "error without line end");
    if (path == 1) require_true(!actual.sof || actual.sol, "frame start without line start");
  endtask

  // Compare the state before the edge, then predict the next state from accepted input bytes.
  always @(posedge i_sys_clk) begin : monitor
    input_entry_t entry;
    beat_t generated[2], selected[2], unused;
    bit emitted[2], old_flush[2], full[2], almost_full[2], expected_ready;
    bit enabled[2], selected_format[2], ready[2], attempted[2], overflow[2];
    bit accept_input, clear_now;
    cycle++;
    if (!i_resetn) begin
      reset_edges++;
      if (reset_edges >= 2) check_started = 1;
      last_taken = 0;
      last_eol = 0;
      if (check_started) begin
        compare_bit("stream valid during reset", o_fbs_valid, 0);
        compare_bit("DMA valid during reset", o_fbd_valid, 0);
        compare_bit("overflow during reset", o_fb_overflow, 0);
      end
    end else if (check_started && !finished) begin
      clear_now = i_soft_reset || i_fb_flush;
      enabled[0] = i_axis_enable;
      enabled[1] = i_dma_enable;
      selected_format[0] = i_stream_format;
      selected_format[1] = i_dma_format;
      ready[0] = i_fbs_ready;
      ready[1] = i_fbd_ready;
      full[0] = stream_expected.size() == int'(PARA_STREAM_FIFO_DEPTH);
      full[1] = dma_expected.size() == int'(PARA_DMA_FIFO_DEPTH);
      almost_full[0] = stream_expected.size() >= int'(PARA_STREAM_FIFO_DEPTH) - 1;
      almost_full[1] = dma_expected.size() >= int'(PARA_DMA_FIFO_DEPTH) - 1;
      old_flush[0] = pending_eol[0] && native_bytes.size() != 0;
      old_flush[1] = pending_eol[1] && expanded_bytes.size() != 0;
      expected_ready = !(old_flush[0] || old_flush[1]) &&
                       !(enabled[0] && almost_full[0]) && !(enabled[1] && almost_full[1]);
      compare_bit("fb_ready", o_fb_ready, expected_ready);
      compare_bit("stream valid", o_fbs_valid, stream_expected.size() != 0);
      compare_bit("DMA valid", o_fbd_valid, dma_expected.size() != 0);
      compare_bit("stream almost_full", o_stream_fifo_almost_full, almost_full[0]);
      compare_bit("DMA almost_full", o_dma_fifo_almost_full, almost_full[1]);
      require_true(!$isunknown({o_fb_ready, o_fbs_valid, o_fbd_valid, o_fb_overflow,
                                o_stream_fifo_almost_full, o_dma_fifo_almost_full}),
                   "unknown control output");
      if (stream_expected.size() != 0) check_beat(0, stream_expected[0]);
      if (dma_expected.size() != 0) check_beat(1, dma_expected[0]);
      for (int path = 0; path < 2; path++) begin
        bit valid_now;
        valid_now = path == 0 ? o_fbs_valid : o_fbd_valid;
        if (previous_output_stall[path] && !previous_clear) begin
          require_true(valid_now, "valid dropped while stalled");
          require_true(sampled_output(path) === previous_output[path], "stalled beat changed");
        end
        previous_output[path] = sampled_output(path);
        previous_output_stall[path] = valid_now && !ready[path];
        if (previous_output_stall[path]) output_stalls[path]++;
        if (almost_full[path]) af_cycles[path]++;
        if (full[path]) full_cycles[path]++;
        if (previous_enable[path] && enabled[path])
          require_true(previous_format[path] == selected_format[path], "format changed enabled");
        previous_enable[path] = enabled[path];
        previous_format[path] = selected_format[path];
      end
      entry = sampled_input();
      if (previous_input_stall && !previous_clear)
        require_true(i_fb_px_valid && entry === previous_entry, "input changed while stalled");
      previous_entry = entry;
      previous_input_stall = i_fb_px_valid && !o_fb_ready;
      if (i_fb_px_valid) begin
        require_true(!$isunknown(entry), "unknown valid input entry");
        require_true(entry.dt ? entry.count == 4 : (entry.count == 2 || entry.count == 4),
                     "illegal input pixel count");
        require_true(!entry.sof || entry.sol, "input frame start without line start");
        require_true(!entry.err || entry.eol, "input alignment error without line end");
        if (entry.count == 2) require_true(entry.data[47:24] == 0, "unused input lanes not zero");
        if (!expected_ready && entry.eol) eol_waits++;
        if (!expected_ready && !entry.eol && !entry.sol) body_waits++;
        if ((old_flush[0] || old_flush[1]) && entry.sol) first_waits[int'(entry.sof)]++;
      end
      accept_input = i_fb_px_valid && expected_ready;
      last_taken = i_fb_px_valid && (o_fb_ready === 1'b1);
      last_eol = last_taken && entry.eol;
      if (accept_input) begin
        takes++;
        if (!clear_now) begin
          require_true(entry.sol ? !line_open : line_open, "input line structure");
          if (!entry.sol)
            require_true(entry.dt == line_dt && entry.vc == line_vc, "input line tag changed");
        end
        if (entry.sol) begin line_dt = entry.dt; line_vc = entry.vc; line_open = 1; end
        if (entry.eol) begin line_error = entry.err; line_open = 0; end
        for (int f = 0; f < 2; f++) begin
          pending_sof[f] |= entry.sof;
          pending_sol[f] |= entry.sol;
          pending_eol[f] |= entry.eol;
        end
        append_pixels(entry);
        format_seen[{i_stream_format, i_dma_format}] = 1;
        if (!entry.dt && previous_count != 0)
          transition_seen[(previous_count == 4 ? 2 : 0) + (entry.count == 4 ? 1 : 0)] = 1;
        previous_count = entry.dt ? 0 : int'(entry.count);
        if (entry.eol) begin
          native_residuals[native_bytes.size() % 8] = 1;
          expanded_residuals[(expanded_bytes.size() % 8) == 4 ? 1 : 0] = 1;
        end
      end
      emitted[0] = old_flush[0] || native_bytes.size() >= 8;
      emitted[1] = old_flush[1] || expanded_bytes.size() >= 8;
      generated[0] = '0;
      generated[1] = '0;
      if (emitted[0]) form_beat(native_bytes, 0, generated[0]);
      if (emitted[1]) form_beat(expanded_bytes, 1, generated[1]);
      for (int path = 0; path < 2; path++) begin
        attempted[path] = enabled[path] && emitted[int'(selected_format[path])];
        selected[path] = generated[int'(selected_format[path])];
        overflow[path] = attempted[path] && full[path];
        if (overflow[path]) overflow_cycles[path]++;
      end
      compare_bit("overflow", o_fb_overflow, overflow[0] || overflow[1]);
      if (clear_now) begin
        if (i_soft_reset) clear_events[1]++;
        if (i_fb_flush) clear_events[2]++;
        if (accept_input) clear_take++;
        if (old_flush[0] || old_flush[1]) clear_flush++;
        if (overflow[0] || overflow[1]) clear_full_write++;
        if ((o_fbs_valid && ready[0]) || (o_fbd_valid && ready[1])) clear_read++;
        clear_model();
      end else begin
        if (stream_expected.size() != 0 && ready[0]) begin
          unused = stream_expected.pop_front(); read_beats[0]++;
        end
        if (dma_expected.size() != 0 && ready[1]) begin
          unused = dma_expected.pop_front(); read_beats[1]++;
        end
        if (attempted[0] && !full[0]) begin
          selected[0].sol = 0;
          stream_expected.push_back(selected[0]); write_beats[0]++;
        end
        if (attempted[1] && !full[1]) begin
          dma_expected.push_back(selected[1]); write_beats[1]++;
        end
      end
      previous_clear = clear_now;
    end
  end

  always @(negedge i_resetn) begin
    clear_model();
    if (check_started) clear_events[0]++;
    #0.1;
    if (check_started) begin
      compare_bit("asynchronous stream valid", o_fbs_valid, 0);
      compare_bit("asynchronous DMA valid", o_fbd_valid, 0);
      compare_bit("asynchronous overflow", o_fb_overflow, 0);
    end
  end

  task automatic set_readies(input int stream_percent, input int dma_percent,
                             input int stream_low = 0, input int dma_low = 0);
    ready_percent[0] = stream_percent;
    ready_percent[1] = dma_percent;
    low_max[0] = stream_low;
    low_max[1] = dma_low;
    low_remaining[0] = 0;
    low_remaining[1] = 0;
    i_fbs_ready = stream_percent != 0;
    i_fbd_ready = dma_percent != 0;
  endtask

  task automatic set_paths(input bit stream_enable, input bit dma_enable,
                           input bit stream_format, input bit dma_format);
    i_axis_enable = 0;
    i_dma_enable = 0;
    tick();
    i_stream_format = stream_format;
    i_dma_format = dma_format;
    tick();
    i_axis_enable = stream_enable;
    i_dma_enable = dma_enable;
  endtask

  // Enqueue a complete legal line. Pattern four reserves four pixels for the last entry.
  task automatic enqueue_line(input int pixels, input bit dt, input logic [1:0] vc,
                               input bit sof, input bit error_at_end, input int pattern);
    int offset, count;
    input_entry_t entry;
    require_true(pixels > 0 && pixels % (dt ? 4 : 2) == 0, "bad line length in stimulus");
    offset = 0;
    while (offset < pixels) begin
      if (dt) count = 4;
      else case (pattern)
        0: count = (offset % 8 == 4) ? 4 : 2;
        1: count = 2;
        2: count = 4;
        4: count = (pixels - offset == 4) ? 4 : 2;
        default: count = $urandom_range(1, 0) ? 4 : 2;
      endcase
      if (count > pixels - offset) count = pixels - offset;
      entry = '0;
      entry.count = 3'(count);
      entry.sol = offset == 0;
      entry.sof = entry.sol && sof;
      entry.eol = offset + count == pixels;
      entry.err = entry.eol && error_at_end;
      entry.dt = dt;
      entry.vc = vc;
      for (int p = 0; p < count; p++)
        entry.data[12*p +: 12] = 12'($urandom_range(dt ? 1023 : 4095, 0));
      source_queue.push_back(entry);
      offset += count;
    end
  endtask

  function automatic bit all_idle();
    return source_queue.size() == 0 && !source_valid && native_bytes.size() == 0 &&
           expanded_bytes.size() == 0 && stream_expected.size() == 0 && dma_expected.size() == 0;
  endfunction

  task automatic wait_idle(input int limit = WAIT_LIMIT);
    int waited;
    waited = 0;
    do begin tick(); waited++; end while (!all_idle() && waited < limit);
    require_true(all_idle(), "drain deadline exceeded");
    repeat (3) tick();
  endtask

  task automatic wait_takes(input int number);
    longint target;
    int waited;
    target = takes + number;
    waited = 0;
    while (takes < target && waited < WAIT_LIMIT) begin tick(); waited++; end
    require_true(takes >= target, "input acceptance deadline exceeded");
  endtask

  task automatic wait_end();
    int waited;
    waited = 0;
    do begin tick(); waited++; end while (!last_eol && waited < WAIT_LIMIT);
    require_true(last_eol, "end-of-line deadline exceeded");
  endtask

  task automatic reset_dut();
    #1;
    discard_source();
    i_resetn = 0;
    i_soft_reset = 0;
    i_fb_flush = 0;
    repeat (3) tick();
    i_resetn = 1;
    tick();
  endtask

  task automatic clear_datapath(input int kind, input int duration = 1);
    if (kind == 0) reset_dut();
    else begin
      if (kind == 1) i_soft_reset = 1;
      else i_fb_flush = 1;
      repeat (duration) tick();
      i_soft_reset = 0;
      i_fb_flush = 0;
    end
  endtask

  // End a bypass line with a drain plus a residual to occupy the last two FIFO entries.
  task automatic fill_path(input int path);
    int depth, pixels, waited;
    depth = path == 0 ? int'(PARA_STREAM_FIFO_DEPTH) : int'(PARA_DMA_FIFO_DEPTH);
    set_paths(path == 0, path == 1, 0, 0);
    set_readies(path == 0 ? 0 : 100, path == 1 ? 0 : 100);
    gap_percent = 0;
    pixels = 2 * ((8 * depth - 3) / 3);
    enqueue_line(pixels, 0, 0, 1, 0, 4);
    waited = 0;
    do begin tick(); waited++; end while (
      (path == 0 ? stream_expected.size() : dma_expected.size()) != depth && waited < WAIT_LIMIT);
    require_true((path == 0 ? stream_expected.size() : dma_expected.size()) == depth,
                 "target FIFO did not fill");
    require_true(source_queue.size() == 0 && !source_valid, "fill left an unfinished input line");
    tick();
  endtask

  // Reset, idle controls, one short frame and another reset.
  task automatic test_reset_idle();
    repeat (20) tick();
    enqueue_line(16, 0, 0, 1, 0, 0);
    wait_idle();
    reset_dut();
    repeat (20) tick();
  endtask

  // Native RAW12 byte order, all residuals, single-entry lines and every intake transition.
  task automatic test_bypass_raw12();
    set_paths(1, 1, 0, 0);
    for (int pattern = 0; pattern < 4; pattern++) begin
      gap_percent = pattern == 3 ? 30 : 0;
      for (int n = 2; n <= 40; n += 2)
        enqueue_line(n, 0, 2'(pattern), n == 2, 0, pattern);
      wait_idle();
    end
    require_true(native_residuals == 8'hff, "missing RAW12 residual");
    require_true(transition_seen == 4'hf, "missing RAW12 intake transition");
  endtask

  // Native RAW10 byte order, every residual and back-to-back one-entry lines.
  task automatic test_bypass_raw10();
    set_paths(1, 1, 0, 0);
    for (int phase = 0; phase < 2; phase++) begin
      gap_percent = phase == 0 ? 0 : 35;
      for (int n = 4; n <= 64; n += 4) enqueue_line(n, 1, 2'(n/4), n == 4, 0, 2);
      enqueue_line(4, 1, 0, 1, 0, 2);
      enqueue_line(4, 1, 1, 0, 0, 2);
      wait_idle();
    end
    require_true(native_residuals == 8'hff, "missing RAW10 residual");
  endtask

  // Right-aligned 16-bit containers, complete beats and four-byte final residuals.
  task automatic test_raw16();
    set_paths(1, 1, 1, 1);
    for (int pattern = 0; pattern < 4; pattern++) begin
      gap_percent = pattern == 3 ? 30 : 0;
      for (int n = 2; n <= 32; n += 2) enqueue_line(n, 0, 2'(pattern), n == 2, 0, pattern);
      wait_idle();
    end
    for (int n = 4; n <= 48; n += 4) enqueue_line(n, 1, 3, n == 4, 0, 2);
    wait_idle();
    require_true(expanded_residuals == 2'b11, "missing expanded residual");
  endtask

  // All independent format selections preserve data and sidebands from the selected formatter.
  task automatic test_format_combinations();
    for (int combo = 0; combo < 4; combo++) begin
      set_paths(1, 1, combo[1], combo[0]);
      for (int n = 2; n <= 32; n += 2) enqueue_line(n, 0, 2'(n), n == 2, n == 6, 0);
      for (int n = 4; n <= 32; n += 4) enqueue_line(n, 1, 2'(n/4), n == 4, n == 12, 2);
      wait_idle();
    end
    require_true(format_seen == 4'hf, "missing format combination");
  endtask

  // Alternate Data Type and Virtual Channel with four pixels on each first entry.
  task automatic test_line_attributes();
    for (int phase = 0; phase < 3; phase++) begin
      set_paths(1, 1, phase == 1, phase == 0);
      for (int line = 0; line < 24; line++)
        enqueue_line(4 + 4 * (line % 8), line[0], 2'(line % 4), line % 4 == 0, 0, 2);
      wait_idle();
    end
  endtask

  // Alignment errors occur only on the final drain or residual beat, including one-entry lines.
  task automatic test_line_error();
    for (int phase = 0; phase < 2; phase++) begin
      set_paths(1, 1, phase[0], !phase[0]);
      enqueue_line(16, 0, 0, 1, 1, 0);
      enqueue_line(10, 0, 0, 0, 0, 0);
      enqueue_line(6, 0, 1, 0, 1, 1);
      enqueue_line(8, 1, 1, 0, 1, 2);
      enqueue_line(12, 1, 0, 0, 0, 2);
      enqueue_line(32, 1, 0, 0, 1, 2);
      enqueue_line(4, 0, 1, 0, 1, 2);
      enqueue_line(2, 0, 1, 0, 1, 1);
      enqueue_line(14, 0, 0, 0, 0, 0);
      wait_idle();
    end
  endtask

  // A waiting next-line entry must not mark the preceding line's residual beat.
  task automatic test_pending_boundaries();
    for (int phase = 0; phase < 2; phase++) begin
      set_paths(1, 1, phase[0], !phase[0]);
      for (int frame = 0; frame < 8; frame++) begin
        enqueue_line(6, 0, 2'(frame), 1, 0, 1);
        enqueue_line(6, 0, 2'(frame), 0, 0, 1);
        enqueue_line(12, 1, 2'(frame), 0, 0, 2);
        enqueue_line(2, 0, 2'(frame), 0, 0, 1);
      end
      wait_idle();
    end
    require_true(first_waits[0] > 0 && first_waits[1] > 0, "missing waiting line/frame start");
  endtask

  // Hold the final entry at almost-full, then test ignored unknown fields in input gaps.
  task automatic test_stalled_input();
    for (int path = 0; path < 2; path++) begin
      int depth;
      longint before_wait;
      depth = path == 0 ? int'(PARA_STREAM_FIFO_DEPTH) : int'(PARA_DMA_FIFO_DEPTH);
      set_paths(path == 0, path == 1, 1, 1);
      set_readies(path == 0 ? 0 : 100, path == 1 ? 0 : 100);
      gap_percent = 0;
      before_wait = eol_waits;
      enqueue_line(4 * depth, 1, 2'(path), 1, 0, 2);
      repeat (depth + 16) tick();
      require_true(eol_waits > before_wait, "EOL was not held at almost-full");
      set_readies(100, 100);
      wait_idle();
      set_readies(path == 0 ? 0 : 100, path == 1 ? 0 : 100);
      before_wait = body_waits;
      enqueue_line(4 * (depth + 1), 1, 2'(path), 1, 0, 2);
      repeat (depth + 16) tick();
      require_true(body_waits > before_wait, "ordinary entry was not held at almost-full");
      set_readies(100, 100);
      wait_idle();
    end
    for (int phase = 0; phase < 2; phase++) begin
      set_paths(1, 1, phase[0], !phase[0]);
      set_readies(45, 55, 60, 35);
      gap_percent = 40;
      for (int line = 0; line < 70; line++) begin
        bit dt;
        dt = $urandom_range(1, 0);
        enqueue_line((dt ? 4 : 2) * int'($urandom_range(20, 1)), dt,
                     2'($urandom), line % 4 == 0, line % 7 == 0, 3);
      end
      wait_idle();
    end
  endtask

  // Exercise both FIFO thresholds, long independent stalls, continuous DMA ready and pointer wrap.
  task automatic test_backpressure();
    for (int path = 0; path < 2; path++) begin
      fill_path(path);
      set_readies(100, 100);
      wait_idle();
      fill_path(path);
      set_readies(100, 100);
      wait_idle();
    end
    for (int phase = 0; phase < 3; phase++) begin
      set_paths(1, 1, phase == 1, phase == 2);
      set_readies(25, phase == 1 ? 100 : 65, 100, phase == 1 ? 0 : 70);
      gap_percent = phase * 10;
      for (int line = 0; line < 40; line++) begin
        bit dt;
        dt = $urandom_range(1, 0);
        enqueue_line((dt ? 4 : 2) * int'($urandom_range(32, 1)), dt,
                     2'($urandom), line % 5 == 0, line % 6 == 0, 3);
      end
      wait_idle();
    end
    for (int path = 0; path < 2; path++) begin
      require_true(af_cycles[path] > 0 && full_cycles[path] > 0, "missing FIFO threshold");
      require_true(output_stalls[path] > 0, "missing output stall");
      require_true(overflow_cycles[path] == 0, "unexpected overflow with stable enables");
      require_true(write_beats[path] > 2 * (path == 0 ? PARA_STREAM_FIFO_DEPTH :
                                                     PARA_DMA_FIFO_DEPTH), "missing FIFO wrap");
    end
  endtask

  // Measure isolated drain and residual latency, plus continuous long-line input.
  task automatic test_throughput_latency();
    for (int phase = 0; phase < 2; phase++) begin
      longint prior_takes;
      int prior_cycle;
      set_paths(1, 1, phase[0], !phase[0]);
      set_readies(100, 100);
      gap_percent = 0;
      enqueue_line(1924, 0, 2'(phase), 1, 0, 0);
      enqueue_line(1920, 1, 2'(phase+1), 0, 0, 2);
      enqueue_line(256, 0, 0, 0, 0, 2);
      wait_idle();
      for (int sample = 0; sample < 4; sample++) begin
        enqueue_line(4, 0, 0, 1, 0, 2);
        wait_takes(1);
        require_true(o_fbs_valid || o_fbd_valid, "drain not visible one cycle after take");
        wait_idle();
        enqueue_line(2, 0, 1, 1, 0, 1);
        wait_takes(1);
        require_true(!o_fbs_valid && !o_fbd_valid, "residual beat appeared before flush edge");
        tick();
        require_true(o_fbs_valid && o_fbd_valid, "residual beat not visible after flush edge");
        wait_idle();
      end
      prior_takes = takes;
      prior_cycle = cycle;
      enqueue_line(256, 1, 0, 1, 0, 2);
      wait_idle();
      require_true(takes - prior_takes == 64, "long line acceptance count");
      require_true(cycle - prior_cycle < 2 * 64 + 20, "unexpected throughput bubbles");
    end
  endtask

  // Disabled FIFOs keep their entries and do not stop the other path; enable acts mid-line.
  task automatic test_path_control();
    for (int path = 0; path < 2; path++) begin
      longint before_takes;
      fill_path(path);
      if (path == 0) begin i_axis_enable = 0; i_dma_enable = 1; end
      else begin i_dma_enable = 0; i_axis_enable = 1; end
      tick();
      if (path == 0) i_stream_format = 1;
      else i_dma_format = 1;
      before_takes = takes;
      for (int line = 0; line < 6; line++) enqueue_line(20, 0, 1, line == 0, 0, 0);
      repeat (200) tick();
      require_true(takes > before_takes, "disabled full path blocked input");
      set_readies(100, 100);
      wait_idle();
    end
    set_paths(0, 0, 0, 1);
    for (int line = 0; line < 6; line++) enqueue_line(6 + 4*line, 0, 0, line == 0, 0, 1);
    wait_idle();
    for (int phase = 0; phase < 2; phase++) begin
      set_paths(1, 1, phase[0], !phase[0]);
      set_readies(80, 80);
      enqueue_line(400, 0, 0, 1, 0, 0);
      wait_takes(15); i_axis_enable = 0;
      wait_takes(15); i_dma_enable = 0;
      wait_takes(15); i_axis_enable = 1;
      wait_takes(15); i_dma_enable = 1;
      wait_idle();
      set_paths(0, 0, !phase[0], phase[0]);
      enqueue_line(8, 1, 1, 1, 0, 2);
      wait_idle();
      i_axis_enable = 1; i_dma_enable = 1;
      enqueue_line(30, 0, 2, 1, 0, 3);
      wait_idle();
    end
  endtask

  // Re-enabling a full FIFO on a flush loses exactly that beat, even with a concurrent read.
  task automatic test_overflow();
    for (int path = 0; path < 2; path++) begin
      for (int concurrent_read = 0; concurrent_read < 2; concurrent_read++) begin
        longint before_overflow;
        fill_path(path);
        if (path == 0) begin i_axis_enable = 0; i_dma_enable = 1; end
        else begin i_dma_enable = 0; i_axis_enable = 1; end
        enqueue_line(6, 0, 1, 1, 0, 1);
        wait_end();
        before_overflow = overflow_cycles[path];
        if (path == 0) i_axis_enable = 1;
        else i_dma_enable = 1;
        if (concurrent_read != 0) set_readies(100, 100);
        tick();
        require_true(overflow_cycles[path] == before_overflow + 1, "missing flush overflow");
        set_readies(100, 100);
        wait_idle();
      end
    end
  endtask

  // Apply each clear during residual, flush, partial/full FIFO, read and full-write activity.
  task automatic exercise_clear(input int kind);
    for (int assignment_id = 0; assignment_id < 2; assignment_id++) begin
      for (int scenario = 0; scenario < 6; scenario++) begin
        longint before_clear;
        reset_dut();
        set_paths(1, 1, assignment_id[0], !assignment_id[0]);
        set_readies(100, 100);
        gap_percent = 0;
        case (scenario)
          0: begin
            enqueue_line(80, 0, 0, 1, 0, 1);
            wait_takes(1);
          end
          1: begin
            enqueue_line(6, 0, 0, 1, 0, 1);
            enqueue_line(6, 0, 1, 0, 0, 1);
            wait_end();
          end
          2: fill_path(assignment_id);
          3: begin
            enqueue_line(256, 0, 0, 1, 0, 2);
            wait_takes(8);
          end
          4: begin
            set_readies(0, 0);
            enqueue_line(4, 0, 0, 1, 0, 2);
            wait_end();
            tick();
          end
          5: begin
            fill_path(assignment_id);
            i_axis_enable = 0; i_dma_enable = 0;
            enqueue_line(6, 0, 1, 1, 0, 1);
            wait_end();
            i_axis_enable = assignment_id == 0;
            i_dma_enable = assignment_id == 1;
            set_readies(100, 100);
          end
          default: fail("unknown clear scenario");
        endcase
        before_clear = clear_events[kind];
        clear_datapath(kind, kind == 1 && scenario[0] ? 5 : 1);
        require_true(clear_events[kind] > before_clear, "clear not observed");
        set_readies(100, 100);
        set_paths(1, 1, assignment_id[0], !assignment_id[0]);
        enqueue_line(12, 0, 0, 0, 0, 0);
        enqueue_line(8, 1, 0, 0, 0, 2);
        enqueue_line(16, 0, 1, 1, 0, 0);
        wait_idle();
      end
    end
    if (kind != 0) begin
      require_true(clear_take > 0 && clear_flush > 0, "missing coincident take/flush clear");
      require_true(clear_full_write > 0 && clear_read > 0, "missing full-write/read clear");
    end
  endtask

  // Asynchronous reset discards pending data immediately and permits a fresh frame.
  task automatic test_async_reset(); exercise_clear(0); endtask

  // Synchronous reset wins for one or several cycles without changing configuration inputs.
  task automatic test_soft_reset(); exercise_clear(1); endtask

  // A single flush pulse discards pending data and boundary flags.
  task automatic test_fb_flush(); exercise_clear(2); endtask

  // Random legal frames, stalls, enables, formats and occasional coordinated clears.
  task automatic test_random_traffic();
    for (int frame = 0; frame < 30; frame++) begin
      int action, action_cycle, elapsed, lines;
      set_paths($urandom_range(3, 0) != 0, $urandom_range(3, 0) != 0,
                1'($urandom), 1'($urandom));
      set_readies(int'($urandom_range(100, 20)), int'($urandom_range(100, 20)), 70, 70);
      gap_percent = int'($urandom_range(50, 0));
      lines = int'($urandom_range(4, 1));
      for (int line = 0; line < lines; line++) begin
        bit dt;
        dt = 1'($urandom);
        enqueue_line((dt ? 4 : 2) * int'($urandom_range(32, 1)), dt, 2'($urandom),
                     line == 0, $urandom_range(4, 0) == 0, 3);
      end
      action = int'($urandom_range(9, 0));
      action_cycle = int'($urandom_range(60, 1));
      elapsed = 0;
      while ((source_queue.size() != 0 || source_valid) && elapsed < WAIT_LIMIT) begin
        tick();
        elapsed++;
        if (elapsed == action_cycle) case (action)
          0: i_axis_enable = !i_axis_enable;
          1: i_dma_enable = !i_dma_enable;
          2: clear_datapath(2);
          3: clear_datapath(1, int'($urandom_range(4, 1)));
          default: ;
        endcase
      end
      wait_idle();
    end
  endtask

  // Corrupt the checker's sampled stream data; the normal beat comparison must report an error.
  task automatic test_checker_fault();
    checker_fault = 1;
    enqueue_line(32, 0, 0, 1, 0, 0);
    wait_idle();
    require_true(errors > 0, "checker failed to detect corrupted observation");
  endtask

  task automatic finish_test(input bit timed_out = 0);
    if (finished) return;
    finished = 1;
    if (timed_out) fail("simulation timeout");
    if (checks == 0 || compared_beats[0] + compared_beats[1] == 0)
      fail("test did not compare any valid data");
    $display("checks=%0d takes=%0d compared=%0d/%0d errors=%0d", checks, takes,
             compared_beats[0], compared_beats[1], errors);
    if (errors == 0) $display("%s PASS", test_name);
    else $display("%s FAIL (errors=%0d)", test_name, errors);
    $finish;
  endtask

  initial begin
    if (!$value$plusargs("TEST=%s", test_name)) test_name = "none";
    if (!$value$plusargs("SEED=%d", seed)) seed = 1;
    void'($urandom(seed));
    $display("test=%s stream_depth=%0d dma_depth=%0d", test_name,
             PARA_STREAM_FIFO_DEPTH, PARA_DMA_FIFO_DEPTH);
    clear_model();
    driver_started = 1;
    #1 i_resetn = 0;
    repeat (3) tick();
    i_resetn = 1;
    tick();
    case (test_name)
      "test_reset_idle": test_reset_idle();
      "test_bypass_raw12": test_bypass_raw12();
      "test_bypass_raw10": test_bypass_raw10();
      "test_raw16": test_raw16();
      "test_format_combinations": test_format_combinations();
      "test_line_attributes": test_line_attributes();
      "test_line_error": test_line_error();
      "test_pending_boundaries": test_pending_boundaries();
      "test_stalled_input": test_stalled_input();
      "test_backpressure": test_backpressure();
      "test_throughput_latency": test_throughput_latency();
      "test_path_control": test_path_control();
      "test_overflow": test_overflow();
      "test_async_reset": test_async_reset();
      "test_soft_reset": test_soft_reset();
      "test_fb_flush": test_fb_flush();
      "test_random_traffic": test_random_traffic();
      "test_checker_fault": test_checker_fault();
      default: fail("unknown test name");
    endcase
    finish_test();
  end

  initial begin
    repeat (TIMEOUT_CYCLES) @(posedge i_sys_clk);
    #1;
    finish_test(1);
  end
endmodule

`default_nettype wire
