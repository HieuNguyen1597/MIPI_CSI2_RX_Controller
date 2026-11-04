# MIPI CSI-2 RX Controller — RTL Coding Rules

| Item | Value |
|---|---|
| Document | MIPI_CSI2_RX_Controller_RTL_Rules |
| Version | v1.0 |
| Date | 2026-11-04 |
| Author | HCMUT team |
| Base guideline | FU1 IP Wrapper Guideline (FPT, Mar 07, 2026) |

## Contents

1. [Introduction](#1-introduction)
2. [Sources of Authority](#2-sources-of-authority)
3. [FTEL Coding Style](#3-ftel-coding-style)
4. [Team Coding Conventions](#4-team-coding-conventions)
5. [Signal Name Mapping](#5-signal-name-mapping)
6. [Deviations from the FTEL Guideline](#6-deviations-from-the-ftel-guideline)
7. [RTL Handover Checklist](#7-rtl-handover-checklist)
- [Appendix A. Revision History](#appendix-a-revision-history)
- [Appendix B. Acronyms](#appendix-b-acronyms)

## 1. Introduction

### 1.1 Purpose

This document defines the rules for writing, organising and handing over the RTL of the MIPI CSI-2
RX Controller IP. It applies the FTEL coding style of the FU1 IP Wrapper Guideline and adds the
conventions the team follows for every RTL file of the IP.

### 1.2 Scope

The IP is partitioned into 14 functional blocks. Each block is implemented as exactly one RTL module;
the sub-blocks described in the specification are implemented inside the module of their block and
are not split into separate modules. The top level and the IP wrapper are described in Section 4.9.

| Block | Name in the specification |
|---|---|
| B01 | PHY Protocol Interface (PPI) |
| B02 | Lane Management |
| B03 | Packet Analyzer |
| B04 | Byte-to-Pixel Unpacking |
| B05 | Test Pattern Generator |
| B06 | TPG Mux |
| B07 | Line Buffer |
| B08 | Format Bridge |
| B09 | Frame Context & Metadata Capture |
| B10 | Control FSM |
| B11 | Interrupt & Error Aggregator |
| B12 | AXI4-Stream Master |
| B13 | Direct Memory Access (DMA) |
| B14 | CSR & AXI4-Lite Subordinate |

Primitives, pre-built IP and hierarchy levels other than these 14 modules and the top level are
added only when they are part of the agreed IP partition.

### 1.3 Rule Categories

Each rule belongs to one of three categories:

| Category | Meaning |
|---|---|
| FTEL | Required by the FU1 IP Wrapper Guideline (Section 3 of this document). |
| Project | Required by the project (for example the 14-module partition and the signal mapping). |
| Team | Convention agreed by the team. It complements the FTEL rules and never replaces them. |

The words **shall** and **shall not** mark mandatory rules; **should** marks a recommended practice;
**may** marks a permitted option.

### 1.4 Reference Documents

| Document | Content |
|---|---|
| FU1 IP Wrapper Guideline | FTEL coding style for in-house RTL and IP wrapper signals (Section 1, Tables 1-1 and 1-2). |
| FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS | Micro-architecture specification (MAS) of the IP: block behaviour, interfaces, clocking, reset, parameters. |
| FTEL_IP_MIPI_CSI-2_CONTROLLER_DIAGRAM | Micro-architecture diagrams, one page per block ("#N <block name>"). |
| FTEL_IP_MIPI_CSI2_RX_CONTROLLER_CSR | CSR specification: register offsets, bit fields, access types and reset values. |

The specification version applied to each RTL file is recorded in the `Spec` field of its header
(Section 4.10).

## 2. Sources of Authority

- **Behaviour** is defined by the specification set: the MAS, the micro-architecture diagrams and the
  CSR specification. The RTL shall keep the ports, widths, clocks and resets, handshakes, latencies and
  error handling defined there.
- **Coding style** is defined by the FTEL guideline (Section 3) and the team conventions (Section 4).
  Specification names are adapted to the FTEL naming rules; every adapted name is recorded in the
  signal mapping (Section 5).
- When the MAS text, the MAS figures and the diagrams disagree, no source wins by default. The team
  selects the correct source, records the reason, and corrects the other sources so that they match.
- When the sources are missing information or conflict in a way that affects behaviour, the affected
  logic is not written until the question is resolved with the block owner. An assumed interpretation
  is not treated as part of the specification.

## 3. FTEL Coding Style

Source: FU1 IP Wrapper Guideline, Section 1.1 (In-house RTL Code), Section 1.2 (IP Wrapper),
Table 1-1 and Table 1-2. All rules in this section are in the **FTEL** category.

### 3.1 In-house RTL Code

- RTL shall be written in synthesisable SystemVerilog.
- Combinational logic shall use `always_comb`; sequential logic shall use `always_ff`. The plain
  `always` keyword shall not be used to describe RTL.
- Signal and module names shall be lowercase. Parameter and define names shall be uppercase.
- Every RTL file shall have a header.
- Ports use the prefix `i_` for inputs, `o_` for outputs and `io_` for inouts.

| Item | Rule | Example (from the guideline) |
|---|---|---|
| Module | `m_fpt_<functional_name>` | `m_fpt_i2c_wrap` |
| Parameter | `PARA_<FUNCTIONAL_NAME>` | `PARA_DATA_WIDTH` |
| Define | `DEF_<FUNCTIONAL_NAME>` | `DEF_MODEL` |
| Flip-flop | `reg_<functional_name>` | `reg_counter` |
| Input port | `i_<functional_name>` | `i_clk` |
| Output port | `o_<functional_name>` | `o_pready` |
| Inout port | `io_<functional_name>` | `io_pad` |
| Instance | `u_<functional_name>_<number>` | `u_apb_i2c_0`, `u_apb_i2c_1` |

### 3.2 IP Wrapper Signals

Ports of the IP wrapper follow the pattern below, where `<dir>` is `i`, `o` or `io`; `<ip>` is the IP
name unified across the project; `<bus>` is the bus protocol (APB, AHB or AXI), written in lowercase
in the signal name.

| Port type | Connects to | Naming pattern |
|---|---|---|
| BUS | System bus | `<dir>_<ip>_<bus>_<signal>` |
| INT | GIC or CPU | `<dir>_<ip>_int_<signal>` |
| CLOCK | Clock Reset Module (CRM) | `<dir>_<ip>_<signal>` |
| RESET | Clock Reset Module (CRM) | `<dir>_<ip>_<signal>_n` |
| PAD | IO pad selection | `<dir>_<ip>_pad_<signal>` |
| CSR | SoC CSR | `<dir>_<ip>_soccsr_<signal>` |
| Others | — | `<dir>_<ip>_<signal>` |

All resets at the IP wrapper shall be active-low. If a reset inside the IP is active-high, the
inverter shall be placed inside the IP wrapper.

These wrapper rules apply to the ports of the IP wrapper only. They do not rename the ports between
internal blocks, and they do not change the polarity or meaning of the soft reset, flush and internal
resets defined by the specification.

## 4. Team Coding Conventions

All rules in this section are in the **Team** category unless stated otherwise.

### 4.1 Scope of the RTL Source

- RTL files shall contain synthesisable logic only. Stimulus, simulation delays, checkers,
  assertions and behavioural models belong to the verification sources, which are kept outside
  `00_src/rtl/` and are never part of the synthesis file list.
- Verification compares the RTL against expected results derived from the specification. The RTL is
  never used as the reference for its own expected results.
- Every synthesis result records the top module, file list, parameter values, tool and version,
  options and constraints. A construct is used only after the selected front end has been confirmed
  to elaborate and synthesise it as intended.

### 4.2 Naming

| Item | Rule | Example |
|---|---|---|
| Module, instance, parameter, define, flip-flop | Section 3.1 | `m_fpt_lane_management`, `u_<name>_0` |
| Block port | `i_` / `o_` + the name in the specification | `i_rxdatahs_0`, `o_merged_valid` |
| Flip-flop | `reg_` + the name in the specification | `reg_sync_seen_0` |
| Combinational or internal connecting signal | `w_` + the name in the specification | `w_sync_ok_0`, `w_read_en` |
| Localparam | `LPARA_<NAME>` | `LPARA_PTR_W` |
| Type (`typedef`) | `<name>_type_def` | `lm_state_type_def` |
| Enum member | `ENUM_<FUNCTION>_<VALUE>`, uppercase | `ENUM_OP_ADD` |
| FSM state register / next state | Section 4.5 | `reg_fsm_lm_current_state` |
| Generate block | `gen_<function>` | `gen_lane` |
| Combinational / sequential process label | `p_com_<function>` / `p_ff_<function>` | `p_com_read_ctrl`, `p_ff_flags` |

- Per-lane and replicated signals use an index starting at 0 (`_0`, `_1`, …). Packed ranges include
  bit 0 (`[PARA_DATA_W-1:0]`).
- Names and values fixed by the specification or by a protocol keep their original form after the
  prefix.
- The `reg_` prefix marks a sequential element; it does not refer to the Verilog `reg` type. Such
  signals are declared as `logic`.
- Signals that exist only in the RTL (pointers, counters, memory arrays, …) receive a descriptive
  name with the correct prefix. The mapping marks them as having no specification name (Section 5).
- Block ports keep the specification name, including the reset name and polarity defined there (for
  example `i_resetn`). The `_n` reset suffix of Section 3.2 applies at the IP wrapper.

The following rules apply **only to names that exist only in the RTL**; names taken from the
specification are exempt:

- Words are abbreviated according to the table below, matching the longest phrase first. A word not
  in the table is written in full. `int` and `config` are SystemVerilog keywords and may only appear
  as part of a longer name.
- A name is at most 30 characters long, including prefix and suffix.
- An active-low signal that is not a reset ends with `_n`.

| Word | Abbreviation | Word | Abbreviation |
|---|---|---|---|
| address | `addr` | data | `data` (unchanged) |
| next | `nxt` | interrupt | `int` |
| counter | `cnt` | configuration, configure | `config` |
| enable | `en` | output enable | `oe` |
| clock | `clk` | reset | `rst` |
| request | `req` | acknowledge | `ack` |
| response | `resp` | command | `cmd` |
| select | `sel` | control | `ctrl` |
| status | `sts` | write / read | `wr` / `rd` |
| transmit / receive | `tx` / `rx` | buffer | `buf` |
| pointer | `ptr` | index | `idx` |
| length | `len` | source / destination | `src` / `dst` |
| error | `err` | frequency | `freq` |
| divider | `div` | threshold | `thr` |
| arbiter | `arb` | grant | `gnt` |
| priority | `prio` | multiplexer | `mux` |
| decoder / encoder | `dec` / `enc` | maximum / minimum | `max` / `min` |
| number | `num` | parity | `par` |

The words `data`, `valid`, `ready`, `mode`, `depth`, `done` and `busy` are always written in full.

**Standard protocol ports.** A standard-protocol port that the specification does not name follows
`<dir>_<protocol>_<signal>`, where `<signal>` is the exact lowercase mnemonic of the protocol
revision, without abbreviation. Protocol tags are `apb`, `axi`, `axil` (AXI4-Lite) and `axis`
(AXI4-Stream); for example `i_axil_awvalid`, `o_axis_tdata`. Only signals that exist in the selected
protocol revision and interface configuration are included. The module documents the protocol name,
revision, role and the optional signals used.

### 4.3 Declarations, Data Types and Parameters

- Hardware signals are unsigned 4-state `logic` vectors with an explicit width. `signed`, `$signed`
  and signed literals are not used unless the specification requires signed arithmetic; any such
  exception is explained in a comment and raised with the reviewer.
- Ports are declared as `input wire [..] i_x` and `output logic [..] o_x`. `inout wire` appears only
  in pad or IO wrappers. Internal signals and states use `logic` or an enum based on `logic`.
- `reg`, `wire` (except for input and inout ports), `integer`, `bit`, `byte`, `shortint`, `int` and
  `longint` are not used for signals. Exceptions: an `int` loop variable declared in the header of a
  `for` loop, and a `genvar` declared in the header of a generate `for` loop.
- Implicit nets are forbidden: every RTL file begins with `` `default_nettype none `` and ends with
  `` `default_nettype wire ``.
- `package`, `import`, SystemVerilog `interface` and `modport` are not used in RTL. Types are
  declared inside the module and ports are packed vectors. These constructs may be used in
  verification code.
- `` `define `` and `` `undef `` are not used in block RTL; parameters and localparams are used
  instead. A define needed by the IP wrapper follows the FTEL name `DEF_<NAME>`.
- An overridable parameter is declared as `parameter logic [31:0] PARA_<NAME> = <default>`. A
  derived constant is declared as `localparam logic [31:0] LPARA_<NAME>` (or with another suitable
  width) in the internal constants section of the module.
- Each `parameter` inside `#( )` is preceded by exactly three comment lines, in this order:
  - `// Parameter Description: ...` purpose, dependencies and constraints on other parameters.
  - `// Parameter values: ...` legal values or range, and the default value.
  - `// Parameter unit: ...` unit (`bit`, `byte`, `cycle`, …); `count` for a quantity without a
    physical unit, `none` for a code or flag.
- Derived widths (pointers, counters, field positions) are computed from parameters or localparams
  instead of repeating literals. Design-specific constants are named by a parameter, localparam or
  enum member. A parameter value that is not defined by the specification is not taken from an
  example.
- Illegal parameter values are stopped at elaboration with `$error` inside a generate `if`. This is
  the only system task allowed in RTL. Before it is used, every lint, simulation and synthesis tool of
  the project shall be confirmed to accept it.

```systemverilog
// Parameter Description: number of lane slices instantiated.
// Parameter values: 1, 2 or 4; default is 4.
// Parameter unit: count
parameter logic [31:0] PARA_MAX_LANES = 4,
```

### 4.4 Combinational Logic

- Combinational logic is written in an `always_comb` process labelled `p_com_<function>`, using
  blocking assignments (`=`). `assign` is used only to connect a `reg_*` signal (or the `w_*` signal
  of a combinational output) to an `o_*` port. Each signal is driven by exactly one process or one
  `assign`.
- Every signal of a process receives a default value at the top of the process, before any `if` or
  `case`, so that every branch drives it and no latch is inferred. `always_latch` and latches are
  forbidden.
- Plain `case` is the default. `casez` is used only for intentional wildcard decoding; `casex` is
  forbidden. Every `case` and `casez` has a `default` branch. `unique case` is used only when the
  branches are truly mutually exclusive and complete.
- The `default` branch of a plain `case` always assigns a defined value (a safe value, or the
  recovery value defined by the specification), never `'x`. A `casez` may assign `'x` in its `default`
  only when its branches cover every legal binary code and the specification treats the remaining
  codes as don't-care; the comment above the process states this. Every `'x` is a review item.
- Combinational loops are forbidden; feedback goes through a register.
- Functions in RTL are `automatic` and deterministic, without side effects or static state.

### 4.5 Finite State Machines

- Every FSM declares its states as an enum based on `logic`, with explicit values, inside the
  module: type `<f>_state_type_def`, members `ENUM_ST_<STATE>`. When the specification defines the
  state encoding (for example because the state is reported in a register), the enum values match
  that encoding.
- The current-state register is `reg_fsm_<f>_current_state` and the combinational next state is
  `w_fsm_<f>_nxt_state`, where `<f>` is at most 8 characters and follows the abbreviation table. The
  next state is computed in `always_comb`; the state register is updated in `always_ff`.
- Unused state codes have defined behaviour: the `default` branch returns to a safe recovery state,
  usually IDLE. This branch is omitted only when the specification states that those codes cannot
  occur and the don't-care treatment of Section 4.4 is accepted by the reviewer.
- When the state is required at a port, the `o_*` port is connected to the state register with
  `assign`.

```systemverilog
// Burst FSM states, encoded as the status register reports them.
typedef enum logic [2:0] {
  ENUM_ST_IDLE   = 3'b000,
  ENUM_ST_ARMING = 3'b001
} lm_state_type_def;

lm_state_type_def reg_fsm_lm_current_state;
lm_state_type_def w_fsm_lm_nxt_state;
```

### 4.6 Width and Arithmetic

- Operands are sized before addition, comparison, shift, concatenation and assignment, so that the
  result width and the extension are intentional. Truncation and overflow are checked; an
  intentional bit drop or overflow follows the specification and is covered by a comment and a test.
- An expression assigned to a result of a given width is cast to the target width, for example
  `w_sum = $bits(w_sum)'(w_a + w_b);`. Carry and overflow are kept in a wider intermediate signal
  before the result is narrowed.
- Literals are sized and unsigned (`8'd3`). Sized literals and simple Boolean constants are not
  considered magic numbers.
- A `for` loop inside `always_comb` has static bounds; its loop variable is an `int` declared in the
  loop header: `for (int idx = 0; idx < PARA_N; idx++)`. `generate` is preferred for replicating
  structure.

### 4.7 Sequential Logic, Clocks and Resets

- Flip-flops are updated with non-blocking assignments (`<=`) in an `always_ff` process labelled
  `p_ff_<function>`. Each state element is updated by one process only; each process uses one
  explicit clock. Combinational and sequential logic are not mixed in one process.
- A registered output is stored in `reg_x` and connected to `o_x` with `assign`; no extra flip-flop
  stage is added only to rename a signal.
- Clock enables are preferred to clock gating for conditional updates.
- Resets follow the specification. Control and valid registers are always reset. A payload or
  datapath register (for example a FIFO memory array) may be left without reset when its consumer
  ignores it until the corresponding valid or initial state is reached; the comment states this
  condition. A register left without reset shall not contradict the reset-state requirements of the
  MAS (Section 6.2.2, Reset state).
- Each process uses the clock, reset polarity, reset type and reset/flush priority defined by the
  specification. Reset types are not mixed within one process, and the reset behaviour maps to cells
  of the target library.
- Clock domain crossings use approved structures only, at the locations defined by the
  specification: a two-flop synchronizer for single-bit level signals; a handshake or an asynchronous
  FIFO with Gray-coded pointers for multi-bit data. The crossing source is registered in its own
  domain. No combinational logic is placed between the launch flop and the synchronizer, or between
  synchronizer stages. Synchronizers are not added on the designer's own initiative, and the bits of
  a bus that must stay coherent are not synchronized individually.
- Clocks are not generated or gated by combinational logic and are not used as data. State is not
  stored on both edges, or on an edge other than the domain convention, without an approved
  architecture decision. Clock generation, dividers and timing exceptions follow the architecture and
  the constraints.

### 4.8 Hierarchy, Instances, Generate and Memories

- Each file contains one synthesisable module, and the file name equals the module name
  (`<module_name>.sv`).
- Ports and parameter overrides are connected by name. Positional connections, `.*` and `defparam`
  are forbidden.
- Replication uses compile-time parameters and generate constructs with static bounds. Generate
  blocks are named `gen_<function>` and indexed from 0; the `genvar` is declared in the loop header:
  `for (genvar gen_idx = 0; ...)`. Elaboration-time selection is kept separate from run-time
  behaviour.
- Arrays and memories are written in a form supported by the synthesis front end and the memory
  flow. The synthesis report is checked to confirm the inferred depth, width and ports. Memory macros
  sit behind a wrapper according to the integration rules of the project.
- Technology-dependent cells (clock-gating cells, synchronizers, memory macros, pads) appear only in
  dedicated wrappers, never in common logic.

### 4.9 Top Level and IP Wrapper

These rules apply once the scope of the top level and the IP wrapper is agreed. No additional module
is created inside the 14 functional blocks for this purpose.

- The top level is a structural wrapper: interface, parameters, connecting declarations and
  instances. Logic resides in the sub-modules.
- An asynchronous reset is synchronized for each receiving clock domain (asynchronous assertion,
  synchronous de-assertion) in a dedicated reset-synchronizer module. Synchronizers and clock-gating
  cells are separate modules (`m_fpt_*_sync`, …) so that constraints and `dont_touch` attributes apply
  cleanly.
- The IP wrapper ports follow Section 3.2.

### 4.10 File Layout and Header

Delivered RTL files are stored in `00_src/rtl/`, one file per module. The content of a file appears in
this order:

1. The five-line header (below).
2. `` `timescale 1ns/1ps `` (used by simulation only; present in every RTL file).
3. `` `default_nettype none ``.
4. ANSI-style module declaration: the `#( )` parameter list (three comment lines per parameter,
   Section 4.3), then the port list.
5. Localparams, types and internal signals.
6. Logic grouped by sub-block, each sub-block under a section title (below).
7. Output connections.
8. `endmodule`, followed by `` `default_nettype wire ``.

The FTEL guideline requires a header without defining its fields. The project header has five lines:

```systemverilog
// Module      : <module name>
// Block       : <Bxx - block name in the specification>
// Spec        : <specification document and version, e.g. FTEL_IP_MIPI_CSI-2_CONTROLLER_MAS_v1.4>
// Date        : <YYYY-MM-DD, date of the last content change>
// Description : <one short line describing the function of the module>
```

A sub-block title (and the titles of the constants, internal signals and output connection sections)
is a three-line frame, indented with the module body. The dash lines contain 90 `-` characters, giving
a line length of 95 characters. The comment of each process sits directly above the process, outside
the frame:

```systemverilog
  // ------------------------------------------------------------------------------------------
  // Lane gate
  // ------------------------------------------------------------------------------------------
```

- Indentation and alignment use spaces only, two spaces per level. Tab characters are forbidden.
- Commented-out RTL code is not kept; version control holds the history.

### 4.11 Comments

- A comment precedes each group of logic (each group of `assign` statements, each process, each
  group of instances). It states the behaviour, the assumptions, the reset and valid conditions and
  the recovery behaviour. Comments explain intent, protocol assumptions, non-obvious sizes, the
  condition under which a register is not reset and synthesis-sensitive structures; they do not
  restate each statement. Comments are kept consistent with the code.
- Comments are short: usually one or two lines per process or group of `assign` statements, and up
  to about four lines when a priority order must be stated. Content worth keeping: function;
  conditions and priority of reset, clear and flush; clock domain and synchronization point of
  asynchronous signals; reason a register is not reset; protocol assumptions; priority order of FSM
  transitions; encodings defined by the specification.
- A comment line is at most 100 characters; a longer sentence is split over several `//` lines.
- RTL comments do not cite table, figure or section numbers of the specification. Specification
  names of signals, registers, CSR fields and blocks (for example `LANE_ALIGN_STATUS`, B14) may be
  used.
- The RTL is a deliverable, so comments describe the design only. They do not contain requirement or
  test-case tracking tags, review or approval references, names of development tools or code
  generators, explanations addressed to lint tools or reviewers (for example why a port is unused),
  mapping remarks such as "not in the specification", or revision history.
- One comment format is used throughout a file. Functional changes are not mixed with reformatting
  of a whole module.

### 4.12 Prohibited Constructs in RTL

| No. | Prohibited | Use instead |
|---|---|---|
| 1 | `#` delays in RTL (`#(...)` parameter overrides are allowed) | No delays in RTL |
| 2 | `initial` blocks, initialization by simulation time | Reset, or macro initialization supported by the flow |
| 3 | `casex` | `case`; `casez` as in Section 4.4 |
| 4 | `defparam` | Named parameter override at the instance |
| 5 | Implicit nets | Full declarations and `` `default_nettype none `` |
| 6 | Plain `always` | `always_comb`, `always_ff` |
| 7 | `always_latch`, latches | Complete assignment, or a flip-flop |
| 8 | Multiple drivers on one signal | A single driver |
| 9 | Functions with side effects or static state | Deterministic `automatic` functions |
| 10 | Unexplained magic numbers | Parameter, localparam, enum member |
| 11 | `package`/`import`, `interface`/`modport` in RTL | Types inside the module, packed-vector ports |
| 12 | `` `define ``/`` `undef `` in block RTL | Parameter, localparam |
| 13 | Positional instance connections, `.*` | Named connections |
| 14 | Testbench system tasks, random stimulus (except `$error` at elaboration) | Verification code |
| 15 | Combinational logic that generates or gates a clock; a clock used as data | Clock enable; approved clock-gating cell in a wrapper |
| 16 | State on both edges, or on an edge other than the domain convention, without approval | Documented edge relationship and constraints when required |
| 17 | `inout` in functional logic | Pad or IO wrapper only |
| 18 | Commented-out RTL code | Delete it; use version control |
| 19 | Assertions in synthesisable RTL files | Verification files |
| 20 | `reg`, `wire`, `integer` for ordinary signals (except `input wire`, `inout wire` ports) | `logic` |
| 21 | 2-state types (`bit`, `byte`, `shortint`, `int`, `longint`, enums based on them), except `for` loop variables and `genvar` | 4-state `logic` |
| 22 | `signed`, `$signed`, signed literals, unless the specification requires them | Unsigned vectors and literals |
| 23 | Combinational loops | Feedback registered through a flip-flop |
| 24 | Internal tri-state (`'z`) outside pad or IO wrappers | Multiplexer |
| 25 | Synthesis pragmas that change function or hide logic (`translate_off`/`synthesis off`, `full_case`, `parallel_case`) | Explicit RTL; `unique case` as in Section 4.4 |
| 26 | Cross-module hierarchical references, `force`/`release`, `fork`/`join`, `wait`, `disable` | Ports and named connections |
| 27 | Non-synthesisable types: `real`, `string`, `event`, classes, dynamic/associative arrays, queues | Static `logic` arrays |
| 28 | Logic between the launch flop and a synchronizer, or between synchronizer stages | Section 4.7 |
| 29 | Tab characters in RTL files | Spaces |
| 30 | Lint pragmas in RTL files (for example `// verilator lint_off`) | Waiver file kept separately (Section 4.13) |

## 5. Signal Name Mapping

This section is in the **Project** category.

Specification names are adapted to the FTEL naming rules, so every RTL module has a mapping file that
links its RTL names to the specification names.

- **Location.** One Markdown file per module in `02_mapping/`, named after the module:
  `02_mapping/<module_name>.md` (for example `02_mapping/m_fpt_tpg.md`). The complete IP has 14
  mapping files, one per functional module.
- **Notes section.** Each file starts with a `## Notes` section stating: the RTL module and its file;
  the specification document and version, with the sections, tables and figures the names come from;
  the index conventions (for example how a spec suffix `_x` maps to the lane index `_0` … `_3`); the
  meaning of the `-` entry.
- **Mapping table.** A `## Mapping` section holds a table with at least the columns `RTL name`,
  `Spec name` and `Kind` (module, parameter, input port, output port, flip-flop, internal signal, …).
  A `Spec reference` or `Note` column may be added for the exact table row or the technical meaning
  (value, condition, lane, clock domain).
- **Coverage.** The table lists the module, its parameters and ports, and the `reg_*` and `w_*`
  signals that have a specification counterpart. An RTL-only name, when listed, has `-` as its spec
  name; no spec name is invented for it.
- **Content.** Like the RTL, mapping files contain no tracking tags, review references or tool names.
- **Maintenance.** The mapping is updated together with the RTL whenever a name is added or changed.
  A new name keeps the meaning of the signal it replaces. A change of width, timing or behaviour
  requires a specification and interface update; the mapping does not replace that update.

## 6. Deviations from the FTEL Guideline

**None.** This revision does not replace any rule of the FU1 IP Wrapper Guideline. The team
conventions of Section 4 only add rules in areas the guideline leaves open:

| Area | FTEL guideline | Team addition |
|---|---|---|
| File header | A header is required; its fields are not defined. | Five-line header (Section 4.10). |
| Signal prefixes | `reg_` for flip-flops, `i_`/`o_`/`io_` for ports. | `w_` for combinational and internal connecting signals (Section 4.2). |
| Constants and types | `PARA_` for parameters, `DEF_` for defines. | `LPARA_` for localparams, `_type_def` for types, `ENUM_` for enum members, three comment lines per parameter (Sections 4.2, 4.3). |
| Block ports | `i_`/`o_` prefixes. | Prefix plus the specification name, keeping the specification reset name and polarity (for example `i_resetn`). The active-low `_n` reset rule applies at the IP wrapper (Section 3.2). |

## Appendix A. Revision History

| Version | Date | Author | Reviewer | Description of Change |
|---|---|---|---|---|
| v1.0 | 2026-11-04 | MIPI CSI-2 RX Controller design team | | First version |

## Appendix B. Acronyms

| Acronym | Description |
|---|---|
| AHB | Advanced High-performance Bus |
| APB | Advanced Peripheral Bus |
| AXI | Advanced eXtensible Interface |
| CDC | Clock Domain Crossing |
| CRM | Clock Reset Module |
| CSR | Control and Status Register |
| DFT | Design for Testability |
| FIFO | First In, First Out |
| FSM | Finite State Machine |
| GIC | Generic Interrupt Controller |
| MAS | Micro-Architecture Specification |
| PPI | PHY Protocol Interface |
| RDC | Reset Domain Crossing |
| RTL | Register Transfer Level |
| SDC | Synopsys Design Constraints |
| TPG | Test Pattern Generator |
