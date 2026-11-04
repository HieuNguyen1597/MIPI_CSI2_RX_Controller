# File        : 00_src/tb/Makefile
# Description : Build and run the block testbenches with Cadence Xcelium (`make help`).

SHELL       := /bin/bash
.SHELLFLAGS := -o pipefail -c

HERE        := $(abspath $(dir $(lastword $(MAKEFILE_LIST))))
XRUN        ?= xrun

# Run selection
BLOCK       ?=
CONFIG      ?=
TEST        ?=
SEED        ?= 1
WAVES       ?= 0

# RTL and output locations; both can be overridden.
RTL_DIR     ?= $(HERE)/../rtl
WORK        ?= $(HERE)/work

TIMESCALE   ?= 1ns/1ps
TOP         ?= tb_top
ERR_REGEX   ?= \*[EF],
ELAB_ERR_REGEX ?= xmelab: \*E,ERRORE

TB_DIR       = $(HERE)/$(BLOCK)

# CONFIG defaults to the first line of configs.list.
ifeq ($(strip $(CONFIG)),)
CONFIG      := $(shell awk '!/^[[:space:]]*(\#|$$)/ { print $$1; exit }' $(HERE)/$(BLOCK)/configs.list 2>/dev/null)
endif
RTL_FILE     = $(RTL_DIR)/m_fpt_$(BLOCK).sv
BUILD_DIR    = $(WORK)/$(BLOCK)/build/$(CONFIG)
RUN_DIR      = $(WORK)/$(BLOCK)/$(CONFIG)
SIM_LOG      = $(RUN_DIR)/logs/$(TEST)_s$(SEED).log
SRC_F        = $(BUILD_DIR)/sources.f
SUMMARY      = $(WORK)/$(BLOCK)/regress.txt

# Parameters of the selected configuration.
PARAMS       = $(shell awk -v c="$(CONFIG)" '$$1 == c { for (i = 2; i <= NF; i++) print $$i }' $(TB_DIR)/configs.list)

XRUN_ARGS    = -64bit -sv -licqueue -timescale $(TIMESCALE) -top $(TOP) \
               $(foreach p,$(PARAMS),-defparam $(TOP).$(p)) -f $(SRC_F)
RUN_ARGS     = -svseed $(SEED) +TEST=$(TEST) +SEED=$(SEED)
WAVE_ARGS    = $(if $(filter 1,$(WAVES)),-access +r -input $(RUN_DIR)/logs/$(TEST)_s$(SEED).tcl)

.PHONY: help list sim regress illegal clean check-block check-test

help:
	@echo "Usage: make <target> BLOCK=<block> [CONFIG=<name>] [TEST=<name>] [SEED=<n>] [WAVES=1]"
	@echo ""
	@echo "  list     list the configurations and the regression entries of BLOCK"
	@echo "  sim      compile if needed and run one test (CONFIG, TEST, SEED)"
	@echo "  regress  run every entry of <BLOCK>/regress.list"
	@echo "  illegal  elaborate every parameter set of <BLOCK>/illegal.list; each must be rejected"
	@echo "  clean    remove $(WORK)/<BLOCK>"
	@echo ""
	@echo "Blocks: $(patsubst $(HERE)/%/,%,$(dir $(wildcard $(HERE)/*/tb.f)))"
	@echo "Variables: RTL_DIR (default ../rtl), WORK (default ./work), XRUN (default xrun)"

check-block:
	@[ -n "$(BLOCK)" ] || { echo "BLOCK is not set, e.g. make $(MAKECMDGOALS) BLOCK=lane_management"; exit 2; }
	@for f in $(TB_DIR)/tb.f $(TB_DIR)/configs.list $(TB_DIR)/regress.list $(RTL_FILE); do \
	  [ -f "$$f" ] || { echo "Missing file: $$f"; exit 2; }; done
	@awk -v c="$(CONFIG)" '$$1 == c { f = 1 } END { exit !f }' $(TB_DIR)/configs.list \
	  || { echo "Configuration '$(CONFIG)' is not in $(TB_DIR)/configs.list"; exit 2; }

check-test:
	@[ -n "$(TEST)" ] || { echo "TEST is not set; see make list BLOCK=$(BLOCK)"; exit 2; }

list: check-block
	@echo "Configurations:"; grep -Ev '^[[:space:]]*(#|$$)' $(TB_DIR)/configs.list | sed 's/^/  /'
	@echo "Regression entries (configuration test seed):"; grep -Ev '^[[:space:]]*(#|$$)' $(TB_DIR)/regress.list | sed 's/^/  /'

# Source list: the RTL file, then <BLOCK>/tb.f. Relative paths in tb.f are taken from the block
# directory.
$(SRC_F): FORCE
	@mkdir -p $(BUILD_DIR)
	@{ echo "$(RTL_FILE)"; awk -v r="$(TB_DIR)" ' \
	     /^[[:space:]]*($$|\/\/|#)/ { next } \
	     /^\+incdir\+/ { p = substr($$1, 9); if (p !~ /^\//) p = r "/" p; print "+incdir+" p; next } \
	     /^[-+]/ { print; next } \
	     { p = $$1; if (p !~ /^\//) p = r "/" p; print p }' $(TB_DIR)/tb.f; } > $(SRC_F)
FORCE:

# PASS: xrun exits with 0, the log has "<TEST> PASS", and no error or fatal message.
sim: check-block check-test $(SRC_F)
	@mkdir -p $(RUN_DIR)/logs
	@$(if $(filter 1,$(WAVES)),printf 'database -open %s -shm -default\nprobe -create -all -depth all -shm\nrun\nexit\n' \
	  "$(RUN_DIR)/logs/$(TEST)_s$(SEED).shm" > $(RUN_DIR)/logs/$(TEST)_s$(SEED).tcl;)
	@cd $(BUILD_DIR) && $(XRUN) $(XRUN_ARGS) $(WAVE_ARGS) $(RUN_ARGS) -l $(SIM_LOG) > /dev/null 2>&1; rc=$$?; \
	if [ $$rc -eq 0 ] && grep -Eq '^$(TEST) PASS' $(SIM_LOG) && ! grep -Eq '$(ERR_REGEX)' $(SIM_LOG); \
	then st=PASS; else st=FAIL; fi; \
	echo "$(CONFIG) $(TEST) seed=$(SEED) $$st rc=$$rc log=$(SIM_LOG)"; [ $$st = PASS ]

regress: check-block
	@mkdir -p $(WORK)/$(BLOCK)
	@{ echo "# block=$(BLOCK) date=$$(date '+%Y-%m-%d %H:%M') host=$$(hostname)"; \
	   echo "# $$($(XRUN) -version 2>/dev/null)"; } > $(SUMMARY)
	@fail=0; \
	while read -r c t s; do \
	  $(MAKE) --no-print-directory sim BLOCK=$(BLOCK) CONFIG=$$c TEST=$$t SEED=$$s \
	    | tee -a $(SUMMARY) || fail=1; \
	done < <(grep -Ev '^[[:space:]]*(#|$$)' $(TB_DIR)/regress.list); \
	echo "# total PASS=$$(grep -c ' PASS ' $(SUMMARY)) FAIL=$$(grep -c ' FAIL ' $(SUMMARY))" | tee -a $(SUMMARY); \
	exit $$fail

# Each line of illegal.list must be rejected: elaboration stops with an RTL error and xrun fails.
illegal: check-block $(SRC_F)
	@[ -f $(TB_DIR)/illegal.list ] || { echo "Missing file: $(TB_DIR)/illegal.list"; exit 2; }
	@mkdir -p $(WORK)/$(BLOCK)/illegal
	@{ echo "# block=$(BLOCK) date=$$(date '+%Y-%m-%d %H:%M') host=$$(hostname)"; \
	   echo "# $$($(XRUN) -version 2>/dev/null)"; } > $(WORK)/$(BLOCK)/illegal.txt
	@fail=0; \
	while read -r tag params; do \
	  log=$(WORK)/$(BLOCK)/illegal/$$tag.log; \
	  opts=$$(for p in $$params; do printf -- '-defparam $(TOP).%s ' "$$p"; done); \
	  (cd $(WORK)/$(BLOCK)/illegal && $(XRUN) -64bit -sv -licqueue -timescale $(TIMESCALE) -top $(TOP) \
	     $$opts -f $(SRC_F) -elaborate -l $$log > /dev/null 2>&1); rc=$$?; \
	  if [ $$rc -ne 0 ] && grep -Eq '$(ELAB_ERR_REGEX)' $$log; then st=PASS; else st=FAIL; fi; \
	  echo "illegal $$tag $$st rc=$$rc log=$$log" | tee -a $(WORK)/$(BLOCK)/illegal.txt; \
	  [ $$st = PASS ] || fail=1; \
	done < <(grep -Ev '^[[:space:]]*(#|$$)' $(TB_DIR)/illegal.list); \
	exit $$fail

clean:
	rm -rf $(WORK)/$(BLOCK)
