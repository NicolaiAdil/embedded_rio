# Convenience wrapper for the offline replay tooling.
#
# Default inputs come from environment variables RIO_LOG / RIO_ULG (e.g.
# put `export RIO_LOG=...` in your shell rc). If those are unset, the
# hard-coded fallbacks below are used. Either can still be overridden on
# the command line, which takes top priority:
#
#     export RIO_LOG=data/field/2105/csv/LOG0007.CSV   # session-wide
#     make run                                         # uses $RIO_LOG
#     make run LOG=some/other.CSV                      # one-off override
#     make compare ARGS=--no-vvo                       # forward flags
#     make all run compare                             # full chain

LOG   ?= $(or $(RIO_LOG),data/field/2105/csv/LOG0012.CSV)
ULG   ?= $(or $(RIO_ULG),data/field/2105/ulog/log_24_2026-5-21-20-47-18.ulg)

# Report where a variable's value came from. Used by `make help`.
# Args: $1 = variable name (e.g. LOG), $2 = env var name (e.g. RIO_LOG).
src = $(if $(filter command line,$(origin $1)),CLI override,$(if $($2),from \$$$2,fallback))
NAME  ?= $(notdir $(basename $(LOG)))
OUT   ?= runs/$(NAME)
PY    ?= .venv/bin/python3
ARGS  ?=

BUILD_DIR := build/replay
BIN       := $(BUILD_DIR)/rio_replay
CACHE     := $(BUILD_DIR)/CMakeCache.txt

# Shortcuts that translate to compare_with_ulog.py flags via $(ARGS).
# Anything else can be passed verbatim, e.g. ARGS="--tol 0.3 --no-vvo".
ifdef NO_VVO
  ARGS += --no-vvo
endif
ifdef TOL
  ARGS += --tol $(TOL)
endif

.PHONY: all build run compare clean help

all: build

# CMake configure step — only re-runs if the top-level CMakeLists.txt changes.
$(CACHE): tools/replay/CMakeLists.txt
	cmake -S tools/replay -B $(BUILD_DIR) -DCMAKE_BUILD_TYPE=Release

# Incremental build via cmake --build (Ninja/Make underneath handles deps).
build: $(CACHE)
	cmake --build $(BUILD_DIR) -j

$(BIN): build

# Aiding flags (baro on/off, differential vs absolute, radar on/off) live
# in src/config.h — shared with firmware. Edit there and rebuild to switch.
RUN_ARGS :=

run: $(BIN)
	mkdir -p "$(OUT)"
	$(BIN) $(RUN_ARGS) "$(LOG)" "$(OUT)"
	@echo "→ $(OUT)/"

compare:
	@test -x $(PY) || { echo "missing $(PY) — create a venv with 'python3 -m venv .venv && .venv/bin/pip install evo pyulog matplotlib pandas scipy'"; exit 1; }
	$(PY) tools/replay/scripts/compare_with_ulog.py "$(OUT)" "$(ULG)" $(ARGS)
	$(PY) tools/replay/scripts/plot_runs.py "$(OUT)" --save

clean:
	rm -rf $(BUILD_DIR)

help:
	@echo "targets:"
	@echo "  make all       — configure + build rio_replay (default)"
	@echo "  make run       — replay LOG into OUT"
	@echo "  make compare   — run compare_with_ulog.py OUT vs ULG"
	@echo "  make clean     — remove $(BUILD_DIR)"
	@echo ""
	@echo "inputs (set RIO_LOG / RIO_ULG in your env, or override on CLI):"
	@echo "  LOG=$(LOG)  [$(call src,LOG,RIO_LOG)]"
	@echo "  ULG=$(ULG)  [$(call src,ULG,RIO_ULG)]"
	@echo ""
	@echo "other variables:"
	@echo "  NAME=$(NAME)         (derived from LOG basename)"
	@echo "  OUT=$(OUT)"
	@echo "  PY=$(PY)"
	@echo ""
	@echo "replay aiding flags: edit src/config.h (shared with firmware):"
	@echo "    BARO_AIDING_ENABLED       (0/1) — master baro on/off"
	@echo "    BARO_AIDING_DIFFERENTIAL  (0/1) — 1=differential, 0=absolute"
	@echo "    RADAR_AIDING_ENABLED      (0/1) — master radar on/off"
	@echo "  then 'make build' to pick up the change."
	@echo ""
	@echo "compare flags (forwarded to compare_with_ulog.py):"
	@echo "  ARGS=$(ARGS)"
	@echo "  shortcuts:"
	@echo "    NO_VVO=1     → adds --no-vvo   (suppress live VVO overlay)"
	@echo "    TOL=0.3      → adds --tol 0.3  (association tolerance, s)"
