# Makefile — camazotz-ixchel RP2350 firmware
# Wraps CMake/Ninja build system with convenient targets

# ── Tool paths (override via env, e.g. CMAKE=/usr/bin/cmake make build)
PICO_SDK      ?= $(HOME)/.pico-sdk
CMAKE         ?= $(PICO_SDK)/cmake/v3.31.5/bin/cmake
HOST_CMAKE    ?= $(shell command -v cmake 2>/dev/null || echo $(CMAKE))
NINJA         ?= $(PICO_SDK)/ninja/v1.12.1/ninja
PICOTOOL      ?= $(PICO_SDK)/picotool/2.2.0-a4/picotool/picotool
OPENOCD       ?= $(PICO_SDK)/openocd/0.12.0+dev/openocd
TOOLCHAIN     ?= $(PICO_SDK)/toolchain/14_2_Rel1/bin
GDB           ?= $(TOOLCHAIN)/arm-none-eabi-gdb
SIZE          := $(TOOLCHAIN)/arm-none-eabi-size
OBJDUMP       := $(TOOLCHAIN)/arm-none-eabi-objdump
NM            := $(TOOLCHAIN)/arm-none-eabi-nm
READELF       := $(TOOLCHAIN)/arm-none-eabi-readelf
BAUD          ?= 115200

# ── Serial monitor script (exported for use in recipes) ──────────
define SERIAL_PY
import serial, signal, sys
port, baud = sys.argv[1], int(sys.argv[2])
s = serial.Serial(port, baud, timeout=0.5)
signal.signal(signal.SIGINT, lambda *a: (s.close(), sys.exit(0)))
try:
    while True:
        data = s.read(s.in_waiting or 1)
        if data:
            sys.stdout.write(data.decode('utf-8', errors='replace'))
            sys.stdout.flush()
finally:
    s.close()
endef
export SERIAL_PY

# ── Project paths ───────────────────────────────────────────────────
BUILD_DIR     := build
TEST_BUILD_DIR := build-test
FW_ELF        := $(BUILD_DIR)/src/mapper.elf
FW_UF2        := $(BUILD_DIR)/src/mapper.uf2
FW_BIN        := $(BUILD_DIR)/src/mapper.bin
FW_MAP        := $(BUILD_DIR)/src/mapper.elf.map
NPROC         := $(shell sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)

# ── Source globs (for format/lint targets) ──────────────────────────
SOURCES       := $(shell find src drivers logic utils -name '*.cpp' -o -name '*.hpp' -o -name '*.h' -o -name '*.c' 2>/dev/null)
SOURCES       += config.h types.h

# ── Colors ──────────────────────────────────────────────────────────
BOLD := \033[1m
GREEN := \033[32m
YELLOW := \033[33m
CYAN := \033[36m
RESET := \033[0m

.PHONY: all build configure clean rebuild flash flash-swd reboot bootsel \
        test test-verbose test-clean serial info size symbols sections stack \
        format format-check lint help monitor debug erase run

.DEFAULT_GOAL := help

# ═══════════════════════════════════════════════════════════════════
# Build
# ═══════════════════════════════════════════════════════════════════

all: build ## Alias for 'build'

configure: ## Run CMake configuration (creates build directory)
	@mkdir -p $(BUILD_DIR)
	@printf "$(CYAN)Configuring...$(RESET)\n"
	@cd $(BUILD_DIR) && $(CMAKE) ..

build: | $(BUILD_DIR)/build.ninja ## Build firmware
	@printf "$(CYAN)Building with $(NPROC) cores...$(RESET)\n"
	@cd $(BUILD_DIR) && $(CMAKE) --build . -j$(NPROC)
	@printf "$(GREEN)Build complete: $(FW_UF2)$(RESET)\n"

$(BUILD_DIR)/build.ninja:
	@$(MAKE) --no-print-directory configure

rebuild: clean build ## Clean then build

clean: ## Remove build artifacts
	@printf "$(YELLOW)Cleaning build directories...$(RESET)\n"
	@rm -rf $(BUILD_DIR) $(TEST_BUILD_DIR)

clean-obj: ## Remove object files only (keep CMake cache for faster rebuilds)
	@printf "$(YELLOW)Cleaning object files...$(RESET)\n"
	@cd $(BUILD_DIR) && $(NINJA) -t clean 2>/dev/null || true

# ═══════════════════════════════════════════════════════════════════
# Flash & Device
# ═══════════════════════════════════════════════════════════════════

flash: build ## Build and flash via USB (picotool)
	@printf "$(CYAN)Flashing via USB...$(RESET)\n"
	@$(PICOTOOL) load $(FW_UF2) -f || \
	{ \
		printf "$(YELLOW)Device didn't enter BOOTSEL automatically.$(RESET)\n"; \
		printf "$(BOLD)Hold BOOTSEL + press RESET (or re-plug USB while holding BOOTSEL)$(RESET)\n"; \
		printf "$(CYAN)Waiting for device in BOOTSEL mode...$(RESET)\n"; \
		$(PICOTOOL) load $(FW_UF2) -F; \
	}
	@printf "$(GREEN)Flashed.$(RESET)\n"
	@$(PICOTOOL) reboot -f 2>/dev/null || true

run: flash ## Build, flash, and open serial monitor
	@printf "$(CYAN)Waiting for USB CDC enumeration...$(RESET)\n"
	@for i in 1 2 3 4 5 6 7 8 9 10; do \
		PORT=$$(ls /dev/cu.usbmodem* /dev/ttyACM* 2>/dev/null | head -1); \
		if [ -n "$$PORT" ]; then break; fi; \
		sleep 0.5; \
	done; \
	if [ -z "$$PORT" ]; then \
		printf "$(YELLOW)No USB CDC device found after 5s. Is the Pico connected?$(RESET)\n"; \
		exit 1; \
	fi; \
	printf "$(GREEN)Connected to $$PORT @ $(BAUD) baud (Ctrl-C to exit)$(RESET)\n"; \
	python3 -c "$$SERIAL_PY" "$$PORT" $(BAUD)

flash-swd: build ## Build and flash via SWD (OpenOCD)
	@printf "$(CYAN)Flashing via SWD...$(RESET)\n"
	$(OPENOCD) -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
		-c "adapter speed 5000" \
		-c "program $(FW_ELF) verify reset exit"

reboot: ## Reboot device (app mode)
	$(PICOTOOL) reboot -f

bootsel: ## Reboot into BOOTSEL mode
	$(PICOTOOL) reboot -f -u

info: ## Show connected device info
	@$(PICOTOOL) info -a 2>/dev/null || printf "$(YELLOW)No device found (is it connected?)$(RESET)\n"

erase: ## Erase device flash via SWD
	@printf "$(YELLOW)Erasing flash...$(RESET)\n"
	$(OPENOCD) -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
		-c "adapter speed 5000" \
		-c "init" -c "reset halt" -c "flash erase_address 0x10000000 0x200000" -c "reset run" -c "exit"

debug: build ## Launch GDB debug session via OpenOCD + SWD
	@printf "$(CYAN)Starting OpenOCD + GDB...$(RESET)\n"
	@printf "$(BOLD)Tip:$(RESET) In GDB use 'monitor reset halt' to restart, 'c' to continue\n"
	$(OPENOCD) -f interface/cmsis-dap.cfg -f target/rp2350.cfg \
		-c "adapter speed 5000" &
	@sleep 1
	$(GDB) $(FW_ELF) \
		-ex "target extended-remote :3333" \
		-ex "monitor reset halt" \
		-ex "load"
	@-pkill -f "openocd.*rp2350" 2>/dev/null || true

# ═══════════════════════════════════════════════════════════════════
# Serial Monitor
# ═══════════════════════════════════════════════════════════════════

serial: ## Open serial monitor (override port: SERIAL=/dev/ttyACM0, baud: BAUD=115200)
	@PORT=$(SERIAL); \
	if [ -z "$$PORT" ]; then \
		PORT=$$(ls /dev/cu.usbmodem* /dev/ttyACM* 2>/dev/null | head -1); \
	fi; \
	if [ -z "$$PORT" ]; then \
		printf "$(YELLOW)No USB CDC device found. Is the Pico connected and running?$(RESET)\n"; \
		exit 1; \
	fi; \
	printf "$(CYAN)Connecting to $$PORT @ $(BAUD) baud (Ctrl-C to exit)...$(RESET)\n"; \
	python3 -c "$$SERIAL_PY" "$$PORT" $(BAUD)

monitor: serial ## Alias for 'serial'

# ═══════════════════════════════════════════════════════════════════
# Testing
# ═══════════════════════════════════════════════════════════════════

test: | $(TEST_BUILD_DIR)/Makefile ## Build and run unit tests
	@printf "$(CYAN)Building tests...$(RESET)\n"
	@cd $(TEST_BUILD_DIR) && $(HOST_CMAKE) --build . -j$(NPROC)
	@printf "$(CYAN)Running tests...$(RESET)\n"
	@cd $(TEST_BUILD_DIR) && ctest --output-on-failure

$(TEST_BUILD_DIR)/Makefile:
	@mkdir -p $(TEST_BUILD_DIR)
	@printf "$(CYAN)Configuring tests...$(RESET)\n"
	@cd $(TEST_BUILD_DIR) && $(HOST_CMAKE) ../test

test-verbose: test ## Build and run tests with verbose output
	@cd $(TEST_BUILD_DIR) && ctest --output-on-failure --verbose

test-clean: ## Remove test build artifacts
	@printf "$(YELLOW)Cleaning test build directory...$(RESET)\n"
	@rm -rf $(TEST_BUILD_DIR)

# ═══════════════════════════════════════════════════════════════════
# Firmware Analysis
# ═══════════════════════════════════════════════════════════════════

size: build ## Show firmware size breakdown (text/data/bss)
	@printf "$(BOLD)Firmware Size:$(RESET)\n"
	@$(SIZE) $(FW_ELF)
	@printf "\n$(BOLD)UF2: $(RESET)"
	@ls -lh $(FW_UF2) | awk '{print $$5}'

symbols: build ## List largest symbols (top 30 by size)
	@printf "$(BOLD)Top 30 symbols by size:$(RESET)\n"
	@$(NM) --print-size --size-sort --reverse-sort $(FW_ELF) | head -30

sections: build ## Show ELF section sizes
	@printf "$(BOLD)ELF Sections:$(RESET)\n"
	@$(SIZE) -A $(FW_ELF) | head -40

stack: ## Show stack usage (.su files from -fstack-usage)
	@printf "$(BOLD)Top 20 stack consumers:$(RESET)\n"
	@find $(BUILD_DIR) -name '*.su' -exec cat {} + 2>/dev/null | \
		sort -t$$'\t' -k2 -rn | head -20 || \
		printf "$(YELLOW)No .su files found. Build first.$(RESET)\n"

map: build ## Show memory map summary (RAM/Flash usage)
	@printf "$(BOLD)Memory regions from linker map:$(RESET)\n"
	@grep -A1 'Memory Configuration' $(FW_MAP) 2>/dev/null | head -10
	@printf "\n$(BOLD)Largest input sections:$(RESET)\n"
	@grep '\.text\.' $(FW_MAP) 2>/dev/null | awk '{print $$1, $$2}' | sort -k2 -rn | head -15

# ═══════════════════════════════════════════════════════════════════
# Code Quality
# ═══════════════════════════════════════════════════════════════════

format: ## Format all source files with clang-format
	@command -v clang-format >/dev/null 2>&1 || \
		{ printf "$(YELLOW)clang-format not found. Install: brew install clang-format$(RESET)\n"; exit 1; }
	@printf "$(CYAN)Formatting $(words $(SOURCES)) files...$(RESET)\n"
	@clang-format -i $(SOURCES)
	@printf "$(GREEN)Done.$(RESET)\n"

format-check: ## Check formatting without modifying files
	@command -v clang-format >/dev/null 2>&1 || \
		{ printf "$(YELLOW)clang-format not found. Install: brew install clang-format$(RESET)\n"; exit 1; }
	@printf "$(CYAN)Checking format...$(RESET)\n"
	@clang-format --dry-run --Werror $(SOURCES) && \
		printf "$(GREEN)All files formatted correctly.$(RESET)\n" || \
		{ printf "$(YELLOW)Run 'make format' to fix.$(RESET)\n"; exit 1; }

lint: build ## Run clang-tidy on project sources (requires compile_commands.json)
	@command -v clang-tidy >/dev/null 2>&1 || \
		{ printf "$(YELLOW)clang-tidy not found. Install: brew install llvm$(RESET)\n"; exit 1; }
	@printf "$(CYAN)Running clang-tidy...$(RESET)\n"
	@clang-tidy -p $(BUILD_DIR) $(filter %.cpp,$(SOURCES)) 2>/dev/null || true

# ═══════════════════════════════════════════════════════════════════
# Utilities
# ═══════════════════════════════════════════════════════════════════

compile-commands: configure ## Generate compile_commands.json for IDE
	@printf "$(GREEN)compile_commands.json generated at $(BUILD_DIR)/compile_commands.json$(RESET)\n"
	@printf "Symlink for IDE: "
	@ln -sf $(BUILD_DIR)/compile_commands.json compile_commands.json 2>/dev/null && \
		printf "$(GREEN)OK$(RESET)\n" || printf "$(YELLOW)already exists$(RESET)\n"

deps: ## Show fetched dependency versions
	@printf "$(BOLD)FetchContent Dependencies:$(RESET)\n"
	@printf "  BNO08x (IMU):    "; cd $(BUILD_DIR)/_deps/bno08x-src 2>/dev/null && git log --oneline -1 || printf "not fetched\n"
	@printf "  AS5600 (Encoder): "; cd $(BUILD_DIR)/_deps/as5600-src 2>/dev/null && git log --oneline -1 || printf "not fetched\n"
	@printf "  FatFS (SD):       "; cd $(BUILD_DIR)/_deps/fatfs-src 2>/dev/null && git log --oneline -1 || printf "not fetched\n"
	@printf "  GoogleTest:       "; cd $(BUILD_DIR)/_deps/googletest-src 2>/dev/null && git log --oneline -1 || printf "not fetched\n"

sdk-version: ## Show Pico SDK and toolchain versions
	@printf "$(BOLD)Pico SDK:$(RESET)    "; cat $(PICO_SDK)/sdk/2.2.0/pico_sdk_version.cmake 2>/dev/null | grep 'PICO_SDK_VERSION_STRING' | sed 's/.*"\(.*\)".*/\1/' || echo "unknown"
	@printf "$(BOLD)Toolchain:$(RESET)   "; $(TOOLCHAIN)/arm-none-eabi-gcc --version | head -1
	@printf "$(BOLD)CMake:$(RESET)       "; $(CMAKE) --version | head -1
	@printf "$(BOLD)Picotool:$(RESET)    "; $(PICOTOOL) version 2>/dev/null || echo "unknown"
	@printf "$(BOLD)OpenOCD:$(RESET)     "; $(OPENOCD) --version 2>&1 | head -1

# ═══════════════════════════════════════════════════════════════════
# Git
# ═══════════════════════════════════════════════════════════════════

diff: ## Show uncommitted changes
	@git diff --stat
	@printf "\n"
	@git diff

status: ## Show git status
	@git status -sb

log: ## Show recent git log
	@git log --oneline --graph -20

# ═══════════════════════════════════════════════════════════════════
# Help
# ═══════════════════════════════════════════════════════════════════

help: ## Show this help
	@printf "$(BOLD)camazotz-ixchel — RP2350 Cave Mapper Firmware$(RESET)\n\n"
	@printf "$(BOLD)Usage:$(RESET) make $(CYAN)<target>$(RESET)\n\n"
	@awk 'BEGIN {FS = ":.*##"} /^[a-zA-Z_-]+:.*##/ { \
		printf "  $(CYAN)%-18s$(RESET) %s\n", $$1, $$2 \
	}' $(MAKEFILE_LIST)
	@printf "\n$(BOLD)Examples:$(RESET)\n"
	@printf "  make build                    Build firmware\n"
	@printf "  make flash                    Build + flash via USB\n"
	@printf "  make run                      Build + flash + serial monitor\n"
	@printf "  make test                     Run unit tests\n"
	@printf "  make size                     Show firmware size breakdown\n"
	@printf "  make serial                   Open serial monitor\n"
	@printf "  make serial SERIAL=/dev/ttyACM0   Specify serial port\n"
	@printf "  make debug                    GDB debug session via SWD\n"
