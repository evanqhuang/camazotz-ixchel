# Makefile — camazotz-ixchel RP2350 firmware
# Wraps CMake/Ninja build system with convenient targets

# ── Load .env (CLOUDFLARE_API_TOKEN for deploy/infra targets) ────
-include .env
export CLOUDFLARE_API_TOKEN

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
NPROC         := $(shell sysctl -n hw.ncpu 2>/dev/null || nproc 2>/dev/null || echo 4)

# ── Colors ──────────────────────────────────────────────────────────
BOLD := \033[1m
GREEN := \033[32m
YELLOW := \033[33m
CYAN := \033[36m
RESET := \033[0m

.PHONY: all build configure clean rebuild flash flash-swd reboot bootsel \
        test test-verbose test-clean serial info size help debug erase run \
        viz-serve viz-deploy viz-3d viz-3d-demo \
        tf-init tf-plan tf-apply tf-destroy tf-fmt tf-validate

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

# ═══════════════════════════════════════════════════════════════════
# Visualizer
# ═══════════════════════════════════════════════════════════════════

INFRA_DIR := visualizer/infra

viz-serve: ## Serve web visualizer locally (http://localhost:8080)
	python3 -m http.server 8080 -d visualizer/web

viz-deploy: ## Deploy web visualizer to Cloudflare Pages
	npx wrangler pages deploy visualizer/web --project-name=camazotz-map

viz-3d: ## Launch PyVista 3D visualizer (usage: make viz-3d CSV=path/to/nav.csv)
	cd visualizer/pyvista && python3 viz.py $(CSV)

viz-3d-demo: ## Launch PyVista 3D visualizer with demo data
	cd visualizer/pyvista && python3 viz.py ../testdata/complex_cave_nav.csv \
		--events ../testdata/complex_cave_events.csv

# ═══════════════════════════════════════════════════════════════════
# Infrastructure
# ═══════════════════════════════════════════════════════════════════

tf-init: ## Initialize Terraform
	terraform -chdir=$(INFRA_DIR) init

tf-plan: ## Preview infrastructure changes
	terraform -chdir=$(INFRA_DIR) plan

tf-apply: ## Apply infrastructure changes
	terraform -chdir=$(INFRA_DIR) apply

tf-destroy: ## Destroy infrastructure
	terraform -chdir=$(INFRA_DIR) destroy

tf-fmt: ## Format Terraform files
	terraform fmt $(INFRA_DIR)

tf-validate: ## Validate Terraform configuration
	terraform -chdir=$(INFRA_DIR) validate

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
