# Dandelion OS - Unikraft Development Makefile
# 
# Usage:
#   make setup          - One-time setup: start buildkitd container
#   make build          - Build the CPIO image via kraft
#   make run            - Run DandelionOs in QEMU (foreground)
#   make run-bg         - Run DandelionOs in QEMU (background)
#   make test           - Run the server test against running instance
#   make stop           - Stop background QEMU instance
#   make rebuild        - Clean unikraft build and rebuild
#   make logs           - Show logs from background QEMU
#   make all            - Build and run in foreground
#
# Quick iteration cycle:
#   Terminal 1: make run
#   Terminal 2: make test

.PHONY: setup build run run-bg test stop rebuild logs all clean help check-buildkit

# Configuration
KERNEL := EXTRACTED_BASE_LATEST_BINARIES/kernel
INITRD := .unikraft/build/initramfs-x86_64.cpio
QEMU_PIDFILE := /tmp/dandelion-qemu.pid
QEMU_LOGFILE := /tmp/dandelion-qemu.log
MEMORY := 1G
CPUS := 2
PORT := 8080

# Export for kraft
export KRAFTKIT_BUILDKIT_HOST := docker-container://buildkitd

# QEMU command (shared between run and run-bg)
QEMU_CMD := qemu-system-x86_64 \
  -kernel $(KERNEL) \
  -initrd $(INITRD) \
  -machine pc,accel=kvm \
  -cpu host,+x2apic,-pmu \
  -m $(MEMORY) \
  -smp cpus=$(CPUS) \
  -device virtio-net-pci,mac=02:b0:b0:d3:d2:01,netdev=hostnet0 \
  -netdev user,id=hostnet0,hostfwd=tcp::$(PORT)-:$(PORT) \
  -nographic -no-reboot -parallel none \
  -rtc base=utc \
  -append 'vfs.fstab=[ "initrd0:/:extract:::" ] env.vars=[ "PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" ] -- /dandelionOs'

help:
	@echo "Dandelion OS - Unikraft Development Makefile"
	@echo ""
	@echo "Setup (once per session):"
	@echo "  make setup          - Start buildkitd container"
	@echo ""
	@echo "Development cycle:"
	@echo "  make build          - Build CPIO image via kraft"
	@echo "  make run            - Run in QEMU (foreground, Ctrl-a x to stop)"
	@echo "  make run-bg         - Run in QEMU (background)"
	@echo "  make test           - Run server test against localhost:8080"
	@echo "  make stop           - Stop background QEMU"
	@echo "  make logs           - Show background QEMU logs"
	@echo ""
	@echo "Cleanup:"
	@echo "  make rebuild        - Clean .unikraft and rebuild"
	@echo "  make clean          - Remove build artifacts"
	@echo ""
	@echo "Quick iteration (two terminals):"
	@echo "  Terminal 1: make run"
	@echo "  Terminal 2: make test"

# One-time setup: ensure buildkitd is running
setup: check-buildkit
	@echo "✓ Buildkit container is ready"
	@echo "KRAFTKIT_BUILDKIT_HOST is set to: $(KRAFTKIT_BUILDKIT_HOST)"

check-buildkit:
	@if ! docker container ps | grep -q buildkitd; then \
		echo "Starting buildkitd container..."; \
		docker start buildkitd 2>/dev/null || docker run -d --name buildkitd --privileged moby/buildkit:latest; \
	fi
	@docker container ps | grep buildkitd > /dev/null || (echo "ERROR: buildkitd not running" && exit 1)

# Build the CPIO image (runs cargo build first to avoid stale binaries)
build: check-buildkit
	@echo "Building Rust binary..."
	cargo build --bin dandelion_server --features "unikraft,reqwest_io"
	@echo "Building CPIO image..."
	kraft build --plat qemu --arch x86_64
	@echo "✓ Build complete: $(INITRD)"

# Run in foreground (interactive)
run: $(INITRD) stop build
	@echo "Starting DandelionOs (Ctrl-a x to stop)..."
	@echo "Waiting for requests on localhost:$(PORT)"
	$(QEMU_CMD)

# Run in background
run-bg: $(INITRD) stop build
	@echo "Starting DandelionOs in background..."
	@nohup $(QEMU_CMD) > $(QEMU_LOGFILE) 2>&1 & echo $$! > $(QEMU_PIDFILE)
	@sleep 2
	@if kill -0 $$(cat $(QEMU_PIDFILE)) 2>/dev/null; then \
		echo "✓ DandelionOs running (PID: $$(cat $(QEMU_PIDFILE)))"; \
		echo "  Logs: $(QEMU_LOGFILE)"; \
		echo "  Stop: make stop"; \
	else \
		echo "ERROR: QEMU failed to start. Check $(QEMU_LOGFILE)"; \
		exit 1; \
	fi

# Check if QEMU is running
.PHONY: status
status:
	@if [ -f $(QEMU_PIDFILE) ] && kill -0 $$(cat $(QEMU_PIDFILE)) 2>/dev/null; then \
		echo "DandelionOs is running (PID: $$(cat $(QEMU_PIDFILE)))"; \
	else \
		echo "DandelionOs is not running"; \
	fi

# Show logs from background run
logs:
	@if [ -f $(QEMU_LOGFILE) ]; then \
		cat $(QEMU_LOGFILE); \
	else \
		echo "No log file found. Run 'make run-bg' first."; \
	fi

# Tail logs (follow mode)
.PHONY: logs-follow
logs-follow:
	@if [ -f $(QEMU_LOGFILE) ]; then \
		tail -f $(QEMU_LOGFILE); \
	else \
		echo "No log file found. Run 'make run-bg' first."; \
	fi

# Stop background QEMU (safe to call even if nothing is running)
stop:
	-@if [ -f $(QEMU_PIDFILE) ]; then \
		PID=$$(cat $(QEMU_PIDFILE)); \
		if kill -0 $$PID 2>/dev/null; then \
			echo "Stopping DandelionOs (PID: $$PID)..."; \
			kill $$PID 2>/dev/null; \
			sleep 1; \
			kill -9 $$PID 2>/dev/null; \
		fi; \
		rm -f $(QEMU_PIDFILE); \
	fi
	-@pkill -f "qemu-system-x86_64.*$(KERNEL)" 2>/dev/null; true
	@echo "✓ Stop complete"

# Run the server test (assumes DandelionOs is running)
test:
	@echo "Running server test against localhost:$(PORT)..."
	@echo "Make sure DandelionOs is running (make run or make run-bg)"
	@echo ""
	cargo test --package dandelion_server --test server_tests --features "unikraft,reqwest_io" \
		-- server_tests::serve_matmul_http_2_qemu --exact --nocapture

# Run test with verbose output
.PHONY: test-verbose
test-verbose:
	RUST_LOG=debug cargo test --package dandelion_server --test server_tests --features "unikraft,reqwest_io" \
		-- server_tests::serve_matmul_http_2_qemu --exact --nocapture

# Wait for server to be ready, then test
.PHONY: wait-and-test
wait-and-test:
	@echo "Waiting for server at localhost:$(PORT)..."
	@for i in 1 2 3 4 5 6 7 8 9 10; do \
		if curl -s -o /dev/null -w '' http://localhost:$(PORT)/ 2>/dev/null; then \
			echo "Server is ready!"; \
			break; \
		fi; \
		echo "  Attempt $$i/10..."; \
		sleep 2; \
	done
	@$(MAKE) test

# Clean and rebuild
rebuild: clean-unikraft build

# Clean unikraft build artifacts
clean-unikraft:
	@echo "Cleaning unikraft build artifacts..."
	rm -rf .unikraft
	rm -f .config.dandelion-os_qemu-x86_64

# Full clean
clean: clean-unikraft stop
	rm -f $(QEMU_LOGFILE)

# Build and run
all: build run

# Ensure initrd exists
$(INITRD):
	@echo "CPIO image not found. Building..."
	@$(MAKE) build

# Cargo build (for local testing without unikraft)
.PHONY: cargo-build
cargo-build:
	cargo build --bin dandelion_server --features "unikraft,reqwest_io"

# Check compilation
.PHONY: cargo-check
cargo-check:
	cargo check -p machine_interface --features unikraft

# Debug: attach GDB to running QEMU
.PHONY: debug-attach
debug-attach:
	@echo "To debug, run QEMU with: make run-debug"
	@echo "Then in another terminal: make gdb"

.PHONY: run-debug
run-debug: $(INITRD)
	@echo "Starting DandelionOs with GDB server (port 1234)..."
	@echo "In another terminal, run: make gdb"
	$(QEMU_CMD) -s -S

.PHONY: gdb
gdb:
	gdb --eval-command="target remote :1234" .unikraft/build/dandelion-os_qemu-x86_64.dbg
