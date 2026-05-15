# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall
TARGET = data_scrapper.out

# Directories
INPUT_DIR = inputs
OUTPUT_DIR = outputs

# Find all .bin files in the inputs directory
BIN_FILES = $(wildcard $(INPUT_DIR)/*.bin)

# Generate output directory markers (one for each bin file)
# These marker files track when processing was completed
OUTPUT_MARKERS = $(patsubst $(INPUT_DIR)/%.bin,$(OUTPUT_DIR)/%/.processed,$(BIN_FILES))

# Default target: ensure executable exists and all input files are processed
.PHONY: all
all: $(TARGET) $(OUTPUT_MARKERS)

# Compile DataScrapper.cpp into executable
$(TARGET): DataScrapper.cpp
	$(CXX) $(CXXFLAGS) -o $@ $<
	@echo "✓ Built $(TARGET)"

# Process each .bin file: when input is newer than output marker, run the program
# Pattern rule: for each output/%/.processed file
$(OUTPUT_DIR)/%/.processed: $(INPUT_DIR)/%.bin $(TARGET)
	@echo "Processing $*.bin..."
	@mkdir -p $(OUTPUT_DIR)/$*
	@cd $(OUTPUT_DIR)/$* && ../../$(TARGET) ../../$(INPUT_DIR)/$*.bin
	@touch $@
	@echo "✓ Completed $* - created output files in $(OUTPUT_DIR)/$*"

# Watch for new .bin files and process them
.PHONY: watch
watch: $(TARGET)
	@echo "Watching for .bin files in $(INPUT_DIR)..."
	@while true; do \
		new_files=$$(find $(INPUT_DIR) -name "*.bin" -newer /tmp/.makefile_watch_marker 2>/dev/null); \
		if [ ! -z "$$new_files" ]; then \
			$(MAKE) all; \
		fi; \
		touch /tmp/.makefile_watch_marker; \
		sleep 2; \
	done

# Clean up compiled executable
.PHONY: clean
clean:
	rm -f $(TARGET)
	@echo "✓ Removed executable"

# Clean up all processed files (but keep CSV outputs)
.PHONY: clean-markers
clean-markers:
	find $(OUTPUT_DIR) -name ".processed" -delete
	@echo "✓ Removed processing markers - next run will reprocess all files"

# Complete clean: remove executable and markers, but preserve generated CSVs
.PHONY: clean-all
clean-all: clean clean-markers
	@echo "✓ Full cleanup complete"

# Remove everything including output folders
.PHONY: distclean
distclean: clean-all
	rm -rf $(OUTPUT_DIR)/*/
	@echo "✓ Removed all output folders"

.PHONY: status
status:
	@echo "Input files:"
	@ls -lh $(INPUT_DIR)/*.bin 2>/dev/null || echo "  (No .bin files found)"
	@echo ""
	@echo "Processed outputs:"
	@for dir in $(OUTPUT_DIR)/*/; do \
		if [ -d "$$dir" ]; then \
			echo "  $$(basename $$dir):"; \
			ls -1 "$$dir"*.csv 2>/dev/null | sed 's/^/    /'; \
		fi; \
	done

.PHONY: help
help:
	@echo "Makefile targets:"
	@echo "  all              - Compile and process all .bin files"
	@echo "  watch            - Monitor inputs folder and auto-process new .bin files"
	@echo "  clean            - Remove compiled executable"
	@echo "  clean-markers    - Mark all outputs for reprocessing"
	@echo "  clean-all        - Remove executable and markers"
	@echo "  distclean        - Remove executable, markers, and all output folders"
	@echo "  status           - Show input and output file status"
	@echo "  help             - Show this help message"
