# Compiler and flags
CXX = g++
CXXFLAGS = -std=c++17 -Wall
TARGET = data_scrapper.out

# Directories
INPUT_DIR = inputs
OUTPUT_DIR = outputs

# Cross-platform command helpers
ifeq ($(OS),Windows_NT)
	SHELL = cmd.exe
	MKDIR_CMD = if not exist "$(subst /,\\,$(OUTPUT_DIR)/$*)" mkdir "$(subst /,\\,$(OUTPUT_DIR)/$*)"
	TOUCH_CMD = type NUL > $(subst /,\\,$@)
	RM_CMD = del /F /Q
	PROCESS_CMD = $(TARGET) "$(subst /,\\,$(INPUT_DIR)/$*.bin)" "$(subst /,\\,$(OUTPUT_DIR)/$*)"
else
	MKDIR_CMD = mkdir -p $(OUTPUT_DIR)/$*
	TOUCH_CMD = touch $@
	RM_CMD = rm -f
	PROCESS_CMD = ./$(TARGET) $(INPUT_DIR)/$*.bin $(OUTPUT_DIR)/$*
endif

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
	@echo "Built $(TARGET)"

# Process each .bin file: when input is newer than output marker, run the program
# Pattern rule: for each output/%/.processed file
$(OUTPUT_DIR)/%/.processed: $(INPUT_DIR)/%.bin $(TARGET)
	@echo "Processing $*.bin..."
	@$(MKDIR_CMD)
	@$(PROCESS_CMD)
	@$(TOUCH_CMD)
	@echo "Completed $* - created output files in $(OUTPUT_DIR)/$*"

# Clean up compiled executable
.PHONY: clean
clean:
	$(RM_CMD) $(TARGET)
	@echo "Removed executable"

# Clean up all processed files (but keep CSV outputs)
.PHONY: clean-markers
clean-markers:
ifeq ($(OS),Windows_NT)
	@for /r $(OUTPUT_DIR) %%F in (.processed) do del "%%F"
else
	@find $(OUTPUT_DIR) -name ".processed" -delete
endif
	@echo "Removed processing markers - next run will reprocess all files"

# Complete clean: remove executable and markers, but preserve generated CSVs
.PHONY: clean-all
clean-all: clean clean-markers
	@echo "Full cleanup complete"

# Remove everything including output folders
.PHONY: distclean
distclean: clean-all
ifeq ($(OS),Windows_NT)
	@for /d %%D in ($(OUTPUT_DIR)\*) do @rmdir /s /q "%%D"
else
	@rm -rf $(OUTPUT_DIR)/*/
endif
	@echo "Removed all output folders"

.PHONY: help
help:
	@echo "Makefile targets:"
	@echo "  all              - Compile and process all .bin files"
	@echo "  clean            - Remove compiled executable"
	@echo "  clean-markers    - Mark all outputs for reprocessing"
	@echo "  clean-all        - Remove executable and markers"
	@echo "  distclean        - Remove executable, markers, and all output folders"
	@echo "  help             - Show this help message"
