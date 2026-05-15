# Root makefile - forwards commands to PostProcessing/makefile
# Usage: make [target] from the root directory

.PHONY: all
all:
	$(MAKE) -C PostProcessing all
	
.PHONY: clean
clean:
	$(MAKE) -C PostProcessing clean

.PHONY: clean-markers
clean-markers:
	$(MAKE) -C PostProcessing clean-markers

.PHONY: clean-all
clean-all:
	$(MAKE) -C PostProcessing clean-all

.PHONY: distclean
distclean:
	$(MAKE) -C PostProcessing distclean
.PHONY: help
help:
	@echo "Root Makefile - PostProcessing targets"
	@echo ""
	@echo "Run 'make [target]' from the root directory:"
	@echo "  all              - Compile and process all .bin files"
	@echo "  watch            - Monitor inputs folder and auto-process new .bin files"
	@echo "  clean            - Remove compiled executable"
	@echo "  clean-markers    - Mark all outputs for reprocessing"
	@echo "  clean-all        - Remove executable and markers"
	@echo "  distclean        - Remove executable, markers, and all output folders"
	@echo "  status           - Show input and output file status"
	@echo "  help             - Show this help message"
