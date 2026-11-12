# Forwards every target to HoverBoardMindMotion/Makefile, so the build runs from the repo root:
#   make MCU=F031 both   make targets   make release   make help
.DEFAULT_GOAL := all
%:
	@$(MAKE) --no-print-directory -C HoverBoardMindMotion $@
all:
	@$(MAKE) --no-print-directory -C HoverBoardMindMotion all
.PHONY: all
