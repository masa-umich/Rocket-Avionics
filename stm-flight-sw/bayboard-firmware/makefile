## This is a template makefile, copy into your project directory and customize

.PHONY: gen-version

ifdef OS
# This is Windows (native CMD or PowerShell environment)
HOMEDIR = $(HOMEPATH)
UV_INSTALL := powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
else
# Otherwise assume Linux/macOS Unix-like
HOMEDIR = $(HOME)
UV_INSTALL := curl -LsSf https://astral.sh/uv/install.sh | sh
endif

GIT_ROOT := $(shell git rev-parse --show-toplevel)
UV_CMD := $(HOMEDIR)/.local/bin/uv run $(GIT_ROOT)/stm-flight-sw/version-info/version-gen.py ./Core/Inc/git-hash.h $(PWD)

gen-version:
	-$(UV_INSTALL) 
	$(UV_CMD)