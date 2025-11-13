## This is a template makefile, copy into your project directory and customize

.PHONY: gen-version

ifdef OS
# This is Windows (native CMD or PowerShell environment)
UVDIR := $(HOMEPATH)/.local/bin/uv
UV_INSTALL := powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
else
# Otherwise assume Linux/macOS Unix-like
PATH := $(PATH):/opt/homebrew/bin:/usr/local/bin
ifeq ($(shell command -v uv),)
UV_INSTALL := curl -LsSf https://astral.sh/uv/install.sh | sh
UVDIR := $(HOME)/.local/bin/uv
else
UV_INSTALL := @(echo "uv already installed")
UVDIR := uv
endif
endif

GIT_ROOT := $(shell git rev-parse --show-toplevel)
UV_CMD := $(UVDIR) run "$(GIT_ROOT)/stm-flight-sw/version-info/version-gen.py" ./Core/Inc/git-hash.h "$(PWD)"

gen-version:
	-$(UV_INSTALL) 
	$(UV_CMD)