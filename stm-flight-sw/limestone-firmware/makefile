## This is a template makefile, copy into your project directory and customize

.PHONY: gen-version

ifdef OS
# This is Windows (native CMD or PowerShell environment)
ifeq ($(powershell where uv 2>$(DEVNUL)),)
UVDIR := $(HOMEPATH)/.local/bin/uv
UV_INSTALL := powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
else
UV_INSTALL := @(echo "uv already installed")
UVDIR := uv
endif
else
# Otherwise assume Linux/macOS Unix-like
ifneq ($(shell command -v ),)
UV_INSTALL := @(echo "uv already installed")
UVDIR := uv
else ifneq ($(shell command -v $(HOME)/.local/bin/uv),)
UV_INSTALL := @(echo "uv already installed")
UVDIR := $(HOME)/.local/bin/uv
else
UV_INSTALL := curl -LsSf https://astral.sh/uv/install.sh | sh
UVDIR := $(HOME)/.local/bin/uv
endif
endif

GIT_ROOT := $(shell git rev-parse --show-toplevel)
UV_CMD := $(UVDIR) run "$(GIT_ROOT)/stm-flight-sw/version-info/version-gen.py" ./Core/Inc/git-hash.h "$(PWD)"

gen-version:
	-$(UV_INSTALL) 
	$(UV_CMD)