ifdef OS
    # This is Windows (native CMD or PowerShell environment)
    UV_CMD = $(HOMEPATH)/.local/bin/uv run version-gen.py $(PWD)/../Core/Inc/git-hash.h

else
    # Otherwise assume Linux/macOS Unix-like
    UV_CMD = $(HOME)/.local/bin/uv run version-gen.py $(PWD)/../Core/Inc/git-hash.h
endif

version-gen:
	@$(UV_CMD)