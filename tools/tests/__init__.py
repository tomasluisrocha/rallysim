import os

# CI gives RALLYSIM_EXE relative to sim/. Some tests run commands from tools/ or a temporary folder, so the path is
# made absolute here, before any test runs.
if os.environ.get("RALLYSIM_EXE"):
    os.environ["RALLYSIM_EXE"] = os.path.abspath(os.environ["RALLYSIM_EXE"])
