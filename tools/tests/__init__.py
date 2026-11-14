import os

# RALLYSIM_EXE may be relative to the folder the tests start in (CI gives build-release/rallysim). Some tests run
# commands from other folders, so it is made absolute once, here, for every test and every process they start.
if os.environ.get("RALLYSIM_EXE"):
    os.environ["RALLYSIM_EXE"] = os.path.abspath(os.environ["RALLYSIM_EXE"])
