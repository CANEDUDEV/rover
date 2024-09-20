#!/usr/bin/env bash

# Only works in WSL
python.exe -m PyInstaller \
    --name=rover-demo \
    --clean \
    --onefile \
    --windowed \
    --add-data='../rover.dbc;.' \
    --workpath='build/rover-demo-tmp' \
    --specpath='build' \
    --distpath='build/rover-demo' \
    --noconfirm \
    rover_py/live_demo.py
