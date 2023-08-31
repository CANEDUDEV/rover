#!/usr/bin/env bash

# Only works in WSL
python.exe -m PyInstaller \
	--name=rover-demo \
	--add-data='../rover.dbc;.' \
	--workpath='build/rover-demo-tmp' \
	--specpath='build' \
	--distpath='build/rover-demo' \
	--noconfirm \
	integration/demo-control.py
