#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

if [ -d "$HOME/.simulation-gazebo" ]; then
	echo "~/.simulation-gazebo directory exists, installing custom content..."
	echo

	cp "$SCRIPT_DIR/custom_worlds/"* "$HOME/.simulation-gazebo/worlds"
	cp -r "$SCRIPT_DIR/custom_models/"* "$HOME/.simulation-gazebo/models"

	# Add gz plugins to search directory
	#export GZ_SIM_SYSTEM_PLUGIN_PATH=$GZ_SIM_SYSTEM_PLUGIN_PATH:"$SCRIPT_DIR/gz_plugins/barge_controller/build/"
	export GZ_SIM_SYSTEM_PLUGIN_PATH=$(find "$SCRIPT_DIR/gz_plugins" -type d -name build | paste -sd:)

else
	echo "No ~/.simulation-gazebo found. Install using python file PX4/Tools/simulation/gz/simulation-gazebo"
fi
