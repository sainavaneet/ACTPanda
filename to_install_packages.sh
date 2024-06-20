#!/bin/bash

# Enforce running the script with superuser privileges
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

echo "Checking for Python executable..."

# More robust detection of Python executable
PYTHON=$(which python3 || echo "")

if [ -z "$PYTHON" ]; then
    echo "No Python executable found."
    exit 1
fi

echo "Using Python at $PYTHON"

echo "Checking for pip..."

if ! command -v pip3 &> /dev/null; then
    echo "pip could not be found, installing pip..."
    apt-get update && apt-get install python3-pip -y
    if [ $? -ne 0 ]; then
        echo "Failed to install pip."
        exit 1
    fi
else
    echo "pip is installed."
fi

echo "Installing packages from requirements.txt..."
if ! pip3 install -r requirements.txt; then
    echo "Failed to install packages."
    exit 1
fi

echo "Packages installed."

# Detect Python site-packages directory
SITE_PACKAGES=$($PYTHON -c 'import site; print(site.getsitepackages()[0])')

echo "Python site-packages directory: $SITE_PACKAGES"

# Check and move detr folder if needed
if [ -d "$SITE_PACKAGES/detr" ]; then
    echo "'detr' folder already exists in the site-packages. No action taken."
else
    echo "Moving 'detr' folder to site-packages..."
    if mv detr $SITE_PACKAGES/detr; then
        echo "Move complete."
    else
        echo "Failed to move 'detr' folder."
        exit 1
    fi
fi

# Install catkin tools
if ! command -v catkin &> /dev/null; then
    echo "Catkin tools not found, installing them..."
    pip install catkin-tools -y  # Adjust python3-catkin-tools for newer distributions
    if [ $? -ne 0 ]; then
        echo "Failed to install Catkin tools."
        exit 1
    fi
else
    echo "Catkin tools are installed."
fi
