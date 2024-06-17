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

if ! command -v pip3 &> /dev/null
then
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
    apt-get install python-catkin-tools -y  # Adjust python3-catkin-tools for newer distributions
    if [ $? -ne 0 ]; then
        echo "Failed to install Catkin tools."
        exit 1
    fi
else
    echo "Catkin tools are installed."
fi

# Initialize rosdep safely
echo "Initializing rosdep..."
if ! sudo rosdep init && rosdep update; then
    echo "Failed to initialize or update rosdep."
    exit 1
fi

# Install gdown for Google Drive downloads
if ! command -v gdown &> /dev/null; then
    echo "Installing gdown..."
    if ! pip3 install gdown; then
        echo "Failed to install gdown."
        exit 1
    fi
fi

cd panda_simulator/ || { echo "Failed to change directory to panda_simulator/"; exit 1; }

# Handle Google Drive download and extraction more robustly
echo "Downloading src.zip from Google Drive..."
if ! gdown "https://drive.google.com/uc?id=1yIaI9Ndl1dIDdq8fLU3-7SlktDi84qVf" -O src.zip; then
    echo "Failed to download src.zip."
    exit 1
fi

if [ -f src.zip ]; then
    echo "Extracting src.zip..."
    if ! unzip src.zip -d .; then
        echo "Failed to extract src.zip."
        exit 1
    fi
    echo "Extraction complete."
else
    echo "src.zip not found."
    exit 1
fi

echo "Installing dependencies with rosdep..."
if ! rosdep install --from-paths src --ignore-src --rosdistro $DISTRO; then
    echo "Failed to install dependencies with rosdep."
    exit 1
fi

# Build workspace with catkin
echo "Building the workspace with catkin build..."
if ! catkin build; then
    echo "Failed to build workspace."
    exit 1
fi

# Source ROS setup in .bashrc robustly
if ! grep -Fxq "source ~/panda_simulator/devel/setup.bash" ~/.bashrc; then
    echo "Adding source to .bashrc..."
    echo "source ~/panda_simulator/devel/setup.bash" >> ~/.bashrc
    echo "Source added to .bashrc."
fi

echo "Setup complete."
