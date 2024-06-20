#!/bin/bash


# Initialize rosdep safely
echo "Initializing rosdep..."
if ! rosdep init && rosdep update; then
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

# Ensure the panda_simulator directory exists
if [ ! -d "panda_simulator" ]; then
  mkdir panda_simulator
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

# Detect ROS distribution dynamically if not set
if [ -z "$ROS_DISTRO" ]; then
    ROS_DISTRO=$(source /opt/ros/*/setup.bash && echo $ROS_DISTRO)
    if [ -z "$ROS_DISTRO" ]; then
        echo "ROS distribution not found."
        exit 1
    fi
fi

echo "ROS distribution: $ROS_DISTRO"

echo "Installing dependencies with rosdep..."
if ! rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO; then
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
