#!/bin/bash

echo "Checking for Python executable..."

PYTHON=$(which python || which python3)

if [ -z "$PYTHON" ]; then
    echo "No Python executable found."
    exit 1
fi

echo "Using Python at $PYTHON"

echo "Checking for pip..."

if ! command -v pip &> /dev/null
then
    echo "pip could not be found, please install it."
    exit 1
fi

echo "pip is installed."

echo "Installing packages from requirements.txt..."
pip install -r requirements.txt

echo "Packages installed."

SITE_PACKAGES=$($PYTHON -c 'import site; print(site.getsitepackages()[0])')

echo "Python site-packages directory: $SITE_PACKAGES"

if [ -d "$SITE_PACKAGES/detr" ]; then
    echo "'detr' folder already exists in the site-packages. No action taken."
else
    
    echo "Moving 'detr' folder to site-packages..."
    mv detr $SITE_PACKAGES/detr
    echo "Move complete."
fi


DISTRO=noetic 

if ! command -v catkin &> /dev/null; then
    echo "Catkin tools not found, installing them..."
    sudo apt-get update
    sudo apt-get install python-catkin-tools
else
    echo "Catkin tools are installed."
fi


echo "Initializing rosdep..."
sudo rosdep init
rosdep update

cd panda_simulator/

echo "Installing dependencies with rosdep..."
rosdep install --from-paths src --ignore-src --rosdistro $DISTRO

# Build the workspace using catkin
echo "Building the workspace with catkin build..."
catkin build


if grep -Fxq "source ~/panda_simulator/devel/setup.bash" ~/.bashrc; then
    echo "Source already added to .bashrc"
else
    echo "Adding source to .bashrc..."
    echo "source ~/panda_simulator/devel/setup.bash" >> ~/.bashrc
fi

echo "Setup complete."
