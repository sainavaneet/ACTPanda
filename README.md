Here is the updated code with the image inclusion and corrected formatting:

```markdown
# ACTPanda Installation Guide

```bash
# Clone the repository
git clone https://github.com/sainavaneet/ACTPanda.git

# Change directory to the cloned repository
cd ACTPanda/

# Make the setup script executable
chmod +x setup.sh

# Run the setup script
./setup.sh
```

```bash
# Install Panda_kinematics

cd ~/ACTPanda/

git clone https://github.com/roboticsleeds/panda_ik.git

cd panda_ik

chmod +x build.sh

./build.sh
```

```bash
# Launch the environment

roslaunch src/panda_simulator/panda_gazebo/launch/pick_and_place.launch
```

## ENVIRONMENT SETUP

![Environment Image](https://github.com/sainavaneet/ACTPanda/raw/main/results/env.jpg)
```

### Note:

- Make sure the URL for the image is correct.
- The code blocks are separated by triple backticks for clarity.
