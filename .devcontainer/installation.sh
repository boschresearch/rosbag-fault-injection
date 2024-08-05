sudo apt-get update && sudo apt-get upgrade -y \
&& pip3 install --user -r .devcontainer/requirements.txt \
&& colcon build \
&& echo "source install/setup.bash" >> ~/.bashrc