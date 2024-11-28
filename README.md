# ROS2 Rosbag Fault Injection

![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)
![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Shell Script](https://img.shields.io/badge/shell_script-%23121011.svg?style=for-the-badge&logo=gnu-bash&logoColor=white)

Fault injection for [ROS2](https://github.com/ros2) rosbags that supports recording, modifying, and plotting the fault injected rosbags.

## File Structure

This section provides an overview of the project, including its purpose, features, and architecture. For detailed instructions on how to run the repository, please refer to [Getting Started](#getting-started).

This repository has the following structure:

```
rosbag-fault-injection/
├── .devcontainer/
│   ├── devcontainer.json
│   └── Dockerfile
│   └── installation.sh
│   └── requirements.txt
├── app/
│   ├── num_generator/
│   └── interfaces/
├── src/
│   ├── config.yaml
│   └── fault_injection.py
│   └── fault_types.py
│   └── read_yaml.py
│   └── plot.py
│   └── inject.sh
│   └── record.sh
└── ...
```

### .devcontainer/
- `devcontainer.json`: Configuration file for setting up the development container.
- `Dockerfile`: Dockerfile to build the development container with all necessary dependencies.
- `installation.sh`: Shell script to automate the installation of additional tools and dependencies.
- `requirements.txt`: A file listing Python dependencies to be installed using pip.

### app/
- Example ROS2 project and custom messages to inject fault into.

### src/
- `config.yaml`: Configuration file for fault injection parameters.
- `fault_injection.py`: Main script to perform fault injection on ROS2 rosbags.
- `fault_types.py`: Defines different types of faults that can be injected.
- `read_yaml.py`: Utility script to read and parse YAML configuration files.
- `plot.py`: Script to plot the results of the fault injection.
- `inject.sh`: Shell script to automate the fault injection process.
- `record.sh`: Shell script to automate the recording of ROS2 rosbags.

## Getting Started

### Prerequisite (at host)

* Visual Code
* Docker
    * If you have a Docker Desktop license, Docker Desktop >= 4.27.2
    * If you don't have a Docker Desktop license, [Docker Engine on WSL2](https://www.youtube.com/redirect?event=video_description&redir_token=QUFFLUhqbU8zY0gzaUpIak1nWWM5NExPNlhDYjdYRGh1QXxBQ3Jtc0trMjgzWVZzeGxXSnMtSEpUNlBxeXAwdWg3a29iaVlJcVVIZEdMbzVaeVhOQ3NNSllSa3JnVXlDVVZMLUY2ZzJxSGZQdWNmSDhjRVJWVTYwbFFmRzhzSnJoQ05MNktGRUlCRjdqTDI3OXJ2T2NLVVBhcw&q=https%3A%2F%2Fdocs.docker.com%2Fengine%2Finstall%2Fubuntu%2F%23install-using-the-convenience-script&v=SDk3pqFXgs8) (Docker Engine != Docker Desktop)
        * In case of another installation, make sure you have deleted the previous Docker completely
        * After installation, check if Docker Container Engine is running well
            ```
            # to check if docker container engine is running
            sudo service docker status
            ```
* Python >= 3.8.10
* The user must be added in the `docker` group with sudo previledge
    ```
    # add a group named docker
    sudo gropuadd docker

    # add the current user to the docker group with sudo privilege
    sudo gpasswd -a $USER docker

    # change the current user's primary group to docker
    newgrp docker
    ``` 

### Setup

To clone the repo:
```
git clone https://github.com/boschresearch/rosbag-fault-injection.git
```

To open the Visual Code:
```
# change directory to the project
cd rosbag-fault-injection

# open visual code
code .
```

The project contains `devcontainer` that runs [ROS2 Galactic](https://docs.ros.org/en/galactic/index.html) as a container with all required dependencies and initial setup. 

To open the `devcontainer`, press `Ctrl`+`Shift`+`P` from Visual Code and select `Reopen in Container`. See [Visual Studio Blog](https://code.visualstudio.com/blogs/2020/07/01/containers-wsl#_getting-started) for visual guidelines.

After `devcontainer` is up and running successfully, you can open a `bash` terminal with `vscode` user name. And `ros2` commands are usable right away. :)

## Fault injection

WIP

## Trouble Shooting
Q1. Opening the container fails with `... "docker-credential-desktop.exe": executable file not found in $PATH, out: ...`.

A1. Delete the line with `credsStore` from `~/.docker/config.json` on WSL2.

## License

This project is licenced under the Apache-2.0 license. See the [LICENSE](LICENSE.md) file for more details.
