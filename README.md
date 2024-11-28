# ROS2 Rosbag Fault Injection

![Docker](https://img.shields.io/badge/docker-%230db7ed.svg?style=for-the-badge&logo=docker&logoColor=white)
![Visual Studio Code](https://img.shields.io/badge/Visual%20Studio%20Code-0078d7.svg?style=for-the-badge&logo=visual-studio-code&logoColor=white)
![ROS](https://img.shields.io/badge/ros-%230A0FF9.svg?style=for-the-badge&logo=ros&logoColor=white)
![Python](https://img.shields.io/badge/python-3670A0?style=for-the-badge&logo=python&logoColor=ffdd54)
![Shell Script](https://img.shields.io/badge/shell_script-%23121011.svg?style=for-the-badge&logo=gnu-bash&logoColor=white)

Fault injection for [ROS2](https://github.com/ros2) rosbags that supports recording, modifying, and plotting the fault injected rosbags.

## Table of Contents

- [File Structure](#file-structure)
    - [.devcontainer/](#devcontainer)
    - [app/](#app)
    - [src/](#src)
- [Getting Started](#getting-started)
    - [Prerequisite (at host)](#prerequisite-at-host)
    - [Setup](#setup)
- [How to Use](#how-to-use)
    - [Recording the Rosbags](#recording-the-rosbags)
    - [Fault Injection](#fault-injection)
- [Trouble Shooting](#trouble-shooting)
- [License](#license)

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
* The user must be added in the `docker` group with sudo privilege
    ```
    # add a group named docker
    sudo groupadd docker

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


## How to Use

The project can be run with two bash scripts: `record.sh` and `inject.sh`, both located in `src` directory.

If you already have a rosbag or rosbags to inject faults into, skip [Recording the Rosbags](#recording-the-rosbags) and read [Fault Injection](#fault-injection).

### Recording the Rosbags

The `record.sh` script is designed to simplify the process of recording rosbags using the `ros2 bag record` and `ros2 launch` commands. This script automates the execution and termination of these commands, making it easier to manage the recording process.

#### Steps to use `record.sh`

1. **Navigate to the Script Directory:**
    Ensure you are in the directory where `record.sh` is located.

    ```sh
    cd /path/to/this/project/src
    ```

2. **Make the Script Executable:**
    If not already executable, you need to change the permissions of the script.

    ```sh
    chmod +x record.sh
    ```

3. **Configure the parameters for `record.sh` script:**
    By default, the script will run the example ROS2 project provided in the `./app/num_generator` directory. If you want to record your own ROS2 package, you need to provide the input parameters to the script. Ensure you know the **launch file name (including the extension)** and the **package name** of your ROS2 project.

    To check the other parameters, execute the script with `-h` flag:
    ```sh
    ./record.sh -h
    ```

4. **Record the rosbags:**
    To start recording rosbags, execute the script with input parameters. For example:

    ```sh
    ./record.sh -d 2000 -l your_launch_file.py -p your_package_name -y
    ```

    The script will handle the execution of the `ros2 bag record` and `ros2 launch` commands, and will also manage the termination of the launch process.

### Fault Injection

Once you have recorded a rosbag or have existing rosbags, you can define your fault injection parameters in the `config.yaml` file located in `./src` directory. This file allows you to specify the types and parameters of faults to be injected. After configuring `config.yaml`, you can use the `inject.sh` script to apply the specified faults to your rosbags.

#### How to create a fault injection model

`config.yaml` in this repository shows an example fault injection model for the example ROS2 project. However, it can be easily modified to inject faults into your rosbags.

Here are explanations on the possible configurations parameters:

| Key | Description | Data Type | Mandatory / Optional | Default value
|----------|---------|---------|---------|---------|
| `seed` | random seed number | unsigned integer | optional | `1` |
| `custom_message_path` | paths to the custom msg files | list of strings | optional | |
| `parameters` | fault injection model per input directory or input file | list of dictionaries | mandatory | |

The `parameters` in the config.yaml file is **a list of dictionaries**, each representing a fault injection model for a specific input directory or input file. Each dictionary within this list can contain various configuration settings that define how faults should be injected into the data. Here are the key parameters that can be included within each dictionary:

| Key | Description | Data type | Mandatory / Optional
|----------|---------|---------|---------|
| `input` | path to the input directory or the input file | string | mandatory |
| `topic_mask_filter` | mask filter for defined fault models in `topics` | list of strings | mandatory |
| `topics` | specific topics to inject faults into | list of dictionaries | mandatory |
| `filters` | specific topics to plot and filter | list of dictionaries | optional (no filters applied) |

The `topics` is a list of dictionaries, where each dictionary defines topics and their associated fault injection parameters. Below is a table describing the keys used in each dictionary:

| Key | Description | Data Type | Mandatory / Optional | Default Value |
|----------|---------|---------|---------|---------|
| `topic` | name of the topic to which the fault injection will be applied | string | mandatory | |
| `type` | type of data being published on the topic | string | mandatory | |
| `fault_type` | type of fault to inject (see `config.yaml` comments) | string | mandatory | |
| `fault_value` | value associated with the fault (see `config.yaml` comments) | varies (see `config.yaml` comments) | mandatory | |
| `start_after_sec`| time in seconds after which the fault injection should start | unsigned integer or unsigned float  | mandatory | `0` |
| `duration_sec` | duration in seconds for which the fault injection should be applied | unsigned integer or unsigned float | mandatory | full duration |

The `filters` is also a list of dictionaries, where each dictionary defines topics and their associated filter parameters for the visual outputs of the fault injection results. Below is a table describing the keys used in each dictionary:

| Key | Description | Data Type | Mandatory / Optional | 
|----------|---------|---------|---------|
| `topic` | name of the topic to which the filter will be applied | string | mandatory |
| `type` | type of data being published on the topic | string | mandatory |
| `filter_type` | type of filter to apply (see `config.yaml` comments) | string | mandatory |
| `filter_value` | value associated with the filter (see `config.yaml` comments) | varies (see `config.yaml` comments) | mandatory |
| `filter_window_size` | filter applied to each window size of data entries | unsigned integer | mandatory |
| `pass_size` | threshold for satisfying the filter conditions when `filter_window_size` is set | unsigned integer | mandatory  when `filter_window_size` is set | 

Example of `filter_window_size` and `pass_size`: When `filter_window_size: 3` and `pass_size: 2` are set, for each window size of `3` data entries, `2` of the data entries should satisfy the filter conditions. Otherwise, the data entries within that window are filtered out.

#### Steps to use `inject.sh`

1. **Navigate to the Script Directory:**
    Ensure you are in the directory where `inject.sh` is located.

    ```sh
    cd /path/to/this/project/src
    ```

2. **Make the Script Executable:**
    If not already executable, you need to change the permissions of the script.

    ```sh
    chmod +x inject.sh
    ```

3. **Configure the `config.yaml`:**
    Edit the `config.yaml` file to define the faults you want to inject.

4. **Configure the parameters for `inject.sh` script:**
    By default, the script will run the example ROS2 project provided in the `./app/num_generator` directory. If you want to inject your own ROS2 package, you need to provide the input parameters to the script. Ensure you know the **package name** of your ROS2 project.

    To check the other parameters, execute the script with `-h` flag:
    ```sh
    ./record.sh -h
    ```

5. **Inject Faults:**
    To inject faults into your rosbags, execute the script with input parameters. For example:

    ```sh
    ./inject.sh -c custom_config.yaml -p your_package_name
    ```

    The script will read the `custom_config.yaml` file (**must be located in `/src` directory**) and apply the specified faults to the rosbags.

    By default, plotting mode is deactivated. You can activate with `n` flag.

    `TODO`: Explanation of execution mode  

## Trouble Shooting
Q1. Opening the container fails with `... "docker-credential-desktop.exe": executable file not found in $PATH, out: ...`.

A1. Delete the line with `credsStore` from `~/.docker/config.json` on WSL2.

## License

This project is licensed under the Apache-2.0 license. See the [LICENSE](LICENSE.md) file for more details.
