> ⚠️ **Warning**
>  
> This package is still under development and not in a working state!

# impedance_control
This package provides an impedance controller for the robot fingers in the SeaClear2.0 grapple. It consists of a base C++ library and a ROS2 wrapper for ease of integration with the rest of the system. A Docker container with all necessary dependencies is provided for plug-and-play deployment.

## Set-Up
### FastDDS Discovery Server
To enable communication between the ROS2 nodes running inside the docker container and the rest of the system, a FastDDS discovery server is required. This is because the ROS2 nodes inside the container need to discover and communicate with the ROS2 nodes running on the host machine and other devices (e.g. other containers) in the network.

1. Install FastDDS on the host machine by following the instructions in the [FastDDS documentation](https://fast-dds.docs.eprosima.com/en/latest/index.html).
2. Find the IP address of the host machine:
``` bash
ip addr show
```
3. Update the IP address in the `docker/fastdds_client_profile.xml` file to match the IP address of the host machine.
4. Start the FastDDS discovery server on the host machine. Replace `-l 192.168.2.95` with the IP address of your host machine:
``` bash
fastdds discovery -i 0 -l 192.168.2.95 -p 11815
```

### Docker
To enable easy deployment of the code, and to support different operating systems/versions, [docker](https://www.docker.com/) is used. Below is a brief guide on how to use docker for this specific application.

1. Make sure you're in the `docker` directory of the repo
``` bash
cd docker
```
2. Build the docker image using the provided Dockerfile
``` bash
docker build -t impedance_controller_ros2_image:dev .
```
3. Run the container
``` bash
docker run -it \
    --name impedance_controller_dev_container \
    --network host \
    --ipc=host \
    -e ROS_DOMAIN_ID=8 \
    -v /Users/lucas/dev/seaclear2.0/impedance-control/impedance_control/:/impedance_control \
    impedance_controller_ros2_image:dev
```
> **Note:** Remember to adjust the path `/Users/lucas/dev/seaclear2.0/impedance-control/impedance_control` when mounting the git directory depending on the file structure of your PC.

After running the container for the first time, it will persist unless explicitly removed. In future sessions, the container can be accessed by starting and executing it with the name `impedance_controller_dev_container`:

1. Start the container
``` bash
docker start impedance_controller_dev_container
```

2. Execute the container
``` bash
docker exec -it impedance_controller_dev_container /bin/bash
```

Always enter the docker container when developing. All the dependencies for the project are already set-up inside the container, so you don't have to worry about that.

Once you're satisfied with your work, leave the container **first** and perform the necessary git operations (add, commit, push, etc.) **second**. You may add changes inside the container, but commiting and pushing is definitely easier outside due to the complexity of SSH identification in the container.

## Usage
### Configuration
To configure the impedance controller, modify the initialization parameters in the `config.toml` file located in the `impedance_controller/` package directory. Additionally, the following parameters can be updated dynamically at runtime, either via a [GUI](#visualization) or through ROS2 service calls: 
- virtual stiffness matrix,
- virtual damping matrix,
- virtual inertia matrix, and
- torque feedback gain.

> **Note:** The setters for the matrices take a scalar value and set the corresponding matrix to a scaled identity matrix. This is a common approach for tuning the controller with a single gain parameter. However, the software can be easily extended to allow setting full matrices, if needed.

### Build & Run
To build the ROS2 workspace, go to the `ros2_ws` directory and run
``` bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
```
The `--symlink-install` flag creates symbolic links to the source files instead of copying them during build. That means that changes to the source code will be reflected immediately without the need to rebuild the workspace. This is especially useful when adjusting the initialization parameters in the `config.toml` file.

After that, source the ROS2 workspace with
``` bash
source install/setup.bash
```
and execute the application with
``` bash
ros2 run impedance_controller impedance_controller_node
```

### Testing
#### Simulation tests
To run the simulation tests for the impedance controller, open the `matlab/` folder with your local MATLAB installation and execute the `MAIN.m` script. For more details on the simulation tests, refer to the dedicated README file in the `impedance_controller_sim/` directory.
#### Integration tests
To run the integration tests for the impedance controller, use the following command in the `ros2_ws/` directory:
``` bash
colcon test --packages-select impedance_controller
```
This will execute all the tests defined for the `impedance_controller` package. You can view the test results in the terminal or generate a report for more detailed information. To generate a test report, run:
``` bash
colcon test-result --all --verbose
```
The `--verbose` flag is optional and provides more detailed output. This will generate a detailed report of the test results, including any failures or errors encountered during testing.

## Visualization
Foxglove is a third-party software tool to interact with ROS2 data. It can handle recorded data e.g. via ROS bags or real time data from ROS interfaces (topics, services, actions). Here are the steps to use the impedance controller together with Foxglove:
1. Install [Foxglove Studio](https://foxglove.dev/download/) on the host machine. If for some reason you can't install it, you can also use the [web version](https://studio.foxglove.dev/).
2. Go to the "Layouts" section, click on the "Add" button and select "Import Layout from File". Then, select the `impedance_controller/foxglove_layout.json` file.
3. Inside the docker container, start the ROS2 impedance controller node as described in the [Build & Run](#build--run) section.
4. Inside the docker container, start the Foxglove websocket bridge by running the following command:
``` bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
```
5. Go back to Foxglove Studio and add a new ROS2 connection by clicking on the "Open connection" button in the "Data sources" panel.
6. Select the websocket option and set the URL to `ws://localhost:8765`.
7. Click on "Connect" to start receiving data from the impedance controller node.

## Documentation
### Doxygen
The implementation of the impedance controller is documented using Doxygen. To generate the documentation, navigate to the `docs/` directory and run:
```bash
doxygen Doxyfile
```
The generated documentation will be available in the `docs/html` directory. Alternatively, you can compile and view the `.pdf` documentation directly by navigating to `docs/latex` and running:
```bash
make
```
The resulting `refman.pdf` file contains the complete documentation.

### Developer Notes
For more information on the impedance control algorithm, refer to the [developer notes](https://sharelatex.tum.de/project/694488e4bccf9df893153f9d). The latest version is also available in the `docs/` directory.

## Contact
For questions or support regarding the impedance controller, please contact the SeaClear2.0 development team at lucasmo.silva@tum.de.