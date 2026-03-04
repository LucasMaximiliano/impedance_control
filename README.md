> ⚠️ **Warning**
>  
> This package is still under development and not in a working state!

# impedance_control
This package provides an impedance controller for the robot fingers in the SeaClear2.0 grapple. It consists of a base C++ library and a ROS2 wrapper for ease of integration with the rest of the system. A Docker container with all necessary dependencies is provided for plug-and-play deployment.

## Set-Up
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
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -p 8765:8765 \
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
To configure the impedance controller, modify the parameters in the `config.toml` file located in the `impedance_controller` package directory. The parameters include the
- virtual stiffness matrix,
- virtual damping matrix,
- virtual inertia matrix, and
- torque feedback gain.
### Build & Run
To build the ROS2 workspace, go to the `ros2_ws` directory and run
``` bash
source /opt/ros/humble/setup.bash
colcon build
```
After that, source the ROS2 workspace with
``` bash
source install/setup.bash
```
and execute the application with
``` bash
ros2 run impedance_controller main
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

## Documentation
The implementation of the impedance controller is documented using Doxygen. To generate the documentation, navigate to the `docs/` directory and run:
```bash
doxygen Doxyfile
```
The generated documentation will be available in the `docs/html` directory. Alternatively, you can compile and view the `.pdf` documentation directly by navigating to `docs/latex` and running:
```bash
make
```
The resulting `refman.pdf` file contains the complete documentation.

## Further resources
For more information on the impedance control algorithm, refer to the [developer notes](https://sharelatex.tum.de/project/694488e4bccf9df893153f9d). The latest version is also available in the `docs/` directory.

## Contact
For questions or support regarding the impedance controller, please contact the SeaClear2.0 development team at lucasmo.silva@tum.de.