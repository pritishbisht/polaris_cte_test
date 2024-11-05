
# polaris_cte_test

`polaris_cte_test` is a repository focused on evaluating the Cross-Track Error (CTE) in a simulation environment derived from the POLARIS_GEM_e2 project. The repository utilizes Docker for isolated execution, Jenkins for automated testing, and a customized test package to assess CTE dynamically. 

## Repository Contents

- **Dockerfile**: Builds a Docker image based on a ROS Noetic environment, installing required dependencies for running the Gazebo simulation and test package.
- **Jenkinsfile**: Manages the pipeline for automated testing and deployment, with customizable parameters for test duration and error threshold.
- **Test Package (`test_cross_track_error`)**: Contains scripts for evaluating CTE in a simulation, with the ability to pass parameters through Jenkins.

## Detailed Workflow

### Docker Configuration

The `Dockerfile` establishes a ROS environment and prepares it for simulation. Key steps include:

1. **Base ROS Image**: The image `ros:noetic-ros-core` serves as the foundation, ensuring compatibility with ROS packages.
2. **Dependency Installation**: Installs required ROS packages and utilities.
3. **Workspace Setup**: Initializes a Catkin workspace, clones the POLARIS_GEM_e2 repository, and builds it.
4. **Environment Configuration**: Sets up the workspace environment, sourcing ROS and workspace files for seamless operation in the container.

### Test Package - `test_cross_track_error`

The core functionality of this repository is encapsulated in the `test_cross_track_error` package, running within the [POLARIS_GEM_e2](https://gitlab.engr.illinois.edu/gemillins/POLARIS_GEM_e2) project framework.

1. **Modified `pure_pursuit_sim_updated.py`**:
   - This script, a modification of `pure_pursuit_sim.py`, controls the duration of the simulation, allowing it to be set dynamically.
   - Default duration is 30 seconds, with the value adjustable through the Jenkins pipeline.

2. **CTE Evaluation (`ct_error_evaluation_test.py`)**:
   - This script subscribes to the CTE output and evaluates the error against a threshold.
   - Configurable parameters:
     - `duration` (in seconds): Time span of the test, default is 30 seconds.
     - `error_threshold` (in meters): Acceptable error margin for the CTE, default is 1 meter.
   - The test results are determined by whether the actual CTE stays within the threshold during the simulation.

### Continuous Integration with Jenkins

The `Jenkinsfile` is configured to automate the testing pipeline with Docker, performing the following steps:

1. **Parameter Setup**:
   - Accepts parameters for `DURATION` (default 30 seconds) and `ERROR_THRESHOLD` (default 1.0 meter).
   - Additional parameters for Docker image configuration and email notifications.

2. **Pipeline Stages**:
   - **Checkout**: Retrieves the latest code from the repository.
   - **Docker Image Management**:
     - Builds the Docker image if changes are detected in the `Dockerfile` or if no image exists.
     - Otherwise, pulls the existing image from Docker Hub.
   - **Container Management**:
     - Starts a container instance for the simulation and copies the test package into it.
   - **Run Test Package**:
     - Executes `ct_error_evaluation_test.py` with the configured `duration` and `error_threshold`.
   - **Image Push**:
     - Upon a successful test, pushes the image to Docker Hub with both `latest` and `build-specific` tags.

3. **Notification**:
   - Sends email notifications based on the pipeline’s success or failure.

## Usage

### Running Locally with Docker

1. **Build the Docker Image**:

   ```bash
   docker build -t polaris_cte_test .
   ```

2. **Run the Docker Container**:

   ```bash
   docker run --rm polaris_cte_test
   ```

This command starts the container with the default test parameters. 

### Running Tests with Jenkins

1. **Set Up Jenkins**:
   - Install Jenkins and configure a pipeline job.
   - Point it to the repository’s `Jenkinsfile`.

2. **Configure Parameters**:
   - Set `DURATION` and `ERROR_THRESHOLD` values as required.
   
3. **Execute Pipeline**:
   - Run the Jenkins job to initiate the test package within Docker.
