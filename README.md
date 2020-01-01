# ompl_experimentation
Repository to store Python and C++ code for different motion planning using OMPL.

## Installation

* Navigate to the `install` directory: `cd install`

* Build the docker image by running `docker build --tag ompl_python_2 .` (can replace ompl_python_2 with the tag name of your choice). This operation takes about 10 hours. It creates a Docker image with all the dependencies to run the software.

* Run a docker container by running `docker run -it --name ompl --rm ompl_python_2`, which creates a container from the image created from the previous step. This brings you into an environment with the desired dependencies.

* Copy the ompl Python script into the docker container by running the following command in another terminal `docker cp ../updated_geometric_planner.py ompl:updated_geometric_planner.py`

* In the docker container, run `python2 updated_geometric_planner.py`. This will run the script. Note: By default, the plotting will not working from a docker container because it does not have access to your screen. To fix this, you can run the following command in another terminal: `xhost +local:root` (be sure to run `xhost -local:root` when you are done to be safe!), then use the following command instead of the previous docker run command:

```
docker run -it \
    --env="DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    ompl_python_2
```

This will allow you to run the script and plot the path.

Please note that you can install this on your host computer instead of in a Docker container if you would like but following the instructions in the Dockerfile.

## Path planning

Once the installation is complete, you can run the path-planning with the following: `python2 update_geometric_planner.py`.

You can explore different planners, objective functions, and maximum runtimes.

## Objective Functions

This is a work in progress. There are multiple weighted objective functions that we would like to optimize. These include:

* Minimize path length

* Maximize object clearance

* Avoid upwind/downwind trajectories

* Minimize turning

