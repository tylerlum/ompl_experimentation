FROM ubuntu:bionic AS builder

## ROS Setup
# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install packages
RUN apt-get update && apt-get install -q -y \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros/ubuntu bionic main" > /etc/apt/sources.list.d/ros1-latest.list

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*

# setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# bootstrap rosdep
RUN rosdep init \
    && rosdep update

# install ros packages
ENV ROS_DISTRO melodic
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-core=1.4.1-0* \
    && rm -rf /var/lib/apt/lists/*
# install ros packages
RUN apt-get update && apt-get install -y \
    ros-melodic-ros-base=1.4.1-0* \
&& rm -rf /var/lib/apt/lists/*

## OMPL Setup
RUN apt-get update && \
    apt-get install -y \
        build-essential  \
        cmake \
        libboost-filesystem-dev \
        libboost-numpy-dev \
        libboost-program-options-dev \
        libboost-python-dev \
        libboost-serialization-dev \
        libboost-system-dev \
        libboost-test-dev \
        libeigen3-dev \
        libexpat1 \
        libflann-dev \
        libode-dev \
        libtinfo5 \
        libtriangle-dev \
        pkg-config \
        python2.7-dev \
        python2.7-numpy \
        python-pip \
        wget && \
    # Install spot
    wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | apt-key add - && \
    echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
    apt-get update && \
    apt-get install -y libspot-dev && \
    # Install newer version of castxml than is available via apt-get:
    wget -q -O- https://data.kitware.com/api/v1/file/5b68c2c28d777f06857c1f48/download | tar zxf - && \
    # Install pypy2:
    wget -q -O- https://bitbucket.org/pypy/pypy/downloads/pypy2-v6.0.0-linux64.tar.bz2 |tar jxf - && \
    pip2 install pygccxml pyplusplus

COPY . /ompl

# Install additional dependencies and useful tools
RUN DEBIAN_FRONTEND=noninteractive apt-get install -y \
        python-matplotlib \
        vim \
        nano

RUN /ompl/install-ompl-ubuntu.sh --python 

# Below is code from the original Dockerfile from ompl, which does not work
# RUN /ompl/install-ompl-ubuntu.sh --app

# WORKDIR /build
# RUN cmake \
#         -DPYTHON_EXEC=/usr/bin/python2.7 \
#         -DOMPL_REGISTRATION=OFF \
#         -DCMAKE_INSTALL_PREFIX=/usr \
#         -DCASTXML=/castxml/bin/castxml \
#         -DPYPY=/pypy2-v6.0.0-linux64/bin/pypy2 \
#         /ompl && \
#     make update_bindings -j `nproc` && \
#     make -j `nproc` && \
#     make install
# 
# FROM ubuntu:bionic
# RUN apt-get update && \
#     apt-get install -y \
#         build-essential  \
#         cmake \
#         libboost-filesystem-dev \
#         libboost-numpy-dev \
#         libboost-program-options-dev \
#         libboost-python-dev \
#         libboost-serialization-dev \
#         libboost-system-dev \
#         libeigen3-dev \
#         libflann-dev \
#         libode-dev \
#         libtriangle-dev \
#         pkg-config \
#         python2.7-dev \
#         python2.7-numpy \
#         python-pip \
#         wget && \
#     # Install spot
#     wget -q -O - https://www.lrde.epita.fr/repo/debian.gpg | apt-key add - && \
#     echo 'deb http://www.lrde.epita.fr/repo/debian/ stable/' >> /etc/apt/sources.list && \
#     apt-get update && \
#     apt-get install -y libspot-dev
# 
# COPY --from=builder /usr /usr
# RUN useradd -ms /bin/bash ompl
# USER ompl
# WORKDIR /home/ompl
