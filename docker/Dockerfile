FROM ubuntu:24.04

###################################################
# install C++ toolchain
###################################################
RUN apt update && apt install -y build-essential
RUN apt install -y cmake

###################################################
# install mujoco library
###################################################
RUN apt install -y git
RUN mkdir -p /home/ubuntu/Downloads/
WORKDIR /home/ubuntu/Downloads/
RUN git clone https://github.com/deepmind/mujoco.git
RUN cd mujoco && git checkout 3.4.0
# configure and build mujoco
RUN apt install -y libwayland-dev pkg-config libx11-dev libxrandr-dev libxinerama-dev libxcursor-dev libxi-dev libxext-dev libwayland-dev libxkbcommon-dev libglfw3-dev
RUN mkdir mujoco_build && cd mujoco_build && cmake ../mujoco && cmake --build . -j
# install
RUN mkdir mujoco_install && cd mujoco_build && cmake ../mujoco -DCMAKE_INSTALL_PREFIX=/home/ubuntu/Downloads/mujoco_install && cmake --install .

###################################################
# install IPOPT optimization library
###################################################
RUN apt-get update && apt-get install -y coinor-libipopt-dev

###################################################
# install IFOPT optimization library wrapper
###################################################
# install Eigen
RUN apt-get update && apt-get install -y libeigen3-dev

# build and install ifopt
RUN git clone https://github.com/ethz-adrl/ifopt.git && cd ifopt && mkdir build && cd build && cmake .. && make && make install

###################################################
# install pinnochio
###################################################
# instructions at: https://stack-of-tasks.github.io/pinocchio/download.html

# install dependencies
RUN apt-get update && apt install -qqy lsb-release curl

# register authentication certificate of robotpkg repository
RUN mkdir -p /etc/apt/keyrings && curl http://robotpkg.openrobots.org/packages/debian/robotpkg.asc \
    | tee /etc/apt/keyrings/robotpkg.asc

# Add robotpkg as source repository to apt
RUN echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list

RUN apt-get update && apt-get install -qqy robotpkg-py3*-pinocchio

###################################################
# install debug tools
###################################################
RUN apt-get update && apt-get install systemd-coredump gdb

###################################################
# create directory to be mapped
###################################################
RUN mkdir /home/ubuntu/local_dir
WORKDIR /home/ubuntu/local_dir
