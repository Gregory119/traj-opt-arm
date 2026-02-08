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
ENV CMAKE_PREFIX_PATH=/home/ubuntu/Downloads/mujoco_install

###################################################
# install IPOPT optimization library
###################################################
RUN apt-get install -y coinor-libipopt-dev

###################################################
# create directory to be mapped
###################################################
RUN mkdir /home/ubuntu/local_dir
WORKDIR /home/ubuntu/local_dir
