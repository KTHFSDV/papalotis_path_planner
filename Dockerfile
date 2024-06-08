# This is a template for a Dockerfile to build a docker image for your ROS package. 
# The main purpose of this file is to install dependencies for your package.

FROM ros:noetic-ros-base-focal        
#FROM ros:melodic-ros-base-bionic       ####<--- TODO: change to your base image
#FROM dustynv/ros:noetic-ros-base-l4t-r32.4.4  
#FROM dustynv/ros:nmelodic-ros-base-l4t-r32.4.4

ENV ROS_DISTRO=noetic
#ENV ROS_DISTRO=melodic          ###<--- TODO: change to your ROS version to mach base image

ENV ROS_ROOT=/opt/ros/${ROS_DISTRO} 

# Set upp workspace
RUN mkdir -p /ws/src   

# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive

# Package apt dependencies
RUN apt-get update && apt-get upgrade -y && \
    apt-get install -y \
    #python3-pip \
    python3-catkin-tools \
    # catkin build needed for docker-entrypoint.sh \
    # EXAMPLE: \
    # libssl-dev \
    # libffi-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws


# Installing of pip dependencies
#RUN pip3 install \
#     # EXAMPLE: \
#     # torch \
#     # torchvision \
#     # tensorboardX \
#     # opencv-python \
#     # scikit-image \
#     # scikit-learn \


# Build dependencies from source
#RUN git clone https://github.com/your-github-username/your-repo.git
#WORKDIR your-repo
#RUN make