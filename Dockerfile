# This is a template for a Dockerfile to build a docker image for your ROS package. 
# The main purpose of this file is to install dependencies for your package.

FROM docker.io/osrf/ros:noetic-desktop

# Install some dependencies


ENV ROS_DISTRO=noetic
ENV ROS_ROOT=/opt/ros/${ROS_DISTRO} 


RUN apt-get update
RUN apt-get install -y git \
	python3-pip \
	ros-noetic-rviz \
	&& apt-get clean \
	&& rm -rf /var/lib/apt/lists/*
    
WORKDIR /ws

RUN pip install -y icecream numba numpy scipy typing_extensions scikit-learn


# Set noninteractive installation
ENV DEBIAN_FRONTEND=noninteractive



RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "source ~/ws/devel/setup.bash" >> ~/.bashrc


RUN echo "ALL DONE!!!"
