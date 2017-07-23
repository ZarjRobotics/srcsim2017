FROM ros:indigo-ros-base

# add ihmc messages
RUN apt-get update \
 && apt-get install -y \
    build-essential \
    python-catkin-tools \
    ros-indigo-catkin \
    ros-indigo-ihmc-msgs \
    ros-indigo-rosbag \
    ros-indigo-tf \
    ros-indigo-tf2 \
    ros-indigo-tf2-geometry-msgs \
    python-opencv \
    python-numpy \
    ros-indigo-cv-bridge \
 && rm -rf /var/lib/apt/lists/*

# clone srcsim
ENV WS /home/docker/ws
RUN mkdir -p ${WS}/src
WORKDIR ${WS}
RUN hg clone https://bitbucket.org/osrf/srcsim ${WS}/src/srcsim

# build srcsim messages
RUN . /opt/ros/indigo/setup.sh \
 && catkin config --cmake-args -DBUILD_MSGS_ONLY=True \
 && catkin config --install \
 && catkin build

# Copy the current directory contents into the container at /app
ADD ./fc ${WS}/fc
ADD ./lib ${WS}/lib

# Make port 2823 available to the world outside this container
EXPOSE 2823

ADD startup.bash startup.bash
CMD ["./startup.bash"]
