FROM ros:noetic

ARG DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update -yqq && \
    apt-get install -yqq \
    git \
    python3-catkin-tools \
    build-essential \
    cmake \
    gedit \
    libboost-all-dev \
    libeigen3-dev \
    ros-noetic-roscpp \
    ros-noetic-catkin \
    ros-noetic-tf2 \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-eigen \
    ros-noetic-sensor-msgs \
    ros-noetic-std-msgs && \
    rm -rf /var/lib/apt/lists/*

# Install GTSAM dependencies
RUN apt-get update -yqq && \
    apt-get install -yqq \
    libtbb-dev \
    libmetis-dev \
    libsuitesparse-dev && \
    rm -rf /var/lib/apt/lists/*

# Clone and build GTSAM
WORKDIR /opt
RUN git clone https://github.com/StironjaVlaho/gtsam && \
    cd gtsam && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release \
             -DGTSAM_USE_SYSTEM_EIGEN=ON \
             -DGTSAM_EIGEN_VERSION=3.3.7 \
             -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
             -DGTSAM_BUILD_EXAMPLES=OFF \
             -DGTSAM_BUILD_TESTS=OFF \
             -DGTSAM_BUILD_UNSTABLE=ON && \
    make -j$(nproc) && make install && \
    cd /opt && rm -rf gtsam

# Set up catkin workspace
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p $CATKIN_WS/src
WORKDIR $CATKIN_WS/src

# Copy and build your ROS package
COPY ./fgo_rio_t ./fgo_rio_t

WORKDIR $CATKIN_WS
RUN /bin/bash -c " \
    source /opt/ros/noetic/setup.bash && \
    export CMAKE_PREFIX_PATH=/usr/local:/opt/ros/noetic:$CMAKE_PREFIX_PATH && \
    export GTSAM_UNSTABLE_DIR=/usr/local/lib/cmake/GTSAM && \
    catkin config --extend /opt/ros/noetic && \
    catkin build --no-status"

# Copy IRS folder to root's home directory
COPY ./IRS /root/IRS
RUN chmod +x /root/IRS/run_rio_t.sh

# Custom entrypoint to source workspace
RUN echo '#!/bin/bash\n\
source "$CATKIN_WS/devel/setup.bash"\n\
exec "$@"' > /entrypoint.sh && \
    chmod +x /entrypoint.sh
    
WORKDIR /root/IRS

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

