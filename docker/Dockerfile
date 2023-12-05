FROM ros:kinetic

# get rid of expired key, install some dependencies
RUN mv /etc/apt/sources.list.d/ros1-snapshots.list /etc/apt/sources.list.d/ros1-snapshots.list.disabled
RUN apt update && apt install -y curl apt-transport-https ca-certificates apt-utils

# Update key
RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
RUN curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - && apt update
RUN apt install -y ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control

# Install node.js
RUN curl -SLO https://deb.nodesource.com/nsolid_setup_deb.sh && chmod 500 nsolid_setup_deb.sh && ./nsolid_setup_deb.sh 16 && apt-get install nodejs -y

# Install rvizweb.
WORKDIR /rvizweb_ws
ARG rvizweb_branch=master
RUN git clone https://github.com/wyattrees/rvizweb/ src/rvizweb -b ${rvizweb_branch}

# Install ur5 package
RUN git clone https://github.com/ros-industrial/universal_robot.git src/universal_robot -b kinetic-devel
# Install other dependencies and build
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/kinetic/setup.sh && catkin_make install

# Workaround for files being installed in the wrong place
RUN rm -rf /rvizweb_ws/install/share/rvizweb/www/*
RUN cp -r /rvizweb_ws/build/rvizweb/www/* /rvizweb_ws/install/share/rvizweb/www/

# Clear apt cache.
RUN apt clean

# Set up entrypoint script
COPY bin/start_webserver.sh /root

ENTRYPOINT ["/bin/bash", "-c", "source /rvizweb_ws/install/setup.bash && /root/start_webserver.sh"]
