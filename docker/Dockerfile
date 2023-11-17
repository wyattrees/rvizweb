FROM ros:kinetic

# Basic tools and dependencies and nodejs LTS PPA.
RUN apt update && apt install -y curl && curl -sL https://deb.nodesource.com/setup_12.x | bash -

# Install rvizweb.
WORKDIR /rvizweb_ws
ARG rvizweb_branch=master
RUN git clone https://github.com/wyattrees/rvizweb/ src/rvizweb -b ${rvizweb_branch}
RUN apt install -y ros-kinetic-pr2-gazebo
RUN rosdep install --from-paths src --ignore-src -r -y
RUN . /opt/ros/kinetic/setup.sh && catkin_make install
RUN rm -rf /rvizweb_ws/install/share/rvizweb/www/*
RUN cp -r /rvizweb_ws/build/rvizweb/www/* /rvizweb_ws/install/share/rvizweb/www/

# Clear apt cache.
RUN apt clean

ENTRYPOINT ["/bin/bash", "-c", "source /rvizweb_ws/install/setup.bash && /bin/bash"]
