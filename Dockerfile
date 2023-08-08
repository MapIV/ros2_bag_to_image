FROM ros:humble-perception
COPY . /app/src
WORKDIR /app
RUN rosdep update && rosdep install -y --from-paths src --ignore-src
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && colcon build'
