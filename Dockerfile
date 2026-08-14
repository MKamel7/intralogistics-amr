# Intralogistics AMR: reproducible build and test.
#
# WHY THIS EXISTS
#
# Every number in docs/validation.md was measured on one laptop. That is not a
# criticism of the numbers, it is a limit on who can check them. This image
# builds the workspace from a clean base and runs the whole test suite, so a
# reader can reproduce the parts that do not need a simulator in one command
# and the parts that do in two.
#
# WHAT IT DOES AND DOES NOT COVER
#
# The test suite, the generators and the headless simulation run here. The
# RViz and Gazebo GUIs are deliberately a host concern: rendering inside a
# container needs a display socket and a GPU passthrough that vary by machine,
# and pretending otherwise would produce an image that works here and nowhere
# else. `gz sim -s` headless is what CI and this image target.
#
#   docker build -t amr .
#   docker run --rm amr                         # build and run 291 tests
#   docker run --rm amr tools/run_stack.sh --cameras off --rviz off --run survey_mission
#
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-lc"]

# Gazebo Harmonic is not in the ROS base image. It comes from the OSRF
# repository, and the ros-jazzy-ros-gz metapackage is the bridge and sim
# wrapper that ties it to ROS 2.
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release \
    && curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# The stack's own dependencies. Listed explicitly rather than resolved by
# rosdep at build time so the image is reproducible: a rosdep run six months
# from now resolves against a different package index and the build stops
# being a record of anything.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
        ros-jazzy-gz-ros2-control \
        ros-jazzy-ros2-control \
        ros-jazzy-ros2-controllers \
        ros-jazzy-navigation2 \
        ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-robot-localization \
        ros-jazzy-vision-msgs \
        ros-jazzy-tf-transformations \
        ros-jazzy-xacro \
        ros-jazzy-rviz2 \
        python3-pytest python3-yaml python3-numpy \
        wmctrl imagemagick \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws
COPY . /ws/src/intralogistics-amr-fleet

# Build. --symlink-install matches how the project is developed, so a
# generator run inside the container updates the installed config exactly as
# it does on a workstation, which is the behaviour half the tests assume.
RUN source /opt/ros/jazzy/setup.bash \
    && cd /ws \
    && ln -s src/intralogistics-amr-fleet/tools tools \
    && ln -s src/intralogistics-amr-fleet/demo.sh demo.sh \
    && colcon build --symlink-install --base-paths src/intralogistics-amr-fleet

# HEADLESS BY DEFAULT. Without this Gazebo tries to open a render window and
# fails in a way that reads as a simulation fault rather than a missing
# display.
ENV QT_QPA_PLATFORM=offscreen
ENV GZ_HEADLESS=1

COPY docker-entrypoint.sh /usr/local/bin/
ENTRYPOINT ["/usr/local/bin/docker-entrypoint.sh"]
CMD ["pytest"]
