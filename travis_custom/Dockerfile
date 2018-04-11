# Pull ros docker image
FROM ros:indigo-ros-base

# Prepapre sudo environment
RUN apt-get update && \
      apt-get -y install sudo && \
      apt-get -y install curl

RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

USER root
ADD /travis_custom/sudoers.txt /etc/sudoers
ADD /travis_custom/setup-opt-robotpkg.sh /home/docker/.bashrc
RUN chmod 440 /etc/sudoers

USER docker
CMD /bin/bash
# Add robotpkg binary repo

RUN sudo sh -c "echo \"deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub trusty robotpkg\" >> /etc/apt/sources.list "
RUN sudo sh -c "echo \"deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub trusty robotpkg\" >> /etc/apt/sources.list "

RUN curl http://robotpkg.openrobots.org/packages/debian/robotpkg.key | sudo apt-key add -

# Update the reference to packages
RUN sudo apt-get update
RUN more /etc/apt/sources.list
#
RUN sudo apt-get install -y g++ python2.7 python2.7-dev
RUN sudo apt-get install -y cppcheck doxygen libboost-system-dev libboost-test-dev libboost-filesystem-dev libboost-program-options-dev libeigen3-dev libtinyxml-dev
RUN sudo apt-get install -y ros-indigo-tf ros-indigo-tf2-bullet ros-indigo-realtime-tools 
RUN sudo apt-get install -y robotpkg-pinocchio 
RUN sudo apt-get install -y robotpkg-dynamic-graph-v3 robotpkg-py27-dynamic-graph-v3 robotpkg-dynamic-graph-bridge-msgs
RUN sudo apt-get install -y robotpkg-sot-core-v3 robotpkg-py27-sot-tools-v3 robotpkg-sot-dynamic-pinocchio-v3
RUN sudo apt-get install -y libboost-python-dev robotpkg-py27-eigenpy python2.7-dev python-numpy python-sphinx
RUN env



