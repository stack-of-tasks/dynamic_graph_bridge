# Pull ros docker image
FROM ros:indigo-desktop-full-trusty

# Prepapre sudo environment
RUN apt-get update && \
      apt-get -y install sudo && \
      apt-get -y install curl

RUN useradd -m docker && echo "docker:docker" | chpasswd && adduser docker sudo

USER root
ADD /travis_custom/sudoers.txt /etc/sudoers
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
RUN sudo apt-get install -y cppcheck doxygen libboost-system-dev libboost-test-dev libboost-filesystem-dev libboost-program-options-dev libeigen3-dev libtinyxml-dev
RUN sudo apt-get install -y ros-indigo-tf
RUN sudo apt-get install -y robotpkg-dynamic-graph-v3 robotpkg-py27-dynamic-graph-v3
RUN sudo apt-get install -y robotpkg-sot-core-v3 libboost-python-dev robotpkg-py27-eigenpy python2.7-dev python-numpy python-sphinx

