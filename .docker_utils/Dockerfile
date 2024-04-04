FROM ros:noetic-robot
ARG USERNAME=moe
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
WORKDIR /tmp
COPY requirements.txt .
RUN pip install -r requirements.txt
ENV SHELL /bin/bash

RUN apt-get update && apt-get install -y python3-catkin-tools

RUN mkdir -p "/home/${USERNAME}/catkin_ws/src/"
WORKDIR "/home/${USERNAME}/catkin_ws"
RUN chown -R $USERNAME "/home/${USERNAME}/catkin_ws"
RUN sudo apt update && sudo apt install git -y
RUN catkin init && . /opt/ros/noetic/setup.sh  && git clone https://github.com/ethz-asl/vicon_bridge src/ && catkin build
USER ${USERNAME}
CMD ["/bin/bash"]



