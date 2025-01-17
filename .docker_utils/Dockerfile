ARG BASE_IMAGE=ros
FROM ${BASE_IMAGE}:noetic
ARG USERNAME=moe
ARG USER_UID=501
ARG USER_GID=20

# Create the user
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip && apt install -y ros-noetic-tf ros-noetic-rospy-message-converter 
WORKDIR /tmp
COPY .docker_utils/requirements.txt .
RUN pip install -r requirements.txt
ENV SHELL /bin/bash


COPY fonts /usr/share/fonts
COPY .docker_utils/install_fonts.py .
RUN python3 install_fonts.py


WORKDIR /libs
COPY libs/ /libs
RUN cd readwrite_global_waypoints && pip3 install -e .
COPY libs/f110_msgs /catkin_ws/src/f110_msgs
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"



COPY .docker_utils/bashrc_ext.txt /home/${USERNAME}/bashrc_ext
RUN cat /home/${USERNAME}/bashrc_ext >> /home/${USERNAME}/.bashrc
WORKDIR /ros_eval
COPY . /ros_eval
RUN chown -R ${USER_UID}:${USER_GID} /ros_eval
RUN pip install -e .
RUN chown -R ${USER_UID}:${USER_GID} /ros_eval
USER ${USERNAME}
CMD ["/bin/bash"]



