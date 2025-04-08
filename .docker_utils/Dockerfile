ARG BASE_IMAGE=ros
FROM ${BASE_IMAGE}:noetic
ARG USERNAME=foo
ARG USER_UID=1
ARG USER_GID=1

RUN groupadd --gid $USER_GID $USERNAME \ 
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip 
RUN apt update && apt install -y ros-noetic-tf ros-noetic-rospy-message-converter ros-noetic-vision-opencv libgl1 libglib2.0-0 python3-opencv

WORKDIR /tmp
COPY .docker_utils/requirements.txt .
RUN pip install -r requirements.txt
ENV SHELL /bin/bash


COPY fonts /usr/share/fonts
COPY .docker_utils/install_fonts.py .
RUN python3 install_fonts.py


WORKDIR /libs
COPY libs/ /libs
RUN cd frenet_conversion && pip3 install -e .

WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && mkdir src && catkin_make"
COPY .docker_utils/bashrc_ext.txt /home/${USERNAME}/bashrc_ext
RUN cat /home/${USERNAME}/bashrc_ext >> /home/${USERNAME}/.bashrc
WORKDIR /ros_eval
COPY . /ros_eval
RUN chown -R ${USER_UID}:${USER_GID} /ros_eval
USER ${USERNAME}
RUN pip install -e .
RUN chown -R ${USER_UID}:${USER_GID} /ros_eval

CMD ["python3", "-m",  "notebook", "--port=7000", "--no-browser", "--notebook-dir=/home/moe/ros_eval/src/notebooks"]
