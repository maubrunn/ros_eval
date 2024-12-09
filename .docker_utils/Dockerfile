ARG USERNAME=moe
ARG USER_UID=501
ARG USER_GID=20
ARG BASE_IMAGE=ros

FROM $BASE_IMAGE:noetic
# Create the user
RUN useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip && apt install -y ros-noetic-tf
WORKDIR /tmp
COPY .docker_utils/requirements.txt .
RUN pip install -r requirements.txt
ENV SHELL /bin/bash


RUN mkdir -p "/home/${USERNAME}/eval"
WORKDIR "/home/${USERNAME}/eval"
RUN chown -R $USERNAME "/home/${USERNAME}/eval"
USER ${USERNAME}
CMD ["/bin/bash"]



