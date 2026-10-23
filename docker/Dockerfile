FROM ubuntu:24.04

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
        sudo \
        build-essential \
        cmake \
        libzmq3-dev \
        cppzmq-dev \
        nlohmann-json3-dev \
        python3 \
        python3-pip \
        bash-completion \
        qt6-base-dev \
        qt6-wayland \
        python3-pyqt6 \
        libgl1 \
        ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# Install python dependencies
RUN pip install --break-system-packages --no-cache-dir pyzmq numpy pillow

# User args
ARG USERNAME=core
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME -s /bin/bash \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to the new user
USER $USERNAME

RUN mkdir -p /home/$USERNAME/robot_ws
WORKDIR /home/$USERNAME/robot_ws