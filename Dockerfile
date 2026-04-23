FROM nvidia/cuda:12.6.3-cudnn-runtime-ubuntu24.04

ENV DEBIAN_FRONTEND=noninteractive

ARG UID=1001
ARG GID=1001

# Install system deps + create user
RUN apt-get update && apt-get install -y \
    sudo \
    python3 \
    python3-pip \
    python3-venv \
    libx11-6 \
    libxext-dev \
    libxrender-dev \
    libxinerama-dev \
    libxi-dev \
    libxrandr-dev \
    libxcursor-dev \
    libxss1 \
    x11-apps \
    libsdl2-2.0-0 \
    libvulkan1 \
    vulkan-tools \
    libgl1 \
    libglu1-mesa \
    libegl1 \
    libfontconfig1 \
    libfreetype6 \
    && rm -rf /var/lib/apt/lists/* && \
    groupadd -g $GID user && \
    useradd -m -u $UID -g $GID -s /bin/bash user && \
    usermod -aG sudo user && \
    echo "user ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/user

USER user

WORKDIR /home/user/biguasim
COPY --chown=user:user . .

RUN pip install . --break-system-packages --retries 10 --timeout 10000

RUN python3 -c "import biguasim; biguasim.install('SkyDive')" || echo "WARNING: The BiguaSim packaged worlds server is offline! Please try again later, or use your own."

WORKDIR /home/user

CMD ["bash"]

