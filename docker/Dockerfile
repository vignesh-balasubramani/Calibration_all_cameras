# Calibration environment: Python deps + RealSense/Basler access + X11 for preview windows.
# Build context is the repository root (see docker-compose.yaml).
FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PIP_NO_CACHE_DIR=1

# libusb/udev: RealSense over USB. libgl1/libglib2.0-0/libsm6/libxext6/libxrender-dev: OpenCV GUI windows.
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 \
        python3-pip \
        python3-dev \
        build-essential \
        libusb-1.0-0 \
        libusb-1.0-0-dev \
        udev \
        libgl1 \
        libglib2.0-0 \
        libsm6 \
        libxext6 \
        libxrender-dev \
        libgomp1 \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

COPY requirements.txt .
RUN pip3 install --upgrade pip && pip3 install -r requirements.txt

CMD ["/bin/bash"]
