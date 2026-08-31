FROM ubuntu:22.04

# Prevent interactive timezone and keyboard prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install system utilities, Python 3, OpenGL/X11 rendering libraries, and RealSense USB drivers
RUN apt update && apt install -y --no-install-recommends \
    python3 \
    python3-pip \
    python3-dev \
    build-essential \
    libusb-1.0-0 \
    libusb-1.0-0-dev \
    udev \
    libgl1-mesa-glx \
    libglib2.0-0 \
    libsm6 \
    libxext6 \
    libxrender-dev \
    libgomp1 \
    curl \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# Set working directory inside container
WORKDIR /workspace

# Install pinned Python dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir --upgrade pip && \
    pip3 install --no-cache-dir -r requirements.txt

# Keep interactive shell active
CMD ["/bin/bash"]