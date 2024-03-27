# Use the latest Debian image as the base
FROM debian:latest

# Set environment variables to avoid user interaction during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
# This includes compilers, CMake, and any libraries your project needs
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    g++ \
    git \
    clang \
    vim \
    libboost-all-dev \
    libpcap-dev \
    libeigen3-dev \
    nano \
 && apt-get clean \
 && rm -rf /var/lib/apt/lists/* \
 && alias ll='ls -alF'

# Optional: Install your preferred text editor (nano, vim, etc.)

# Set the working directory in the container
WORKDIR /workspace

# Optional: Copy your project files into the container
# COPY . /workspace

# Avoid automatically running build commands. Instead, provide a script or
# instructions in your documentation on how to build and run your project.
