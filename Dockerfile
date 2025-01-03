# Use a lightweight base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV DOCKER_BUILD=1

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libglfw3-dev \
    libglew-dev \
    freeglut3-dev \
    libx11-dev \
    libxi-dev \
    libxrandr-dev \
    libxinerama-dev \
    libxcursor-dev \
    && apt-get clean

# Set the working directory
WORKDIR /app

# Clone and build OpenMesh
RUN git clone --recursive https://gitlab.vci.rwth-aachen.de:9000/OpenMesh/OpenMesh.git dependencies/OpenMesh && \
    cd dependencies/OpenMesh && mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Clone Polyscope as a dependency
RUN git clone --recurse-submodules https://github.com/nmwsharp/polyscope.git dependencies/polyscope

# Clone and build Google Test
RUN git clone https://github.com/google/googletest.git dependencies/googletest && \
    cd dependencies/googletest && mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Fetch and install Eigen
RUN mkdir -p dependencies/eigen && \
    wget -qO- https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz | tar xz -C dependencies/eigen --strip-components=1

# Copy tinyfiledialogs directory
COPY dependencies/tinyfiledialogs /app/dependencies/tinyfiledialogs

# Copy the project files into the container
COPY . .

# Copy object-files directory
COPY object-files /app/object-files

# Prepare the build directory for your project
RUN mkdir -p build && cd build && cmake .. && make -j$(nproc)

# Default command to run the project executable
CMD ["./build/QEMSimplifier"]
