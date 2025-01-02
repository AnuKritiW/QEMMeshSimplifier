# Use a lightweight base image
FROM ubuntu:22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive

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

# Copy the entire project into the container
COPY . .

# Copy object-files directory
COPY object-files /app/object-files

# Clean and rebuild OpenMesh
RUN cd dependencies/OpenMesh && \
    rm -rf build && mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Clean and rebuild Polyscope
RUN cd dependencies/polyscope && \
    rm -rf build && mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Clean and rebuild Google Test
RUN cd dependencies/googletest && \
    rm -rf build && mkdir -p build && cd build && \
    cmake .. && make -j$(nproc) && make install

# Prepare the build directory for your project
RUN mkdir -p build && cd build && cmake .. && make -j$(nproc)

# Default command to run the project executable
CMD ["./build/QEMSimplifier"]
