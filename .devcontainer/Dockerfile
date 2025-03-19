# Start with a base image
FROM ubuntu:20.04

# Set non-interactive mode
ENV DEBIAN_FRONTEND=noninteractive

ARG USER_UID=1001
ARG USER_GID=1001
ARG USERNAME=user

WORKDIR /workspaces

# Create a non-root user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    && chown -R $USER_UID:$USER_GID /workspaces \
    && apt-get update \
    && apt-get install -y sudo \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Install necessary packages
RUN apt-get update \
    && apt-get install -y \
    bash-completion \
    build-essential \
    clang-6.0 \
    clang-format-6.0 \
    clang-tidy-6.0 \
    cmake \
    doxygen \
    graphviz \
    lcov \
    libeigen3-dev \
    libpoco-dev \
    rename \
    valgrind \
    lsb-release \
    dpkg \
    git \
    libeigen3-dev \
    libpoco-dev \
    libfmt-dev \
    python3-dev \
    python3-pip \
    liburdfdom-headers-dev \
    libconsole-bridge-dev \
    libtinyxml2-dev \
    lsb-release \
    curl \
    wget \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/* \
    && ln -s $(which clang-tidy-6.0) /usr/bin/clang-tidy \
    && ln -s $(which clang-format-6.0) /usr/bin/clang-format

# Install Pinocchio
RUN mkdir -p /etc/apt/keyrings && \
    curl -fsSL http://robotpkg.openrobots.org/packages/debian/robotpkg.asc | tee /etc/apt/keyrings/robotpkg.asc && \
    echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/robotpkg.asc] http://robotpkg.openrobots.org/packages/debian/pub $(lsb_release -cs) robotpkg" \
    | tee /etc/apt/sources.list.d/robotpkg.list && \
    apt-get update && \
    apt-get install -y robotpkg-pinocchio && \
    apt-get clean -y && \
    rm -rf /var/lib/apt/lists/*

# Set environment variables
ENV PATH=/opt/openrobots/bin:$PATH \
    PKG_CONFIG_PATH=/opt/openrobots/lib/pkgconfig:$PKG_CONFIG_PATH \
    LD_LIBRARY_PATH=/opt/openrobots/lib:$LD_LIBRARY_PATH \
    PYTHONPATH=/opt/openrobots/lib/python3.10/site-packages:$PYTHONPATH \
    CMAKE_PREFIX_PATH=/opt/openrobots/lib/cmake:$CMAKE_PREFIX_PATH

ENV TRACY_NO_INVARIANT_CHECK=1
RUN git clone --branch v0.10 https://github.com/wolfpld/tracy.git \
    && cd tracy \
    && cmake -DBUILD_SHARED_LIBS=1 -DTRACY_ENABLE=1 -DTRACY_ON_DEMAND=1 -DCMAKE_BUILD_TYPE=Release -DCMAKE_CXX_STANDARD=17 -S . -B build \
    && cmake --build build --parallel `nproc` --target install

RUN git clone --branch v0.9.5 https://github.com/Neargye/magic_enum.git                                                                        \
    && cd magic_enum                                                                                                                           \
    && cmake -S . -B build -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DMAGIC_ENUM_OPT_BUILD_EXAMPLES=0 -DMAGIC_ENUM_OPT_BUILD_TESTS=0 -DCMAKE_CXX_STANDARD=20 \
    && cmake --build build --parallel `nproc` --target install

USER $USERNAME
SHELL ["/bin/bash", "-c"]
