ARG UBUNTU_VERSION="20.04"
ARG PYTHON_VERSION="3.10"

# Start with a base image
FROM ubuntu:${UBUNTU_VERSION}
ARG UBUNTU_VERSION
ARG PYTHON_VERSION

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

# Add LLVM repository for newer clang versions
RUN apt-get update && \
    apt-get install -y wget gnupg lsb-release software-properties-common && \
    wget https://apt.llvm.org/llvm.sh && \
    chmod +x llvm.sh && \
    ./llvm.sh 18 && \
    apt-get update && \
    apt-get install -y clang-format-18 clang-tidy-18 \
    && ln -s $(which clang-tidy-18) /usr/bin/clang-tidy \
    && ln -s $(which clang-format-18) /usr/bin/clang-format

# Install necessary packages (without Python - we'll install specific version later)
# Note: pybind11-dev kept for devcontainer users (works with Ubuntu's default Python)
# For CI wheel builds, pip-installed pybind11 in venv is used for each Python version
RUN apt-get update \
    && apt-get install -y \
    bash-completion \
    build-essential \
    curl \
    doxygen \
    dpkg \
    git \
    graphviz \
    lcov \
    libeigen3-dev \
    libfmt-dev \
    libpoco-dev \
    lsb-release \
    pybind11-dev \
    rename \
    valgrind \
    wget \
    software-properties-common \
    && apt-get clean -y \
    && rm -rf /var/lib/apt/lists/*

# Install specific Python version
# On Ubuntu 22.04, Python 3.10 is the default system Python (packages are python3-dev, python3-venv)
# For other versions, use deadsnakes PPA (packages are python3.X-dev, python3.X-venv)
RUN set -ex && \
    # Check if Python version is already available as system default
    SYSTEM_PYTHON_VERSION=$(python3 --version 2>/dev/null | grep -oP '\d+\.\d+' || echo "none") && \
    echo "System Python: $SYSTEM_PYTHON_VERSION, Requested: ${PYTHON_VERSION}" && \
    if [ "$SYSTEM_PYTHON_VERSION" = "${PYTHON_VERSION}" ]; then \
        echo "Python ${PYTHON_VERSION} is already the system default, installing dev packages..." && \
        apt-get update && \
        apt-get install -y python3-dev python3-venv && \
        apt-get clean -y && \
        rm -rf /var/lib/apt/lists/*; \
    else \
        echo "Installing Python ${PYTHON_VERSION} from deadsnakes PPA..." && \
        add-apt-repository ppa:deadsnakes/ppa -y && \
        apt-get update && \
        apt-get install -y \
            python${PYTHON_VERSION} \
            python${PYTHON_VERSION}-dev \
            python${PYTHON_VERSION}-venv && \
        (apt-get install -y python${PYTHON_VERSION}-distutils 2>/dev/null || true) && \
        apt-get clean -y && \
        rm -rf /var/lib/apt/lists/* && \
        update-alternatives --install /usr/bin/python3 python3 /usr/bin/python${PYTHON_VERSION} 1 && \
        update-alternatives --set python3 /usr/bin/python${PYTHON_VERSION}; \
    fi && \
    echo "Installed Python version:" && python3 --version

# Install CMake; use Kitware repo on Ubuntu 20.04 for >=3.22
RUN if [ "${UBUNTU_VERSION}" = "20.04" ]; then \
        apt-get update && \
        apt-get install -y wget gnupg ca-certificates software-properties-common && \
        wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc | gpg --dearmor | tee /usr/share/keyrings/kitware-archive-keyring.gpg > /dev/null && \
        echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ focal main" > /etc/apt/sources.list.d/kitware.list && \
        apt-get update && \
        apt-get install -y --no-install-recommends cmake=3.22.2-0kitware1ubuntu20.04.1 cmake-data=3.22.2-0kitware1ubuntu20.04.1; \
    else \
        apt-get update && apt-get install -y --no-install-recommends cmake; \
    fi && \
    cmake --version && \
    dpkg --compare-versions "$(cmake --version | head -n1 | awk '{print $3}')" ge 3.22 || (echo "Error: CMake >= 3.22 is required. Got ubuntu version ${UBUNTU_VERSION}" >&2; exit 1)

# Add the necessary 3rd party dependencies for the robot-service
# Note: the order is important, change at your own risk.
RUN git clone --depth 1 --recurse-submodules --shallow-submodules --branch boost-1.77.0 https://github.com/boostorg/boost.git \
    && cd boost \
    && ./bootstrap.sh --prefix=/usr \
    && ./b2 install \
    && cd ../.. \
    && rm -rf boost

RUN git clone --depth 1 --branch 10.0.0 https://github.com/leethomason/tinyxml2.git \
    && cd tinyxml2 \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    && make -j4 && make install \
    && cd ../.. \
    && rm -rf tinyxml2

RUN git clone --depth 1 --branch 1.0.2 https://github.com/ros/console_bridge.git \
    && cd console_bridge \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    && make -j4 \
    && make install \
    && cd ../.. \
    && rm -rf console_bridge

RUN git clone --depth 1 --branch 1.0.5 https://github.com/ros/urdfdom_headers.git \
    && cd urdfdom_headers \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    && make -j4 && make install \
    && cd ../.. \
    && rm -rf urdfdom_headers

COPY ./urdfdom.patch /tmp/urdfdom.patch
RUN git clone --depth 1 --branch 4.0.0 https://github.com/ros/urdfdom.git \
    && cd urdfdom \
    && git apply  /tmp/urdfdom.patch \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    && make -j4 && make install \
    && cd ../.. \
    && rm -rf urdfdom

RUN git clone --depth 1 --recurse-submodules --shallow-submodules --branch v5.4.3 https://github.com/assimp/assimp.git \
    && cd assimp \
    && mkdir build && cd build \
    && cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DASSIMP_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    && make -j4 && make install \
    && cd ../.. \
    && rm -rf assimp

COPY ./pinocchio.patch /tmp/pinocchio.patch
RUN git clone --depth 1 --recurse-submodules --shallow-submodules --branch v3.4.0 https://github.com/stack-of-tasks/pinocchio.git \
    && cd pinocchio \
    && git apply /tmp/pinocchio.patch \
    && mkdir build  && cd build \
    && cmake .. -DBoost_USE_STATIC_LIBS=ON -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DBUILD_PYTHON_INTERFACE=OFF -DBUILD_DOCUMENTATION=OFF -DBUILD_TESTING=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    && make -j4 && make install \
    && cd ../.. \
    && rm -rf pinocchio

COPY ./gtest.patch /tmp/gtest.patch
RUN git clone --depth 1 --branch v1.17.0 https://github.com/google/googletest.git \
    && cd googletest \
    && git apply /tmp/gtest.patch \
    && mkdir build && cd build \
    && cmake .. -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DBUILD_SHARED_LIBS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr -DCMAKE_INSTALL_LIBDIR=/usr/lib \
    && make -j4 && make install \
    && cd ../.. \
    && rm -rf googletest

# Install Python wheel building tools inside a virtualenv to satisfy PEP 668
# Note: python3 -m venv uses ensurepip to bootstrap pip automatically
RUN python3 -m venv /opt/venv && \
        /opt/venv/bin/pip install --upgrade pip setuptools wheel && \
        /opt/venv/bin/pip install \
            auditwheel \
            build \
            cibuildwheel \
            twine \
            patchelf \
            flake8 \
            numpy \
            pybind11 \
            cmake && \
    echo "Python version in venv:" && \
    /opt/venv/bin/python --version && \
    /opt/venv/bin/pip --version && \
    chown -R $USER_UID:$USER_GID /opt/venv

# Expose the virtualenv via env var for downstream scripts (e.g., Jenkinsfile)
ENV VIRTUAL_ENV="/opt/venv"
ENV PATH="${VIRTUAL_ENV}/bin:${PATH}"

USER $USERNAME
