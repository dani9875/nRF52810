FROM ubuntu:22.04 as base
WORKDIR /workdir

# Set environment variables for non-interactive installs
ENV DEBIAN_FRONTEND=noninteractive
ENV ZEPHYR_SDK_INSTALL_DIR=/root/ncs/toolchains/e9dba88316/opt/zephyr-sdk
ENV ZEPHYR_TOOLCHAIN_VARIANT=zephyr

ARG toolchain_version=v2.7.0
ARG NORDIC_COMMAND_LINE_TOOLS_VERSION="10-24-0/nrf-command-line-tools-10.24.0"
ARG ZEPHYR_SDK_VERSION=0.16.0

# Install dependencies
RUN apt-get -y update && \
    apt-get -y upgrade && \
    apt-get -y install wget unzip clang-format gcc-multilib make libffi-dev python3 python3-pip cmake ninja-build git curl gpg apt-transport-https && \
    apt-get -y clean && \
    rm -rf /var/lib/apt/lists/*

# Install nrfutil and toolchain manager
RUN wget -q https://developer.nordicsemi.com/.pc-tools/nrfutil/x64-linux/nrfutil && \
    mv nrfutil /usr/local/bin && \
    chmod +x /usr/local/bin/nrfutil && \
    nrfutil install toolchain-manager && \
    nrfutil install toolchain-manager search && \
    nrfutil toolchain-manager install --ncs-version ${toolchain_version} && \
    nrfutil toolchain-manager list && \
    rm -f /root/ncs/downloads/*

# Install west using pip
RUN pip3 install --upgrade pip && \
    pip3 install west && \
    pip3 install pyelftools && \
    pip3 install intelhex