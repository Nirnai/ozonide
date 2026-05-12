FROM ubuntu:26.04

# Accept build arguments for user configuration
ARG USER_UID=1000
ARG USER_GID=1000
ARG USERNAME=vscode

# Create user matching host user
# Remove any existing user/group at the target UID/GID first
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends sudo \
    # Remove any existing user/group at target UID/GID
    && (userdel -r $(getent passwd ${USER_UID} | cut -d: -f1) 2>/dev/null || true) \
    && (groupdel $(getent group ${USER_GID} | cut -d: -f1) 2>/dev/null || true) \
    # Create the user
    && groupadd --gid ${USER_GID} ${USERNAME} \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}

# Install system dependencies
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
    && apt-get -y install --no-install-recommends \
    build-essential \
    curl \
    git \
    openssh-client \
    ca-certificates \
    pkg-config \
    libudev-dev \
    libusb-1.0-0-dev \
    && apt-get clean \
    && rm -rf /var/lib/apt/lists/*

# Switch to the created user
USER ${USERNAME}
WORKDIR /home/${USERNAME}

# Install Rust
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y --default-toolchain stable

# Setup environment
ENV PATH="/home/${USERNAME}/.cargo/bin:${PATH}"

# Install embedded Rust target for Cortex-M4F/M7F
RUN rustup target add thumbv7em-none-eabihf

# Install essential cargo tools
RUN cargo install cargo-binutils flip-link \
    && rustup component add llvm-tools-preview

# Install probe-rs for flashing and debugging
RUN curl --proto '=https' --tlsv1.2 -LsSf https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh

USER ${USERNAME}
WORKDIR /workspaces
