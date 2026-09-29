# Pre-built dev container image for litearm-mujoco.
#
# Python 3.12 matches the CI test runner. The image pre-installs the project
# with the dev and mirror extras, so a container starts ready to run tests and
# examples. The OpenGL/X11 libraries let the MuJoCo viewer open when the
# container can reach a display; without one, rendering degrades to headless.
FROM python:3.12-slim-bookworm

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        curl \
        git \
        libegl-mesa0 \
        libegl1 \
        libgl1 \
        libglfw3 \
        libglx-mesa0 \
        libosmesa6 \
        libx11-6 \
        libxext6 \
        libxi6 \
        libxrandr2 \
        libxrender1 \
        sudo \
    && rm -rf /var/lib/apt/lists/*

# Run as a non-root user. uid/gid 1000 matches Docker Desktop's default
# workspace ownership mapping, so mounted files are writable.
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID --create-home $USERNAME \
    && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME

# Pre-install the project and every dependency. The workspace is mounted over
# this path at runtime; the editable install keeps working because the path
# does not change.
WORKDIR /workspaces/litearm-mujoco
COPY --chown=$USERNAME:$USERNAME . .
USER $USERNAME

# pip installs into the user site when run as a non-root user; make its
# console scripts (pytest) reachable on PATH.
ENV PATH="/home/$USERNAME/.local/bin:$PATH" \
    PYTHONUNBUFFERED=1
RUN pip install --no-cache-dir -e ".[dev,mirror]"

LABEL org.opencontainers.image.source="https://github.com/nexform-tech/litearm-mujoco" \
      org.opencontainers.image.description="Pre-built dev container for litearm-mujoco"
