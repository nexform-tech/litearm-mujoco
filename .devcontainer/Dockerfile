# Pre-built dev container image for litearm-mujoco.
#
# Python 3.12 matches the CI test runner. The image pre-installs the project
# with the dev extra, so a container starts ready to run tests and examples.
# The OpenGL/X11 libraries let the MuJoCo viewer open when the container can
# reach a display. With no display at all the viewer cannot start and the
# examples exit non-zero; run them under `xvfb-run -a` instead, which is why
# xvfb is installed below.
FROM python:3.12-slim-bookworm

# libglfw3 and the libX*/libgl/libegl/libosmesa entries are the viewer's X11
# and OpenGL runtime; the MuJoCo wheel loads libglfw.so.3 at run time. xauth and
# x11-utils provide X11 authentication and `xdpyinfo`, which is how you tell a
# missing display apart from a missing socket mount. xvfb gives tests and
# examples a virtual display on hosts that have none.
#
# No compiler toolchain: every dependency is a wheel or pure Python, which is
# the point of this package. Install `build-essential` with apt if you need one.
RUN apt-get update && apt-get install -y --no-install-recommends \
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
        libxcursor1 \
        libxext6 \
        libxi6 \
        libxinerama1 \
        libxkbcommon0 \
        libxrandr2 \
        libxrender1 \
        sudo \
        x11-utils \
        xauth \
        xvfb \
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
# Create the workspace with the right owner before copying into it: WORKDIR
# alone would leave it root-owned, and the container user could not write
# .pytest_cache beside the sources.
RUN mkdir -p /workspaces/litearm-mujoco && chown $USERNAME:$USERNAME /workspaces/litearm-mujoco
WORKDIR /workspaces/litearm-mujoco
COPY --chown=$USERNAME:$USERNAME . .
USER $USERNAME

# pip installs into the user site when run as a non-root user; make its
# console scripts (pytest) reachable on PATH.
ENV PATH="/home/$USERNAME/.local/bin:$PATH" \
    PYTHONUNBUFFERED=1

# No `mirror` extra: the Zenoh SDK was replaced by the litearm-python
# dependency in the base install. MUJOCO_GL is left unset so MuJoCo picks its
# default backend. It only selects the backend for offscreen rendering, so it
# does not make the viewer work on a host without a display -- `xvfb-run -a`
# does, which is why xvfb is installed above.
RUN pip install --no-cache-dir -e ".[dev]"

LABEL org.opencontainers.image.source="https://github.com/nexform-tech/litearm-mujoco" \
      org.opencontainers.image.description="Pre-built dev container for litearm-mujoco"
