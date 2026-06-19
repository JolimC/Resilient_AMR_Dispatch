FROM osrf/ros:jazzy-desktop

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update \
    && apt-get install --yes --no-install-recommends \
        python3-colcon-common-extensions \
        python3-matplotlib \
        python3-paho-mqtt \
        python3-pytest \
        python3-pytest-cov \
    && rm -rf /var/lib/apt/lists/*

RUN printf '%s\n' \
      'source /opt/ros/jazzy/setup.bash' \
      '[ -f /workspace/install/setup.bash ] && source /workspace/install/setup.bash' \
      >> /root/.bashrc

WORKDIR /workspace
