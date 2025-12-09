# syntax=docker/dockerfile:1.7-labs
# needed until COPY --parents is available in stable

FROM osrf/ros:kilted-desktop-full
WORKDIR /ws

RUN git config --global --add safe.directory /ws
ADD https://astral.sh/ruff/install.sh /tmp/install-ruff.sh
RUN sh /tmp/install-ruff.sh && rm /tmp/install-ruff.sh

ADD https://cyberbotics.com/Cyberbotics.asc /etc/apt/keyrings/Cyberbotics.asc
RUN chown _apt /etc/apt/keyrings/Cyberbotics.asc

RUN <<EOF
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" > /etc/apt/sources.list.d/Cyberbotics.list
apt-get update
EOF
RUN apt-get upgrade -y clangd webots python3-pylsp

COPY --parents src/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]
