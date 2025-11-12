# syntax=docker/dockerfile:1.7-labs
# needed until COPY --parents is available in stable

FROM osrf/ros:kilted-desktop-full
WORKDIR /ws

ADD https://github.com/helix-editor/helix/releases/download/25.07.1/helix_25.7.1-1_amd64.deb /tmp/helix.deb
RUN apt-get install -y /tmp/helix.deb && rm /tmp/helix.deb
RUN mkdir -p /root/.config/helix && ln -rs hx.toml /root/.config/helix/config.toml

ADD https://cyberbotics.com/Cyberbotics.asc /etc/apt/keyrings/Cyberbotics.asc
RUN chown _apt /etc/apt/keyrings/Cyberbotics.asc

RUN <<EOF
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" > /etc/apt/sources.list.d/Cyberbotics.list
apt-get update
EOF
RUN apt-get upgrade -y clangd webots

COPY --parents src/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]
