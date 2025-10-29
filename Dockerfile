# syntax=docker/dockerfile:1.7-labs
# needed until COPY --parents is available in stable

FROM osrf/ros:rolling-desktop-full
WORKDIR /ws

ADD https://cyberbotics.com/Cyberbotics.asc /etc/apt/keyrings/Cyberbotics.asc
RUN chown _apt /etc/apt/keyrings/Cyberbotics.asc

COPY --parents src/*/package.xml .
RUN <<EOF
set -eux
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" > /etc/apt/sources.list.d/Cyberbotics.list
apt-get update
apt-get install -y webots
rosdep install --from-path --ignore-src -y /ws/src
# apt-get clean
# rm -rf /var/lib/apt/lists/*
EOF

CMD ["sleep", "inf"]
