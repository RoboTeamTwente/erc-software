# syntax=docker/dockerfile:1.7-labs
# needed until COPY --parents is available in stable

FROM osrf/ros:rolling-desktop-full
WORKDIR /ws

ADD https://cyberbotics.com/Cyberbotics.asc /etc/apt/keyrings/Cyberbotics.asc
RUN chown _apt /etc/apt/keyrings/Cyberbotics.asc

RUN <<EOF
echo "deb [arch=amd64 signed-by=/etc/apt/keyrings/Cyberbotics.asc] https://cyberbotics.com/debian binary-amd64/" > /etc/apt/sources.list.d/Cyberbotics.list
apt-get update
EOF
RUN apt-get upgrade -y webots

COPY --parents src/*/package.xml .
RUN rosdep install --from-path --ignore-src -y /ws/src

CMD ["sleep", "inf"]

TODO: progress report for reveal deay, to show that we're not doing nothing
TODO: could: interactive sim for events
