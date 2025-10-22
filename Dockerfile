FROM osrf/ros:kilted-desktop-full
RUN apt update && apt install mesa-vulkan-drivers && apt clean && rm -rf /var/lib/apt/lists/*
