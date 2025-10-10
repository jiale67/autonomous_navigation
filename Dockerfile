# 使用官方 ROS Noetic Robot 镜像作为基础
FROM ros:noetic-robot

# 维护者信息
LABEL maintainer="1821377855@qq.com"

# 设置非交互式安装，避免 apt 提示
ENV DEBIAN_FRONTEND=noninteractive

# ===== 替换为阿里云镜像源 =====
RUN sed -i 's/archive.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.aliyun.com/g' /etc/apt/sources.list && \
    #
    # 更新包列表并安装所需软件包
    apt-get update && \
    apt-get install -y \
        # 构建工具
        python3-catkin-tools \
        python3-pip \
        build-essential \
        # ROS 依赖包
        ros-noetic-octomap-ros \
        ros-noetic-pcl-ros \
        ros-noetic-gazebo-ros \
        ros-noetic-pcl-conversions \
        ros-noetic-libg2o \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-tf2-geometry-msgs \
        ros-noetic-rviz \
        ros-noetic-rqt \
        ros-noetic-xacro \
        # 系统库依赖
        libpcap-dev \
        libpng-dev \
        libusb-1.0-0-dev \
        # GUI 支持工具
        mesa-utils \
        net-tools \
        iputils-ping \
    && \
    # 清理缓存，减小镜像大小
    rm -rf /var/lib/apt/lists/*

# 可选：设置工作空间目录（仅为提示，实际由挂载决定）
WORKDIR /catkin_ws

# 提示用户如何运行容器
CMD ["bash", "-c", "echo 'ROS Noetic 自定义镜像准备就绪。请挂载工作空间并运行：'; echo 'docker run -it --name ros_dev --env=\"DISPLAY\" --env=\"QT_X11_NO_MITSHM=1\" --volume=\"/tmp/.X11-unix:/tmp/.X11-unix:rw\" --volume=\"$HOME/automous_navigation:/catkin_ws\" --network host --privileged ros_noetic_automous:latest /bin/bash'"]

