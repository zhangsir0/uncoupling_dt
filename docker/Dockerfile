# ROS2 Humble Docker开发环境模板
# 基于Ubuntu 22.04
ARG ROS_DISTRO=humble
FROM ros:humble-ros-base

# 构建参数：是否使用国内镜像源
ARG USE_CN_MIRROR=true

# 避免交互式提示
ENV DEBIAN_FRONTEND=noninteractive

# 设置时区为中国时区
RUN ln -sf /usr/share/zoneinfo/Asia/Shanghai /etc/localtime && \
    echo "Asia/Shanghai" > /etc/timezone

# 配置镜像源（根据构建参数选择）
RUN if [ "$USE_CN_MIRROR" = "true" ]; then \
    sed -i 's|http://archive.ubuntu.com/ubuntu|https://mirrors.ustc.edu.cn/ubuntu|g' /etc/apt/sources.list && \
    sed -i 's|http://security.ubuntu.com/ubuntu|https://mirrors.ustc.edu.cn/ubuntu|g' /etc/apt/sources.list && \
    sed -i 's|http://ports.ubuntu.com/ubuntu-ports|https://mirrors.ustc.edu.cn/ubuntu-ports|g' /etc/apt/sources.list; \
    fi

# 更新包列表并安装 ca-certificates
RUN apt-get update && apt-get install -y ca-certificates && rm -rf /var/lib/apt/lists/*

# 更新 ROS2 仓库 GPG 密钥（解决密钥过期问题）
# 先删除过期的密钥，再添加新密钥
RUN apt-get update --allow-insecure-repositories && apt-get install -y curl gnupg2 ca-certificates && \
    rm -f /usr/share/keyrings/ros-archive-keyring.gpg && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg && \
    rm -rf /var/lib/apt/lists/*

# 合并所有 apt 安装步骤，减少层数并优化缓存
RUN apt-get update --allow-insecure-repositories && apt-get install -y \
    vim \
    wget \
    htop \
    tmux \
    build-essential \
    cmake \
    git \
    python3-pip \
    python3-argcomplete \
    python3-dev \
    python3-setuptools \
    python3-vcstool \
    bash-completion \
    curl \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-tf2-tools \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-tf2-geometry-msgs \
    ros-${ROS_DISTRO}-navigation2 \
    ros-${ROS_DISTRO}-nav2-bringup \
    ros-${ROS_DISTRO}-launch-xml \
    ros-${ROS_DISTRO}-launch-yaml \
    && rm -rf /var/lib/apt/lists/*

# 创建用户（与主机用户保持一致，避免权限问题）
ARG USER_NAME=sysadmin
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd -g ${USER_GID} ${USER_NAME} || true && \
    useradd -m -u ${USER_UID} -g ${USER_GID} -s /bin/bash ${USER_NAME} && \
    echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# 切换到用户
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}

# 安装 Python 依赖（在创建工作空间之前）
COPY --chown=${USER_NAME}:${USER_NAME} docker/requirements.txt /tmp/requirements.txt
RUN pip3 install --user --no-cache-dir -r /tmp/requirements.txt || true && \
    rm /tmp/requirements.txt

# 设置ROS2工作空间
RUN mkdir -p ~/workspace/src
WORKDIR /home/${USER_NAME}/workspace

# 复制自定义 bashrc 配置
COPY --chown=${USER_NAME}:${USER_NAME} docker/config/bashrc /home/${USER_NAME}/.bashrc_docker

# 配置ROS2环境（加载 Docker 专用配置）
RUN echo "# Load Docker-specific ROS2 configuration" >> ~/.bashrc && \
    echo "if [ -f ~/.bashrc_docker ]; then" >> ~/.bashrc && \
    echo "    source ~/.bashrc_docker" >> ~/.bashrc && \
    echo "fi" >> ~/.bashrc

# 健康检查：检查 bash 进程是否运行
HEALTHCHECK --interval=30s --timeout=10s --start-period=5s --retries=3 \
    CMD pgrep -x "bash" > /dev/null || exit 1

# 默认启动bash
CMD ["/bin/bash"]