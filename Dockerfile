FROM ros:humble

# 安装 ROS 2 构建工具 + Python 硬件通信依赖
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    ros-humble-tf2-ros \
    python3-pip \
    python3-serial \
    python3-smbus \
    i2c-tools \
    && rm -rf /var/lib/apt/lists/*

# 安装 Python 包：串口通信 + I2C smbus2
RUN pip3 install --no-cache-dir \
    pyserial \
    smbus2

# 复制包到工作空间
RUN mkdir -p /ros2_ws/src
COPY . /ros2_ws/src/ssmr_slip_odom

# 构建
WORKDIR /ros2_ws
RUN . /opt/ros/humble/setup.sh && colcon build --packages-select ssmr_slip_odom

# 入口脚本
RUN printf '#!/bin/bash\nsource /opt/ros/humble/setup.bash\nsource /ros2_ws/install/setup.bash\nexec "$@"\n' > /entrypoint.sh \
    && chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "ssmr_slip_odom", "slip_odom_launch.py"]
