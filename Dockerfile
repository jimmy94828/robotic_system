FROM ros:humble-ros-base-jammy

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1

SHELL ["/bin/bash", "-c"]

# 1. 系統級與 ROS 2 依賴套件（單一 RUN 指令以最大化快取效率）
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS 2 核心與 Sensors 套件
    ros-humble-rviz2 ros-humble-desktop ros-humble-realsense2-camera ros-humble-realsense2-description \
    ros-humble-rosbag2 ros-humble-rosbag2-storage-default-plugins ros-humble-rosbag2-storage-mcap \
    ros-humble-rosbag2-transport ros-humble-rmw-cyclonedds-cpp ros-humble-ackermann-msgs \
    # ROS 2 工具與 Python 支援
    python3-colcon-common-extensions python3-rosdep python3-venv python3-pip python3-dev python3-setuptools \
    # 系統工具與編譯工具
    build-essential cmake git curl wget nano tmux ca-certificates pkg-config \
    libssl-dev libprotobuf-dev protobuf-compiler zlib1g-dev bzip2 \
    # Intel RealSense 編譯依賴與 GUI 支援
    libusb-1.0-0-dev libgtk-3-dev libcanberra-gtk-module libcanberra-gtk3-module \
    libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    # 語音節點依賴
    portaudio19-dev ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# 2. 初始化 rosdep
RUN rosdep init || true && rosdep update

# 3. 編譯安裝 Intel RealSense C++ SDK (Release 效能最佳化版)
RUN git clone --depth 1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense && \
    cd /tmp/librealsense && mkdir build && cd build && \
    cmake .. -DBUILD_EXAMPLES=false -DBUILD_GRAPHICAL_EXAMPLES=false -DBUILD_WITH_TM2=false -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc) && make install && ldconfig && rm -rf /tmp/librealsense

# 4. 安裝 Miniconda (釘版本 + 自動偵測架構)
RUN wget -q "https://repo.anaconda.com/miniconda/Miniconda3-py312_24.9.2-0-Linux-$(uname -m).sh" -O /tmp/miniconda.sh && \
    bash /tmp/miniconda.sh -b -p /opt/conda && rm /tmp/miniconda.sh && \
    /opt/conda/bin/conda clean --all -y

ENV PATH=/opt/conda/bin:$PATH
WORKDIR /robot_ws

# 5. 建立基礎 Conda 環境
RUN conda create -n robot_ros python=3.10 -y && conda clean --all -y

# 6. 設定 Conda 與 ROS 2 Python 互操作性 (.pth 橋接) 並安裝 Python 工具
RUN mkdir -p /opt/conda/envs/robot_ros/lib/python3.10/site-packages && \
    echo "/opt/ros/humble/lib/python3.10/site-packages" > /opt/conda/envs/robot_ros/lib/python3.10/site-packages/ros_humble.pth && \
    echo "/robot_ws/install" > /opt/conda/envs/robot_ros/lib/python3.10/site-packages/robot_ws_install.pth && \
    /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir --upgrade pip setuptools wheel Cython colcon-common-extensions && \
    /opt/conda/bin/conda install -n robot_ros -c conda-forge -y grpcio=1.59.3 open3d pyrealsense2 && \
    /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir \
        kachaka-api==3.14.4.0 pyyaml tqdm transforms3d catkin_pkg lxml lark-parser empy==3.3.4 && \
    conda clean --all -y

# 7. 應用層 Python 依賴（版本釘死，兩台機器才會一致）
COPY robot_ws/requirements.txt /tmp/requirements.txt
RUN /opt/conda/bin/conda run -n robot_ros pip install --no-cache-dir -r /tmp/requirements.txt

# 8. 複製 entrypoint 腳本
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 設定 ~/.bashrc，讓每個新終端都自動載入環境
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /opt/conda/etc/profile.d/conda.sh' >> ~/.bashrc && \
    echo 'conda activate robot_ros' >> ~/.bashrc && \
    echo 'source /robot_ws/install/setup.bash 2>/dev/null || true' >> ~/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
