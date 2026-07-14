# ── DJI PSDK Base Image ─────────────────────────────────────────────
# Contains pre-built libpayloadsdk.a + headers + system dependencies.
# Shared by all DJI drone drivers (Mavic 3E, Matrice 30, etc.)
#
# Build (仅 PSDK 版本更新时需要重建):
#   docker buildx build -f deploy/Dockerfile.psdk-base \
#     --platform linux/arm64 --push \
#     --tag bj-warehouse.tencentcloudcr.com/phanthy-motus/psdk-base:3.16.0 \
#     --tag bj-warehouse.tencentcloudcr.com/phanthy-motus/psdk-base:latest .
# ────────────────────────────────────────────────────────────────────

ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

# ── System dependencies for PSDK ────────────────────────────────────
RUN apt-get -o Acquire::AllowInsecureRepositories=true update && \
    apt-get install -y --no-install-recommends --allow-unauthenticated \
        cmake \
        build-essential \
        curl \
        libusb-1.0-0-dev \
        libssl-dev \
        libopus-dev \
        ffmpeg \
        libavcodec-dev \
        libavformat-dev \
        libavutil-dev \
        libswscale-dev \
        libjpeg-dev \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-libav \
        udev \
        usbutils \
        iproute2 && \
    rm -rf /var/lib/apt/lists/*

# ── Download Payload SDK (pre-built static library) ───────────────
ARG PSDK_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/psdk_lib-3.16.0.tar.gz
RUN curl -fsSL ${PSDK_URL} | tar xz -C /opt && \
    mv /opt/Payload-SDK-* /opt/Payload-SDK

# PSDK install location for downstream images
# Pre-built libs: /opt/Payload-SDK/psdk_lib/lib/aarch64-linux-gnu-gcc/libpayloadsdk.a
# Headers:        /opt/Payload-SDK/psdk_lib/include/
ENV PSDK_DIR=/opt/Payload-SDK
