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
        udev \
        usbutils && \
    rm -rf /var/lib/apt/lists/*

# ── Download and build Payload SDK ──────────────────────────────────
ARG PSDK_URL=https://agi-phanthy-dev-1252788780.cos.ap-beijing.myqcloud.com/public/Payload-SDK-3.16.0.tar.gz
RUN curl -fsSL ${PSDK_URL} | tar xz -C /opt && \
    mv /opt/Payload-SDK-* /opt/Payload-SDK && \
    mkdir -p /opt/Payload-SDK/build && \
    cd /opt/Payload-SDK/build && \
    cmake ../psdk_lib -DCMAKE_BUILD_TYPE=Release && \
    make -j$(nproc)

# PSDK install location for downstream images
ENV PSDK_DIR=/opt/Payload-SDK
