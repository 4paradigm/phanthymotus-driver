ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE} AS ros-base
# Use a public mirror; the base image's Tencent mirror requires VPC DNS.
ARG APT_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports
ARG ROS_APT_MIRROR=http://packages.ros.org/ros2/ubuntu

# Some ros-base images remove the source link while retaining ros2-apt-source.
# Restore its signed repository configuration for both build and runtime stages.
RUN if [ ! -e /etc/apt/sources.list.d/ros2.sources ] \
        && [ -f /usr/share/ros-apt-source/ros2.sources ]; then \
        ln -s /usr/share/ros-apt-source/ros2.sources /etc/apt/sources.list.d/ros2.sources; \
    fi \
    && sed -i -e "s|http://mirrors.tencentyun.com/ubuntu-ports|${APT_MIRROR}|g" \
        -e "s|http://ports.ubuntu.com/ubuntu-ports|${APT_MIRROR}|g" /etc/apt/sources.list \
    && find -L /etc/apt/sources.list.d -type f -exec sed -i \
        -e "s|http://packages.ros.org/ros2/ubuntu|${ROS_APT_MIRROR}|g" \
        -e 's/^Types: deb deb-src/Types: deb/' -e '/^deb-src /d' {} +

FROM ros-base AS build-deps
# The robot link uses libfranka TCP/UDP; DDS stays on the fleet loopback profile.
# Pinocchio comes from the ROS repository, whose packages support ARM64.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates python3-yaml ros-humble-rmw-fastrtps-cpp \
        ros-humble-pinocchio libpocofoundation80 libpoconet80 libfmt8 \
        libtinyxml2-9 libconsole-bridge1.0 \
    && rm -rf /var/lib/apt/lists/*

FROM build-deps AS builder

RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        build-essential cmake pkg-config patch libpoco-dev libeigen3-dev libfmt-dev \
        libtinyxml2-dev libconsole-bridge-dev \
    && rm -rf /var/lib/apt/lists/*

# libfranka 0.18.1 uses FCI protocol 10 (robot system >= 5.9.0).
# Keep pinned official source archives in the context so SDK compilation does
# not depend on GitHub access from the build container. See vendor/SOURCE.md.
COPY vendor/ /tmp/franka-source/
COPY cmake/ /tmp/franka-cmake/
RUN echo "b682dbb2c6c564608b7b3ec443f332545dde94dee6969237f182441f21c23951  /tmp/franka-source/libfranka-182c9c4f7ffb512d4b2e2efbc86643e751389771.tar.gz" | sha256sum -c - \
    && echo "75f26fc8794e4ae149086b59dda9de9d61cba845c375a51d13dd886ba7a9c104  /tmp/franka-source/libfranka-common-d96c221a32c343384cbe2f77c61116ed58b84231.tar.gz" | sha256sum -c - \
    && mkdir -p /tmp/libfranka/common \
    && tar -xzf /tmp/franka-source/libfranka-182c9c4f7ffb512d4b2e2efbc86643e751389771.tar.gz --strip-components=1 -C /tmp/libfranka \
    && tar -xzf /tmp/franka-source/libfranka-common-d96c221a32c343384cbe2f77c61116ed58b84231.tar.gz --strip-components=1 -C /tmp/libfranka/common \
    && patch --batch --forward --fuzz=0 -d /tmp/libfranka -p1 < /tmp/franka-cmake/libfranka-offline-version.patch

RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
    && cmake -S /tmp/libfranka -B /tmp/libfranka/build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
        -DCMAKE_MODULE_PATH=/tmp/franka-cmake \
        -DCMAKE_INSTALL_LIBDIR=lib -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF \
        -DBUILD_DOCUMENTATION=OFF -DGENERATE_PYLIBFRANKA=OFF \
    && cmake --build /tmp/libfranka/build --parallel 2 \
    && cmake --install /tmp/libfranka/build'

COPY native/ /tmp/franka-native/
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash \
    && cmake -S /tmp/franka-native -B /tmp/franka-native/build \
        -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
    && cmake --build /tmp/franka-native/build --parallel 2 \
    && cmake --install /tmp/franka-native/build'

FROM ros-base

# ros-humble-pinocchio depends on all Boost development packages, MPI and a
# Fortran compiler. Keep that SDK build environment out of the final image.
# These runtime packages and five ROS library families were checked with ldd
# against the ARM64 reader; apt resolves the Ubuntu libraries' dependencies.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ca-certificates python3-yaml ros-humble-rmw-fastrtps-cpp \
        libpocofoundation80 libpoconet80 libfmt8 libtinyxml2-9 libconsole-bridge1.0 \
        libboost-filesystem1.74.0 libboost-serialization1.74.0 \
        libassimp5 liboctomap1.9 libtinyxml2.6.2v5 \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /usr/local/lib/libfranka.so.0.18.1 /usr/local/lib/
COPY --from=builder /opt/ros/humble/lib/*/libpinocchio_default.so.* \
    /opt/ros/humble/lib/*/libpinocchio_parsers.so.* \
    /opt/ros/humble/lib/*/libcoal.so.* \
    /opt/ros/humble/lib/*/liburdfdom_model.so.* \
    /opt/ros/humble/lib/*/liburdfdom_world.so.* /usr/local/lib/
COPY --from=builder /usr/local/bin/franka-state-reader /usr/local/bin/
COPY --from=builder /tmp/libfranka/LICENSE /tmp/libfranka/NOTICE /usr/share/licenses/libfranka/
COPY --from=builder /tmp/libfranka/common/LICENSE /tmp/libfranka/common/NOTICE /usr/share/licenses/libfranka-common/
COPY --from=builder /usr/share/doc/ros-humble-pinocchio/copyright /usr/share/licenses/pinocchio/copyright
COPY --from=builder /usr/share/doc/ros-humble-coal/copyright /usr/share/licenses/coal/copyright
COPY --from=builder /usr/share/doc/ros-humble-urdfdom/copyright /usr/share/licenses/urdfdom/copyright
# Resolve copied SDK libraries without running ldconfig under QEMU.
# Keep the no-argument smoke check: it loads all libraries and must exit 2.
ENV LD_LIBRARY_PATH=/usr/local/lib
RUN ln -s libfranka.so.0.18.1 /usr/local/lib/libfranka.so.0.18 \
    && (/usr/local/bin/franka-state-reader; test "$?" -eq 2)

WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py hardware.py config.yaml driver.yaml requirements.txt /work/franka/fr3/
COPY resource/ /work/franka/fr3/resource/
COPY deploy/ /deploy/

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    ROS_LOCALHOST_ONLY=0 \
    PYTHONUNBUFFERED=1 \
    RCUTILS_COLORIZED_OUTPUT=0 \
    PYTHONPATH=/work

EXPOSE 15723
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 /work/franka/fr3/main.py"]
