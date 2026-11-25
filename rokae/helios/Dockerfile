# Build with driver files + common/ in the context (the build.sh convention).
# Use the established driver base; do not carry Core or its virtualenv.
ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /driver
COPY requirements.txt /driver/rokae/helios/requirements.txt
# The vendor chassis SDK (VMR AMR SDK 2.1.51) ships CPython 3.12 extension modules only,
# while ROS Humble uses 3.10. The SDK therefore runs in its own worker process under a
# pinned standalone CPython 3.12 (python-build-standalone, PSF licence), used for nothing
# else. Its binaries need glibc >= 2.17 / GLIBCXX >= 3.4.22, satisfied by Ubuntu 22.04.
ARG SDK_PYTHON_URL=https://github.com/astral-sh/python-build-standalone/releases/download/20261110/cpython-3.12.16%2B20261110-aarch64-unknown-linux-gnu-install_only.tar.gz
ARG SDK_PYTHON_SHA256=4792f81750282597fb2a3389fe18b945ed2ba6aad5e45ad406d122c58cb33081
RUN python3 -c "import urllib.request; urllib.request.urlretrieve('${SDK_PYTHON_URL}', '/tmp/sdk-python.tar.gz')" && \
    echo "${SDK_PYTHON_SHA256}  /tmp/sdk-python.tar.gz" | sha256sum -c - && \
    mkdir -p /opt/sdk-python && tar xzf /tmp/sdk-python.tar.gz -C /opt/sdk-python --strip-components=1 && \
    rm -f /tmp/sdk-python.tar.gz && /opt/sdk-python/bin/python3 -c "import sys; assert sys.version_info[:2] == (3, 12)"
# Vendor chassis SDK (closed binaries): fetched from the project COS at build time and
# hash-checked, never committed to Git (same as agibot/A3 AimDK, dji/base PSDK,
# noetix/bumi). Archive layout: lib/ with the cp312 aarch64 nav / py_client_common /
# exception / odometry modules and their libclient_*.so. Without a URL the image builds
# normally and base_drive reports the SDK as unavailable.
ARG VMR_SDK_URL=
ARG VMR_SDK_SHA256=
RUN if [ -n "${VMR_SDK_URL}" ]; then \
      test -n "${VMR_SDK_SHA256}" && \
      python3 -c "import urllib.request; urllib.request.urlretrieve('${VMR_SDK_URL}', '/tmp/vmr-sdk.tar.gz')" && \
      echo "${VMR_SDK_SHA256}  /tmp/vmr-sdk.tar.gz" | sha256sum -c - && \
      mkdir -p /opt/vmr-sdk && tar xzf /tmp/vmr-sdk.tar.gz -C /opt/vmr-sdk && \
      rm -f /tmp/vmr-sdk.tar.gz && \
      PYTHONPATH=/opt/vmr-sdk/lib LD_LIBRARY_PATH=/opt/vmr-sdk/lib /opt/sdk-python/bin/python3 -c "import nav, py_client_common, odometry"; \
    fi
ARG PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /driver/rokae/helios/requirements.txt
COPY common/__init__.py common/vendor_runtime.py common/logsafe.py /driver/common/
COPY __init__.py main.py device.py model.py feedback.py battery.py chassis_battery_pb2.py chassis_move.py chassis_sdk_worker.py config.yaml driver.yaml /driver/rokae/helios/
COPY resource/ /driver/rokae/helios/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/driver PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import rclpy; from rokae.helios.battery import HeliosBattery; from rokae.helios.device import HeliosJoints; from rokae.helios.chassis_move import HeliosChassisMove"'
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -m rokae.helios.main"]
