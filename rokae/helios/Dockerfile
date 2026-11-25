# Build with driver files + common/ in the context (the build.sh convention).
# Use the established driver base; do not carry Core or its virtualenv.
ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
WORKDIR /driver
COPY requirements.txt /driver/rokae/helios/requirements.txt
# Vendor chassis SDK (closed binaries): fetched from the project COS at build time and
# hash-checked, never committed to Git (same as agibot/A3 AimDK, dji/base PSDK,
# noetix/bumi). Archive layout: lib/ with the cp312 aarch64 nav / py_client_common /
# exception / odometry modules and their libclient_*.so.
# The SDK ships CPython 3.12 extension modules only, while ROS Humble uses 3.10, so it runs
# in its own worker process under a pinned standalone CPython 3.12 (python-build-standalone,
# PSF licence, stripped build: 88 MB). Its binaries need glibc >= 2.17 / GLIBCXX >= 3.4.22,
# satisfied by Ubuntu 22.04. Both are installed only when VMR_SDK_URL is set: without the
# SDK the runtime would be unusable, so the default image does not grow and base_drive
# reports the SDK as unavailable.
# The build host reaches GitHub release downloads slowly or not at all, so the runtime is
# fetched from the npmmirror CDN first and GitHub second, with a socket timeout and progress
# output per attempt (the bot kills silent builds); the pinned SHA256 is checked either way.
ARG VMR_SDK_URL=
ARG VMR_SDK_SHA256=
ARG SDK_PYTHON_URL=https://registry.npmmirror.com/-/binary/python-build-standalone/20261110/cpython-3.12.16%2B20261110-aarch64-unknown-linux-gnu-install_only_stripped.tar.gz
ARG SDK_PYTHON_FALLBACK_URL=https://github.com/astral-sh/python-build-standalone/releases/download/20261110/cpython-3.12.16%2B20261110-aarch64-unknown-linux-gnu-install_only_stripped.tar.gz
ARG SDK_PYTHON_SHA256=b6a41defec4975d94b4ff1950236e7383e5c02ba40ea48c9ec33c66087a35aab
COPY build_fetch.py /tmp/build_fetch.py
RUN if [ -n "${VMR_SDK_URL}" ]; then \
      test -n "${VMR_SDK_SHA256}" && \
      python3 /tmp/build_fetch.py /tmp/sdk-python.tar.gz "${SDK_PYTHON_SHA256}" "${SDK_PYTHON_URL}" "${SDK_PYTHON_FALLBACK_URL}" && \
      mkdir -p /opt/sdk-python && tar xzf /tmp/sdk-python.tar.gz -C /opt/sdk-python --strip-components=1 && \
      rm -f /tmp/sdk-python.tar.gz && /opt/sdk-python/bin/python3 -c "import sys; assert sys.version_info[:2] == (3, 12)" && \
      python3 /tmp/build_fetch.py /tmp/vmr-sdk.tar.gz "${VMR_SDK_SHA256}" "${VMR_SDK_URL}" && \
      mkdir -p /opt/vmr-sdk && tar xzf /tmp/vmr-sdk.tar.gz -C /opt/vmr-sdk && \
      rm -f /tmp/vmr-sdk.tar.gz && \
      PYTHONPATH=/opt/vmr-sdk/lib LD_LIBRARY_PATH=/opt/vmr-sdk/lib /opt/sdk-python/bin/python3 -c "import nav, py_client_common, odometry"; \
    else \
      echo "VMR_SDK_URL not set: building without the chassis SDK and its Python runtime (base_drive reports the SDK as unavailable)"; \
    fi && rm -f /tmp/build_fetch.py
ARG PYPI_MIRROR=https://pypi.tuna.tsinghua.edu.cn/simple
RUN python3 -m pip install --no-cache-dir -i ${PYPI_MIRROR} -r /driver/rokae/helios/requirements.txt
COPY common/__init__.py common/vendor_runtime.py common/logsafe.py /driver/common/
COPY __init__.py main.py device.py model.py feedback.py battery.py chassis_battery_pb2.py chassis_move.py chassis_sdk_worker.py config.yaml driver.yaml /driver/rokae/helios/
COPY resource/ /driver/rokae/helios/resource/
COPY deploy/ /deploy/
ENV PYTHONPATH=/driver PYTHONUNBUFFERED=1 RCUTILS_COLORIZED_OUTPUT=0 ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && python3 -c "import rclpy; from rokae.helios.battery import HeliosBattery; from rokae.helios.device import HeliosJoints; from rokae.helios.chassis_move import HeliosChassisMove"'
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && exec python3 -m rokae.helios.main"]
