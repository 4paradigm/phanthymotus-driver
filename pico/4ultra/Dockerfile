ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
ARG PYPI_MIRROR=https://pypi.org/simple/
WORKDIR /work
COPY requirements.txt ./
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r requirements.txt
COPY common/logsafe.py common/teleop_contract.py /work/common/
COPY ext_vr/*.py ext_vr/requirements.transport.lock ext_vr/NOTICE.md ext_vr/ADOPTED_SOURCE.json /work/ext_vr/
COPY ext_vr/openxr_capture_native/scripts/fetch_apk.py /work/ext_vr/openxr_capture_native/scripts/
COPY ext_vr/openxr_capture_native/package-manifest.json /work/ext_vr/openxr_capture_native/
COPY ext_vr/openxr_capture_native/app/build.gradle.kts /work/ext_vr/openxr_capture_native/app/
RUN pip3 install --no-cache-dir --require-hashes -i ${PYPI_MIRROR} -r /work/ext_vr/requirements.transport.lock
RUN python3 /work/ext_vr/openxr_capture_native/scripts/fetch_apk.py /work/ext_vr/openxr_capture_native/package-manifest.json /work/onboarding
COPY main.py identity.py config.yaml driver.yaml ./
COPY dds-local.xml /opt/phanthy-motus/dds-local.xml
COPY deploy/ /deploy/
ENV RCUTILS_COLORIZED_OUTPUT=0 PYTHONUNBUFFERED=1 PYTHONPATH=/work ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
ENV FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml ROS_LOCALHOST_ONLY=0
EXPOSE 15741 15742
CMD ["/bin/bash", "-lc", "source /opt/ros/humble/setup.bash && exec python3 /work/main.py"]
