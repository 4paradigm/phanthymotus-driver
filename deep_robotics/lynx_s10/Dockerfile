FROM python:3.11-slim
ARG PYPI_MIRROR=https://pypi.org/simple/
RUN apt-get update && apt-get install -y --no-install-recommends ca-certificates && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /tmp/lynx-s10-requirements.txt
RUN pip3 install --no-cache-dir -i ${PYPI_MIRROR} -r /tmp/lynx-s10-requirements.txt && rm /tmp/lynx-s10-requirements.txt
WORKDIR /work
COPY common/ /work/common/
COPY main.py device.py basic_server.py config.yaml driver.yaml /work/deep_robotics/lynx_s10/
COPY resource/ /work/deep_robotics/lynx_s10/resource/
COPY deploy/ /deploy/
ENV PYTHONUNBUFFERED=1 PYTHONPATH=/work
EXPOSE 15724
CMD ["python3", "/work/deep_robotics/lynx_s10/main.py"]
