FROM ros:humble-ros-base
WORKDIR /work
RUN apt-get update && apt-get install -y --no-install-recommends python3-pip \
    && rm -rf /var/lib/apt/lists/*
COPY requirements.txt /work/
RUN pip3 install --no-cache-dir -r requirements.txt
COPY common /work/common
COPY main.py device.py client.py telemetry.py config.yaml /work/
ENV PYTHONPATH=/work PYTHONUNBUFFERED=1
CMD ["python3", "/work/main.py"]
