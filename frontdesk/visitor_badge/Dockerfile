ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}
# The base image's apt mirror resolves only inside the Tencent VPC.
ARG UBUNTU_MIRROR=http://mirrors.tuna.tsinghua.edu.cn/ubuntu-ports

RUN grep -RIl 'mirrors.tencentyun.com/ubuntu-ports' /etc/apt 2>/dev/null | \
    xargs -r sed -i "s#http://mirrors.tencentyun.com/ubuntu-ports#${UBUNTU_MIRROR}#g" && \
    apt-get update && apt-get install -y --no-install-recommends openssh-client curl && \
    rm -rf /var/lib/apt/lists/*

WORKDIR /work

# Windows-side dependencies, pinned by sha256 and shipped to the front-desk PC
# by `setup`, so that PC needs no internet access. First reachable URL wins.
COPY windows/deps.txt /work/windows/deps.txt
RUN mkdir -p /work/windows/deps && cd /work/windows/deps && \
    grep -v '^#' ../deps.txt | while read -r name sha url fallback; do \
        [ -n "$name" ] || continue; \
        curl -fsSL --retry 3 -o "$name" "$url" || curl -fsSL --retry 3 -o "$name" "$fallback" || exit 1; \
        echo "$sha  $name" | sha256sum -c - || exit 1; \
    done

COPY common/       /work/common/
COPY windows/      /work/windows/
COPY main.py device.py frontdesk.py config.yaml /work/
COPY deploy/       /deploy/

ENV PYTHONUNBUFFERED=1
ENV PYTHONPATH=/work

EXPOSE 15750

CMD ["python3", "/work/main.py"]
