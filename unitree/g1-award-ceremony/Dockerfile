ARG ROS_BASE_IMAGE=bj-warehouse.tencentcloudcr.com/phanthy-motus/ros-base:latest
FROM ${ROS_BASE_IMAGE}

WORKDIR /work
COPY main.py /work/main.py

ENV PYTHONUNBUFFERED=1
ENV RCUTILS_COLORIZED_OUTPUT=0
ENV AGENT_CORE_URL=https://localhost:15678
ENV G1_MCP_URL=http://localhost:15701/mcp

EXPOSE 15799

CMD ["python3", "/work/main.py"]
