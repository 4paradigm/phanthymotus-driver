# Reuse an existing installed runtime; never pull a new interpreter or packages.
ARG BASE_IMAGE
FROM ${BASE_IMAGE} AS verify
USER root
WORKDIR /opt/tron2-head-follow
COPY client.py engine.py server.py Dockerfile ./
COPY tests/ ./tests/
ENV PYTHONDONTWRITEBYTECODE=1
RUN FOLLOW_HTTP_TEST=1 python3 -m unittest discover -s tests -v

FROM ${BASE_IMAGE} AS runtime
WORKDIR /opt/tron2-head-follow
COPY --from=verify /opt/tron2-head-follow/client.py /opt/tron2-head-follow/engine.py /opt/tron2-head-follow/server.py ./
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
USER 65532:65532
EXPOSE 15793
STOPSIGNAL SIGTERM
ENTRYPOINT []
CMD ["python3", "/opt/tron2-head-follow/server.py"]
