FROM python:3.12-slim-bookworm

WORKDIR /work

COPY requirements.txt /tmp/teleop-shadow-requirements.txt
RUN pip install --no-cache-dir -r /tmp/teleop-shadow-requirements.txt

COPY main.py protocol.py runtime.py dispatch.py rtc.py smoke_recording.py config.yaml driver.yaml /work/
COPY deploy/ /deploy/

ENV PYTHONDONTWRITEBYTECODE=1
ENV PYTHONUNBUFFERED=1
ENV CONFIG_PATH=/work/config.yaml
ENV MOTUS_MCP_PORT=15711

EXPOSE 15711

RUN useradd --create-home --uid 10001 teleop
USER teleop

HEALTHCHECK --interval=30s --timeout=3s --start-period=5s --retries=3 \
  CMD ["python", "-c", "import json,os,urllib.request; p=int(os.environ['MOTUS_MCP_PORT']); d=os.environ.get('MOTUS_DRIVER_ID','teleop-shadow-driver'); s=json.load(urllib.request.urlopen(f'http://127.0.0.1:{p}/health', timeout=2)); assert s['driver_id']==d and s['actuation_enabled'] is False and s['ready'] is True and s['rtc_enabled'] is True"]

CMD ["python", "/work/main.py"]
