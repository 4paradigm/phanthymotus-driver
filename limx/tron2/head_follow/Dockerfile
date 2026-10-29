FROM python:3.12-slim
WORKDIR /app
COPY client.py engine.py server.py /app/
COPY tests/ /app/tests/
ENV PYTHONUNBUFFERED=1 PYTHONDONTWRITEBYTECODE=1
RUN FOLLOW_HTTP_TEST=1 python -m unittest discover -s tests -v
USER 65532:65532
EXPOSE 15793
STOPSIGNAL SIGTERM
CMD ["python", "/app/server.py"]
