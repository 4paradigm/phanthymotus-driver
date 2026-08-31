FROM python:3.11-slim
WORKDIR /work
RUN pip install --no-cache-dir pymavlink>=2.4.41,<3 PyYAML>=6
COPY main.py device.py config.yaml driver.yaml ./
ENV PYTHONUNBUFFERED=1
EXPOSE 15719
CMD ["python3", "main.py"]
