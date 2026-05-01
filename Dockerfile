ARG BUILD_FROM=python:3.12-alpine
FROM $BUILD_FROM

RUN pip3 install --no-cache-dir pyserial paho-mqtt

COPY srne_modbus.py /srne_modbus.py
COPY run.sh /run.sh
RUN chmod +x /run.sh

CMD ["/run.sh"]
