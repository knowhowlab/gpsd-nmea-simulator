FROM alpine:latest

EXPOSE 2947 8888
COPY simulator /opt/nmea/simulator/
WORKDIR /opt/nmea/simulator
RUN apk add --update --no-cache python gpsd && chmod +x /opt/nmea/simulator/start.sh

CMD ["./start.sh"]
