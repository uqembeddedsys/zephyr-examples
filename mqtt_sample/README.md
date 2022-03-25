# MQTT Python Sample

Sample code on PC for receiving messages via MQTT

## Requirements
- [paho-mqtt](https://pypi.org/project/paho-mqtt/)  
- [mosquitto broker](https://mosquitto.org/)

Ports may need to be opened also, such as 1883.

## Testing

mosquitto_pub -h localhost -p 1883 -t testtopic -m message