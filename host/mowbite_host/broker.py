"""The broker of the OS seen from mowbite-host, over paho-mqtt 1.x (Ubuntu 24.04) or 2.x, MQTT 5 only.

The client reconnects by itself; on_connect runs after every connect, on_message in the thread of the client. A
request names its answer topic and correlation data as MQTT 5 properties, as the app sends them.
"""
import os
import secrets

import paho.mqtt.client as mqtt
from paho.mqtt.packettypes import PacketTypes
from paho.mqtt.properties import Properties

KEEPALIVE = 30


class Message:
    def __init__(self, topic, payload, retained, response_topic=None, correlation=None):
        self.topic, self.payload, self.retained = topic, payload, retained
        self.response_topic, self.correlation = response_topic, correlation


def _client(client_id):
    # 2.x asks for the callback form; the form of 1.x keeps one set of callbacks for both.
    if hasattr(mqtt, "CallbackAPIVersion"):
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION1, client_id=client_id, protocol=mqtt.MQTTv5)
    return mqtt.Client(client_id=client_id, protocol=mqtt.MQTTv5)


class Broker:
    """subscriptions: topics with their QoS, subscribed after every connect; will: (topic, payload) retained."""

    def __init__(self, host, port, name, subscriptions, on_message, on_connect=lambda: None, will=None):
        self.host, self.port = host, int(port)
        # The answers of the OS to this client come back here and nowhere else.
        self.answers = f"rpc/response/{name}-{secrets.token_hex(4)}"
        self.subscriptions = [*subscriptions, (self.answers, 1)]
        self._on_message, self._on_connect = on_message, on_connect
        self.connected = False
        self.client = _client(f"{name}-{os.getpid()}")
        self.client.on_connect = self._connected
        self.client.on_disconnect = self._disconnected
        self.client.on_message = self._message
        self.client.reconnect_delay_set(min_delay=1, max_delay=30)
        if will:
            self.client.will_set(will[0], will[1], qos=1, retain=True)

    def start(self):
        self.client.connect_async(self.host, self.port, keepalive=KEEPALIVE)
        self.client.loop_start()

    def stop(self):
        self.client.disconnect()
        self.client.loop_stop()

    def _connected(self, client, userdata, flags, rc, properties=None):
        if rc != 0:
            return
        self.connected = True
        for topic, qos in self.subscriptions:
            client.subscribe(topic, qos)
        self._on_connect()

    def _disconnected(self, client, userdata, rc, properties=None):
        self.connected = False

    def _message(self, client, userdata, m):
        props = getattr(m, "properties", None)
        response = getattr(props, "ResponseTopic", None) if props is not None else None
        correlation = getattr(props, "CorrelationData", None) if props is not None else None
        self._on_message(Message(m.topic, m.payload, bool(m.retain), response, correlation))

    # Sent only while connected; a retained text goes again with on_connect.
    def publish(self, topic, payload, retain=False, correlation=None):
        if not self.connected:
            return False
        props = None
        if correlation is not None:
            props = Properties(PacketTypes.PUBLISH)
            props.CorrelationData = correlation
        self.client.publish(topic, payload, qos=1 if retain else 0, retain=retain, properties=props)
        return True

    # A request to the OS whose answer comes back on self.answers.
    def request(self, payload):
        if not self.connected:
            return False
        props = Properties(PacketTypes.PUBLISH)
        props.ResponseTopic = self.answers
        self.client.publish("rpc/request", payload, qos=1, properties=props)
        return True
