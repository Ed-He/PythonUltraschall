# GPIO-Pins benutzen
import RPi.GPIO as GPIO
# Einbindung der Zeit
import time
# from datetime import datetime
# MQTT Dateb versebdeb
import json
from paho.mqtt import client as mqtt_client
# import ssl
# Routine bei Programmstop
# import signal
# import sys


def main():
    # Variable Werte zum Einstellen/ Einrichten
    array_size = 2  # Die Menge an Distanzen die überprüft werden
    tolerance = 3  # Toleranz einstellung wie genau berechnet wird ob die Maschine läuft
    sleep_time = 0.5  # Wartezeit zwischen Messungen
    gpio_trigger = 18
    gpio_echo = 23

    # Vordefinierte Werte für die allgemeine Funktion
    iteration = 0  # initialisierung der ersten Iteration
    status_running = False  # Initialisierung des Maschinenstatus
    measurements = [0] * array_size  # Array Größe initialisieren
    GPIO.setmode(GPIO.BCM)
    speed_sound = 34300  # Schallgeschwindikeit
    publish_interval = 1
    data = {}

    # MQTT-Konfiguration für Kommunikation mit Node-Red
    broker = 'localhost'
    port = 1883
    # topic = "python/ultrasonic/distance"
    client_id = f'python-mqtt-ultrasonic'

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    def on_message(client, userdata, msg_par):
        settings = json.loads(msg_par.payload)
        print("received: " + str(settings))
        # setzen von publishInterval
        nonlocal publish_interval
        publish_interval = float(settings["publish_interval"])

    def get_distance():
        trigger = gpio_trigger
        echo = gpio_echo
        GPIO.setup(trigger, GPIO.OUT)  # Trigger
        GPIO.setup(echo, GPIO.IN)  # Echo

        GPIO.output(trigger, False)
        time.sleep(sleep_time)
        GPIO.output(trigger, True)
        time.sleep(0.00001)
        GPIO.output(trigger, False)
        trigger_time = time.time()

        start = 0
        stop = 0

        while GPIO.input(echo) == 0:
            start = time.time()

            if time.time() - trigger_time > 0.1:  # war 0,1, wenn länger als 0.1 Sekunden FALSE --> Messung ungültig
                return -2

        while GPIO.input(echo) == 1:
            stop = time.time()

        # berechnet distanz
        distance = ((stop - start) * speed_sound) / 2

        if distance > 400:
            return -1

        return distance

    def read_sensor(iteration_par):

        distance = get_distance()

        measurements[iteration_par] = distance

        time.sleep(sleep_time)

        return distance

    def publish_broker(client_par, status, topic_name, active_time_par, inactive_time_par):
        data["status"] = status
        data["active_time"] = int(active_time_par)
        data["inactive_time"] = int(inactive_time_par)
        json_message = json.dumps(data)
        result = client_par.publish(topic_name, json_message, qos=1)  # Status an Broker senden
        status = result[0]
        if status == 0:
            print(f"Send `{json_message}` to topic `{topic_name}`")
        else:
            print(f"Failed to send message to topic {topic_name}")

        # MQTT-Client anlegen, Callbacks registrieren und zum Broker verbinden

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)

    client.loop_start()  # Background-Task starten, der die Callbacks ausführt
    client.subscribe("python/ultrasonic/settings", qos=2)  # auf Topic subscriben

    try:
        start_time = time.time()
        active_time = time.time() - start_time
        inactive_time = active_time
        while True:
            read_sensor(iteration)
            iteration = iteration + 1

            # Nach einer vollständigen Messung wird die Iteration zurückgesetzt und Daten erfasst
            if iteration >= array_size:
                iteration = 0

                # Durchlauf der gemessenen Werte und Erfassen des Maschinenstatus
                for x in range(array_size):
                    if measurements[x] == -1:
                        print("Distanz ist zu Weit! Größer 4 Meter")
                        status_running = False
                    elif measurements[x] == -2:
                        print("Messung ist ungültig!!")
                    elif abs(measurements[x] - measurements[0]) >= tolerance:
                        status_running = True
                    else:
                        status_running = False

            if status_running:
                active_time = time.time() - (start_time + inactive_time)
                msg = "Maschine is Running"
            else:
                inactive_time = time.time() - (start_time + active_time)
                msg = "Maschine is off"

            publish_broker(client, msg, "status", active_time, inactive_time)

    # Programm beenden
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Programm abgebrochen")
        client.disconnect()


main()
