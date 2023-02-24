import time
import json
import RPi.GPIO as GPIO
from paho.mqtt import client as mqtt_client
from datetime import datetime


def main():
    array_size = 10  # Die Menge an Distanzen die überprüft werden
    tolerance = 1  # Toleranz einstellung wie genau berechnet wird ob die Maschine läuft
    sleep_time = 0.5  # Wartezeit zwischen Messungen

    iteration = 0  # initialisierung der ersten Iteration
    status_running = False  # Initialisierung des Maschinenstatus
    measurements = [0] * array_size  # Array Größe initialisieren

    GPIO.setmode(GPIO.BCM)
    speed_sound = 34300
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

    def get_distance(trigger, echo):

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

        if distance > 4000:
            return -1

        return distance

    def read_sensor(trigger, echo, iteration_par):
        GPIO.setup(trigger, GPIO.OUT)  # Trigger
        GPIO.setup(echo, GPIO.IN)  # Echo

        distance = get_distance(trigger, echo)

        measurements[iteration_par] = distance

        time.sleep(sleep_time)

    def publish_broker(client_par, status, topic_name, active_time_par, inactive_time_par):
        data["status"] = status
        data["active_time"] = active_time_par.strftime("%H:%M:%S")
        data["inactive_time"] = inactive_time_par.strftime("%H:%M:%S")
        json_message = json.dumps(data)
        result = client_par.publish(topic_name, json_message)  # Status an Broker senden
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
        start_time = datetime.now()
        active_time = datetime.now() - start_time
        inactive_time = datetime.now() - start_time
        while True:
            read_sensor(20, 12, iteration)

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
                active_time = datetime.now() - (start_time + inactive_time)
                msg = "Maschine is Running"
            else:
                inactive_time = datetime.now() - (start_time + active_time)
                msg = "Maschine is off"

            publish_broker(client, msg, "status", active_time, inactive_time)

    # Programm beenden
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Programm abgebrochen")


main()
