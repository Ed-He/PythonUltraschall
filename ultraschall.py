import time
import json
import RPi.GPIO as GPIO
from paho.mqtt import client as mqtt_client
from datetime import datetime

def main():
    arraySize = 10  # Die Menge an Distanzen die überprüft werden
    tolerance = 1  # Toleranz einstellung wie genau berechnet wird ob die Maschine läuft
    sleepTime = 0.5 # Wartezeit zwischen Messungen

    iteration = 0  # initialisierung der ersten Iteration
    statusRunning = False  # Initialisierung des Maschinenstatus
    measurements = [0] * arraySize  # Array Größe initialisieren

    data = {}

    GPIO.setmode(GPIO.BCM)
    speedSound = 34300

    #MQTT-Konfiguration für Kommunikation mit Node-Red
    broker = 'localhost'
    port = 1883
    #topic = "python/ultrasonic/distance"
    client_id = f'python-mqtt-ultrasonic'

    # Definition von Variablen/ Listen
    publishInterval = 1 #Wartezeit nach jedem Messvorgang
    #data={
    #    "status": "" #enthält letzten Status
    #    }

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)

    def on_message(client, userdata, msg):
        settings=json.loads(msg.payload)
        print("received: " + str(settings))
        #setzen von publishInterval
        nonlocal publishInterval
        publishInterval=float(settings["publishInterval"])

    def get_distance(trigger, echo):

        GPIO.output(trigger, False)

        time.sleep(sleepTime)

        GPIO.output(trigger, True)

        time.sleep(0.00001)

        GPIO.output(trigger, False)

        triggerTime = time.time()

        start = 0
        stop = 0

        while GPIO.input(echo) == 0:
            start = time.time()

            if time.time()-triggerTime > 0.1:  # war 0,1, wenn länger als 0.1 Sekunden FALSE --> Messung ungültig
                return -2


        while GPIO.input(echo)==1:
            stop = time.time()

        #berechnet distanz
        distance = ((stop-start) * speedSound) / 2

        if(distance > 4000):
            return -1

        return distance

    def read_sensor(trigger, echo, itteration):
        GPIO.setup(trigger,GPIO.OUT)  # Trigger
        GPIO.setup(echo,GPIO.IN)      # Echo

        distance = get_distance(trigger, echo)

        measurements[itteration] = distance

        time.sleep(sleepTime)

    def publishBroker(client, status, topicName):
        data["status"] = status
        msg = json.dumps(data)
        result = client.publish(topicName, msg)  # Status an Broker senden
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topicName}`")
        else:
            print(f"Failed to send message to topic {topicName}")

        # MQTT-Client anlegen, Callbacks registrieren und zum Broker verbinden

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)

    client.loop_start()  # Background-Task starten, der die Callbacks ausführt
    # client.subscribe("python/ultrasonic/settings", qos=2)  # auf Topic subscriben

    try:
        start = datetime.now()
        aktiv = datetime.now() - start
        inaktiv = datetime.now() - start
        while True:
            read_sensor(20, 12, iteration)

            iteration = iteration + 1

            # Nach einer vollständigen Messung wird die Iteration zurückgesetzt und Daten erfasst
            if (iteration >= arraySize):
                iteration = 0

                # Durchlauf der gemessenen Werte und Erfassen des Maschinenstatus
                for x in range(arraySize):
                    if (measurements[x] == -1):
                        print("Distanz ist zu Weit! Größer 4 Meter")
                        statusRunning = False
                    elif (measurements[x] == -2):
                        print("Messung ist ungültig!!")
                        # statusRunning = False
                    elif (abs(measurements[x] - measurements[0]) >= tolerance):
                        statusRunning = True
                    else:
                        statusRunning = False

            if (statusRunning == True):
                aktiv = datetime.now() - (start + inaktiv)
                msg = "Maschine is Running"
            else:
                inaktiv = datetime.now() - (start + aktiv)
                msg = "Maschine is off"

            publishBroker(client, msg, "status")

    # Programm beenden
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Programm abgebrochen")

main()