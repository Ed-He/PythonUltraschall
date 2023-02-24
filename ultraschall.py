import time
import RPi.GPIO as GPIO
import re
import array as arr

def main():
    arraySize = 10  # Die Menge an Distanzen die überprüft werden
    tolerance = 1  # Toleranz einstellung wie genau berechnet wird ob die Maschine läuft
    sleepTime = 0.5 # Wartezeit zwischen Messungen

    iteration = 0  # initialisierung der ersten Iteration
    statusRunning = False  # Initialisierung des Maschinenstatus
    measurements = [0] * arraySize  # Array Größe initialisieren

    GPIO.setmode(GPIO.BCM)
    speedSound = 34300

    #MQTT-Konfiguration für Kommunikation mit Node-Red
    broker = 'localhost'
    port = 1883
    topic = "python/ultrasonic/distance"
    client_id = f'python-mqtt-ultrasonic'

    data={
        "distance": 0 #enthält zuletzt gemessene Distanz
        }

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

        if(distance > 400):
            return -1

        return distance

    def read_sensor(trigger, echo, name, itteration):
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
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")

        # MQTT-Client anlegen, Callbacks registrieren und zum Broker verbinden

    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(broker, port)

    client.loop_start()  # Background-Task starten, der die Callbacks ausführt
    # client.subscribe("python/ultrasonic/settings", qos=2)  # auf Topic subscriben

    try:
        while True:
            read_sensor(20, 12, "Lang", iteration)

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
                #Aktive Zeit mit einbringen - Wie lange Maschine läuft
                msg = "Maschine is Running"
                #print("Maschine is Running: Distanz = " + str("%.2f" % measurements[iteration]))
            else:
                msg = "Maschine is off"
                #print("Maschine is off: Distanz = " + str("%.2f" % measurements[iteration]))

            publishBroker(client, msg, "status")

    # Programm beenden
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Programm abgebrochen")

main()