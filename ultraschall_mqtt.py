#Bibliotheken einbinden
import RPi.GPIO as GPIO
import time
import json
from paho.mqtt import client as mqtt_client


def main():
    time.sleep(2)
    #MQTT-Konfiguration für Kommunikation mit Node-Red
    broker = 'localhost'
    port = 1883
    topic = "python/ultrasonic/distance"
    client_id = f'python-mqtt-ultrasonic'
    
    #Definition von Variablen/ Listen
    publishInterval=1 #Wartezeit nach jedem Messvorgang
    data={
        "distance": 0 #enthält zuletzt gemessene Distanz
        }

    #GPIO Konfiguration für Ultraschallsensor
    GPIO.setmode(GPIO.BCM) #GPIO Modus (BOARD / BCM)

    #Callback, der bei Verbindung zum Broker aufgerufen wird
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
        else:
            print("Failed to connect, return code %d\n", rc)
    
    #Callback, der bei Erhalt einer Message aufgerufen wird
    #auslesen der Einstellungen
    def on_message(client, userdata, msg):
        settings=json.loads(msg.payload)
        print("received: " + str(settings))
        #setzen von publishInterval
        nonlocal publishInterval
        publishInterval=float(settings["publishInterval"])
        

    def get_distance(trigger, echo):
        GPIO.output(trigger, False)
    
        time.sleep(0.5)
        # setze Trigger auf HIGH --> Senden des Schallimpulses
        GPIO.output(trigger, True)
        # setze Trigger nach 0.01ms auf LOW
        time.sleep(0.0001)
        GPIO.output(trigger, False)
        
        TriggerZeit= time.time()
        StartZeit = time.time()
        StopZeit = time.time()

        # speichere Startzeit
        while GPIO.input(echo) == 0:    #warten bis ECHO auf HIGH gesetzt
            StartZeit = time.time()
            if time.time()-TriggerZeit > 0.1:  #wenn länger als 0.1 Sekunden FALSE --> Messung ungültig
                return -1

        # speichere Ankunftszeit
        while GPIO.input(echo) == 1:   #warten bis ECHO wieder auf FALSE gesetzt
            StopZeit = time.time()

        # Zeitdifferenz zwischen Start und Ankunft
        Laufzeit = StopZeit - StartZeit
        # mit der Schallgeschwindigkeit (343000 mm/s) multiplizieren
        # und durch 2 teilen, da hin und zurueck
        distance = int((Laufzeit * 343000) / 2)
        
        if (distance > 4000) :
            return -2
        
        return distance

    #Funktion, die Messdaten vom Sensor entgegennimmt und an Broker sendet
    def publish(client, trigger, echo, topicName):
        GPIO.setup(trigger, GPIO.OUT)
        GPIO.setup(echo, GPIO.IN)
    
        topic = "ultrasonic/" + topicName
        data["distance"] = get_distance(trigger, echo)   #Messung durchführen
        msg = json.dumps(data)
        result = client.publish(topic, msg)  #Messwert an Broker senden
        status = result[0]
        if status == 0:
            print(f"Send `{msg}` to topic `{topic}`")
        else:
            print(f"Failed to send message to topic {topic}")


    #MQTT-Client anlegen, Callbacks registrieren und zum Broker verbinden
    client = mqtt_client.Client(client_id)
    client.on_connect = on_connect
    client.on_message=on_message
    client.connect(broker, port)

    client.loop_start() #Background-Task starten, der die Callbacks ausführt
    client.subscribe("python/ultrasonic/settings", qos=2)   #auf Topic subscriben

    while True:
        publish(client, 20, 12, "Sensor1") #Messung durchführen und senden
        time.sleep(publishInterval) #eingestellte Zeit warten
        publish(client, 6, 5, "Sensor2") #Messung durchführen und senden
        time.sleep(publishInterval) #eingestellte Zeit warten

main()