#!/usr/bin/env python
'''12_09_2018 - 09:22'''
            
#GPIO-Pins benutzen
import RPi.GPIO as GPIO
#Einbinden der Zeit
import time
from datetime import datetime
#MQTT Daten versenden
import json
import paho.mqtt.publish as publish
import paho.mqtt.client as mqtt
import ssl
#Routine bei Programmstop
import signal
import sys

#BCM-Bezeichnung der Pin's verwenden
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

"""
Programmteil fuer Ultraschallsensor
"""
#Zuordnen der GPIO-Pin's zur Funktion
GPIO_TRIGGER = 18
GPIO_ECHO = 23
#Einstellen der GPIO-Pin's als Ein-/Ausgang
GPIO.setup(GPIO_TRIGGER, GPIO.OUT)
GPIO.setup(GPIO_ECHO, GPIO.IN)
#Toleranz-Wert fuer Hysterese zur Richtungsbetimmung
Tolerance = 3
# Tolerance = 1.8

#Funktion zum Ermitteln vom Abstand des Objekts
def distanz():

    #Starten der Messung ueber 10uS Puls am Triggerausgang des RPi
    GPIO.output(GPIO_TRIGGER, True)
    time.sleep(0.00001)
    GPIO.output(GPIO_TRIGGER, False)

    #Anlegen von Variablen zum Speichern von Start- und Stopzeit
    StartZeit = time.time()
    StopZeit = time.time()

    #Bestimmung der Singalstartzeit
    while GPIO.input(GPIO_ECHO) == 0:
        StartZeit = time.time()

    #Bestimmung der Signalstopzeit
    while GPIO.input(GPIO_ECHO) == 1:
        StopZeit = time.time()

    #Berechnen der doppelten Signallaufzeit
    TimeElapsed = StopZeit - StartZeit

    #Umrechnung vom Zeit in zugehoerigen Abstand ueber Ausbreitungsgeschwindigkeit
    distanz = (TimeElapsed*34300)/2

    #Rueckgabe der Distanz an die aufrufende Stelle
    return distanz

#Funktion zum Analysieren der Bewegungsrichtung
def analysis():

    #Refferenzvariable als ein ehemaliger Wert
    global reff
    global abstand
    global client

    buffer = distanz()

    #Messen des neuen Abstandes vom Objekt
    if (buffer<40):
        abstand = buffer

    #print(abstand)
    client.publish("201000000/machines/AMB-Motor/distance", abstand, qos=1)

    #If-Abfrage zur Zustandsauswertung
    if (abstand > (reff + Tolerance)) or (abstand < (reff - Tolerance)):
        analysis = 1
        reff = abstand
    else:
        analysis = 10

    #Rueckgabe des Zustandes
    return analysis


"""
Funktionen fuer MQTT-Kommunikation
"""
#Callback-Function when client receives CONNACK response from server.
#def on_connect(client, userdata, flags, rc):
    #print("Connected with result code " + str(rc))

#Callback-Function for when a PUBLISH message is received from the server.
#def on_message(client, userdata, msg):
    #print(msg.topic+" "+str(msg.payload))

#Callback-Funktion zum Ausgeben der empfangenen MQTT-Nachricht
#def on_message_print(client, userdata, message):
    #print("%s %s" % (message.topic, message.payload))

#Funktion zum Initialisieren der MQTT Verbindung
def init_mqtt():
    global client
    client = mqtt.Client()
    #client.on_connect = on_connect
    #client.on_message = on_message
    #client.tls_set(tls_version=ssl.PROTOCOL_TLS, cert_reqs=ssl.CERT_NONE)
    #client.username_pw_set("mqtt.grobvertrieb","cD2mz3LfL7c3aNah.UmY")
    client.will_set("201000000/machines/AMB-Motor/productionstatesU", json.dumps({"rule_Id":"rule_amb","state":11}), qos=1)
    client.connect("127.0.0.1", 1883, 60)
    client.loop_start()

#Funktion zum Senden der Daten
def send_mqtt():
    global status
    global client
    json = '{ "ruleId":"rule_amb", "state":' + str(status) + ', "startTime":"' + datetime.utcnow().isoformat() + 'Z" }'
    print(json)
    client.publish("201000000/machines/AMB-Motor/productionstatesU", json, qos=1)


"""
Routine bei Programmstop
"""
def signal_term_handler(signal, frame):
    global client
    client.publish("201000000/machines/AMB-Motor/productionstatesU", json.dumps({"ruleId":"rule_amb", "state":11}), qos=1)
    client.disconnect()
    GPIO.cleanup()
    sys.exit(0)

signal.signal(signal.SIGTERM, signal_term_handler)


"""
Main-Funktion des Algorithmus
"""
if __name__ == '__main__':

    #globale Variablen fuer Produktionszustand
    global status
    status = 10
    #zuweisen auf 0 um den ersten Status auch zu senden
    global statusalt
    statusalt = 0

    #globale Variablen fuer Ultraschallsensor
    global reff
    reff = distanz()
    global abstand
    abstand = reff

    #MQTT Verbindung initialisieren
    global client
    init_mqtt()

    #Hauptschleife des Algorithmus
    try:
        while True:
            #Bestimmung des Statuses
            status = analysis()

            #Senden des Zustandes bei Aenderung
            if (status != statusalt):
                send_mqtt()
                statusalt = status
                client.publish("201000000/machines/AMB-Motor/distance", abstand, qos=1)

            #Warten um Abtastfrequenz zu realisieren
            time.sleep(0.2)

    #Abbrechen der Messung bei KeyboardInterrupt (Strg + C)
    except KeyboardInterrupt:
        GPIO.cleanup()
        client.publish("201000000/machines/AMB-Motor/productionstatesU", json.dumps({"ruleId":"rule_amb", "state":11}), qos=1)
        client.disconnect()



