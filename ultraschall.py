import time
import RPi.GPIO as GPIO
import re
import array as arr

GPIO.setmode(GPIO.BCM)
speedSound = 34300

def get_distance(trigger, echo):

    GPIO.output(trigger, False)

    time.sleep(0.1)

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

    time.sleep(0.1)

if __name__ == '__main__':
    arraySize = 5 # Die Menge an Distanzen die überprüft werden
    tolerance = 1 # Toleranz einstellung wie genau berechnet wird ob die Maschine läuft

    iteration = 0 # initialisierung der ersten Iteration
    statusRunning = False # Initialisierung des Maschinenstatus
    measurements = [0] * arraySize # Array Größe initialisieren


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
                        statusRunning = False
                    elif (abs(measurements[x] - measurements[0]) >= tolerance):
                        statusRunning = True
                    else:
                        statusRunning = False

            if (statusRunning == True):
                print("Maschine is Running: Distanz = " + str("%.2f" % measurements[iteration]))
            else:
                print("Maschine is off: Distanz = " + str("%.2f" % measurements[iteration]))

    # Programm beenden
    except KeyboardInterrupt:
        GPIO.cleanup()
        print("Programm abgebrochen")