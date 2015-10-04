import RPi.GPIO as GPIO
from time import sleep

GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

button = 14
light = 15

GPIO.setup(button, GPIO.IN, GPIO.PUD_UP)
GPIO.setup(light, GPIO.OUT)

print("Ready...")
lighton = False

count = 0
while (count < 6):
    GPIO.wait_for_edge(button, GPIO.FALLING)
    lighton = not lighton
    GPIO.output(light, lighton)
    count = count + 1
    sleep(2)
