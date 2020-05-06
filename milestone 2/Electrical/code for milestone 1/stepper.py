from time import sleep
import RPi.GPIO as GPIO
from keyboard import *

def stepperTurn(step):
        for i in range(0,step,1):
                GPIO.output(STEP, GPIO.HIGH)
                sleep(delay)
                GPIO.output(STEP, GPIO.LOW)
                sleep(delay)


DIR = 20 #2 #6
STEP = 21 #3 #5
CW = 1
CCW = 0
SPR = 200

GPIO.setmode(GPIO.BCM)
GPIO.setup(DIR, GPIO.OUT)
GPIO.setup(STEP, GPIO.OUT)
GPIO.output(DIR, CW)

delay = 0.01000

print("1")
while(True):
        char = getch()
        print(char)
        if(char=="w"):
                GPIO.output(DIR, CW)
                #while(getch()=="w"):
                stepperTurn(500)
                #print("forwarding")
        if(char=="s"):
                GPIO.output(DIR, CCW)
                #while(getch()=="s"):
                stepperTurn(500)

        if(char==" "):
                break
        sleep(0.02)
print("done")
