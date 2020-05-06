from time import sleep
import RPi.GPIO as GPIO
from keyboard import *

def turn(topDIR,topSTEP,delay,step):
   CW = 1
   CCW = 0
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(topDIR,GPIO.OUT)
   GPIO.setup(topSTEP,GPIO.OUT)
   GPIO.output(topDIR,CW)
   for i in range(0,step,1):
      GPIO.output(topSTEP,GPIO.HIGH)
      sleep(delay)
      GPIO.output(topSTEP,GPIO.LOW)
      sleep(delay)

def back(topDIR,topSTEP,delay,step):
   CW = 1
   CCW = 0
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(topDIR,GPIO.OUT)
   GPIO.setup(topSTEP,GPIO.OUT)
   GPIO.output(topDIR,CCW)
   for i in range(0,step,1):
      GPIO.output(topSTEP,GPIO.HIGH)
      sleep(delay)
      GPIO.output(topSTEP,GPIO.LOW)
      sleep(delay)

def wheelForward(leftDIR,rightDIR,leftSTEP,rightSTEP,delay,step):
   CW = 0
   CCW = 1
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(leftDIR,GPIO.OUT)
   GPIO.setup(leftSTEP,GPIO.OUT)
   GPIO.setup(rightDIR,GPIO.OUT)
   GPIO.setup(rightSTEP,GPIO.OUT)
   GPIO.output(leftDIR,CW)
   GPIO.output(rightDIR,CCW)
   for i in range(0,step,1):
      GPIO.output(leftSTEP,GPIO.HIGH)
      GPIO.output(rightSTEP,GPIO.HIGH)
      sleep(delay)
      GPIO.output(leftSTEP,GPIO.LOW)
      GPIO.output(rightSTEP,GPIO.LOW)
      sleep(delay)

def wheelBackward(leftDIR,rightDIR,leftSTEP,rightSTEP,delay,step):
   CW = 0
   CCW = 1
   GPIO.setmode(GPIO.BCM)
   GPIO.setup(leftDIR,GPIO.OUT)
   GPIO.setup(leftSTEP,GPIO.OUT)
   GPIO.setup(rightDIR,GPIO.OUT)
   GPIO.setup(rightSTEP,GPIO.OUT)
   GPIO.output(leftDIR,CCW)
   GPIO.output(rightDIR,CW)
   for i in range(0,step,1):
      GPIO.output(leftSTEP,GPIO.HIGH)
      GPIO.output(rightSTEP,GPIO.HIGH)
      sleep(delay)
      GPIO.output(leftSTEP,GPIO.LOW)
      GPIO.output(rightSTEP,GPIO.LOW)
      sleep(delay)

