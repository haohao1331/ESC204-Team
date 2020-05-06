import RPI.GPIO as GPIO  # controls position percisely
import pigpio  # offers PWM but no accurate position
from time import sleep
import numpy as np

class Stepper:
    def __init__(self, dirPin, stepPin, steps = 200):
        self.dirPin = dirPin
        self.stepPin = stepPin
        self.steps = steps
        self.CW = 1
        self.CCW = 0
        self.dir = 1    # default direction CW
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.dirPin, GPIO.OUT)
        GPIO.setup(self.stepPin, GPIO.OUT)
        GPIO.output(self.dirPin, self.CW)

    def forward(self, dx, Vmax):
        self.dir = self.CW
        self.drive(dx, Vmax)

    def backward(self, dx, Vmax):
        self.dir = self.CCW
        self.drive(dx, Vmax)

    def drive(self, dx, Vmax):
        (steps, delays) = self.generateRamp(dx, Vmax)
        print("Drive: steps = ", steps)
        print("Drive: delays = ", delays)

        for i in range(len(steps)):
            for j in range(steps[i]):
                GPIO.output(self.stepPin, GPIO.HIGH)
                sleep(delays[i])
                GPIO.output(self.stepPin, GPIO.LOW)
                sleep(delays[i])
            print("Driving ", i)

    def generateRamp(self, dx, Vmax):
        """Generates the ramp up, middle constant
        velocity section, and ramp down np arrays
        for the forward function
        """
        acc = 100  # acceleration
        VStepSize = 100  # size of velocity leaps
        distToStepFactor = 100  # converstion factor between distance to stepper degrees

        # detecting if Vmax above limit
        if Vmax ** 2 > acc * dx:
            print("Generate Ramp: edge case triggered")
            newVmax = np.sqrt(acc * dx)
        else:
            newVmax = Vmax

        Nstep = int(newVmax / VStepSize)
        t = newVmax / acc  # total time given for acceleration
        dt = t / Nstep  # individual time interval during ramp

        vel = np.linspace(VStepSize, newVmax, Nstep)  # velocity array during ramp
        dists = vel * dt  # array of distances for each velocity step
        middleDist = dx - 2 * np.sum(dists)  # the middle distance between the ramps
        print("middleDist = ", middleDist)

        steps = dists / distToStepFactor
        delays = dt / steps
        middleSteps = middleDist / distToStepFactor
        middleDelay = delays[-1]

        if len(steps) != len(delays):
            print("Error Generate Ramp: length of steps and delay not matched")

        return np.concatenate((steps, np.array([middleSteps]), np.flipud(steps)), axis=1), \
               np.concatenate((delays, np.array([middleDelay]), np.flipud(delays)), axis=1)


def bothDrive(motorA, motorB, dx, Vmax):
    (steps, delays) = motorA.generateRamp(dx, Vmax)
    print("Drive: steps = ", steps)
    print("Drive: delays = ", delays)

    for i in range(len(steps)):
        for j in range(steps[i]):
            GPIO.output(motorA.stepPin, GPIO.HIGH)
            GPIO.output(motorB.stepPin, GPIO.HIGH)
            sleep(delays[i])
            GPIO.output(motorA.stepPin, GPIO.LOW)
            GPIO.output(motorB.stepPin, GPIO.LOW)
            sleep(delays[i])
        print("Both Driving ", i)


def bothForward(motorA, motorB, dx, Vmax):
    motorA.dir = motorA.CW
    motorB.dir = motorB.CCW

    bothDrive(motorA, motorB, dx, Vmax)


def bothBackward(motorA, motorB, dx, Vmax):
    motorA.dir = motorA.CCW
    motorB.dir = motorB.CW

    bothDrive(motorA, motorB, dx, Vmax)













