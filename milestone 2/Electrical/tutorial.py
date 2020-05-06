from time import sleep
import pigpio

DIR = 20
STEP = 21
SWITCH = 16

pi = pigpio.pi()


# setup the switch input
# pi.set_mode(SWITCH, pigpio.INPUT)
# pi.set_pull_up_down(SWITCH, pigpio.PUD_UP)

pi.set_mode(DIR, pigpio.OUTPUT)
pi.set_mode(STEP, pigpio.OUTPUT)


pi.set_PWM_dutycycle(STEP, 128)
pi.set_PWM_frequency(STEP, 500)  # limited sample rates, check table for complete list
# alternative: hardware_PWM(18, frequency, duty cycle)


def generate_ramp(ramp):
    """Generate ramp wave forms.
    ramp: List of [Frequency, Steps]
    """
    pi.wave_clear()
    length = len(ramp)
    wid = [-1] * length

    # Generate a wave per ramp level
    for i in range(length):
        frequency = ramp[i][0]
        micros = int(500000 / frequency)    # the length of pulse
        wf = []
        wf.append(pigpio.pulse(1 << STEP, 0, micros))   # pulse on
        wf.append(pigpio.pulse(0, 1 << STEP, micros))   # pulse off
        pi.wave_add_generic(wf)
        wid[i] = pi.wave_create()

    # Generate a chain of waves
    chain = []
    for i in range(length):
        steps = ramp[i][1]
        x = steps & 255
        y = steps >> 8
        chain += [255, 0, wid[i], 255, 1, x, y]

    pi.wave_chain(chain)    # Transmit chain


try:
    old_ramp = 0
    while True:
        new_ramp = pi.read(SWITCH)
        if new_ramp != old_ramp:
            if new_ramp:
                # Ramp up
                generate_ramp([[320, 200],
                               [500, 400],
                               [800, 500],
                               [1000, 700],
                               [1600, 900],
                               [2000, 10000]])
            else:
                # Ramp down
                generate_ramp([[1600, 900],
                               [1000, 700],
                               [500, 500],
                               [320, 400]])
            eld_ramp = new_ramp
        sleep(0.1)

except KeyboardInterrupt:
    print("/nCtrl-C pressed. Stopping PIGPIO and exiting...")
finally:
    pi.set_PWM_dutycycle(STEP, 0)
    pi.stop()


