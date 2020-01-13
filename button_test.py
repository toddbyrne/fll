#!/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.sensor.lego import GyroSensor
from ev3dev2.sensor.lego import ColorSensor
from ev3dev2.button import Button
from ev3dev2.display import Display
import ev3dev2.fonts as fonts
from ev3dev2.sound import Sound
import logging
import time

logging.basicConfig(level=logging.DEBUG)

gyro = GyroSensor(INPUT_4)
disp = Display()
b = Button()
s = Sound()

color_left = ColorSensor(INPUT_1)
color_right = ColorSensor(INPUT_3)

# 14, 18, ...
# See all: https://python-ev3dev.readthedocs.io/en/latest/display.html
f = fonts.load('luBS18')

def show(text):
    #disp.text_grid(t, x=5, y=5)
    disp.clear()
    # 0 = left, 80 = far right?
    # align="center" doesn't seem to work
    lr = 0
    down = 20
    #down = 50
    disp.draw.text((lr, down), text, font=f, align="left")
    disp.update()

def follow_line(speed=10, want_rli=65, edge="right"):
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    while True:
        rli = color_left.reflected_light_intensity
        t = "rli: {}".format(rli)
        show(t)
        diff = rli - want_rli
        # bigger adjustment for faster speed?
        #factor = speed / 100.0
        # light difference is +/- 40 but
        # motor difference should be small.
        factor = 0.05
        bias = diff * factor
        if edge == "left":
            bias = -bias
        m1.on(speed - bias)
        m2.on(speed + bias)
        time.sleep(0.01)

def follow_line_left():
    follow_line(speed=10, want_rli=65, edge="left")

def follow_line_right():
    follow_line(speed=10, want_rli=65, edge="right")

# Needs 2 color sensors!!!
def align_black(speed=10, black_thresh=20):
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    m1.on(speed)
    m2.on(speed)
    left_running = True
    right_running = True
    while left_running or right_running:
        rli_left = color_left.reflected_light_intensity
        rli_right = color_right.reflected_light_intensity
        t = "rli: {} {}".format(rli_left, rli_right)
        show(t)
        if rli_left <= black_thresh:
            m1.off()
            left_running = False
        if rli_right <= black_thresh:
            m2.off()
            right_running = False

def align_forever():
    while True:
        s.beep()
        #follow_line(speed=20, edge="left")
        align_black(speed=10)
        s.beep()
        while not b.any():
            rli_left = color_left.reflected_light_intensity
            rli_right = color_right.reflected_light_intensity
            t = "rli: {} {}".format(rli_left, rli_right)
            show(t)
            time.sleep(0.1)

def display_rfi():
    while not b.any():
        rli = color_left.reflected_light_intensity
        t = "rli: {}".format(rli)
        show(t)
        logging.info(t)
        time.sleep(0.1)

def beep_and_wait_for_button():
    # Don't do anything else
    s.beep()
    while not b.any():
        time.sleep(0.1)
    s.beep()

def motor_test():
    m = LargeMotor(OUTPUT_D)
    m.on_for_rotations(SpeedPercent(75), 5)
    #hoi bois!!!!

def gyro_test():
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    m1.on(20)
    m2.on(-20)
    gyro.wait_until_angle_changed_by(90)
    m1.off()
    m2.off()

# Resets to 0, does not fix drift
gyro.reset()

def long_gyro_test():
    s.beep()
    show("push a button")
    #b.wait_for_pressed([Button.enter, Button.right])
    #b.wait_for_bump([Button.enter, Button.right])
    b.wait_for_pressed(["enter"])
    s.beep()
    gyro_test()
    #motor_test()
    #for i in range(50):
    # Loop until button pressed, displaying gyro angle
    while not b.any():
        ang = gyro.angle
        t = "Angle: {}".format(ang)
        show(t)
        logging.info(t)
        time.sleep(0.1)

done = False
wait_for = None
choice = 0
progs = [
    ("gyro test", gyro_test),
    ("motor test", motor_test),
    ("align black", align_black),
    ("line left", follow_line_left),
    ("line right", follow_line_right),
]

def change(changed_buttons):
    global done
    global choice
    # changed_buttons is a list of 
    # tuples of changed button names and their states.
    logging.info('These buttons changed state: ' + str(changed_buttons))
    if wait_for is not None and wait_for in changed_buttons:
        logging.info('You pressed the done button')
        done = True
    if ("up", True) in changed_buttons:
        choice -= 1
        if choice < 0:
            choice = len(progs) - 1
    elif ("down", True) in changed_buttons:
        choice += 1
        if choice >= len(progs):
            choice = 0
    logging.info('Done is: ' + str(done))
    # will also beep if release button
    s.beep()

# Set callback from b.process()
b.on_change = change

def run_program():
    # This loop checks button states
    # continuously and calls appropriate event handlers
    global done
    global wait_for
    done = False
    wait_for = ("enter", True)
    logging.info("Waiting for enter button.")
    while not done:
        #ang = gyro.angle
        ang = 0
        rli_left = color_left.reflected_light_intensity
        rli_right = color_right.reflected_light_intensity
        t = "Angle: {}\nrli: {} {}\nProg: {}\nWaiting for enter".format(ang, rli_left, rli_right, progs[choice][0])
        show(t)
        b.process()
        time.sleep(0.1)

    logging.info("And done.")
    logging.info("Running {}".format(progs[choice][0]))
    progs[choice][1]()
    s.beep()

s.beep()
s.beep()
s.beep()
while True:
    run_program()
