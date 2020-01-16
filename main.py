#!/usr/bin/env micropython
# #/usr/bin/env python3
from ev3dev2.motor import LargeMotor, OUTPUT_A, OUTPUT_B, OUTPUT_D, SpeedPercent, MoveTank
from ev3dev2.sensor import INPUT_1, INPUT_2, INPUT_3, INPUT_4
from ev3dev2.sensor.lego import TouchSensor
from ev3dev2.led import Leds
from ev3dev2.motor import *
from ev3dev2.wheel import *
from ev3dev2.sensor.lego import *
from ev3dev2.sensor import *
from ev3dev2.button import *
# from ev3dev2.display import Display
from ev3dev2.console import Console
from ev3dev2.sound import Sound
# import ev3dev2.fonts as fonts
import logging
import time

logging.basicConfig(level=logging.DEBUG)

# Use the gyro for turns
TURN_WITH_GYRO = True

GYRO_RESET_WAIT = 4

WHEEL_DIAMATER = 6.88
M_MOTOR = OUTPUT_B
S_MOTOR = OUTPUT_C

gyro = GyroSensor(INPUT_4)
disp = Console()
b = Button()
s = Sound()

color_left = ColorSensor(INPUT_2)
color_right = ColorSensor(INPUT_3)
# 14, 18, ...
# See all: https://python-ev3dev.readthedocs.io/en/latest/display.html
# f = fonts.load('luBS18')

"""
Please depending on the robot change the number below:                                <<<       IMPORTANT!!!!                                         HEY!!!!
WHICH_ROBOT = 0 is if the robot has one color sensor                               <<<<<<OOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOOO
WHICH_ROBOT = 1 is if the robot has two color sensor                                  <<<                          LOOK!!!!
WHICH_ROBOT = 2 is the final robot design
PLEASE AND THANK YOU
"""
WHICH_ROBOT = 2
if WHICH_ROBOT == 0:
    L_MOTOR = OUTPUT_A
    R_MOTOR = OUTPUT_B
    GYRO_PORT = INPUT_3
    IS_INVERTED = True
    DEGREES_PER_INCH = 43.0 * 2  # guess
elif WHICH_ROBOT == 1:
    # LRPB
    L_MOTOR = OUTPUT_D
    R_MOTOR = OUTPUT_A
    GYRO_PORT = INPUT_4
    IS_INVERTED = False
    DEGREES_PER_INCH = 53.0
elif WHICH_ROBOT == 2:
    # NEW ROBOT
    L_MOTOR = OUTPUT_A
    R_MOTOR = OUTPUT_D
    GYRO_PORT = INPUT_4
    IS_INVERTED = False
    DEGREES_PER_INCH = 53.0
    WHEEL_DIAMETER = 6.88
    M_MOTOR = OUTPUT_B
    S_MOTOR = OUTPUT_C
else:
    L_MOTOR = OUTPUT_A
    R_MOTOR = OUTPUT_D
    GYRO_PORT = INPUT_2
    IS_INVERTED = False
    DEGREES_PER_INCH = 43.0 + 1 - 1

lifter = MediumMotor(M_MOTOR)
side_motor = MediumMotor(S_MOTOR)


class WideWheel(Wheel):
    def __init__(self):
        Wheel.__init__(self, 68.8, 36)


tank_drive = MoveTank(L_MOTOR, R_MOTOR)
# 180 at 15 is about 3-5 degrees too much
# tank_diff = MoveDifferential(L_MOTOR, R_MOTOR, WideWheel, 146.75)
# Works great at 180 and speed 15 at 143
# was turning too much on 1/5
# tank_diff = MoveDifferential(L_MOTOR, R_MOTOR, WideWheel, 143)
# was turning too much on 1/6
# tank_diff = MoveDifferential(L_MOTOR, R_MOTOR, WideWheel, 141.5)
tank_diff = MoveDifferential(L_MOTOR, R_MOTOR, WideWheel, 141)


def show(text):
    disp.reset_console()
    disp.text_at(text, 1, 1)


# test line follow edge
def follow_line(speed=10, want_rli=65, edge="right"):
    m1 = LargeMotor(L_MOTOR)
    m2 = LargeMotor(R_MOTOR)
    # drive forever
    while True:
        # right color sensor
        rli = color_left.reflected_light_intensity
        t = "rli: {}".format(rli)
        show(t)
        diff = rli - want_rli
        # light difference is +/- 40 but
        # motor difference should be small.
        # divide by 20
        factor = 0.05
        bias = diff * factor
        if edge == "left":
            bias = -bias
        m1.on(speed - bias)
        m2.on(speed + bias)
        time.sleep(0.01)


# Timothy
# updated follow_line to stop after dist
# use right color sensor
def follow_line_inches(dist=1, speed=10, want_rli=65, edge="right"):
    m1 = LargeMotor(L_MOTOR)
    m2 = LargeMotor(R_MOTOR)

    rotations_total = inches_to_rotations(dist)

    r1_start = m1.rotations
    r2_start = m2.rotations
    r1_rotations = 0
    r2_rotations = 0

    rotations_traveled = 0
    while rotations_traveled < rotations_total:
        # right color sensor
        rli = color_right.reflected_light_intensity
        t = "rli: {}\n{}\n{}".format(rli, rotations_traveled, rotations_total)
        show(t)
        diff = rli - want_rli
        # light difference is +/- 40 but
        # motor difference should be small.
        # divide by 40
        factor = 0.025
        bias = diff * factor
        if edge == "left":
            bias = -bias
        m1.on(speed - bias)
        m2.on(speed + bias)
        time.sleep(0.01)

        r1_rotations = m1.rotations - r1_start
        r2_rotations = m2.rotations - r2_start

        # average
        rotations_traveled = (r1_rotations + r2_rotations) / 2.0
        log.info("rli: {} {} {}".format(
            rli, rotations_traveled, rotations_total))

    # stop
    m1.off()
    m2.off()

    log.info("Drove (line) rot1={} rot2={}".format(r1_rotations, r2_rotations))

# Spencer
# like follow_line_inches but uses gyro
# Everett: added factor as parameter
# 5 was crazy. 1.5 didn't work.
def drive_inches_gyro(dist, speed=20, factor=1):
    # wheel power +-factor for each degree off
    m1 = LargeMotor(L_MOTOR)
    m2 = LargeMotor(R_MOTOR)

    rotations_total = inches_to_rotations(dist)

    r1_start = m1.rotations
    r2_start = m2.rotations
    r1_rotations = 0
    r2_rotations = 0

    # subtract amount to get right amount from gyro
    angle_start = gyro.value()

    rotations_traveled = 0
    while rotations_traveled < rotations_total:
        angle = gyro.value()
        t = "angle: {}".format(angle)
        show(t)
        # zero is straight
        diff = angle - angle_start
        # fix if turns the wrong way
        bias = diff * factor
        m1.on(speed - bias)
        m2.on(speed + bias)
        time.sleep(0.01)

        r1_rotations = m1.rotations - r1_start
        r2_rotations = m2.rotations - r2_start

        # average
        rotations_traveled = (r1_rotations + r2_rotations) / 2.0
        log.info("angle: {} {} {}".format(
            angle, rotations_traveled, rotations_total))

    m1.off()
    m2.off()

    log.info("Drove (gyro) rot1={} rot2={}".format(r1_rotations, r2_rotations))


# Needs 2 color sensors!!!
# stop when intensity is less than threshold
# only used to test
def align_black_intensity(speed=10, black_thresh=20):
    m1 = LargeMotor(L_MOTOR)
    m2 = LargeMotor(R_MOTOR)
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


# drive until both color sensors see black or white
def align_color(c_name, speed=10):
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    m1.on(speed)
    m2.on(speed)
    left_running = True
    right_running = True
    while left_running or right_running:
        name_l = color_left.color_name
        name_r = color_right.color_name
        t = "{} {}".format(name_l, name_r)
        show(t)
        if name_l == c_name:
            m1.off()
            left_running = False
        if name_r == c_name:
            m2.off()
            right_running = False


# Don't stop on color2 until at least one
# sensor found color1
def align_color_via_color(color2, color1, speed=10):
    m1 = LargeMotor(OUTPUT_A)
    m2 = LargeMotor(OUTPUT_D)
    m1.on(speed)
    m2.on(speed)
    left_found_color1 = False
    right_found_color1 = False
    left_running = True
    right_running = True
    while left_running or right_running:
        name_l = color_left.color_name
        name_r = color_right.color_name
        t = "{} {}".format(name_l, name_r)
        show(t)
        if left_found_color1 or right_found_color1:
            if name_l == color2:
                m1.off()
                left_running = False
        else:
            if name_l == color1:
                left_found_color1 = True
        if right_found_color1 or left_found_color1:
            if name_r == color2:
                m2.off()
                right_running = False
        else:
            if name_r == color1:
                right_found_color1 = True


def align_accurate(speed=10, num_passes=2, white_first=True):
    if white_first:
        align_color_via_color("Black", "White", speed)
    else:
        align_color("Black")
    for i in range(num_passes - 1):
        align_color("White", -speed / 2)
        align_color("Black", speed / 2)


# Testing
# drive to a black line
def align_forever(gyro=None):
    while True:
        s.beep()
        align_color("Black", speed=10)
        s.beep()
        while not b.any():
            rli_left = color_left.reflected_light_intensity
            rli_right = color_right.reflected_light_intensity
            t = "rli: {} {}".format(rli_left, rli_right)
            show(t)
            time.sleep(0.1)


# try running a motor
def motor_test1():
    m = LargeMotor(OUTPUT_D)
    m.on_for_rotations(SpeedPercent(75), 5)
    # hoi bois!!!!


# gyro: gyro sensor
# deg: degrees to turn (positive is clockwise)
# speed: motor speed 0-100 (must be positive)
def turn_degrees_gyro(gyro, deg, speed=20):
    #gyro.mode = GyroSensor.MODE_GYRO_ANG

    if deg > 0:
        lspeed = speed
    else:
        lspeed = -speed
    rspeed = -lspeed

    # reset crashes!!
    #time.sleep(0.5)
    #gyro.reset()

    start = gyro.value()
    log.info("wheel speed: {} {}".format(lspeed, rspeed))
    log.info("gyro inital value: {}".format(start))
    log.info("gyro in mode: {}".format(gyro.mode))
    # log.info("gyro inital value after mode set: {}".format(gyro.value))

    tank_drive.on(lspeed, rspeed)
    log.info("turning {}: {} {}".format(deg, lspeed, rspeed))

    # last part turn slow
    slow_degrees = 10
    turn_guess = abs(deg) - slow_degrees

    if turn_guess > 0:
         gyro.wait_until_angle_changed_by(turn_guess)

    # finish slow
    slow_speed = 5
    if lspeed > 0:
        lspeed = slow_speed
        rspeed = -slow_speed
    else:
        lspeed = -slow_speed
        rspeed = slow_speed
    log.info("wheel speed: {} {}".format(lspeed, rspeed))
    tank_drive.on(lspeed, rspeed)
    #log.info("gyro intermediate value: {}".format(gyro.value()))

    # code from gyro.wait_until_angle_changed_by
    delta = deg
    if delta > 0:
        while (gyro.value() - start) < delta:
            time.sleep(0.01)
    else:
        delta *= -1
        while (start - gyro.value()) < delta:
            time.sleep(0.01)

    tank_drive.off()
    time.sleep(0.5)

    final = gyro.value()

    log.info("gyro final value: {}".format(final))
    log.info("angle: {} actual: {}".format(deg, final - start))
    log.info("stopping")


def my_turn_left(speed=20, angle=90):
    if TURN_WITH_GYRO:
        turn_degrees_gyro(gyro, -angle, speed)
    else:
        tank_diff.turn_left(speed, angle)


def my_turn_right(speed=20, angle=90):
    if TURN_WITH_GYRO:
        turn_degrees_gyro(gyro, angle, speed)
    else:
        tank_diff.turn_right(speed, angle)


def turn_degrees(gyro, deg, speed=15):
    if deg > 0:
        my_turn_right(speed, deg)
    else:
        my_turn_left(speed, -deg)


def inches_to_mill(inches):
    return 25.4 * inches


def inches_to_rotations(distance):
    # wheel diameter is 43.2mm (?), 68.8 for larger
    circum_inches = WHEEL_DIAMETER * 3.14 / 2.54
    rotations = distance / circum_inches

    return rotations


def drive_inches(distance, speed=20):
    rotations = inches_to_rotations(distance)
    speed = -speed if IS_INVERTED else speed
    # tank_drive.on_for_rotations(SpeedPercent(speed), SpeedPercent(speed), rotations)
    tank_diff.on_for_distance(SpeedPercent(
        speed), inches_to_mill(distance))


def mission_2_crane(gyro):
    drive_inches(12)
    # 6 inches out 6 inches over
    turn_degrees(gyro, 45)
    drive_inches(7*1.414)
    turn_degrees(gyro, -45)
    drive_inches(9)
    drive_inches(6, -20)
    turn_degrees(gyro, -45)
    drive_inches(3*1.414)
    turn_degrees(gyro, 45)
    drive_inches(4)


def mission_white_blocks(gyro):

    drive_inches(.85, speed=12)
    # turn_degrees(gyro, -72, 15)
    my_turn_right(15, 69.25)
    drive_inches(61, 30)
    # LIFTER.on_for_rotations(10, 1)
    drive_inches(-4, 30)
    # LIFTER.on_for_rotations(10, -1, False)

    # turn_degrees(gyro, 180, 7)

    # turn_degrees(gyro, -85, 35)

    # turn_degrees(gyro, -85, 35)

    # drive_inches(26)


# Program 1
def big_O_by_crane(gyro):

    # This is driving to the crane and doing two missions: M02 and M12.

    # Drives out quickly and then slower for more accuracy.
    drive_inches_gyro(dist=25, speed=20, factor=1)
    drive_inches_gyro(1.5, 10)
    time.sleep(1.0)
    drive_inches(-2, 20)

    # Raises side holder, or medium motor.
    side_motor.on_for_degrees(speed=20, degrees=109)

    # Attempts to not crash into the load.
    my_turn_left(20, 10)
    drive_inches(-2, 20)
    my_turn_right(20, 10)

    # Returns to the home base.
    drive_inches(-12.5, 40)
    my_turn_right(20, 90)
    drive_inches(-20, 40)
    drive_inches(2, 40)
    my_turn_left(40, 80)


def circle_straight(gyro):
    drive_inches(16, 30)
    lifter.on_for_rotations(50, 1)
    drive_inches(-8, 30)
    my_turn_right(15, 45)
    drive_inches(-19, 30)
    drive_inches(1, 30)
    my_turn_left(15, 90)
    s.beep()
    s.beep()
    s.beep()
    s.beep()


# use the color sensor to drive down the board
def drive_out_black_line_with_cs():
    # extend lowwer bar two studs and add lift after placement of tan blocks, lower it again and lift after earthquake
    # Use gyro cause weight turns us
    gyro_zero = gyro.value()
    drive_inches_gyro(5, 15)
    # lots of weight on left so read how much turned
    time.sleep(0.01)
    gyro_start = gyro.value()
    turned = gyro_start - gyro_zero
    log.info("Initial gyro angle: {}".format(turned))
    # undo the turn and an extra degree to not miss the line
    extra_turn = -turned + 1
    my_turn_right(15, 90 + extra_turn)

    # first get to line to follow
    edge = "right"
    drive_inches_gyro(11.7, 30)

    # 20.5 along line
    # drive a little lift then drive the rest
    follow_line_inches(dist=18, speed=30, edge=edge, want_rli=32)
    # lift
    side_motor.on_for_degrees(speed=20, degrees=109)
    follow_line_inches(dist=2.5, speed=30, edge=edge, want_rli=32)

    # a little more where it bends
    drive_inches(3, 20)


def mission_tan_blocks_plus(gyro):

    # This mission does these seven missions: M01, M07, M08, M09, M11, M12 and M13.

    # Drives out for a good start, then folows the black line to the red circle.
    # Leaves red blocks and train(M11) in circle.
    drive_out_black_line_with_cs()

    # Get pointed toward the black line
    # Back hits ramp if turn all at once
    my_turn_left(speed=15, angle=20)
    drive_inches(3, 20)
    my_turn_left(speed=15, angle=10)
    align_accurate(10, num_passes=3)

    # how far tan circle is from black line
    tan_dist = 10
    drive_inches(tan_dist, 30)
    my_turn_left(20, 90)
    # get closer to black line
    drive_inches(2, 20)
    # Stop at the line and goes a little bit further.
    align_accurate(10, num_passes=3)
    # push to tan circle and come back a little less for elevator
    tan_push = 5.35
    drive_inches(tan_push, 20)
    drive_inches(-tan_push + 0.5, 20)

    # M08_Elevator
    my_turn_right(20, 90)
    # flip the elevator
    # already came tan_dist from the black line
    drive_inches(19 - tan_dist, 40)
    # lift bar over elevator and back up
    lifter.on_for_rotations(50, 1, brake=False)
    drive_inches(-10, speed=20)
    lifter.on_for_rotations(-50, 1, brake=False)

    # M09_safety factor
    turn_extra = 12
    my_turn_right(15, 30 + turn_extra)
    lifter.on_for_degrees(50, 120, brake=False)
    drive_inches(6, 20)
    # swing front pointer 20 left, (20 right to straighten then) 20 right
    swing_turn = 20
    swing_first = 5
    my_turn_left(15, turn_extra + swing_first)
    # push building support down
    lifter.on_for_degrees(-50, 90, brake=False)
    building_drive_more = 0.25
    drive_inches(building_drive_more, 10)
    my_turn_left(15, swing_turn - swing_first)
    drive_inches(-1 - building_drive_more, 20)

    # M07 Swing
    # point towards the swing
    my_turn_right(20, swing_turn + 90 - 2)
    # lift arm to push swing
    lifter.on_for_degrees(50, 90, brake=False)
    # drive to swing
    drive_inches(4, 30)
    # push swing by turning a little
    my_turn_left(20, 30)
    my_turn_right(20, 15)

    # backup 6 inches for safe turn
    drive_inches(-6, 30)
    lifter.on_for_degrees(50, 600, brake=False)

    # M01 Elevated Places
    # align on line perp to ramp line
    # watch out for the lettering over white
    my_turn_left(speed=20, angle=185)
    align_accurate(10, num_passes=3, white_first=False)
    # get color sensors away from black line
    drive_inches(1.5, 20)
    # get to ramp line
    my_turn_left(10, 90)
    drive_inches(4, 20)
    align_accurate(10, num_passes=2)
    drive_inches(1.75, 15)
    my_turn_right(10, 90)
    drive_inches(10, 15)

    # Lower lift for less tippyness.
    lifter.on_for_degrees(50, -300, brake=False)
    # go up ramp

    # The Big Ending
    align_accurate(10, num_passes=2)
    drive_inches(-2, 20)
    # drive with gyro cause slides on ramp
    drive_inches_gyro(46, 40)
    s.set_volume(100)
    play_sound = 0
    while play_sound < 3:
        s.play_file('sound/crazy-mono.wav')
        play_sound = play_sound + 1

    # TODO Lock The Motors


def mission_red_blocks(gyro):
    """
    Setup:
    Line up robot from wall to the 5th from the right hashmark.
    Line up blocks so it looks like a rectangle.
    Two pieces of LEGO block will be sticking up on oppisite ends.
    Place an upgrade on the furthest LEGO block sticking up.
    Make sure that the blocks are lined up on the left side of the attachment.
    Your good to go!
    """

    drive_inches(9, 20)
    my_turn_right(15, 85)
    drive_inches(22.5, 30)
    drive_inches(-37, 80)
    my_turn_left(80, 80)


done = False
wait_for = None
choice = 0
choice_incr = 1


def turn_test(gyro):
    turn_ang = 180
    my_turn_right(15, turn_ang)
    time.sleep(5)
    my_turn_left(15, turn_ang)


def gyro_reset(gyro):
    gyro.mode = GyroSensor.MODE_GYRO_ANG
    gyro.reset()
    time.sleep(GYRO_RESET_WAIT)

def drop_frame(gyro):
    """
    Lowers side motor in between missions
    """

    side_motor.on_for_degrees(speed=20, degrees=-109)


progs = [

    ("m: Big O Crane", big_O_by_crane),
    ("m: drop frame", drop_frame),
    ("m: tan blocks", mission_tan_blocks_plus),
    ("m: circle_straight", circle_straight),
    # ("m: ReD EnDiNg", red_ending),
    ("gyro_reset", gyro_reset),
    # ("m: turn_test", turn_test),
    # ("m: white blocks", mission_white_blocks),
    # ("m: red blocks", mission_red_blocks),
    # ("m: 2 crane", mission_2_crane),
]


# called when a button is pushed
def change(changed_buttons):
    global done
    global choice
    # changed_buttons is a list of
    # tuples of changed button names and their states.
    if len(changed_buttons) == 0:
        return False
    logging.info('These buttons changed state: ' + str(changed_buttons))
    if wait_for is not None and wait_for in changed_buttons:
        logging.info('You pressed the done button')
        done = True
    else:
        done = False
    if "up" in changed_buttons:
        choice -= 1
        if choice < 0:
            choice = len(progs) - 1
        s.beep()
    elif "down" in changed_buttons:
        choice += 1
        if choice >= len(progs):
            choice = 0
        s.beep()
    elif "left" in changed_buttons:
        lifter.on_for_degrees(-100, 15, brake=True)
    elif "right" in changed_buttons:
        lifter.on_for_degrees(100, 15, brake=True)
    logging.info('Done is: ' + str(done))
    return done


def run_program(gyro):
    # This loop checks button states
    # continuously and calls appropriate event handlers
    global done
    global wait_for
    global choice
    done = False
    wait_for = "enter"
    logging.info("Waiting for enter button.")
    logging.info(sys.implementation.name == "micropython")
    ang = 0
    count = 0
    while not done:
        # reading gyro fast can break
        if count % 20 == 0:
            ang = gyro.angle
            count = 0
        rli_left = color_left.reflected_light_intensity
        rli_right = color_right.reflected_light_intensity
        # t = "rli: {} {}".format(rli_left, rli_right)
        t = "rli: {} {}\nP: {}\nA: {}\nWaiting for l button".format(
            rli_left, rli_right, progs[choice][0], ang)
        show(t)
        done = change(b.buttons_pressed)
        time.sleep(0.05)
        count += 1

    logging.info("And done.")
    logging.info("Running {}".format(progs[choice][0]))

    progs[choice][1](gyro)
    s.beep()

    choice = choice + choice_incr


if __name__ == "__main__":
    # Resets to 0, does not fix drift
    # broken and gyro might start at 23
    time.sleep(1)
    gyro.mode = GyroSensor.MODE_GYRO_ANG
    gyro.reset()
    time.sleep(GYRO_RESET_WAIT)
    logging.info("Starting angle: {}".format(gyro.angle))
    b.on_change = change
    s.beep()
    s.beep()
    s.beep()
    while True:
        run_program(gyro)
