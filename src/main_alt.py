# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       jessestout                                                   #
# 	Created:      11/13/2025, 8:29:14 AM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

class PID:
    def __init__(self, kP: float, kI: float, kD: float, integral_error_limit: float, speed_mult: float) -> None:
        self.Kp = kP
        self.Ki = kI
        self.Kd = kD
        self.integral_error_limit = integral_error_limit
        self.error = 0
        self.previous_error = 0
        self.integral = 0
        self.speed_mult = speed_mult

    def loop_instance(self, current_value: float, target_value: float) -> float:
        self.error = target_value - current_value
        self.integral = self.integral + self.error
        if self.error == 0:
            self.integral = 0
        if abs(self.error) > self.integral_error_limit:
            self.integral = 0
        derivative = self.error - self.previous_error
        self.previous_error = self.error
        drive_speed = (self.error * self.Kp + self.integral * self.Ki + derivative * self.Kd) * self.speed_mult
        print(self.error)
        return drive_speed
    

brain = Brain()
controller = Controller()

ratio = GearSetting.RATIO_6_1
speed = 6
intake_speed = 1000
intake_lock = False
descore_state = False
matchload_state = False
drive_PID = PID(32, 0.0005, 0.0007, 99, 0.25)
turn_PID = PID(3, 0.000005, 0.000001, 91, 1)

right_motor_front = Motor(Ports.PORT18, ratio, False)
right_motor_back = Motor(Ports.PORT20, ratio, False)
right_motor_top = Motor(Ports.PORT19, ratio, True)
right_drivetrain_motors = MotorGroup(right_motor_front, right_motor_back, right_motor_top)

left_motor_front = Motor(Ports.PORT8, ratio, True)
left_motor_back = Motor(Ports.PORT10, ratio, True)
left_motor_top = Motor(Ports.PORT9, ratio, False)
left_drivetrain_motors = MotorGroup(left_motor_front, left_motor_back, left_motor_top)

intake_motor = Motor(Ports.PORT7, ratio, False)
score_motor = Motor(Ports.PORT15, ratio, True)

brain_inertial = Inertial(Ports.PORT17)

descore = Pneumatics(brain.three_wire_port.a)
matchload = Pneumatics(brain.three_wire_port.b)

drivetrain = DriveTrain(left_drivetrain_motors, right_drivetrain_motors, 300, 320, 320, MM, 3/5)

# Start calibration.
brain_inertial.calibrate()
# Print that the Inertial Sensor is calibrating while
# waiting for it to finish calibrating.
while brain_inertial.is_calibrating():
    brain.screen.clear_screen()
    brain.screen.print("Inertial Sensor Calibrating")
    wait(50, MSEC)

def toggle_descore():
    global descore_state
    descore_state = not descore_state
    if descore_state:
        descore.open()
    else:
        descore.close()

def toggle_matchloader():
    global matchload_state
    matchload_state = not matchload_state
    if matchload_state:
        matchload.open()
    else:
        matchload.close()

def drive_distance(distance):
    left_drivetrain_motors.reset_position()
    distance_traveled = 0
    drive_PID.error = distance - distance_traveled
    drive_PID.previous_error = drive_PID.error
    counter = 0
    while counter < 150:
        distance_traveled = (left_drivetrain_motors.position() * 3.0/5.0) * 10.2101761242/360.0
        drivetrain.drive(FORWARD, drive_PID.loop_instance(distance_traveled, distance), RPM)
        if abs(drive_PID.error) <= 1:
            counter += 1
        else:
            counter = 0
        print(counter)
        wait(10, MSEC)
    drivetrain.stop()

def turn_degrees(measure):
    brain_inertial.reset_rotation()
    turn_PID.error = measure - brain_inertial.rotation()
    turn_PID.previous_error = turn_PID.error
    counter = 0
    while counter < 75:
        drivetrain.turn(RIGHT, turn_PID.loop_instance(brain_inertial.rotation(), measure), RPM)
        if abs(turn_PID.error) <= 2:
            counter += 1
        else:
            counter = 0
        print(counter)
        wait(10, MSEC)
    drivetrain.stop()

def toggle_intake_lock():
    global intake_lock
    intake_lock = not intake_lock
    if intake_lock:
        intake_motor.spin(FORWARD, intake_speed)

def unjam():
    for i in range(10):
        score_motor.spin(REVERSE, intake_speed)
        intake_motor.spin(FORWARD, intake_speed)
        wait(0.5, SECONDS)
        intake_motor.spin(REVERSE, intake_speed)
        wait(0.5, SECONDS)

def autonomous():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    intake_motor.spin(FORWARD, intake_speed)
    drive_distance(40)
    turn_degrees(-124)
    drive_distance(-13)
    score_motor.spin(FORWARD, 120)
    wait(2, SECONDS)
    score_motor.stop()
    drive_distance(50)
    turn_degrees(-45)
    toggle_matchloader()
    # score_motor.spin(FORWARD, intake_speed)

def user_control():
    global intake_lock
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    controller.buttonY.pressed(toggle_descore)
    controller.buttonDown.pressed(toggle_matchloader)
    controller.buttonRight.pressed(toggle_intake_lock)
    controller.buttonUp.pressed(unjam)
    while True:
        wait(20, MSEC)
        right_drivetrain_motors.spin(FORWARD, controller.axis3.position() * speed - controller.axis1.position() * speed * 2/3)
        left_drivetrain_motors.spin(FORWARD, controller.axis3.position() * speed + controller.axis1.position() * speed * 2/3)

        # if controller.buttonDown.pressing():
        #     intake_lock = True
        #     intake_motor.spin(FORWARD, intake_speed)
        # elif controller.buttonUp.pressing():
        #     intake_lock = False

        if controller.buttonR2.pressing():
            intake_motor.spin(FORWARD, intake_speed)
        elif controller.buttonL2.pressing():
            intake_motor.spin(REVERSE, intake_speed)
        elif not intake_lock:
            intake_motor.stop()

        if controller.buttonR1.pressing():
            score_motor.spin(FORWARD, intake_speed)
        elif controller.buttonL1.pressing():
            score_motor.spin(REVERSE, intake_speed)
        else:
            score_motor.stop()

        # if abs(controller.axis1.position()) == 100:
        #     controller.rumble("-")
        # if abs(controller.axis3.position()) == 100:
        #     controller.rumble(".")
        

# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()