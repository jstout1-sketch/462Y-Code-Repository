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
    def __init__(self, name, kP: float, kI: float, kD: float, integral_error_limit: float) -> None:
        self.Kp = kP
        self.Ki = kI
        self.Kd = kD
        self.name = name
        self.integral_error_limit = integral_error_limit
        self.error = 0
        self.previous_error = 0
        self.integral = 0

    def loop_instance(self, current_value: float, target_value: float) -> float:
        self.error = target_value - current_value
        self.integral = self.integral + self.error
        if self.error == 0:
            self.integral = 0
        if abs(self.error) > self.integral_error_limit or abs(self.error) < 0.05:
            self.integral = 0
        derivative = self.error - self.previous_error
        self.previous_error = self.error
        drive_speed = self.error * self.Kp + self.integral * self.Ki + derivative * self.Kd
        print(self.name + ": " + str(self.error))
        return drive_speed

brain = Brain()
controller = Controller()

ratio = GearSetting.RATIO_6_1
speed = 24
auton_speed = 0.0625
auton__turn_speed = 0.5
intake_speed = 1000
intake_lock = False
descore_state = False
matchload_state = False
target_heading = 0
left_drive_PID = PID("LEFT", 0.03, 0.0016, 0.125, 20)
right_drive_PID = PID("RIGHT", 0.025, 0.0016, 0.125, 20)
heading_PID = PID("HEADING", 0, 0, 0, 25)
under_80_turn_PID = PID("TURN", 0.156, 0.0005, 0, 2)
over_100_turn_PID = PID("TURN", 0.11, 0, 0.05, 2)
middle_turn_PID = PID("TURN", 0.117, 0, 0.06, 2)

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
matchload = Pneumatics(brain.three_wire_port.d)

drivetrain = DriveTrain(left_drivetrain_motors, right_drivetrain_motors, 300, 320, 320, MM, 3/5)

# Start calibration.
brain_inertial.calibrate()
# Print that the Inertial Sensor is calibrating while
# waiting for it to finish calibrating.
while brain_inertial.is_calibrating():
    brain.screen.clear_screen()
    brain.screen.print("Inertial Sensor Calibrating")
    wait(50, MSEC)

brain_inertial.reset_heading()

def toggle_descore():
    print("descore toggle attempt")
    global descore_state
    descore_state = not descore_state
    if descore_state:
        descore.open()
        print("descore toggled on")
    else:
        descore.close()
        print("descore toggled off")

def toggle_matchloader_auton(): 
    global matchload_state
    matchload_state = not matchload_state
    if matchload_state:
        matchload.open()
    else:
        matchload.close()

def toggle_matchloader():
    global matchload_state
    matchload_state = not matchload_state
    if matchload_state:
        matchload.open()
    else:
        matchload.close()

def drive_distance(distance):
    left_motor_front.reset_position()
    right_motor_front.reset_position()
    left_drive_PID.error = distance
    right_drive_PID.error = distance
    left_drive_PID.previous_error = left_drive_PID.error
    right_drive_PID.previous_error = right_drive_PID.error
    counter = 0
    time_counter = 0
    distanceInDeg = (distance / (3.25 * math.pi)) * (5.0/3.0) * 360 #dist (in) *    1 rev /  (diam * pi) in * (5 / 3) * 360 deg / 1rv
    while counter < 150 and time_counter < 2000:
        #distance_traveled = (left_drivetrain_motors.position() * 3.0/5.0) * 10.2101761242/360.0
        lmp = left_motor_front.position()
        rmp = right_motor_front.position()
        left_drive_speed = left_drive_PID.loop_instance(lmp, distanceInDeg) + heading_PID.loop_instance(brain_inertial.rotation(), target_heading)
        right_drive_speed = right_drive_PID.loop_instance(rmp, distanceInDeg) - heading_PID.loop_instance(brain_inertial.rotation(), target_heading)
        left_motor_back.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_top.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_front.spin(FORWARD, left_drive_speed, VOLT)
        right_motor_back.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_top.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_front.spin(FORWARD, right_drive_speed, VOLT)
        if abs(left_drive_PID.error) <= 10 and abs(right_drive_PID.error) <= 10:
            counter += 10
        else:
            counter = 0
        time_counter += 10
        wait(10, MSEC)
        #print(drive_PID.error)
    drivetrain.stop(BRAKE)

def turn_under_80_degrees(measure):
    under_80_turn_PID.error = measure
    under_80_turn_PID.previous_error = under_80_turn_PID.error
    counter = 0
    time_counter = 0
    while counter < 150 and time_counter < 2000:
        output = under_80_turn_PID.loop_instance(brain_inertial.rotation(), measure)
        l_output_clamped = 5 if output > 5 else output
        l_output_clamped = -5 if l_output_clamped < -5 else l_output_clamped
        left_drive_speed =  l_output_clamped
        right_drive_speed = -l_output_clamped
        left_motor_back.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_top.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_front.spin(FORWARD, left_drive_speed, VOLT)
        right_motor_back.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_top.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_front.spin(FORWARD, right_drive_speed, VOLT)
        if abs(under_80_turn_PID.error) <= 2:
            counter += 10
        else:
            counter = 0
        time_counter += 10
        wait(10, MSEC)
    drivetrain.stop()

def turn_over_80_degrees(measure):
    middle_turn_PID.error = measure
    middle_turn_PID.previous_error = middle_turn_PID.error
    counter = 0
    time_counter = 0
    while counter < 150 and time_counter < 2000:
        output = middle_turn_PID.loop_instance(brain_inertial.rotation(), measure)
        l_output_clamped = 8 if output > 8 else output
        l_output_clamped = -8 if l_output_clamped < -8 else l_output_clamped
        left_drive_speed =  l_output_clamped
        right_drive_speed = -l_output_clamped
        left_motor_back.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_top.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_front.spin(FORWARD, left_drive_speed, VOLT)
        right_motor_back.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_top.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_front.spin(FORWARD, right_drive_speed, VOLT)
        if abs(middle_turn_PID.error) <= 2:
            counter += 10
        else:
            counter = 0
        time_counter += 10
        wait(10, MSEC)
    drivetrain.stop()

def turn_over_135_degrees(measure):
    over_100_turn_PID.error = measure
    over_100_turn_PID.previous_error = under_80_turn_PID.error
    counter = 0
    time_counter = 0
    while counter < 150 and time_counter < 2000:
        output = over_100_turn_PID.loop_instance(brain_inertial.rotation(), measure)
        l_output_clamped = 8 if output > 8 else output
        l_output_clamped = -8 if l_output_clamped < -8 else l_output_clamped
        left_drive_speed =  l_output_clamped
        right_drive_speed = -l_output_clamped
        left_motor_back.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_top.spin(FORWARD, left_drive_speed, VOLT)
        left_motor_front.spin(FORWARD, left_drive_speed, VOLT)
        right_motor_back.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_top.spin(FORWARD, right_drive_speed, VOLT)
        right_motor_front.spin(FORWARD, right_drive_speed, VOLT)
        if abs(over_100_turn_PID.error) <= 2:
            counter += 10
        else:
            counter = 0
        time_counter += 10
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
    # turn_over_80_degrees(90)
    turn_under_80_degrees(-25)
    drive_distance(20)
    drive_distance(8)
    turn_over_80_degrees(-135)
    drive_distance(-13)
    score_motor.spin(FORWARD, 120)
    wait(2, SECONDS)
    score_motor.stop()
    drive_distance(47)
    turn_under_80_degrees(-180)
    toggle_matchloader_auton()
    wait(0.25, SECONDS)
    drive_distance(12)
    wait(0.5, SECONDS)
    drive_distance(-25)
    score_motor.spin(FORWARD, 120)

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("driver control")
    while True:
        wait(20, MSEC)
        left_motor_back.spin(FORWARD, controller.axis3.position() * 0.12, VOLT)
        left_motor_top.spin(FORWARD, controller.axis3.position() * 0.12, VOLT)
        left_motor_front.spin(FORWARD, controller.axis3.position() * 0.12, VOLT)
        right_motor_back.spin(FORWARD, controller.axis2.position() * 0.12, VOLT)
        right_motor_top.spin(FORWARD, controller.axis2.position() * 0.12, VOLT)
        right_motor_front.spin(FORWARD, controller.axis2.position() * 0.12, VOLT)
        
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
        
        
controller.buttonY.pressed(toggle_descore)
controller.buttonRight.pressed(toggle_matchloader)
controller.buttonDown.pressed(toggle_intake_lock)
controller.buttonUp.pressed(unjam)
# create competition instance
comp = Competition(user_control, autonomous)

# actions to do when the program starts
brain.screen.clear_screen()
