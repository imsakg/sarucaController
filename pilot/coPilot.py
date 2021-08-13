from pilot import mrPilot
from control import spiralScanning
import time

movements = spiralScanning.spiralMovements()
movements.generate(2)

pilot = mrPilot.Pilot()
pilot.launch()
pilot.changeMode("MANUAL")
pilot.armForce()


def speed2pwm(speed):
    if speed > 0:
        pwm = 1500 + speed * 10
        return int(pwm)
    else:
        pwm = 1500 + speed * 10
        return int(pwm)


def goT(channel, pwm, duration):
    t1 = time.time()
    while time.time() - t1 < duration:
        pilot.channelOverRide(channel, pwm)
        time.sleep(0.1)
    pilot.channelOverRide(channel, 1500)


def gotoTest(north, east, down):
    vehicle = pilot.goto_position_target_local_ned(north, east, down)


def rotate(value):
    pilot.set_target_attitude(0, 0, value)


def setDepth(depth):
    pilot.changeMode("ALT_HOLD")
    pilot.vehicle.location.global_relative_frame.alt = depth


def dive(depth, tolorance=0.7, diveSpeed=-15):
    pilot.changeMode("ALT_HOLD")
    while pilot.vehicle.location.global_relative_frame.alt >= tolorance * depth:
        # print(pilot.vehicle.location.global_relative_frame.alt)
        pwm = speed2pwm(diveSpeed)
        pilot.channelOverRide(3, pwm)
    pilot.channelOverRide(3, 1500)


def spin(rotation="left", speed=30):
    roll_angle = pitch_angle = 0
    if rotation == "left":
        # spin the other way with 3x larger steps
        for yaw_angle in range(500, 0, -speed):
            pilot.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
            time.sleep(1)
    elif rotation == "right":
        for yaw_angle in range(0, 500, speed):
            pilot.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
            time.sleep(1)  # wait for a second
    else:
        raise Exception("Unknown rotation")


def roll(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=6, pwm=pwm, duration=duration)


def throotle(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def yaw(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=4, pwm=pwm, duration=duration)


def pitch(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=5, pwm=pwm, duration=duration)


def transform(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=2, pwm=pwm, duration=duration)


def forward(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=5, pwm=pwm, duration=duration)


def back(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def right(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def left(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def down(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def up(speed=10, duration=1):
    pwm = speed2pwm(speed)
    goT(channel=3, pwm=pwm, duration=duration)


def testDrone():
    pilot.changeMode("GUIDED")
    pilot.arm()
    pilot.takeoff(20)

    while True:
        for x, y in movements:
            currentLocation, targetLocation, targetDistance = pilot.goto(x, y)
            while pilot.vehicle.mode.name == "GUIDED":
                remainingDistance = pilot.get_distance_metres(
                    pilot.vehicle.location.global_relative_frame, targetLocation
                )
                print("Distance to target: ", remainingDistance, targetDistance * 0.2)
                if remainingDistance <= targetDistance * 0.2:
                    print("Reached target")
                    break
                time.sleep(1)
