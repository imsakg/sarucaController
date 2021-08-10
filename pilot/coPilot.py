from pilot import mrPilot
from control import spiralScanning
import time
movements = spiralScanning.spiralMovements()
movements.generate(2) 

pilot = mrPilot.Pilot()

pilot.launch()

def posHoldSpin():
    pilot.changeMode("ALT_HOLD")
    pilot.armForce()
    pilot.set_target_depth(-0.5)
    
    roll_angle = pitch_angle = 0
    for yaw_angle in range(0, 500, 10):
        pilot.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
        time.sleep(1) # wait for a second

    # spin the other way with 3x larger steps
    for yaw_angle in range(500, 0, -30):
        pilot.set_target_attitude(roll_angle, pitch_angle, yaw_angle)
        time.sleep(1)

def testDrone():
    pilot.changeMode("GUIDED")
    pilot.arm()
    pilot.takeoff(20)

    while True:
        for x,y in movements:
            currentLocation,targetLocation, targetDistance = pilot.goto(x,y)
            while pilot.vehicle.mode.name=="GUIDED":
                remainingDistance=pilot.get_distance_metres(pilot.vehicle.location.global_relative_frame, targetLocation)
                print("Distance to target: ", remainingDistance, targetDistance*0.2)
                if remainingDistance<=targetDistance*0.2:
                    print("Reached target")
                    break;
                time.sleep(1)