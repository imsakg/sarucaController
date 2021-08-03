from pilot import mrPilot
from control import spiralScanning
import time
movements = spiralScanning.spiralMovements()
movements.generate(2) 

pilot = mrPilot.Pilot()

pilot.launch()
pilot.changeMode()
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