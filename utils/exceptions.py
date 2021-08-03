
class VehicleModeErr(Exception):
    def __init__(self, vehicleMode, message='Vehicle is not "GUIDED". '):
        self.vehicleMode = vehicleMode
        self.message = message + f"Please Set it to {vehicleMode}"
        super().__init__(self.message)

    def __str__(self):
        return f'{self.vehicleMode} -> {self.message}'

class VehicleConnectionErr(Exception):
    def __init__(self, vehiclePort, message='Cant connected to Vehicle '):
        self.vehiclePort = vehiclePort
        self.message = message + f"at {vehiclePort} port."
        super().__init__(self.message)

    def __str__(self):
        return f'{self.vehiclePort} -> {self.message}'

class VehicleCameraErr(Exception):
    def __init__(self, cameraStatus, message='Did not connected to camera. Camera status :  '):
        self.cameraStatus = cameraStatus
        self.message = message + f"at {cameraStatus} port."
        super().__init__(self.message)

    def __str__(self):
        return f'{self.cameraStatus} -> {self.message}'

class MissionInitErr(Exception):
    def __init__(self, mission, message='Mission initialiaze error at Mission : '):
        self.mission = mission
        self.message = message + f"{mission}"
        super().__init__(self.message)

    def __str__(self):
        return f'{self.mission} -> {self.message}'


class MissionCompErr(Exception):
    def __init__(self, mission, message='Mission complete error at Mission : '):
        self.mission = mission
        self.message = message + f"{mission}. Mission did not completed."
        super().__init__(self.message)

    def __str__(self):
        return f'{self.mission} -> {self.message}'
