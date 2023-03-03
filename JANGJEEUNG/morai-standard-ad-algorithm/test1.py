from autonomous_driving import AutonomousDriving
from vehicle_state import VehicleState

ad = AutonomousDriving()

_input = ad.execute(VehicleState(0,0,0,0),[],[])

print("_input", _input)