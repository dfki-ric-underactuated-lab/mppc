import pyvicon_datastream as pv
from pyvicon_datastream import tools
import json
import zmq

# vicon
VICON_TRACKER_IP = "10.250.223.40"
vicon_client = pv.PyViconDatastream()
vicon_client.connect(VICON_TRACKER_IP)
mytracker = tools.ObjectTracker(VICON_TRACKER_IP)

# zmq
context = zmq.Context()
socket = context.socket(zmq.PUSH)
socket.bind("tcp://127.0.0.1:5555")
base = mytracker.get_position('BoomstickBase')[2][0][2:5]
socket.send_json(base)
obs1_ref = mytracker.get_position('ParkourObstacle_1')[2][0][2:]
obs2_ref = mytracker.get_position('ParkourObstacle_2')[2][0][2:]
try:
    while True:
        try:
            obs1 = mytracker.get_position('ParkourObstacle_1')[2][0][2:]
            obs2 = mytracker.get_position('ParkourObstacle_2')[2][0][2:]
            message = json.dumps([obs1, obs2])
            socket.send_json(message)
        except Exception as e:
            print(f"-- vicon failed: {e} --")
except KeyboardInterrupt:
    print("KeyboardInterrupt received. Exiting the loop.")
finally:
    print("trackng session closed")
    socket.close()
    context.term()