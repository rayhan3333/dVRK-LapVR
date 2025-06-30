#from simulation_manager import SimulationManager
import rospy
from ambf_client import Client
from ambf_msgs.msg import ObjectState, WorldState
import time
import math
import socket
import json

#vars
port = 48000
UNITY_IP = "10.162.34.171"

#ambf setup
_client = Client()
_client.connect()
print(_client.get_obj_names())

psm1 = _client.get_obj_handle('/ambf/env/psm1/baselink')
psm2 = _client.get_obj_handle('/ambf/env/psm2/baselink')
print(psm1.get_num_joints())
print(psm2.get_num_joints())

#IP broadcast Setup
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

msg = json.dumps([1.0, 2.0, 3.0])

#start socket
#recieve
psm1_pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
psm1_pos_sock.bind(("0.0.0.0", port+5))
psm1_pos_sock.settimeout(0.01)
psm2_pos_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
psm2_pos_sock.bind(("0.0.0.0", port+3))
psm2_pos_sock.settimeout(0.01)

#Recieve from Unity
psm1_js_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
psm1_js_sock.bind(("0.0.0.0", port+14))
psm1_js_sock.settimeout(0.01)


psm2_js_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
psm2_js_sock.bind(("0.0.0.0", port+12))
psm2_js_sock.settimeout(0.01)


#send to Unity
psm1_joint_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#psm1_joint_sock.settimeout(0.01)


psm2_joint_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#psm2_joint_sock.settimeout(0.01)


def recieve(sock):
    # data, address = socket.recvfrom(4096)
    
    # jarr = json.loads(data.decode())
    # return jarr
    latest = None
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            latest = json.loads(data.decode())
        except socket.timeout:
            break
    return latest if latest is not None else []
def send(socket, joints, ip, port):
    print("sending: " + str(joints))
    data = {"GetStateJoint": {'Position': joints}}
    jsend = json.dumps(data)
    socket.sendto(jsend.encode('utf-8'), (ip, port))
    #print("data sent")

#Always receive PSM Position Only (not rotation) (RCM) from unity

#IF Unity Gizmo moving - recieve unity PSM Joint Positions

#ELSE send AMBF Joint Positions to Unity
#ALWAYS send AMBF PSM joint states (orientation) to unity


PSM1_JOINT_PORT = port + 4
PSM2_JOINT_PORT = port + 2

#helper function for updating AMBF PSM Joint States
def update_js(obj, js_list):
    for i, js in enumerate(js_list):
        obj.set_joint_pos(i, math.radians(js))

#broadcast IP to Hololens for 30 seconds
for i in range(300):
    sock.sendto(msg.encode(), (UNITY_IP, 47999))
    print("sent ip")
    time.sleep(0.1)

print("entering main loop")
prevjoints1 = [0, 0, 0, 0, 0, 0]
prevjoints2 = [0, 0, 0, 0, 0, 0]
while True:
    

    #SUJ State / PSM RCM Position - Unity always Priority
    try:
        trans1 = recieve(psm1_pos_sock)
        
    except socket.timeout:
        print("socket timeout1")
        trans1 = []
    try:
        trans2 = recieve(psm2_pos_sock) 
    except socket.timeout:
        print("socket timeout2")

        trans2 = []

    if len(trans1) != 0:
        pass
        #print("SUJ1 recieved from Unity" + str(trans1))
        psm1.set_pos(trans1[2], trans1[0], trans1[1])
        psm1.set_rpy(0, 0, -math.radians(trans1[4]))



    if len(trans2) != 0:
        pass
        #print("SUJ4 recieved from Unity" + str(trans2))
        psm2.set_pos(trans2[2], trans2[0], trans2[1])
        psm2.set_rpy(0, 0, -math.radians(trans2[4]))


    #psm1.set_rpy(0, 0, 0)
    #psm2.set_rpy(0, 0, 0)



    #PSM Joint States - Unity sends empty list if not moving (AMBF priority)
    try:
        joints1 = recieve(psm1_js_sock)
        
    except socket.timeout:
        print("socket timeout3")

        joints1 = []
    try:
        joints2 = recieve(psm2_js_sock) 
    except socket.timeout:
        print("socket timeout4")

        joints2 = []

    if len(joints1) != 0:
        print("PSM1 Joints: " + str(joints1))
        update_js(psm1, joints1)
        prevjoints1 = joints1

    else:
        #print("PSM1 Nothing recieved from unity: AMBF Priority")
        #update_js(psm1, prevjoints1)

        pass

    if len(joints2) != 0:
        print("PSM2 Joints: " + str(joints2))
        update_js(psm2, joints2)
        prevjoints2 = joints2

    else:
        #update_js(psm2, prevjoints2)

        pass
        #print("PSM2 Nothing recieved from unity: AMBF Priority")
    #UNITY decides if update or ignore
    send(psm1_joint_sock, psm1.get_all_joint_pos()[0:6], UNITY_IP, PSM1_JOINT_PORT)
    send(psm2_joint_sock, psm2.get_all_joint_pos()[0:6], UNITY_IP, PSM2_JOINT_PORT)

    #send(psm1_joint_sock, [math.degrees(i) for i in psm1.get_all_joint_pos()[0:6]], UNITY_IP, PSM1_JOINT_PORT)
    #send(psm2_joint_sock, [math.degrees(i) for i in psm2.get_all_joint_pos()[0:6]], UNITY_IP, PSM2_JOINT_PORT)


