# Using the controller trigger it is then possible to 'grab' the Crazyflie
# and to make it move.

import time

import openvr

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from cflib.utils.power_switch import PowerSwitch

# URI to the Crazyflie to connect to
uri = 'radio://0/80/2M'

print('Init VR...')
vr = openvr.init(openvr.VRApplication_Other)
print('Init VR done')


# Find first controller or tracker
controllerId = None
poses = vr.getDeviceToAbsoluteTrackingPose(openvr.TrackingUniverseStanding, 0,
                                           openvr.k_unMaxTrackedDeviceCount)
for i in range(openvr.k_unMaxTrackedDeviceCount):
    if poses[i].bPoseIsValid:
        device_class = vr.getTrackedDeviceClass(i)
        if device_class == openvr.TrackedDeviceClass_Controller or \
           device_class == openvr.TrackedDeviceClass_GenericTracker:
            controllerId = i
            break

print('Found VR controller')


def vector_substract(v0, v1):
    return [v0[0] - v1[0], v0[1] - v1[1], v0[2] - v1[2]]


def vector_add(v0, v1):
    return [v0[0] + v1[0], v0[1] + v1[1], v0[2] + v1[2]]

def pose_to_drone_pos(pose) -> (float, float, float):
    # converts between the space of my OpenVR device to the drone coordinate system
    pos = [pose[0][3], pose[1][3], pose[2][3]]
    return (
        .707*(pos[2] - pos[0]),
        .707*(pos[2] + pos[0]),
        pos[1],
    )

def grab_control(scf, initial_setpoint):
    cf = scf.cf

    poses = vr.getDeviceToAbsoluteTrackingPose(
        openvr.TrackingUniverseStanding, 0, openvr.k_unMaxTrackedDeviceCount)
    controller_pose = poses[controllerId]
    pose = controller_pose.mDeviceToAbsoluteTracking
    setpoint = initial_setpoint[:3]

    grabbed = False
    grab_controller_start = [0, 0, 0]
    grab_setpoint_start = [0, 0, 0]

    while True:
        poses = vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0,
            openvr.k_unMaxTrackedDeviceCount)
        controller_state = vr.getControllerState(controllerId)[1]

        trigger = ((controller_state.ulButtonPressed & 0x200000000) != 0)
        # print(hex(controller_state.ulButtonPressed))

        # Exit when "B" button is pressed
        if (controller_state.ulButtonPressed & 0x2) != 0:
            print("VR exit button pressed.")
            break

        controller_pose = poses[controllerId]
        pose = controller_pose.mDeviceToAbsoluteTracking

        if not grabbed and trigger:
            print('Grab started')
            grab_controller_start = pose_to_drone_pos(pose)
            grab_setpoint_start = setpoint

        if grabbed and not trigger:
            print('Grab ended')

        grabbed = trigger

        if trigger:
            curr = pose_to_drone_pos(pose)
            setpoint = vector_add(grab_setpoint_start,
                                  vector_substract(curr,
                                                   grab_controller_start))

        cf.commander.send_position_setpoint(setpoint[0],
                                            setpoint[1],
                                            setpoint[2],
                                            0)
        time.sleep(0.02)

def wait_for_controller_start_button():
    print("waiting on VR start button...")
    while True:
        poses = vr.getDeviceToAbsoluteTrackingPose(
            openvr.TrackingUniverseStanding, 0,
            openvr.k_unMaxTrackedDeviceCount)
        controller_state = vr.getControllerState(controllerId)[1]

        if (controller_state.ulButtonPressed & 0x4) != 0:
            print('start button pressed')
            return

        time.sleep(0.02)



HOVER_HEIGHT = 0.2

def take_off(scf):
    log_config = LogConfig(name='starting state', period_in_ms=500)
    log_config.add_variable('stateEstimate.x', 'float')
    log_config.add_variable('stateEstimate.y', 'float')
    log_config.add_variable('stateEstimate.z', 'float')
    log_config.add_variable('stateEstimate.yaw', 'float')

    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            break

    # hover about starting location with no yaw change
    setpoint = (
        data['stateEstimate.x'],
        data['stateEstimate.y'],
        data['stateEstimate.z'] + HOVER_HEIGHT,
        data['stateEstimate.yaw']
    )
    scf.cf.commander.send_position_setpoint(*setpoint)
    time.sleep(1)

    # yaw = 0
    setpoint = setpoint[:3] + (0,)
    scf.cf.commander.send_position_setpoint(*setpoint)
    time.sleep(.5)

    return setpoint

def land(scf):
    print('landing...')
    cf = scf.cf

    for _ in range(30):
        cf.commander.send_position_setpoint(0, 0, HOVER_HEIGHT, 0)
        time.sleep(.1)

    cf.commander.send_stop_setpoint()

    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)


class FakeCommander:
    def send_position_setpoint(self, *args, **kwargs):
        # print('position: ', *args)
        pass
class FakeCF:
    commander = FakeCommander()
class FakeSCF:
    cf = FakeCF()

if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # print('fake mode.')
    # grab_control(FakeSCF(), (0,0,0,0))



    print('connecting to drone..')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('connected')

        wait_for_controller_start_button()

        initial_setpoint = take_off(scf)

        try:
            print('starting grab control')
            grab_control(scf, initial_setpoint)
        except KeyboardInterrupt:
            pass

        land(scf)

    openvr.shutdown()
