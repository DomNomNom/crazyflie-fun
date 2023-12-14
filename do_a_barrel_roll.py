import traceback

import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

from scipy.spatial.transform import Rotation


uri = 'radio://0/80/2M'

def take_off(scf):
    print('takeoff...')

    good_supervisor = False
    for i in range(4):

        log_config = LogConfig(name='starting state', period_in_ms=500)
        log_config.add_variable('stateEstimate.x', 'float')
        log_config.add_variable('stateEstimate.y', 'float')
        log_config.add_variable('stateEstimate.z', 'float')
        log_config.add_variable('stateEstimate.yaw', 'float')
        log_config.add_variable('supervisor.info', 'uint16_t')
        cf.param.set_value('stabilizer.controller', '2')


        with SyncLogger(scf, log_config) as logger:
            for log_entry in logger:
                data = log_entry[1]
                break

        # reset lockout state
        print('supervisor:', data['supervisor.info']);
        if data['supervisor.info'] == 68:
            print('locked out.')
            print('rebooting....')
            from cflib.utils.power_switch import PowerSwitch
            power_switch = PowerSwitch(uri)
            power_switch.stm_power_cycle()
            time.sleep(10);
            print('rebooting done')
            continue
        good_supervisor = True
        break

    if not good_supervisor:
        raise Exception("couldn't get a good supervisor reading")

    HOVER_HEIGHT = 0.5
    # hover about starting location with no yaw change
    setpoint = (
        data['stateEstimate.x'],
        data['stateEstimate.y'],
        data['stateEstimate.z'] + HOVER_HEIGHT,
        data['stateEstimate.yaw']
    )
    scf.cf.commander.send_position_setpoint(*setpoint)
    time.sleep(1.5)

    # yaw = 0
    setpoint = setpoint[:3] + (0,)
    scf.cf.commander.send_position_setpoint(*setpoint)
    time.sleep(.5)
    print('takeoff done')

    return setpoint

def land(scf):
    print('landing...')
    cf = scf.cf

    HOVER_HEIGHT = 0.2
    for _ in range(30):
        cf.commander.send_position_setpoint(0, 0, HOVER_HEIGHT, 0)
        time.sleep(.1)

    cf.commander.send_stop_setpoint()

    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    print('landing done')



def send_full_setpoint(cf, duration, pos, vel, acc, orientation, rollrate, pitchrate, yawrate):
    # Set points must be sent continuously to the Crazyflie, if not it will think that connection is lost
    end_time = time.time() + duration
    while time.time() < end_time:
        cf.commander.send_full_state_setpoint(pos, vel, acc, orientation, rollrate, pitchrate, yawrate)
        time.sleep(0.1)

def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion

    Args:
        roll (float): roll, in radians
        pitch (float): pitch, in radians
        yaw (float): yaw, in radians

    Returns:
        array: the quaternion [x, y, z, w]
    """
    return Rotation.from_euler(seq='xyz', angles=(roll, pitch, yaw), degrees=False).as_quat()



if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # print('fake mode.')
    # grab_control(FakeSCF(), (0,0,0,0))

    print('connecting to drone..')
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('connected')


        try:
            cf = scf.cf
            initial_setpoint = take_off(scf)

            # go high for flip
            for _ in range(30):
                cf.commander.send_position_setpoint(0, 0, 1, 0)
                time.sleep(.1)

            # # flip
            # send_full_setpoint(cf, 2.0,
            #               [0.0, 0.0, 1.0],
            #               [0.0, 0.0, 0.0],
            #               [0.0, 0.0, 0.0],
            #               quaternion_from_euler(0.0, 0.0, 0.0),
            #               0.0, 0.0, 0.0)

            MAX_TURN_RATE = 32.76
            send_full_setpoint(cf, 0.2,
                          [0.0, 0.0, 1.0],
                          [0.0, 0.0, 0.0],
                          [0.0, 0.0, 0.0],
                          quaternion_from_euler(1.0, 0.0, 0.0),
                          0, 0, 0)


            # stabilize high high for flip
            for _ in range(30):
                cf.commander.send_position_setpoint(0, 0, 0.9, 0)
                time.sleep(.1)

            # print('land')
            # send_full_setpoint(cf, 2.0,
            #               [0.0, 0.0, 0.2],
            #               [0.0, 0.0, 0.0],
            #               [0.0, 0.0, 0.0],
            #               quaternion_from_euler(0.0, 0.0, 0.0),
            #               0.0, 0.0, 0.0)


        except KeyboardInterrupt:
            print("Ctrl-C caught once - landing")
            pass
        except Exception:
            traceback.print_exc()

        land(scf)
