from cflib.utils.power_switch import PowerSwitch

uri = 'radio://0/80/2M'

print('toggling power...')
power_switch = PowerSwitch(uri)
power_switch.stm_power_cycle()
print('toggling power done')
