from itertools import cycle
from queue import Queue
from pygnssutils.gnssntripclient import GNSSNTRIPClient
from pymavlink import mavutil
from socket import gaierror
from dotenv import dotenv_values

config = dotenv_values('.env')
try:
    connection = mavutil.mavlink_connection(config['MAVLINK_CONNECTION_URL'])
except gaierror:
    print('Connection to autopilot failed')
    exit(1)

connection.wait_heartbeat()
print(f"Heartbeat from system (system {connection.target_system} \
      component {connection.target_component})")
# Main loop to listen for messages
while True:
    # msg = connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    msg = connection.recv_match(type='GPS_RAW_INT', blocking=True)
    # Print GPS fix type
    print("GPS Fix Type:", msg.fix_type)