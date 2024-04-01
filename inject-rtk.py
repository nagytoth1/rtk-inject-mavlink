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
try:
    ntrip_client = GNSSNTRIPClient(None, verbosity=0)
except:
    print('ntrip connection failed')
    exit(2)

buffer = Queue()
STREAMING = ntrip_client.run(server=config['PYGPSCLIENT_HOST'], 
                             mountpoint=config['PYGPSCLIENT_MOUNTPOINT'],
                             user=config['PYGPSCLIENT_USER'],
                             password=config['PYGPSCLIENT_PASSWORD'],
                             output=buffer)
SEQ_NO = cycle(range(32))
MAX_FRAGMENT_SIZE = 180
print('NTRIP RTK correction injected')
while STREAMING:
    (raw_rtcm, formatted_rtcm) = buffer.get()
    n = len(raw_rtcm)
    if n > MAX_FRAGMENT_SIZE:
        # fragmented packet
        slices = [
            raw_rtcm[i: (i + MAX_FRAGMENT_SIZE)]
            for i in range(0, n, MAX_FRAGMENT_SIZE)
        ]

        if len(slices[-1]) == MAX_FRAGMENT_SIZE:
            # if the last fragment is full, we need to add an extra empty
            # one according to the protocol
            slices.append(b"")

        if len(slices) > 4:
            raise Exception(f"Dropping oversized RTCM packet: {n} bytes")

        seq_no = next(SEQ_NO)

        for fragment_id, packet in enumerate(slices):
            flags = (seq_no << 3) + (fragment_id << 1) + 1

            connection.mav.gps_rtcm_data_send(
                flags,
                len(packet),
                packet.ljust(180, b"\x00")
            )
    else:
        # not fragmented packet
        connection.mav.gps_rtcm_data_send(
            0,
            len(raw_rtcm),
            raw_rtcm.ljust(180, b"\x00"))
