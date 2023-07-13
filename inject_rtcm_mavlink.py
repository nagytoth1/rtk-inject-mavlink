from itertools import cycle
from pygnssutils.gnssntripclient import GNSSNTRIPClient
from queue import Queue
from pymavlink import mavutil

import argparse

parser = argparse.ArgumentParser()
parser.add_argument("ip_caster", type=str,
                    help="ip of caster ntrip")
parser.add_argument("port_caster", type=int,
                    help="port of caster ntrip")
parser.add_argument("mountpoint", type=str,
                    help="mountpoint from ntrip caster")

args = parser.parse_args()

SERVER = args.ip_caster
PORT = args.port_caster
MOUNTPOINT = args.mountpoint

the_connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

the_connection.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (the_connection.target_system, the_connection.target_component))

GPS_RTCM_DATA_ID = 233
GPS_RTCM_DATA_SIZE = 180

# STRUCTURE
#uint8_t flags; /*<  LSB: 1 means message is fragmented, next 2 bits are the fragment ID, the remaining 5 bits are used for the sequence ID. Messages are only to be flushed to the GPS when the entire message has been reconstructed on the autopilot. The fragment ID specifies which order the fragments should be assembled into a buffer, while the sequence ID is used to detect a mismatch between different buffers. The buffer is considered fully reconstructed when either all 4 fragments are present, or all the fragments before the first fragment with a non full payload is received. This management is used to ensure that normal GPS operation doesn't corrupt RTCM data, and to recover from a unreliable transport delivery order.*/
#uint8_t len; /*< [bytes] data length*/
#uint8_t data[180]; /*<  RTCM message (may be fragmented)*/

# --------------------------

SERVER = "caster.centipede.fr"
PORT = 2101
MOUNTPOINT = "MOTE1"

buffer = Queue()

ntrip_client = GNSSNTRIPClient(None, verbosity=0)

streaming = ntrip_client.run(
    server=SERVER, 
    mountpoint=MOUNTPOINT,
    output=buffer
)

SEQ_NO = cycle(range(32))
MAX_FRAGMENT_SIZE = 180

while streaming:
    (raw_rtcm, formatted_rtcm) = buffer.get()

    if len(raw_rtcm) > MAX_FRAGMENT_SIZE:
        # fragmented packet
        slices = [
            raw_rtcm[i : (i + MAX_FRAGMENT_SIZE)]
            for i in range(0, len(raw_rtcm), MAX_FRAGMENT_SIZE)
        ]

        if len(slices[-1]) == MAX_FRAGMENT_SIZE:
            # if the last fragment is full, we need to add an extra empty
            # one according to the protocol
            slices.append(b"")

        if len(slices) > 4:
            raise Exception(f"Dropping oversized RTCM packet: {len(raw_rtcm)} bytes")

        seq_no = next(SEQ_NO)

        for fragment_id, packet in enumerate(slices):
            flags = (seq_no << 3) + (fragment_id << 1) + 1

            the_connection.mav.gps_rtcm_data_send(
                flags,
                len(packet),
                packet.ljust(180, b"\x00")
            )
    else:
        # not fragmented packet
        the_connection.mav.gps_rtcm_data_send(
            0,
            len(raw_rtcm),
            raw_rtcm.ljust(180, b"\x00")
        )