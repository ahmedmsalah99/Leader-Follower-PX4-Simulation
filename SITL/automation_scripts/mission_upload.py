#!/usr/bin/env python3
"""Upload a QGroundControl .plan mission to a drone and start it.

Talks MAVLink to the PX4 SITL instance over its offboard link
(UDP 14540 + instance, where instance = MAVLink system id - 1).
"""

import argparse
import json
import math
import os
import sys

from pymavlink import mavutil

TIMEOUT = 5  # seconds to wait for each MAVLink reply
GCS_SYSTEM_ID = 254  # our own system id, must not clash with a drone's
GLOBAL_FRAMES = {
    mavutil.mavlink.MAV_FRAME_GLOBAL,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    mavutil.mavlink.MAV_FRAME_GLOBAL_TERRAIN_ALT_INT,
}


def load_plan(path):
    """Read the mission items of a QGroundControl .plan file"""
    with open(path) as f:
        plan = json.load(f)

    items = []
    for item in plan['mission']['items']:
        if item.get('type') != 'SimpleItem':
            raise ValueError(f"unsupported mission item type '{item.get('type')}' "
                             f'(only simple waypoints/commands are supported)')
        params = [math.nan if p is None else float(p) for p in item['params']]
        items.append({
            'command': item['command'],
            'frame': item['frame'],
            'autocontinue': 1 if item.get('autoContinue', True) else 0,
            'params': params,
        })
    if not items:
        raise ValueError('the plan has no mission items')
    return items


def connect(system_id):
    """Connect to the PX4 instance with the given MAVLink system id"""
    port = 14540 + system_id - 1
    print(f'Connecting to drone with system id {system_id} on UDP port {port}...')
    conn = mavutil.mavlink_connection(f'udpin:0.0.0.0:{port}', source_system=GCS_SYSTEM_ID)
    while True:
        heartbeat = conn.recv_match(type='HEARTBEAT', blocking=True, timeout=10)
        if heartbeat is None:
            raise TimeoutError(f'no heartbeat on UDP port {port}; is the simulation running?')
        if heartbeat.get_srcSystem() == system_id and heartbeat.get_srcComponent() == 1:
            conn.target_system = system_id
            conn.target_component = 1
            return conn


def upload_mission(conn, items):
    """Upload the items with the MAVLink mission protocol"""
    conn.mav.mission_count_send(conn.target_system, conn.target_component, len(items),
                                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
    while True:
        msg = conn.recv_match(type=['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'],
                              blocking=True, timeout=TIMEOUT)
        if msg is None:
            raise TimeoutError('drone stopped answering during mission upload')

        if msg.get_type() == 'MISSION_ACK':
            if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                raise RuntimeError(f'mission rejected by the drone (MAV_MISSION_RESULT {msg.type})')
            return

        seq = msg.seq
        item = items[seq]
        p = item['params']
        if item['frame'] in GLOBAL_FRAMES:
            x, y = int(p[4] * 1e7), int(p[5] * 1e7)
        else:
            x, y = (0 if math.isnan(p[4]) else int(p[4])), (0 if math.isnan(p[5]) else int(p[5]))
        print(f'  sending item {seq + 1}/{len(items)} (command {item["command"]})')
        conn.mav.mission_item_int_send(
            conn.target_system, conn.target_component, seq,
            item['frame'], item['command'],
            1 if seq == 0 else 0,  # current
            item['autocontinue'],
            p[0], p[1], p[2], p[3], x, y, p[6],
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)


def start_mission(conn, num_items):
    """Switch the drone to mission mode and run the mission from the first item"""
    conn.mav.command_long_send(conn.target_system, conn.target_component,
                               mavutil.mavlink.MAV_CMD_MISSION_START, 0,
                               0, num_items - 1, 0, 0, 0, 0, 0)
    while True:
        ack = conn.recv_match(type='COMMAND_ACK', blocking=True, timeout=TIMEOUT)
        if ack is None:
            raise TimeoutError('no answer to the mission start command')
        if ack.command != mavutil.mavlink.MAV_CMD_MISSION_START:
            continue
        if ack.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f'mission start rejected (MAV_RESULT {ack.result}); '
                               'is the drone armed and in the air?')
        return


def main():
    default_plan = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                '..', 'missions', 'simple_mission.plan')
    parser = argparse.ArgumentParser(description='Upload a .plan mission to a drone and start it')
    parser.add_argument('system_id', type=int, help='MAVLink system id of the drone (drone number + 1)')
    parser.add_argument('plan', nargs='?', default=default_plan,
                        help='QGroundControl .plan file (default: SITL/missions/simple_mission.plan)')
    args = parser.parse_args()

    try:
        items = load_plan(args.plan)
        conn = connect(args.system_id)
        print(f'Uploading {len(items)} mission items from {args.plan}')
        upload_mission(conn, items)
        start_mission(conn, len(items))
    except (OSError, ValueError, KeyError, TimeoutError, RuntimeError) as e:
        print(f'Mission failed: {e}')
        sys.exit(1)
    print('Mission started')


if __name__ == '__main__':
    main()
