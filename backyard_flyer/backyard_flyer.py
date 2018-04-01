import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection, target_altitude = 3, square_offset = 10):
        '''
        Parameters:
            connection - connection instance
            target_altitude - altitude
            square_offsets - offset for the square vertices
        '''
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.cur_waypoint = None

        self.target_altitude = target_altitude
        self.offset = square_offset

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data
        """
        if self.flight_state == States.TAKEOFF:
            altitude = -1.0 * self.local_position[2]

            # check if altitude is within 95% of target
            if altitude > 0.95 * self.target_position[2]:
                self.landing_transition()

        if self.flight_state == States.WAYPOINT:
            if self.is_done():
                self.landing_transition()
            elif self.has_arrived(self.all_waypoints[self.cur_waypoint]):
                self.cur_waypoint += 1
                self.waypoint_transition()
                
    def has_arrived(self, waypoint):
        dest_n, dest_e, _, _ = waypoint
        cur_n, cur_e, _ = self.local_position
        
        return abs(cur_n - dest_n) < 0.1 and abs(cur_e - dest_e) < 0.1
    
    def velocity_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data
        """
                
        if self.flight_state == States.LANDING:
            if ((self.global_position[2] - self.global_home[2] < 0.1) and
            abs(self.local_position[2]) < 0.01):
                self.disarming_transition()

    def state_callback(self):
        """
        TODO: Implement this method

        This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data
        """
        if not self.in_mission:
            return

        if self.flight_state == States.MANUAL:
            self.arming_transition()

        elif self.flight_state == States.ARMING:
            if self.armed:
                self.takeoff_transition()

        # took off, start flying
        elif self.flight_state == States.TAKEOFF:
            self.all_waypoints = self.calculate_box()
            self.cur_waypoint = 0
            self.waypoint_transition()

        elif self.flight_state == States.DISARMING:
            self.manual_transition()

    def is_done(self):
        return self.cur_waypoint >= len(self.all_waypoints)
    
    def calculate_box(self):
        """TODO: Fill out this method

        1. Return waypoints to fly a box
        """
        offset = self.offset

        n, e, a = self.target_position
        h = 0
        return [(n + offset, e, a, h), (n + offset, e + offset, a, h), (n, e + offset, a, h), (n, e, a, h)]

    def arming_transition(self):
        """TODO: Fill out this method

        1. Take control of the drone
        2. Pass an arming command
        3. Set the home location to current position
        4. Transition to the ARMING state
        """
        print("arming transition")
        self.take_control()
        self.arm()
        self.set_home_position(*self.global_position)
        self.flight_state = States.ARMING

    def takeoff_transition(self):
        """TODO: Fill out this method

        1. Set target_position altitude to 3.0m
        2. Command a takeoff to 3.0m
        3. Transition to the TAKEOFF state
        """
        print("takeoff transition")
        self.target_position[2] = self.target_altitude
        self.takeoff(self.target_position[2])

        self.all_waypoints = self.calculate_box()
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):
        """TODO: Fill out this method

        1. Command the next waypoint position
        2. Transition to WAYPOINT state
        """
        if self.is_done():
            return
  
        print(f"waypoint transition: {self.all_waypoints[self.cur_waypoint]}")

        self.cmd_position(*self.all_waypoints[self.cur_waypoint])
        self.flight_state = States.WAYPOINT


    def landing_transition(self):
        """TODO: Fill out this method

        1. Command the drone to land
        2. Transition to the LANDING state
        """
        print("landing transition")
        self.land()
        self.flight_state = States.LANDING

    def disarming_transition(self):
        """TODO: Fill out this method

        1. Command the drone to disarm
        2. Transition to the DISARMING state
        """
        print("disarm transition")
        self.disarm()
        self.flight_state = States.DISARMING

    def manual_transition(self):
        """This method is provided

        1. Release control of the drone
        2. Stop the connection (and telemetry log)
        3. End the mission
        4. Transition to the MANUAL state
        """
        print("manual transition")

        self.release_control()
        self.stop()
        self.in_mission = False
        self.flight_state = States.MANUAL

    def start(self):
        """This method is provided

        1. Open a log file
        2. Start the drone connection
        3. Close the log file
        """
        print("Creating log file")
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        self.connection.start()
        print("Closing log file")
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False, timeout=1200)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
