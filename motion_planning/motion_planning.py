import os
import argparse
import time
import msgpack
import csv
import time

from enum import Enum, auto

import numpy as np

from planning_utils import *
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local

from networkx.readwrite import gpickle

np.set_printoptions(precision=4, suppress=True)

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()

# we are flying based on a random graph.
# increase accessibility by flying high
TARGET_ALTITUDE = 25
SAFETY_DISTANCE = 5

class MotionPlanning(Drone):

    def __init__(self, connection, mapfile='colliders.csv', graph_data=None):
        '''
        Parameters:
            connection - connection to the simulator
            mapfile - collistion map
            graph_data - a pickle of the previously created graph
        '''
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}

        self.graph_path = graph_data
        self.graph = None

        # parameters for stochastic sampling
        # and NN algorithm
        self.sample_size = 2000
        self.neighbors = 12

        # read lat0 and long0
        with open(mapfile) as f:
            row = list(csv.reader(f))[0]
        self.lat0 = float(row[0].split()[1])
        self.lon0 = float(row[1].split()[1])

        # read the data
        self.data = np.loadtxt(mapfile, delimiter=',', dtype='Float64', skiprows=2)

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

        print("Preparing flight graph...")
        start_time = time.time()
        self.prepare_flight_graph()
        print("Graph prepared in {:.3f} sec".format(time.time() - start_time))

    def local_position_callback(self):
        if self.local_position[0] > 1000.:
            raise ValueError("pos")

        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()

        elif self.flight_state == States.WAYPOINT:
            if self.has_arrived():
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def has_arrived(self):
        loc_pos = self.local_position
        loc_pos[-1] = - loc_pos[-1]
        good_norm = np.linalg.norm(self.target_position[:3] - loc_pos) < 1.0
        return good_norm

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                 self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        print('local position', self.local_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self, waypoints):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(waypoints)
        self.connection._master.write(data)

    def get_current_grid(self):
        '''
        Create the grid for visualization & offsets
        '''
        self.grid, self.north_offset, self.east_offset = create_grid(self.data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    def prepare_flight_graph(self):
        '''
        Creates the graph used to fly the drone
        '''
        if self.graph_path is not None and os.path.exists(self.graph_path):
            self.graph = gpickle.read_gpickle(self.graph_path)

        # Sample some random points on the current grid
        self.sampler = Sampler(self.data, SAFETY_DISTANCE, zmin=TARGET_ALTITUDE//2, zmax=TARGET_ALTITUDE)
        self.polygons = self.sampler.polygons
        self.heights = self.sampler.heights

        # if we don't have the graph by now - let's create it
        if self.graph is None:
            nodes = self.sampler.sample(self.sample_size)
            self.graph = create_graph(nodes, sampler.polygons, sampler.heights, self.neighbors)
            # if we have specified the path - we want to save it
            if self.graph_path is not None:
                gpickle.write_gpickle(self.graph, self.graph_path)

    def pick_a_start(self, start):
        if not add_point_to_graph(start, self.graph, self.polygons, self.heights, self.neighbors):
            raise ValueError("Cannot set start to the start location")
        return tuple(start)

    def pick_a_goal(self):
        '''
        Try random sampling the space and picking an appropriate goal
        '''
        nodes = list(self.graph.nodes)
        candidates = self.sampler.sample(300)
        for pt in candidates:
            if add_point_to_graph(pt, self.graph, self.polygons, self.heights, self.neighbors):
                return tuple(pt)
        raise ValueError("Could not pick a goal point!")

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")

        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(self.lon0, self.lat0, 0)

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        # Make sure we are at the right place.
        north, east, alt = global_to_local(self.global_home, self.global_home)

        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                          self.local_position))
        # Define starting point on the grid (this is just grid center)
        # we need to add it to the graph
        graph_start = self.pick_a_start((north, east, alt))

        # Set goal as some arbitrary position on the grid
        path = []
        tries = 0
        while(len(path) < 13 and tries < 30):
            graph_goal = self.pick_a_goal()
            path, _ = a_star_graph(self.graph, heuristic, graph_start, graph_goal)
            tries += 1

        print('Local Start and Goal: ', graph_start, graph_goal)

        # for visualization
        self.start = graph_start
        self.goal = graph_goal
        self.path = path

        print('Path length: ', len(path))

        # Set self.waypoints
        self.waypoints = np.array(path).reshape(-1, 3)[1:]
        zeros = np.zeros((self.waypoints.shape[0], 1))
        self.waypoints = list(np.hstack([self.waypoints, zeros]))

        # Convert path to waypoints
        waypoints = np.array(self.waypoints, dtype=np.int, copy=True)
        waypoints = [[int(w[0]), int(w[1]), int(w[2]), int(w[3])] for w in waypoints]

        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints(waypoints)

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False, timeout=1200)
    drone = MotionPlanning(conn, graph_data='flygraph.gpickle')
    time.sleep(1)

    drone.start()
