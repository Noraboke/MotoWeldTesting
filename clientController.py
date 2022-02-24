from moto.simple_message import *
from moto import motion_connection
from moto.simple_message_connection import SimpleMessageConnection
from moto.simple_message import SimpleMessage
from moto import Moto
from moto import Moto, ControlGroupDefinition
from moto.simple_message import (
    JointFeedbackEx,
    JointTrajPtExData,
    JointTrajPtFull,
    JointTrajPtFullEx,
)

import roboticstoolbox as rtb
import spatialmath as sm
import spatialgeometry as gm
from swift import Swift
import time
import sys

import numpy as np

from outputAdresses import ot

import numpy as np

class MotoController(object):
    """ Controller for the Yaskawa motoman Moto library """

    def __init__(self):
        super(MotoController, self).__init__()

        # controller IP
        self.ip = "192.168.255.200"

        self.position = [0]*6
        self.robot = None

        self.simulation_real_time = 0
        self._sequence = 0

        # apparently this increment also works the best with the robot
        self.time_increment = 0.05 # this can be observed as the loop interval in INIT_ROS.jbi as well

        self.c_speed = 1/250 # connection is supposedly 250Hz

        self.home_move_time = 5

        self.connected = False


        # SIMULATION PARAMETERS
        self.sim_loaded = False
        self.sim_robot = rtb.models.URDF.Motoman_gp25_12()
        self.sim_robot.base = sm.SE3(-0.3,0,0.8)



    def connect(self, ip = None):
        # Establish connection
        # should add retry etc.

        self.robot: Moto = Moto(
                self.ip,
                [ControlGroupDefinition("robot", 0, 6, ["s", "l", "u", "r", "b", "t"])],
            )

        self.robot.motion.start_servos()
        self.robot.motion.start_trajectory_mode()

        status = self.robot.motion.check_motion_ready()


        self._update_position()

        print("Status:\n" + str(status))
        print("Position:\n"+str(self.position))

        self.connected = True

        time.sleep(1)
        self.move_mode() # redundant?

    def move_mode(self):
        self._write_bit(ot['MOVE'], 1)
        while self.robot.motion.check_motion_ready().body.result is not ResultType.SUCCESS:
            self.robot.motion.stop_trajectory_mode()
            self.robot.motion.start_trajectory_mode()
            time.sleep(0.004)


    def arcon_mode(self):
        self._write_bit(ot['MOVE'], 0)
        self.robot.motion.stop_trajectory_mode()
        self._write_bit(ot['ARCON'], 1)

        # deactive ARCON signal when ARCON has been turned on successfully - so that move/arcof signals can be sent
        while self._read_bit(ot['Process active']) == 0:
            time.sleep(0.004)

        self._write_bit(ot['ARCON'], 0)



    def arcof_mode(self):
        self._write_bit(ot['MOVE'], 0)
        self.robot.motion.stop_trajectory_mode()
        self._write_bit(ot['ARCOF'], 1)

        # deactive ARCON signal when ARCON has been turned on successfully - so that move/arcof signals can be sent
        while self._read_bit(ot['Process active']) == 1:
            time.sleep(0.004)

        self._write_bit(ot['ARCOF'], 0)


    def _update_position(self):
        # does not work when the position is in 0?
        try:
            self.position = self.robot.state.joint_feedback(0).pos[:6] # position for robot group 0
        except Exception as e:
            self.position = [0,0,0,0,0,0]

        return self.position


    def disconnect(self):
        self.robot.motion.stop_servos()
        self.robot.motion.stop_trajectory_mode()
        self.robot.motion.disconnect()

        self.connected = False

        # state server and IO server should be closed as well
        # should look for existing function - should exist in Moto?


    def _move_from_to(self, traj):
        # don't know if this is neccessary
        track_time = time.perf_counter()
        for pos in traj:
            # velocity arr
            dqt = (self.position - pos)/self.time_increment

            # update position
            self.position = pos

            self._generate_trajectory_point(pos, dqt)  # sends trajectory if status is not BUSY
            # _generate_trajectory_point(pos, vel) # using pre-calculated velcoty from roboticstoolbox

            # to stream to buffer - attempts to send at connection speed/frequency? - not sure if this is neccessary
            time.sleep(self.c_speed - (track_time-time.perf_counter()))
            track_time = time.perf_counter()


    def _generate_trajectory_point(self, position, dqt):
        self.simulation_real_time += self.time_increment

        current_trajectory_point = JointTrajPtFull(
                        groupno=0, # should remember to extend for which robot is being controlled
                        sequence=self._sequence,
                        valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                        time=self.simulation_real_time,
                        pos=np.array([position[0], position[1], position[2], position[3], position[4], position[5], 0.0, 0.0, 0.0, 0.0]),
                        vel=np.array([dqt[0], dqt[1], dqt[2], dqt[3], dqt[4], dqt[5], 0.0, 0.0, 0.0, 0.0]),
                        acc=[0.0] * 10
                        )

        # apparently the controller buffers up to 200 traj points - returns BUSY if full?
        # attempt to send joint trajectory point
        while True:
            if self.robot.motion.send_joint_trajectory_point(current_trajectory_point).body.result == ResultType.SUCCESS:
                break

            time.sleep(0.1)

        self._sequence += 1

    def _force_move_home(self):
        # just a manual move to home operation for reseting and experimenting
        p = JointTrajPtFull(
                groupno=0,
                sequence=self._sequence,
                valid_fields=ValidFields.TIME | ValidFields.POSITION | ValidFields.VELOCITY,
                time=self.home_move_time,
                pos=np.deg2rad([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                vel=[0.0] * 10,
                acc=[0.0] * 10
            )

        self.simulation_real_time += self.home_move_time

        self.robot.motion.send_joint_trajectory_point(p0)


    # maybe add waiting until success function for these
    def _write_bit(self, adress, value):
        if self.connected:
            res = self.robot.io.write_bit(adress, value)

            # print(res)
        else:
            return False

    def _read_bit(self, adress):
        if self.connected:
            res = self.robot.io.read_bit(adress)

            # print(res)
            return res.body.value
        else:
            return False

    def _write_group(self, group_adress, value):
        pass

    def _read_group(self, group_adress):
        pass



    # mainly for testing
    def _relative_movement_trajectory(self, trans_array, steps=50):
        # generate a trajectory for relative translational movement to current position
        # requires simulation robot
        current_ = self.sim_robot.fkine(self.sim_robot.q)

        T0 = (sm.SE3(current_.t)) * sm.SE3.OA(current_.o, current_.a)
        T1 = (sm.SE3(current_.t+trans_array)) * sm.SE3.OA(current_.o, current_.a)

        Ts = rtb.ctraj(T0, T1, steps)
        sol = self.sim_robot.ikine_min(Ts)

        traj = np.array([x.q for x in sol])

        return traj

    def _start_simulation_environment(self):

        self.env = Swift()
        self.env.launch()
        self.env.realtime=True
        self.env.add(self.sim_robot)
        self.env.step()

        self.sim_loaded = True

    def _sync_sim_robot_position(self):
        if self.connected:
            self.sim_robot.q = self.position
            if self.sim_loaded:
                self.env.step()
        else:
            return False

    def _simulation_move_from_to(self, traj):
        for pos in traj:
            self.sim_robot.q = pos
            self.env.step(self.time_increment)

if __name__ == '__main__':
    c = MotoController()
