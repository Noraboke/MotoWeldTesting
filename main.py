from clientController import MotoController
import time
from outputAdresses import ot


c = MotoController()


def arcof_test():
    # just a function to test arcon by attempting to immediately deactivate the welding
    c._sync_sim_robot_position()
    m_traj = c._relative_movement_trajectory([0,0,0.02], 50)

    print('welding')
    c.arcon_mode()
    # should add function to check if the welder is active - before turning off the starter signal - same for stopping signal
    time.sleep(0.4)

    c.arcof_mode()

    c.move_mode()
    c._move_from_to(m_traj)


def perform_weld(arcweld=False):
    c._sync_sim_robot_position()
    weld_t_arr = [-0.15,0,0]
    weld_traj = c._relative_movement_trajectory(weld_t_arr, 250)

    if arcweld:
        print('STARTING WELD')
        time.sleep(1)
        c.arcon_mode()
        time.sleep(0.05)
        c.move_mode()

    c._move_from_to(weld_traj)

    if arcweld:
        c.arcof_mode()
        # c.move_mode()

def return_to_start():
    c._sync_sim_robot_position()
    weld_t_arr = [0.15,0,0]
    weld_traj = c._relative_movement_trajectory(weld_t_arr, 100)
    c._move_from_to(weld_traj)

#careful
def yolo_movement(t_arr, steps=200):
    c._sync_sim_robot_position()
    m_traj = c._relative_movement_trajectory(t_arr, steps)
    c._move_from_to(m_traj)
