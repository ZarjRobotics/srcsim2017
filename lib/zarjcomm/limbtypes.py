"""
    Types for hands and arms, to be shared by oc and fc

"""

from copy import deepcopy

class LimbTypes(object):
    """ This gives us types and ranges for arms and hands """

    hand_styles = {
        'open': [0.0, 0.0, 0.0, 0.0, 0.0],
        'oppose': [1.8, 0.0, 0.0, 0.0, 0.0],
        'grab': [float('nan'), 0.6, 1.1, 0.9, 1.0],
        'claw': [1.8, 0.0, 0.4, 0.4, 0.4],
        'claw_no_thumb': [0.0, 0.0, 0.4, 0.4, 0.4],
        'open_down': [1.8, 0.4, 0.0, 0.0, 0.0],
        'rake': [0.0, 0.0, 0.3, 0.3, 0.3],
        'partial_grab': [1.8, 0.4, 0.3, 0.3, 0.3],
        'partial_grab2': [1.8, 0.45, 0.6, 0.6, 0.6],
        'partial_ungrab': [1.8, 0.4, 0.15, 0.15, 0.15],
        'repair_grab1': [1.4, 0.15, 0.1, 0.2, 0.1],
        'repair_grab2': [1.4, 0.3, 0.3, 0.3, 0.3],
        'repair_grab3': [1.4, 0.45, 0.6, 0.6, 0.6],
        'repair_ungrab': [1.4, 0.2, 0.15, 0.15, 0.15],
        'grab_object': [1.8, 0.6, 0.6, 0.6, 0.6],
        'dabird': [1.8, 0.6, 1.1, 0.3, 1.0],
        'purebird': [1.8, 0.6, 1.1, 0.0, 1.0],
        'grab_cable': [1.8, 0.6, 0.0, 0.9, 0.0],
        'fist': [1.8, 0.6, 1.1, 0.9, 1.0],
        }
    invert_left_hand = [1, 2, 3, 4]

    arm_styles = {
        'home': [-0.2, 1.3, 0.5, 1.4, 1.1, 0.0, 0.0],
        'tuck': [-0.2, 1.5182, 0.414, 2.0, 0, 0.0, 0.0],
        'tuck_out': [-0.2, 1.5182, -1, 2.0, 0, 0.0, 0.0],
        'pass_football': [-1.8, 1.5, 0., 2, 1.7, 0.0, 0.0],
        'carry_array': [-0.2, 1.3, 1, 2.174, 0, 0.2, 0.36],
        'carry_array_higher': [-1.0, 1.5, 1, 2.174, 0, 0.2, 0.36],
        'carry_array_middle': [-0.5, 1.5, 1, 2.174, 0, 0.2, 0.36],
        'spin_array_1': [-0.8, 0, 2.18, 2.174, 0.7, 0.0, -0.49],
        'spin_array_2': [-0.8, 0, 2.18, 1.0, 0.7, 0.0, -0.49],
        'spin_array_3': [-0.8, 0, 2.18, 1.3, 0.7, 0.0, -0.49],
        'spin_array_clear': [-0.8, -1, 2.18, 1.3, 0.7, 0.0, -0.49],
        'release_array_1': [-0.07, -0.07, -0.36, 1.18, -1.49, -0.40, -0.21],
        'look_at_object': [-2.2, 1.4, 2.18, 1.14, 0.19, 0.62, -0.19],
        'look_at_object_with_disgust': [-1.2, 1.4, -3.1, 0, 0.0, 0.0, 0.0],
        'turn_valve': [-1.0, 1.0, 0.5, 0.8, 1.0, 0.0, 0.0],
        'king_tut': [-2.2, 1.10, 1.65, 0.05, 1.7, 0.2, 0.2],
        'king_tut_high': [-2.7, 1.10, 1.65, 0.05, 1.7, 0.2, 0.2],
        'shut': [float('nan'), 1.5, float('nan'), float('nan'), float('nan'),
                  float('nan'), float('nan')],
        'open': [float('nan'), 0, float('nan'), float('nan'), float('nan'),
                  float('nan'), float('nan')],
        'control_out': [-0.6, 1.5, -1.2, 2.1, 2.3, 0.31, 0.0],
        'control_preside_grab': [-0.6, 1.5, -0.9, 1.0, 1.5, 0.62, 0.0],
        'control_side_grab': [-0.6, 1.5, 0.0, 1.05, 1.5, 0.62, 0.0],
        'control_pretop_grab': [-1.50, 1.519, 0.0, 0.0, 3.14, 0.0, 0.0],
        'control_top_grab': [-1.3, 1.519, 0.0, 0.0, 3.14, 0.0, 0.0],
        'door_out': [-1.83, 1.0, 1.85, 0.69, 1.20, 0.61, 0],
        'door_top': [-1.83, 1.519, 1.85, 0.69, 1.20, 0.61, 0],
        'door_bottom': [-0.5, 1.519, 1.85, 0.69, 1.20, 0.61, 0],
        'door_push': [-1.5, 1.5, 0, 0, 3.14, 0, 0],
        'spin_object_1': [0.0, -1.0, 1.4, 2.174, 3.14, 0.0, -0.49],
        'spin_object_2': [0.0, -0.3, 1.4, 2.174, 3.14, 0.0, -0.49],
        'rake_above': [0, -.3, 1.4, 0, 0, 0.0, 0.0],
        'rake_start': [0, 0.45, 1.4, 0, 0, 0.0, 0.0],
        'rake_end': [0, -.3, 1.4, 1.7, 0, 0.0, 0.0],
        'front_wipe_top': [-2.3, 1.3, 0.0, 0.0, 0.0, -0.15, 0.0],
        'front_wipe_bottom': [-0.5, 1.3, 0.0, 0.0, 0.0, -0.15, 0.0],
        'back_wipe_top': [2.0, 1.3, -0.5, 0.0, 0.0, 0.35, 0.0],
        'back_wipe_bottom': [1.0, 1.3, 0.0, 0.0, 0.0, 0.35, 0.0],
        'repair_prep': [0.0, 0.7, 0.0, 0.0, 3.14, 0.62, 0.0],
        'repair_start': [-1.5, 0.7, 0.0, 0.5, 3.14, 0.62, 0.0],
        'repair_start_side': [-1.2, 1.3, 0, 1.4, 0, 0.2, 0],
        }

    invert_left_arm = [1, 3, 5, 6]

    def __init__(self):

        # Constants
        self.arm_descriptions = [
            [ 'shoulderpitch',  [ -2.85, 2.0 ]],
            [ 'shoulderroll',   [ -1.266, 1.519 ]],
            [ 'shoulderyaw',    [ -3.1, 2.18 ]],
            [ 'elbowpitch',     [ -0.12, 2.174 ]],
            [ 'forearmyaw',     [ -2.019, 3.14 ]],
            [ 'wristroll',      [ -0.625, 0.62 ]],
            [ 'wristpitch',     [ -0.49, 0.36 ]]
            ]

        self.hand_descriptions = [
            [ 'thumbroll',  [ 0.0, 1.8 ]],
            [ 'thumbpitch', [ 0.0, 0.55 ]],
            [ 'indexpitch', [ 0.0, 1.1 ]],
            [ 'middlepitch',[ 0.0, 0.9 ]],
            [ 'pinkypitch', [ 0.0, 1.0 ]]
            ]

    @staticmethod
    def invert_arm_configuration(sidename, joints):
        if not sidename == 'left':
            return joints
        for inverter in LimbTypes.invert_left_arm:
            joints[inverter] = joints[inverter] * -1.0

        return joints

    @staticmethod
    def invert_hand_configuration(sidename, joints):
        if not sidename == 'left':
            return joints
        for inverter in LimbTypes.invert_left_hand:
            joints[inverter] = joints[inverter] * -1.0

        return joints


