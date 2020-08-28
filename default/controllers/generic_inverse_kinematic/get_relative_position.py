import numpy as np

class RelativePositions():
    def __init__(self, Supervisor):
        self.robot = Supervisor

    def get_pos(self, DEF_target, DEF_base=None):
        target = self.robot.getFromDef(DEF_target)
        if DEF_base is None:
            base = self.robot.getSelf()            
        else:
            base = self.robot.getFromDef(DEF_base)
        
        # Get the transposed rotation matrix of the base, so we can calculate poses of
        # everything relative to it.
        # Get orientation of the Node we want as our new reference frame and turn it into
        # a numpy array. Returns 1-dim list of len=9.
        rot_base = np.array(base.getOrientation())
        # reshape into a 3x3 rotation matrix
        rot_base = rot_base.reshape(3, 3)
        # Transpose the matrix, because we need world relative to the base, not the
        # base relative to world.
        rot_base = np.transpose(rot_base)
        # Get the translation between the base and the world (basically where the origin
        # of our new relative frame is).
        # No need to use the reverse vector, as we will subtract instead of add it later.
        pos_base = np.array(base.getPosition())


        # target position relative to world.
        target_pos_world = np.array(target.getPosition())
        # Calculate the relative translation between the target and the base.
        target_pos_world = np.subtract(target_pos_world, pos_base)
        # Matrix multiplication with rotation matrix: target posistion relative to base.
        target_pos_base = np.dot(rot_base, target_pos_world)

        # Calculate the orientation of the target, relative to the base, all in one line.
        target_rot_base = np.dot(rot_base, np.array(target.getOrientation()).reshape(3, 3))
        return target_pos_base, target_rot_base