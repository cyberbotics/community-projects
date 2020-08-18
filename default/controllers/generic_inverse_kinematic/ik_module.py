# Copyright 2020 Simon Steinmann
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import sys
import tempfile
import numpy as np

try:
    from scipy.spatial.transform import Rotation as R
except ImportError:
    sys.exit('The "scipy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install scipy with this command: "pip install scipy"')

try:
    import ikpy
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')

class inverseKinematics():
    def __init__(self, supervisor):
        # Initialize the Webots Supervisor.
        self.supervisor = supervisor
        self.timeStep = int(self.supervisor.getBasicTimeStep())

        # Get names for the arm's motors and sensors. Needed for correct armChain configuration
        n = self.supervisor.getNumberOfDevices()
        self.motor_names = []
        self.sensor_names = []
        for i in range(n):
            device = self.supervisor.getDeviceByIndex(i)
            if device.getNodeType() == 54:
                sensor = device.getPositionSensor()
                self.motor_names.append(device.getName())
                self.sensor_names.append(sensor.getName())

     
        # Create the arm chain.
        filename = None
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.supervisor.getUrdf().encode('utf-8'))
        self.armChain = ikpy.chain.Chain.from_urdf_file(filename)

        # make sure only links with a motor are active for IK calculations
        active_links = [False] * len(self.armChain.links)
        for i in range(len(self.armChain.links)):  
            link_name = self.armChain.links[i].name
            active_links[i] = link_name in self.motor_names or link_name in self.sensor_names
            if active_links[i]:
                # ikpy includes the bounds as valid, In Webots they have to be less than the limit
                new_lower = self.armChain.links[i].bounds[0] + 0.0000001
                new_upper = self.armChain.links[i].bounds[1] - 0.0000001       
                self.armChain.links[i].bounds = (new_lower, new_upper)            
        self.armChain.active_links_mask = active_links    

        # ----------------------------------------------------------
        # Setup for correct orientation calculations
        # ----------------------------------------------------------

        # convert urdf rpy of last link into rotation matrix
        r = R.from_euler('xyz', self.armChain.links[-1].orientation, degrees=False).as_matrix()
        # get the translation vector of the last link
        pos = self.armChain.links[-1].translation_vector
        # calculate the final link vector using the dot product
        self.final_link_vector = np.round(np.dot(r,pos), 2)

        # which column we have to take from the rotation matrix, for our toolSlot axis
        try:
            self.rot_index = self.final_link_vector.tolist().index(np.sum(self.final_link_vector))
        except:
            self.rot_index=2
            print('WARNING!!!!!!!!!!!')
            print('The vector of the last link is not an axis. Make sure the orientation and translation of this link are set up in a way, that either the x, y, or z axis points out of the toolSlot')
            print('The IK solver will not solve correctly for orientation. The controller will now use the z-axis as the final link vector')
            print('rotationi matrix:')
            print(r)
            print('translation vector: ', pos)
            print('final link vector:', self.final_link_vector)   

        # define for ikpy, which axis we are using for the last link
        self.toolSlot_axis = ['X', 'Y', 'Z'][self.rot_index]

    def get_ik(self, target_pos, target_rot=None):   
        # check if a target_rot is requested (3x3 numpy array)
        if np.sum(target_rot) == None:
            # Call "ikpy" to compute the inverse kinematics of the arm WITHOUT orientation
            return self.armChain.inverse_kinematics(target_pos)
        else:
            # get the rotation vector from the target_rot rotation matrix, depending on which axis points out of the toolSlot
            rot_vector = [target_rot[0,self.rot_index],target_rot[1,self.rot_index],target_rot[2,self.rot_index]]
            
            # Call "ikpy" to compute the inverse kinematics of the arm WITH orientation   
            ikResults = self.armChain.inverse_kinematics(target_pos, target_orientation=rot_vector, orientation_mode=self.toolSlot_axis)

            # if ikResults is all zeros, we are dealing with a singularity
            if sum(ikResults) == 0: 
                # tiny change in the orientation vector to avoid singularity
                rot_vector_new = np.array(rot_vector) + np.full((1,3), 0.0000001) 
                ikResults = self.armChain.inverse_kinematics(target_pos, target_orientation=rot_vector_new, orientation_mode=toolSlot_axis)
            return ikResults
