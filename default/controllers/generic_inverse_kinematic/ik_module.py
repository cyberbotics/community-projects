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
    def __init__(self, supervisor, last_link_vector=None):
        # Initialize the Webots Supervisor.
        self.supervisor = supervisor
        self.timeStep = int(self.supervisor.getBasicTimeStep())

        # Get names for the arm's motors and sensors. Needed for correct armChain configuration
        n = self.supervisor.getNumberOfDevices()
        self.motor_names = []
        self.sensor_names = []
        for i in range(n):
            device = self.supervisor.getDeviceByIndex(i)
            print(device.getName() , '  -  NodeType = ' , device.getNodeType())
            if device.getNodeType() == 54:
                sensor = device.getPositionSensor()
                self.motor_names.append(device.getName())
                try:
                    self.sensor_names.append(sensor.getName())
                except:
                    print('Rotational Motor: ' + device.getName() + ' has no Position Sensor')

     
        # Create the arm chain.         
        filename = None
        # (uncomment next two lines, if you want to save a copy of the generated urdf file)
        with open('filename.urdf',  'w') as file:
            file.write(supervisor.getUrdf())   
        with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
            filename = file.name
            file.write(self.supervisor.getUrdf().encode('utf-8'))        
        self.armChain = ikpy.chain.Chain.from_urdf_file(filename) #,last_link_vector=last_link_vector)
        

        # make sure only links with a motor are active for IK calculations
        active_links = [False] * len(self.armChain.links)
        for i in range(len(self.armChain.links)):
            link_name = self.armChain.links[i].name
            active_links[i] = link_name in self.motor_names or link_name in self.sensor_names
            if active_links[i]:
                # ikpy includes the bounds as valid, In Webots they have to be less than the limit
                new_lower = new_upper = None
                if self.armChain.links[i].bounds[0] is not None:
                    new_lower = self.armChain.links[i].bounds[0] + 0.0000001
                if self.armChain.links[i].bounds[1] is not None:
                    new_upper = self.armChain.links[i].bounds[1] - 0.0000001       
                self.armChain.links[i].bounds = (new_lower, new_upper)   
                self.last_active = i  
        if not any(active_links):
            print('WARNING: could not identify which links are active. Setting all links to active by default.')      
            active_links = [True] * len(self.armChain.links)    
        self.armChain.active_links_mask = active_links    
        print(self.armChain)

        # ----------------------------------------------------------
        # Setup for correct orientation calculations
        # ----------------------------------------------------------

        # convert urdf rpy of last link into rotation matrix
        try:
            self.last_link_rot = R.from_euler('xyz', self.armChain.links[-1].orientation, degrees=False).as_matrix()
        except:
            print('Last link has no orientation. Parsing identity matrix (no rotation).')
            self.last_link_rot = np.identity(3)
        # get the translation vector of the last link
        pos = self.armChain.links[-1].translation_vector
        # calculate the final link vector using the dot product
        self.final_link_vector = np.round(np.dot(self.last_link_rot,pos), 2)
        print('---final vector: ',self.final_link_vector)

        # which column we have to take from the rotation matrix, for our toolSlot axis
        try:
            self.rot_index = self.final_link_vector.tolist().index(np.sum(self.final_link_vector))
        except:
            self.rot_index=2
            print('WARNING!!!!!!!!!!!')
            print('The vector of the last link is not an axis. Make sure the orientation and translation of this link are set up in a way, that either the x, y, or z axis points out of the toolSlot')
            print('The IK solver will not solve correctly for orientation. The controller will now use the z-axis as the final link vector')
            print('translation vector: ', pos)
            print('final link vector:', self.final_link_vector)   

        # define for ikpy, which axis we are using for the last link
        self.toolSlot_axis = ['X', 'Y', 'Z'][self.rot_index]
        self.direction = None


    def rotateToolSlot(self, target_rot, ikResults):
        # calculate the difference in Rotation around the axis between what we requested, and what we computed
        fk = self.armChain.forward_kinematics(ikResults)
        # rotation of the calculated ik solution
        R_fk = fk[:3,:3]
        # rotation to get from our ik rotation to our target rotation
        R_new = np.dot(target_rot, R_fk)
        # convert our rotation matrix into euler angles, so we know how much we have to change the last motor
        r = R.from_matrix(R_new)
        euler = r.as_euler('xyz', False)
        # the first time we call this function we have to figure out in which direction we have to rotate
        if self.direction is not None: 
            ikResults[6] = ikResults[self.last_active ] + self.direction * euler[self.rot_index]
        else:
            # here we test adding the delta-angle, if the resulting rotation - target rotation is not 0, we flip the direction
            ikResults[6] = ikResults[self.last_active ] + 1 * euler[self.rot_index]
            fk_check = self.armChain.forward_kinematics(ikResults)
            if np.sum(np.abs(np.round(fk_check[:3, :3] - target_rot, 3))) == 0:
                self.direction = 1
            else:
                self.direction = -1
        return ikResults


    def get_ik(self, target_pos, target_rot=None):   
        # check if a target_rot is requested (3x3 numpy array)
        if np.sum(target_rot) == None:
            # Call "ikpy" to compute the inverse kinematics of the arm WITHOUT orientation
            return self.armChain.inverse_kinematics(target_pos)
        else:
            # get the rotation vector from the target_rot rotation matrix, depending on which axis points out of the toolSlot
            rot_vector = [target_rot[0,self.rot_index],target_rot[1,self.rot_index],target_rot[2,self.rot_index]]
            
            # Call "ikpy" to compute the inverse kinematics of the arm WITH orientation   
            transMat = np.identity(4)
            transMat[:3,:3] = target_rot
            transMat[:3,3] = target_pos
            ikResults = self.armChain.inverse_kinematics(target_pos, target_orientation=rot_vector, orientation_mode=self.toolSlot_axis)
            # if ikResults is all zeros, we are dealing with a singularity
            if sum(abs(ikResults)) < 0.001:  
                # tiny change in the orientation vector to avoid singularity
                rot_vector_new = np.array(rot_vector) + np.full((1,3), 0.0000001)                 
                ikResults = self.armChain.inverse_kinematics(target_pos, target_orientation=rot_vector_new, orientation_mode=self.toolSlot_axis)
                # if the above step was not enough to avoid the singularity, we try to increase the deviation from the requested orientation step by step
                n = -5
                while sum(abs(ikResults)) < 0.001: 
                    print('Simple singulaity avoidance was not suffiecient. Trying larger deviation from requested orientation.')
                    print('Requested rotation vector: ' + str(rot_vector))
                    rot_vector_new = np.array(rot_vector) + np.full((1,3), 1 * 10 ** n) 
                    print('New rotation vector: ' + str(rot_vector_new))
                    ikResults = self.armChain.inverse_kinematics(target_pos, target_orientation=rot_vector_new, orientation_mode=self.toolSlot_axis)
                    n += 1            
            return self.rotateToolSlot(target_rot, ikResults)
