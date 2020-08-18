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

"""Demonstration of inverse kinematics using the "ikpy" Python module."""

import sys
import tempfile
from math import sin, pi
import math
import numpy as np
from controller import Supervisor
from get_relative_position import RelativePositions
from scipy.spatial.transform import Rotation as R

try:
    import ikpy
except ImportError:
    sys.exit('The "ikpy" Python module is not installed. '
             'To run this sample, please upgrade "pip" and install ikpy with this command: "pip install ikpy"')

if ikpy.__version__[0] < '3':
    sys.exit('The "ikpy" Python module version is too old. '
             'Please upgrade "ikpy" Python module to version "3.0" or newer with this command: "pip install --upgrade ikpy"')


# ----------------------------------------------------------
# CONFIGURATION
# ----------------------------------------------------------
# how many simulationsteps before calculating the next IK solution. This 
# is only relevant, if the target is constantly changing, as no new IK
# solution gets calculated, if the target did not change.
IKstepSize = 10 


# ----------------------------------------------------------
# ----------------------------------------------------------

# Initialize the Webots Supervisor.
supervisor = Supervisor()
timeStep = int(supervisor.getBasicTimeStep())

# Initialize the RelativePositions class
RelPos = RelativePositions(supervisor)

# Initialize the arm motors and sensors. 
n = supervisor.getNumberOfDevices()
motors = []
sensors = []
motor_names = []
sensor_names = []
for i in range(n):
    device = supervisor.getDeviceByIndex(i)
    #print(device.getName(), '   - NodeType:', device.getNodeType())
    if device.getNodeType() == 54:
        motors.append(device)
        sensors.append(device.getPositionSensor())
        motor_names.append(device.getName())
        sensor_names.append(sensors[-1].getName())
        sensors[-1].enable(timeStep)

       
       
        
# Create the arm chain.
filename = None
with tempfile.NamedTemporaryFile(suffix='.urdf', delete=False) as file:
    filename = file.name
    file.write(supervisor.getUrdf().encode('utf-8'))
armChain = ikpy.chain.Chain.from_urdf_file(filename)




# make sure only links with a motor are active for IK calculations
active_links = [False] * len(armChain.links)

for i in range(len(armChain.links)):  
    link_name = armChain.links[i].name
    active_links[i] = link_name in motor_names or link_name in sensor_names
    if active_links[i]:
        # ikpy includes the bounds as valid, In Webots they have to be less than the limit
        new_lower = armChain.links[i].bounds[0] + 0.0000001
        new_upper = armChain.links[i].bounds[1] - 0.0000001       
        armChain.links[i].bounds = (new_lower, new_upper)
    
armChain.active_links_mask = active_links    

# ----------------------------------------------------------
# Setup for correct orientation calculations
# ----------------------------------------------------------

# convert urdf rpy of last link into rotation matrix
r = R.from_euler('xyz', armChain.links[-1].orientation, degrees=False).as_matrix()
# get the translation vector of the last link
pos = armChain.links[-1].translation_vector
# calculate the final link vector using the dot product
final_link_vector = np.round(np.dot(r,pos), 2)

# which column we have to take from the rotation matrix, for our toolSlot axis
try:
    rot_index = final_link_vector.tolist().index(np.sum(final_link_vector))
except:
    rot_index=2
    print('WARNING!!!!!!!!!!!')
    print('The vector of the last link is not an axis. Make sure the orientation and translation of this link are set up in a way, that either the x, y, or z axis points out of the toolSlot')
    print('The IK solver will not solve correctly for orientation. The controller will now use the z-axis as the final link vector')
    print('rotationi matrix:')
    print(r)
    print('translation vector: ', pos)
    print('final link vector:', final_link_vector)   

# define for ikpy, which axis we are using
toolSlot_axis = ['X', 'Y', 'Z'][rot_index]


# ----------------------------------------------------------
#  Main loop, following the target sphere
# ----------------------------------------------------------

target_pos_old = np.zeros((3))
target_rot_old = [0]*3
print('Move the yellow and black sphere to move the arm...')
while supervisor.step(IKstepSize*timeStep) != -1:
    # Get the target position relative to the arm
    target_pos, target_rot = RelPos.get_pos('TARGET')
    
    # get the rotation vector from the target_rot rotation matrix, depending on which axis points out of the toolSlot
    rot_vector = [target_rot[0,rot_index],target_rot[1,rot_index],target_rot[2,rot_index]]
    
    # Call "ikpy" to compute the inverse kinematics of the arm.   
    if any(target_pos != target_pos_old) or rot_vector != target_rot_old: 
        ikResults = armChain.inverse_kinematics(target_pos, target_orientation=rot_vector, orientation_mode=toolSlot_axis)
        if sum(ikResults) == 0: # dealing with singularity
            rot_vector_new = np.array(rot_vector) + np.full((1,3), 0.0000001) # tiny change in the orientation vector to avoid singularity
            ikResults = armChain.inverse_kinematics(target_pos, target_orientation=rot_vector_new, orientation_mode=toolSlot_axis)
        for i in range(len(motors)):
            motors[i].setPosition(ikResults[i + 1])
    target_pos_old = target_pos
    target_rot_old = rot_vector

