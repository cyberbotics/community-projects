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
import numpy as np
from controller import Supervisor

# import custom scripts. Located in the same directory as this controller
from get_relative_position import RelativePositions
from ik_module import inverseKinematics
from spawn_target import spawnTarget

# ----------------------------------------------------------
# CONFIGURATION
# ----------------------------------------------------------
# how many simulationsteps before calculating the next IK solution. This 
# is only relevant, if the target is constantly changing, as no new IK
# solution gets calculated, if the target did not change.
IKstepSize = 10 
useRotation = True
last_link_vector = None#[0, 0.12, 0]


# ----------------------------------------------------------
# ----------------------------------------------------------

# Initialize the Webots Supervisor.
supervisor = Supervisor()
if not supervisor.getSupervisor():
    sys.exit('WARNING: Your robot is not a supervisor! Set the supervisor field to True and restart the controller.')

timeStep = int(supervisor.getBasicTimeStep())
# Initialize our inverse kinematics module
ik = inverseKinematics(supervisor,last_link_vector)

# check if our world already has the TARGET node. If not, we spawn it.
target = supervisor.getFromDef('TARGET')
try:
    target.getPosition()
except:
    print('No TARGET defined. Spawning TARGET sphere')
    spawnTarget(supervisor)
    

# Initialize the RelativePositions module
RelPos = RelativePositions(supervisor)

# Initialize the arm motors and sensors. 
n = supervisor.getNumberOfDevices()
motors = []
sensors = []
for i in range(n):
    device = supervisor.getDeviceByIndex(i)
    #print(device.getName(), '   - NodeType:', device.getNodeType())
    if device.getNodeType() == 54:
        motors.append(device)
        sensor = device.getPositionSensor()
        try:
            sensor.getName()
            sensors.append(sensor)
            sensor.enable(timeStep)  
        except:
            print('Rotational Motor: ' + device.getName() + ' has no Position Sensor')
       
 
target_pos_old = np.zeros((3))
target_rot_old = np.zeros((3,3))
print('-------------------------------------------------------')
print('Move or rotate the TARGET sphere to move the arm...')
while supervisor.step(IKstepSize*timeStep) != -1:
    # Get the target position relative to the arm
    target_pos, target_rot = RelPos.get_pos('TARGET')    
    # check if our TARGET position or rotation has changed. If not, skip ik calculations (computationally intensive)
    if not np.array_equal(target_pos, target_pos_old) or not np.array_equal(target_rot, target_rot_old):
        # Call the ik_module to compute the inverse kinematics of the arm.   
        if useRotation:
            ikResults = ik.get_ik(target_pos, target_rot)
        else: 
            ikResults = ik.get_ik(target_pos)
        # set the motor positions to the calculated ik solution
        for i in range(len(motors)):
            motors[i].setPosition(ikResults[i + 1])
    target_pos_old = target_pos
    target_rot_old = target_rot

