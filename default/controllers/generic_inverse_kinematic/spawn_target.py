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
import os
from shutil import copyfile

class spawnTarget():
    def __init__(self, supervisor):
        targetSphereString = """
        DEF TARGET Solid {
        translation 0.68 0.4 0.69
        rotation 0.5773519358512958 -0.5773469358518515 -0.5773519358512958 2.0944
        scale 0.4 0.4 0.4
        children [
            Shape {
            appearance PBRAppearance {
                baseColorMap ImageTexture {
                url [
                    "textures/target.png"
                ]
                }
                roughnessMap ImageTexture {
                url [
                    "textures/target.png"
                ]
                }
                metalnessMap ImageTexture {
                url [
                    "textures/target.png"
                ]
                }
                emissiveColorMap ImageTexture {
                url [
                    "textures/target.png"
                ]
                }
                textureTransform TextureTransform {
                scale 2 1
                }
            }
            geometry Sphere {
                radius 0.1
                subdivision 2
            }
            }
        ]
        }
        """
        # copy the target.png texture into our wold
        filePath = supervisor.getWorldPath()
        fileNameLength = len(filePath.split('/')[-1])
        worldPath = filePath[:-fileNameLength]        
        controllerPath = os.path.dirname(os.path.abspath(__file__))
        worldPath = worldPath + 'textures/'
        if not os.path.exists(worldPath):
            os.makedirs(worldPath)
        copyfile(controllerPath + '/textures/target.png', worldPath + 'target.png')

        # spawn the TARGET node
        root = supervisor.getRoot()
        rootChildren = root.getField('children')
        rootChildren.importMFNodeFromString(rootChildren.getCount(), targetSphereString)