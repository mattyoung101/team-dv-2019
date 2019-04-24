Deus Vult ESP32 IDF
====================

This repository contains the code powering Team Deus Vult's 2019 robot, competing in RoboCup Jr Soccer. It is written almost entirely in C, without some Python scripts for code generation and debugging.

For more information on our robot, please see our team's PowerPoint and poster.

Contact Matt Young (25070@bbc.qld.edu.au) for any questions, queries, qualms or concerns.

## Directory structure
- components/
    - eMD: contains the Embedded MotionDriver from Invensense, modified by us to run on the ESP32. With the exception of simple_imu.c and simple_imu.h, all code in this directory is covered under Invensense's license (see License.txt).
- docs: contains various bits of documentation
- main: contains the main code that runs on the ESPs
- openmv: contains the code that runs on our camera, the OpenMV H7
- scripts: contains various Python scripts for generating code and simple debugging
    - midis: contains MIDI files to be converted and played on the robot

## License
This code is currently proprietary and confidential to Brisbane Boys' College and Team Deus Vult. No redistribution or use outside BBC teams is permitted. 

Note: At some point, the code may be re-licensed under the BSD 4-Clause or MPL.

## Libraries used
- ESP-IDF: Apache 2 license
- Embedded MotionDriver: see License.txt in components/eMD
- DG_dynarr: Public Domain
- HandmadeMath: Public Domain