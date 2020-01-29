Team Deus Vult (2019)
====================

Welcome to the repository for Team Deus Vult's code and designs, which powered our robot in the 2019 RoboCup Jr Internationals in Sydney, Australia. We competed in RoboCup Jr Lightweight at the time,
and placed 11th in gameplay. However, we placed highly in SuperTeam and won the innovation prize for our Protocol Buffers technology.

## Team members
- Ethan Lo: electrical and mechanical designer, software engineer, hardware engineer, documentation
- Matt Young: software engineer, documentation
- Daniel Aziz: hardware engineer
- Henry Hulbert: hardware engineer, electrical engineer
- Taehwan Kim (uncredited at the time): documentation, hardware engineer

### Contacting us
If you have any questions about our code or designs, don't hesitate to get in touch. Although this is an old project, we're always happy to answer any questions we may get.

Since Daniel, Henry and Taehwan have all graduated, it may be difficult to get in touch with them directly. Feel free to contact [Matt](matt.young.1@outlook.com) or [Ethan](ethanlo2010@gmail.com), and we can forward you to the right person if need be.

## Disclaimers
Please take into account the following before using our code:

- This repository is very old and the code is now defunct. Our team has merged with another BBC team to form Team Omicron, and we are competing in the 2020 Bordeaux RoboCup Jr Open Soccer competition.
- Our documentation website is no longer readily available. If you have any questions, please contact us directly. This isn't really under our control, so we apologise.
- At this time, we do not have time to write proper installation instructions. Please lookup the ESP IDF v3.3 install instructions and treat this like a regular ESP-IDF project.
- We apologise for our potentially distasteful team name at the time. This is no longer the case.
- We also apologise for any offensive language in commits and code, but we feel like preserving the whole repository history is important.

## Original description
This repository contains the code powering Team Deus Vult's 2019 robot, competing in RoboCup Jr Soccer. It is written almost entirely in C, without some Python scripts for code generation and debugging.

### Important notice about IDF version
Due to the fact that we sync the file `sdkconfig`, it's important that you use the exact same IDF version that we do, to avoid merge conflicts.
This project currently uses the **[v3.3 release branch](https://github.com/espressif/esp-idf/tree/release/v3.3) version** (i.e. the latest commit on origin/release/v3.3).

Please visit [the docs](https://docs.espressif.com/projects/esp-idf/en/latest/versions.html) for more information about IDF versions.

## Directory structure
- .vscode: VSCode settings, mainly spellchecking. In order to create `c_cpp_properties.json` for the C/C++ extension (which is not synced on Git), please see `docs/c_properties_template.txt`.
- components/
    - eMD: contains the Embedded Motion Driver, which is no longer used. All InvenSense files in this directory are licensed under their own license, the rest (e.g. simple_imu.*) are under the main repo's license.
    - led_strip: ESP32 Addressable LED strip library: [https://github.com/Lucas-Bruder/ESP32_LED_STRIP](https://github.com/Lucas-Bruder/ESP32_LED_STRIP)
    - mpl: InvenSense Motion Processing Library, no longer used.
    - vl53l0x: ST's VL53L0X LRF driver and docs. We use Kyle Hendrick's [ESP32 port](https://github.com/kylehendricks/esp32-vl53l0x).
    - nanopb: A C library for encoding/decoding ProtoBuf data
- docs: contains various bits of documentation, templates, etc
- ls_slave: contains the code which runs on the Arduino Nano light sensor slave device
- main: contains the main code that runs on the master and slave ESP32
- openmv: contains the code that runs on our camera, the OpenMV H7
- scripts: contains various Python scripts for generating code and simple debugging
    - midis: contains MIDI files to be converted and played on the robot

## License
Everything in the repository written by us is available under the Mozilla Public Licenses 2.0. Please see LICENSE.txt.

## Libraries and licenses
- ESP-IDF: Apache 2
- DG_dynarr: Public Domain
- HandmadeMath: Public Domain
- nanopb: zlib license
- ESP32 LED_STRIP: Apache 2
