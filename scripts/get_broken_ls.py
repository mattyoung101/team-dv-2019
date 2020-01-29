#
# Copyright (c) 2019 Team Deus Vult (Ethan Lo, Matt Young, Henry Hulbert, Daniel Aziz, Taehwan Kim). 
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
broken_string = "111111111111111100111111111111001111111100110000"

for i, sensor in enumerate(broken_string):
    if sensor == "0":
        print("Broken sensor:", str(i))