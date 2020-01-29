#
# Copyright (c) 2019 Team Deus Vult (Ethan Lo, Matt Young, Henry Hulbert, Daniel Aziz, Taehwan Kim). 
#
# This Source Code Form is subject to the terms of the Mozilla Public
# License, v. 2.0. If a copy of the MPL was not distributed with this
# file, You can obtain one at http://mozilla.org/MPL/2.0/.
#
print_str = "ESP_LOGD(TAG, \"Values: {}\", {})"
float_formats = ", ".join(["(%.2f, %.2f)"] * 24)
format_str = ", ".join([f"readings[{i}].X, readings[{i}].Y" for i in range(24)])

print(print_str.format(float_formats, format_str))