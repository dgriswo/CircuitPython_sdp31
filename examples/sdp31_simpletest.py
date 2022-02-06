# SPDX-FileCopyrightText: Copyright (c) 2022 Daniel Griswold
#
# SPDX-License-Identifier: MIT

import board
import sdp31

sdp31 = sdp31(board.I2C())

print(f"Differential Pressure: {sdp31.differential_pressure}")
print(f"Temperature: {sdp31.temperature}")
