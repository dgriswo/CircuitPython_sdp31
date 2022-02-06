# SPDX-FileCopyrightText: Copyright (c) 2022 Daniel Griswold
#
# SPDX-License-Identifier: MIT

import time
import board
import sdp31


sdp31 = sdp31.SDP31(board.I2C())

sdp31.start_continuous_measurement(average=True)

for i in range(5):
    print(
        f"Differential Pressure: {sdp31.differential_pressure}\tTemperature: {sdp31.temperature}"
    )
    time.sleep(1)

sdp31.stop_continuous_measurement()
