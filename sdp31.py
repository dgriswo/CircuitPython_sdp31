# SPDX-FileCopyrightText: Copyright (c) 2022 Daniel Griswold
#
# SPDX-License-Identifier: MIT
"""
`sdp31`
================================================================================

CircuitPython helper library for the SDP31 differential pressure sensor


* Author(s): Daniel Griswold

Implementation Notes
--------------------

**Hardware:**

.. todo:: Add links to any specific hardware product page(s), or category page(s).
  Use unordered list & hyperlink rST inline format: "* `Link Text <url>`_"

**Software and Dependencies:**

* Adafruit CircuitPython firmware for the supported boards:
  https://github.com/adafruit/circuitpython/releases

.. todo:: Uncomment or remove the Bus Device and/or the Register library dependencies
  based on the library's use of either.

# * Adafruit's Bus Device library: https://github.com/adafruit/Adafruit_CircuitPython_BusDevice
# * Adafruit's Register library: https://github.com/adafruit/Adafruit_CircuitPython_Register
"""

import time
from adafruit_bus_device.i2c_device import I2CDevice
from micropython import const

__version__ = "0.0.0-auto.0"
__repo__ = "https://github.com/dgriswo/CircuitPython_sdp31.git"

_SDP31_I2CADDR_DEFAULT = const(0x21)
_SDP31_CONTINUOUS_DIFF_PRESSURE_AVERAGE = b"\x36\x15"
_SDP31_CONTINUOUS_DIFF_PRESSURE = b"\x36\x1E"
_SDP31_STOP_CONTINUOUS_MEASURE = b"\x3F\xF9"
_SDP31_TRIGGER_DIFF_PRESSURE_STRETCH = b"\x37\x2D"
_SDP31_ENTER_SLEEP = b"\x36\x77"
_SDP31_READ_PRODUCT_NUMBER = b"\x36\x7C"
_SDP31_READ_SERIAL_NUMBER = b"\xE1\x02"

_SDP31_CRC8_POLYNOMIAL = const(0x31)
_SDP31_CRC8_INIT = const(0xFF)
_SDP31_WORD_LEN = const(2)


class SDP31:
    """
    A driver for the SDP31 differential pressure sensor.

    :param ~busio.I2C i2c: The I2C bus the SDP31 is connected to.
    :param int address: The I2C address of the device. Defaults to :const: '0x21'

    **Quickstart: Importing and using the SDP31 pressure sensor**
      Here is one way of importing the `SDP31` class so you
      can use it with the name ``sdp31``.
      First you will need to import the libraries to use the sensor
      .. code-block:: python
          import busio
          import board
          import sdp31
      Once this is done you can define your `busio.I2C` object and define your sensor object
      .. code-block:: python
          i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
          sdp31 = sdp31.SDP31(i2c)
      Now you have access to the differential pressure using the
      :attr:`differential_pressure` attribute and the temperature using the :attr:`temperature`
      .. code-block:: python
          diff_pressure = sdp31.differential_pressure
          temperature = sdp31.temperature
    """

    def __init__(self, i2c, address=_SDP31_I2CADDR_DEFAULT):
        """Initialize the sensor, get the product number and verify that we found a SDP31."""
        self._device = I2CDevice(i2c, address)
        self._pressure = None
        self._temperature = None
        self._mode = None
        self.soft_reset(i2c)
        time.sleep(0.25)

        result = self.read_identifiers()
        if result == b"\x03\x01\x01\x01":  # SDP31
            self._pressure_scale_factor = 60
            self._temperature_scale_factor = 200
        elif result == b"\x03\x01\x02\x01":  # SDP32
            self._pressure_scale_factor = 240
            self._temperature_scale_factor = 200
        else:
            raise RuntimeError("SDP not detected.")

    def start_continuous_measurement(self, average=False):
        """In continuous mode the sensor is measuring at the highest speed
        and writes the measurement values to the I2C results buffer, where
        the I2C master can read out the value when it requires.

        If the ‘average till read’ option is chosen, the sensor averages
        all values prior to the read out. This has the benefit that
        the user can read out the sensor at its own desired speed, without
        losing information, which thus prevents aliasing.
        """
        if average is True:
            mode = _SDP31_CONTINUOUS_DIFF_PRESSURE_AVERAGE
        else:
            mode = _SDP31_CONTINUOUS_DIFF_PRESSURE

        if self._continuous_measurement is True and mode != self._mode:
            raise RuntimeError(
                "Continuous measurement already started.  Stop measurement before switching modes."
            )
        if self._continuous_measurement is True and mode is self._mode:
            return  # Continuous measurement active

        with self._device:
            self._device.write(bytes(mode))

        self._mode = mode
        self._continuous_measurement = True

    def read_measurement(self, type):
        if type == "pressure":
            return self._i2c_read_words(1)

        if type == "temperature":
            result = self._i2c_read_words(2)
            return result[2:4]

    def stop_continuous_measurement(self):
        """This command stops the continuous measurement and puts the sensor
        in idle mode. It powers off the heater and makes the sensor receptive
        for another command after 500us.  The Stop command is also required
        when switching between different continuous measurement commands."""
        with self._device:
            self._device.write(bytes(_SDP31_STOP_CONTINUOUS_MEASURE))
        time.sleep(0.005)
        self._continuous_measurement = False

    def triggered_measurement(self, type):
        """During a triggered measurement the sensor measures both differential
        pressure and temperature.  The measurement starts directly after the
        command has been sent. The command needs to be repeated with every
        measurement."""
        if type == "pressure":
            result = self._i2c_read_words_from_cmd(
                _SDP31_TRIGGER_DIFF_PRESSURE_STRETCH, 0, 1
            )

        if type == "temperature":
            result = self._i2c_read_words_from_cmd(
                _SDP31_TRIGGER_DIFF_PRESSURE_STRETCH, 0, 2
            )
            result = result[2:4]

        return result

    def soft_reset(self, i2c):
        while not i2c.try_lock():
            pass
        i2c.writeto(0x0, bytes([0x0006]))
        i2c.unlock()
        self._continuous_measurement = False

    def enter_sleep(self):
        if self._continuous_measurement is True:
            self.stop_continuous_measurement()
        with self._device:
            self._device.write(bytes(_SDP31_ENTER_SLEEP))

    def read_identifiers(self):
        with self._device:
            self._device.write(bytes(_SDP31_READ_PRODUCT_NUMBER))
            self._device.write(bytes(_SDP31_READ_SERIAL_NUMBER))

            crc_result = bytearray(6)
            self._device.readinto(crc_result)
            result = bytearray(0)
            for i in range(2):
                word = [crc_result[3 * i], crc_result[3 * i + 1]]
                crc = crc_result[3 * i + 2]
                if self._generate_crc(word) != crc:
                    raise RuntimeError("CRC Error")
                result.append(word[0])
                result.append(word[1])
            return result

    def _i2c_read_words(self, reply_size):
        """Read from SDP and CRC results if necessary"""
        with self._device:
            result = bytearray(reply_size)
            self._device.readinto(result)
            if not reply_size:
                return None
            crc_result = bytearray(reply_size * (_SDP31_WORD_LEN + 1))
            self._device.readinto(crc_result)
            result = bytearray()
            for i in range(reply_size):
                word = [crc_result[3 * i], crc_result[3 * i + 1]]
                crc = crc_result[3 * i + 2]
                if self._generate_crc(word) != crc:
                    raise RuntimeError("CRC Error")
                result.append(word[0])
                result.append(word[1])
            return result

    def _i2c_read_words_from_cmd(self, command, delay, reply_size):
        """Run an SDP command query, get a reply and CRC results if necessary"""
        with self._device:
            self._device.write(bytes(command))
            time.sleep(delay)
            if not reply_size:
                return None
            crc_result = bytearray(reply_size * (_SDP31_WORD_LEN + 1))
            self._device.readinto(crc_result)
            result = bytearray()
            for i in range(reply_size):
                word = [crc_result[3 * i], crc_result[3 * i + 1]]
                crc = crc_result[3 * i + 2]
                if self._generate_crc(word) != crc:
                    raise RuntimeError("CRC Error")
                result.append(word[0])
                result.append(word[1])
            return result

    # pylint: disable=no-self-use
    def _generate_crc(self, data):
        """8-bit CRC algorithm for checking data"""
        crc = _SDP31_CRC8_INIT
        # calculates 8-Bit checksum with given polynomial
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ _SDP31_CRC8_POLYNOMIAL
                else:
                    crc <<= 1
        return crc & 0xFF

    @property
    def temperature(self):
        if self._continuous_measurement is True:
            return (
                int.from_bytes(self.read_measurement("temperature"), "big")
                / self._temperature_scale_factor
            )
        return (
            int.from_bytes(self.triggered_measurement("temperature"), "big")
            / self._temperature_scale_factor
        )

    @property
    def differential_pressure(self):
        if self._continuous_measurement is True:
            return (
                int.from_bytes(self.read_measurement("pressure"), "big")
                / self._pressure_scale_factor
            )
        return (
            int.from_bytes(self.triggered_measurement("pressure"), "big")
            / self._pressure_scale_factor
        )
