# Copyright 2021 AUTHORS
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the AUTHORS nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


# Connector for UART integration of the BNO-055
# See also https://pyserial.readthedocs.io/en/latest/pyserial_api.html
import sys
import serial

from rclpy.node import Node

from bno055 import registers
from bno055.connectors.Connector import Connector


class UART(Connector):
    """Connector implementation for serial UART connection to the sensor."""

    CONNECTIONTYPE_UART = 'uart'
    MAX_RETRIES = 10

    def __init__(self, node: Node, baudrate, port, timeout, uart_first_bytes_timeout):
        """Initialize the UART class.

        :param node: a ROS node
        :param baudrate: baudrate to configure UART communication to
        :param port: UART port to connect to
        :param timeout: timeout to use
        :return:
        """
        super().__init__(node)

        self.node = node
        self.baudrate = baudrate
        self.port = port
        self.timeout = timeout
        self.uart_first_bytes_timeout = uart_first_bytes_timeout
        self.serialConnection = None

    def connect(self):
        """Connect to the sensor.

        :return:
        """
        self.node.get_logger().info('Opening serial port: "%s"...' % self.port)

        try:
            self.serialConnection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except serial.serialutil.SerialException:
            self.node.get_logger().info('Unable to connect to IMU at port ' + self.port)
            self.node.get_logger().info('Check to make sure your device is connected')
            sys.exit(1)

    def read(self, reg_addr, length):
        """Read data from sensor via UART.

        :param reg_addr: The register address
        :param length: The data length
        :return:
        """
        self.serialConnection.timeout = self.uart_first_bytes_timeout

        buf_out = bytearray()
        buf_out.append(registers.COM_START_BYTE_WR)
        buf_out.append(registers.COM_READ)
        buf_out.append(reg_addr)
        buf_out.append(length)

        for attempt in range(0, self.MAX_RETRIES):
            try:
                self.serialConnection.write(buf_out)
                buf_in = bytearray(self.serialConnection.read(2))
            except Exception as e:  # noqa: B902
                self.node.get_logger().error('Transmission error: %s' % e)
                continue

            # Check for valid response length (the smallest (error) message has at least 2 bytes):
            if buf_in.__len__() < 2:
                self.node.get_logger().error('Unexpected length of READ-request response: %s'
                                             % buf_in.__len__())
                continue

            # Check for READ result (success or failure):
            if buf_in[0] == registers.COM_START_BYTE_ERROR_RESP:
                # Error 0x07 (BUS_OVER_RUN_ERROR) can be "normal" if data fusion is not yet ready
                if buf_in[1] == 7:
                    # see #5
                    self.node.get_logger().debug('Data fusion not ready, resend read request')
                    continue
                else:
                    self.node.get_logger().error('READ-request failed with error code %s'
                                                 % hex(buf_in[1]))
                    continue

            # Check for correct READ response header:
            if buf_in[0] != registers.COM_START_BYTE_RESP:
                self.node.get_logger().error('Wrong READ-request response header %s'
                                             % hex(buf_in[0]))
                continue

            self.serialConnection.timeout = self.timeout
            try:
                buf_in.extend(self.serialConnection.read(length))
            except Exception as e:  # noqa: B902
                self.node.get_logger().error('Transmission error: %s' % e)
                continue

            if (buf_in.__len__()-2) != buf_in[1]:
                self.node.get_logger().error('Payload length mismatch detected: '
                                             + '  received=%s, awaited=%s'
                                             % (buf_in.__len__()-2, buf_in[1]))
                continue

            # Check for correct READ-request response length
            if buf_in.__len__() != (2 + length):
                self.node.get_logger().error('Incorrect READ-request response length: %s'
                                             % (2 + length))
                continue

            # remove the 0xBB:
            buf_in.pop(0)

            # remove the length information:
            buf_in.pop(0)

            # Return the received payload:
            return buf_in

        raise Exception(f'Terminating, number of tries in serial read function \
                        exceeded max_tries ({self.MAX_RETRIES}).')

    def write(self, reg_addr, length, data: bytes):
        """
        Transmit data packages to the sensor.

        :param reg_addr: The register address
        :param length: The data length
        :return:
        """
        self.serialConnection.timeout = self.timeout

        buf_out = bytearray()
        buf_out.append(registers.COM_START_BYTE_WR)
        buf_out.append(registers.COM_WRITE)
        buf_out.append(reg_addr)
        buf_out.append(length)
        buf_out += data

        for attempt in range(0, self.MAX_RETRIES):
            try:
                self.serialConnection.write(buf_out)
                buf_in = bytearray(self.serialConnection.read(2))
            except Exception as e:  # noqa: B902
                self.node.get_logger().error('Write exception: %s' % e)
                continue

            if (buf_in.__len__() != 2) or (buf_in[1] != 0x01):
                self.node.get_logger().error(f'Incorrect Bosh IMU device response. buf_in: \
                                             {buf_in}')
                continue

            return True

        self.node.get_logger().error(f'Number of tries in serial write function exceeded \
                                     max_tries ({self.MAX_RETRIES}).')
        return False
