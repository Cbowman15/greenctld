"""
greenctld.py

Driver and TCP server for the Green Heron Engineering RT-21 Digital Rotor Controller,
interfacing with the Alfa Spid RAS az/el mount used in the TeraLink-1 ground station.

The RT-21 communicates over RS-232 (or USB-serial) using the DCU-1 superset protocol
at 4800 8N1. Two separate RT-21 units are used: one for azimuth, one for elevation.

The TCP server exposes a subset of the rotctld protocol so that antenna-tracking
software (e.g., GPredict) can command the mount over a network socket.

Protocol references:
  - RT-21 User Guide Rev 1.3a (https://www.greenheronengineering.com/wp-content/uploads/2019/08/RT-21_Manual_1_3.pdf), Appendix G (Communications Protocol) and Appendix A.8 (SPID rotor setup)
"""

import argparse
import os
import re
import select
import socket
import sys
import time
import traceback

import serial


class DummyRotor(object):
    """
    Software stub that mimics the GreenHeronRotor interface.

    Useful for testing the TCP server and upstream software (e.g., GPredict)
    without a physical RT-21 connected. Position converges in 4-degree steps
    per set_pos() call, which loosely models the latency of a real mount.
    """

    # Starting position (degrees). Arbitrary non-zero values to make
    # convergence behaviour visible during testing.
    az = 50
    el = 69

    def set_pos(self, az, el):
        """Step az/el toward the target by 4 degrees each call."""
        print('==> Setting position to {},{}'.format(az, el))
        if self.az > az:
            self.az -= 4
        elif self.az < az:
            self.az += 4

        if self.el > el:
            self.el -= 4
        elif self.el < el:
            self.el += 4

    def get_pos(self):
        """Return (az, el) tuple."""
        print('<== Current position is {},{}'.format(self.az, self.el))
        return (self.az, self.el,)

    def stop(self):
        print("==> Stop")


class GreenHeronRotor(object):
    """
    Serial driver for the Green Heron Engineering RT-21 Digital Rotor Controller.

    Two RT-21 units are used in the TeraLink-1 ground station — one controlling
    the azimuth axis and one controlling the elevation axis of the Alfa Spid RAS
    mount. Each RT-21 is connected via its RS-232 (or USB) port to the host PC
    as a separate serial device.

    RT-21 serial settings (fixed by the device): 4800 baud, 8N1.

    The RT-21 is configured with OPTION = SPID (Appendix A.8), which sets the
    pulse divider to 360 (1 pulse/degree) and uses the pulse-counter position
    feedback of the Alfa Spid RAS.

    Command summary used here (RT-21 User Guide, Appendix G):
      AP1<xxx.y>\\r;   — set target heading to xxx.y degrees (tenths supported)
      BI1;             — query current heading; response is "xxx.y;" (6 chars)
      ;                — stop current rotation immediately
    """

    az_serial = None  # Serial port object for the azimuth RT-21
    el_serial = None  # Serial port object for the elevation RT-21

    def __init__(self, az_device, el_device, baud, timeout):
        """
        Open serial connections to both RT-21 units.

        Args:
            az_device: OS path to the azimuth serial device (e.g., '/dev/ttyUSB0').
            el_device: OS path to the elevation serial device (e.g., '/dev/ttyUSB1').
            baud:      Baud rate — must be 4800 for RT-21 DCU-1 protocol.
            timeout:   Read timeout in seconds. 1.5 s is a safe default; increase
                       for slow mounts or long cable runs.
        """
        self.az_serial = serial.Serial(
            az_device, baudrate=baud, timeout=timeout)
        self.el_serial = serial.Serial(
            el_device, baudrate=baud, timeout=timeout)
        print('--- Serial timeout set to', timeout)

    def stop(self):
        """
        Send an immediate stop command to both RT-21 units.

        The RT-21 interprets a bare semicolon (';') as a stop/abort,
        applying a short ramp-down before halting motor power.
        (RT-21 User Guide, Appendix G: ';' command)
        """
        print("==> Stop")
        self.az_serial.write(b';')
        self.el_serial.write(b';')

    def set_pos(self, az, el):
        """
        Command both RT-21 units to rotate to the specified az/el heading.

        Uses the AP1 (Antenna Position) command with 0.1-degree resolution.
        Format: 'AP1<xxx.y>\\r;'  e.g., 'AP1045.5\\r;' for 45.5 degrees.

        The RT-21 begins movement immediately upon receiving the command and
        applies configured ramp-up/ramp-down profiles autonomously.

        Args:
            az: Target azimuth in degrees (float), 0–359.9.
            el: Target elevation in degrees (float), 0–90.
        """
        print(f'==> Setting position to {az:05.1f},{el:05.1f}')
        self.az_serial.write(f'AP1{az:05.1f}\r;'.encode())
        self.el_serial.write(f'AP1{el:05.1f}\r;'.encode())
        # Brief pause to avoid flooding the RT-21 serial buffer before a
        # subsequent get_pos() call flushes input.
        time.sleep(0.1)

    def __parse_response(self, buf):
        """
        Parse a position response string from the RT-21.

        The RT-21 BI1 response format is 'xxx.y;' or ' xx.y;' (leading spaces
        for values under 100 degrees). This method extracts the numeric value.

        Args:
            buf: Raw decoded string read from the serial port.

        Returns:
            Position in degrees as a float, or -1 on parse failure.
        """
        print("---buf----", buf)
        # Pattern matches 1–3 digits, a decimal point, one digit, optional semicolon.
        match = re.match(r'^\s*(\d{1,3}.\d{1});?$', buf)
        if not match:
            return -1
        ret = match.groups()[0]
        ret = float(ret)
        return ret

    def get_pos(self):
        """
        Query both RT-21 units for their current reported heading.

        Issues the BI1 (Bearing Inquiry) command to each unit.
        The RT-21 responds with the current heading as 'xxx.y;' (6 bytes).

        Note: Input buffers are flushed before querying to discard any stale
        data that may have accumulated since the last call.

        Returns:
            (az, el) tuple of floats on success.
            False if the serial read fails or the response cannot be parsed.
        """
        # Flush stale bytes that may have accumulated in OS serial buffers.
        self.az_serial.flushInput()
        self.el_serial.flushInput()

        # BI1; — "Bearing Inquiry" command (RT-21 User Guide, Appendix G).
        self.az_serial.write(b'BI1;')
        self.el_serial.write(b'BI1;')

        # Read exactly 6 bytes: format is 'xxx.y;'
        # (3 digits + decimal point + 1 digit + semicolon terminator)
        az_buf = self.az_serial.read(6).decode()
        el_buf = self.el_serial.read(6).decode()
        print(f'az_buf: {az_buf}, el_buf: {el_buf}')

        az = self.__parse_response(az_buf)
        el = self.__parse_response(el_buf)

        if az < 0 or el < 0:
            print('!!! Failed to parse response, received {} and {}'.format(
                repr(az_buf), repr(el_buf)))
            return False

        print('<== Current position is {},{}'.format(az, el))
        return (az, el)


class TCPServer(object):
    """
    TCP server exposing a subset of the rotctld (Hamlib) protocol.

    rotctld is the standard rotor control daemon used by GPredict and other
    satellite tracking applications. This server implements the three commands
    that GPredict issues, plus a stop command:

        'p'          — query current az/el position
        'P <az> <el>'— command a move to the specified az/el
        'S'          — stop current rotation
        'q'          — close the connection

    Multiple simultaneous clients are supported via select()-based I/O
    multiplexing (non-blocking).

    Default port: 4533 (standard rotctld port).
    """

    # Maps each connected client socket to its pending receive buffer.
    # Commands may arrive fragmented across multiple TCP segments; partial
    # data is held here until a newline delimiter is received.
    client_buf = {}

    def __init__(self, port, rotor, ip=''):
        """
        Bind and listen on the specified TCP port.

        Args:
            port:  TCP port to listen on (default 4533).
            rotor: A GreenHeronRotor (or DummyRotor) instance to drive.
            ip:    Bind address. Empty string binds to all interfaces (0.0.0.0).
        """
        self.rotor = rotor
        self.listener = socket.socket()
        # SO_REUSEADDR avoids 'address already in use' errors on restart.
        self.listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.listener.bind((ip, port))
        self.listener.listen(4)
        addr = self.listener.getsockname()
        print(
            '--- Listening for connections on {}:{}'.format(addr[0], addr[1]))

    def close_client(self, fd):
        """
        Close a client connection and issue a motor stop as a safety measure.

        Stopping on disconnect prevents the mount from continuing a commanded
        move if the controlling application crashes or disconnects unexpectedly.
        """
        self.rotor.stop()
        try:
            fd.close()
            del self.client_buf[fd]
        except:
            pass

    def parse_client_command(self, fd, cmd):
        """
        Parse and dispatch a single rotctld command from a client.

        rotctld response codes used here:
          'RPRT 0'   — success
          'RPRT -1'  — out-of-range argument
          'RPRT -4'  — unknown command
          'RPRT -6'  — communication error (rotor did not respond)
          'RPRT -8'  — parse error (malformed argument)

        Args:
            fd:  Client socket.
            cmd: Command string (stripped of leading/trailing whitespace).
        """
        cmd = cmd.strip()

        if cmd == '':
            return

        print('<-- Received command: {}'.format(repr(cmd)))

        # 'q' — client requests connection close
        if cmd == 'q':
            self.close_client(fd)
            return

        # 'S' — stop rotation immediately
        if cmd == 'S':
            self.rotor.stop()
            print('--> RPRT 0')
            fd.send(b'RPRT 0\n')
            return

        # 'p' — return current az/el position
        # Response format (rotctld): two lines, one float per line: "az\nel\n"
        if cmd == 'p':
            pos = self.rotor.get_pos()
            if not pos:
                print('--> RPRT -6')
                fd.send(b'RPRT -6\n')
            else:
                az, el = pos
                print('--> {},{}'.format(az, el))
                fd.send('{:.6f}\n{:.6f}\n'.format(az, el).encode())
            return

        # 'P <az> <el>' — command a move to az/el
        # GPredict sends floating-point values; we truncate to integer degrees
        # since the RT-21's practical accuracy with the Alfa Spid is ~1 degree.
        match = re.match(r'^P\s+([\d.]+)\s+([\d.]+)$', cmd)
        if match:
            az = match.groups()[0]
            el = match.groups()[1]
            try:
                az = int(float(az))
                el = int(float(el))
            except:
                print('--> RPRT -8 (could not parse)')
                fd.send(b'RPRT -8\n')
                return

            # rotctld uses 360 to mean North (same as 0) but AP1 command
            # range is 0–359; remap 360 -> 359 to avoid RT-21 out-of-range error.
            if az == 360:
                az = 359

            # RT-21 azimuth range: 0–359 degrees (OPTION=SPID, 360 pulses/rev)
            if az > 359:
                print('--> RPRT -1 (az too large)')
                fd.send(b'RPRT -1\n')
                return

            # Alfa Spid RAS elevation range: 0–90 degrees
            if el > 90:
                print('--> RPRT -1 (el too large)')
                fd.send(b'RPRT -1\n')
                return

            self.rotor.set_pos(az, el)
            print('--> RPRT 0')
            fd.send(b'RPRT 0\n')
            return

        # Any other command is not implemented
        print('--> RPRT -4 (unknown command)')
        fd.send(b'RPRT -4\n')

    def read_client(self, fd):
        """
        Read available data from a client socket and process complete commands.

        Commands are newline-delimited. Partial commands are held in client_buf
        until the delimiter arrives, accommodating TCP stream fragmentation.
        """
        buf = fd.recv(1024).decode()

        if len(buf) == 0:
            # Zero-length read indicates the client closed the connection (EOF).
            print('<-- EOF')
            self.close_client(fd)
            return

        self.client_buf[fd] += buf

        while self.client_buf[fd]:
            # Partition on newline: cmd is the next complete command,
            # tail is any remaining data for future processing.
            cmd, sep, tail = self.client_buf[fd].partition('\n')

            if not sep:
                # No newline found yet — treat entire buffer as one command
                # and process it immediately (handles clients that don't
                # terminate commands with '\n').
                cmd = self.client_buf[fd]
                self.client_buf[fd] = ''
            else:
                self.client_buf[fd] = tail

            self.parse_client_command(fd, cmd)

            # parse_client_command may have removed fd from client_buf ('q' command)
            if fd not in self.client_buf:
                return

        print('<-- Waiting for more data')

    def __run_once(self):
        """
        Single iteration of the select() event loop.

        Monitors the listening socket for new connections and all active
        client sockets for incoming data. Exceptions on a client connection
        are caught, logged, and cleaned up without taking down the server.
        """
        rlist = [self.listener] + list(self.client_buf.keys())
        wlist = []
        xlist = []

        rlist, wlist, xlist = select.select(rlist, wlist, xlist)

        for fd in rlist:
            if fd == self.listener:
                # New client connecting
                new_fd, addr = self.listener.accept()
                new_fd.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 1024*16)
                new_fd.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1024*16)
                new_fd.setblocking(False)
                self.client_buf[new_fd] = ''
                print('<-- Connect {}:{}'.format(addr[0], addr[1]))

            else:
                # Data available from an existing client
                try:
                    self.read_client(fd)
                except Exception:
                    print(
                        'Unhandled exception, killing client and issuing motor stop command:')
                    traceback.print_exc()
                    self.close_client(fd)

            print()

    def loop(self):
        """Run the event loop indefinitely."""
        while True:
            self.__run_once()


if __name__ == '__main__':
    # Force line-buffered stdout so log output appears immediately even when
    # the process is run under nohup or piped to a log file.
    sys.stdout = os.fdopen(sys.stdout.fileno(), 'w', buffering=1)

    parser = argparse.ArgumentParser(
        description='RT-21 rotctld-compatible TCP server for the TeraLink-1 ground station.')
    parser.add_argument('--az-device', '-a', type=str,
                        required=False, help='Serial device for azimuth RT-21 (e.g., /dev/ttyUSB0)')
    parser.add_argument('--el-device', '-e', type=str,
                        required=False, help='Serial device for elevation RT-21 (e.g., /dev/ttyUSB1)')
    parser.add_argument('--speed', '-s', type=int,
                        default=4800, help='Serial baud rate (must be 4800 for RT-21)')
    parser.add_argument('--timeout', '-t', type=float,
                        default=1.5, help='Serial read timeout in seconds')
    parser.add_argument('--port', '-p', type=int,
                        default=4533, help='TCP port to listen on (default: 4533, rotctld standard)')
    parser.add_argument('--get-pos', '-g', action='store_true',
                        help='Query position once and exit (serial comms sanity check)')
    parser.add_argument('--dummy', action='store_true',
                        help='Use DummyRotor instead of real hardware (no serial required)')
    args = parser.parse_args()

    if args.dummy:
        rotor = DummyRotor()
    else:
        rotor = GreenHeronRotor(
            args.az_device, args.el_device, args.speed, args.timeout)

    if args.get_pos:
        # One-shot position query for verifying serial connectivity
        print(rotor.get_pos())
        sys.exit(0)

    server = TCPServer(args.port, rotor)
    server.loop()
