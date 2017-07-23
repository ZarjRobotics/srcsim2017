#!/usr/bin/env python
""" Team ZARJ Communications
    This module provides simple message sending and recieving
    functions to be used between the field computer and the
    operator computer.

    The messages can be any Python object
"""

import threading
import socket
import select
import sys
import Queue
import time
import struct
import traceback
from .message import zarj_marshall_message
from .message import zarj_unmarshall_message

class ZarjComm(threading.Thread):
    def __init__(self, port, host):
        """ Specify a port; if host is None, we will listen on that port.
            If host is present, we will connect to that host.  """
        self.port = port
        self.send_queue = Queue.Queue()
        self.recv_queue = Queue.Queue()
        self.stopped = False
        self.debug = False
        self.remote_socket = None
        self.connect_to = host
        self._connected = False
        self.write_in_progress = False
        self.callback = None
        self.bytes_read = 0
        self.bytes_written = 0
        threading.Thread.__init__(self)

    def push_message(self, status):
        """ Push any Python object across the wire. """
        self.send_queue.put(status)

    def pop_message(self):
        """ Retrieve a Python object, or None if none is ready """
        if self.recv_queue.empty():
            return None
        return self.recv_queue.get()

    def run(self):
        """ Standard thread entry point for the class """
        while not self.stopped:
            try:
                if self.connect_to == None:
                    self._run_listener()
                else:
                    self._connect()

            except (IOError, socket.error, socket.timeout) as e:
                print "A socket error occurred; sleeping for a second, and retrying.", e
                time.sleep(1)
                if self.debug:
                    raise e
            except Exception as e:
                print "Exception: ", e
                raise e

            time.sleep(0.1)

    def debug(self, val):
        """ This will primarily raise an exception instead of ignoring it. """
        self.debug = val

    @property
    def connected(self):
        """ Return whether or not we are connected. """
        return self._connected

    def stats(self):
        """ Return bytes read and written """
        return (self.bytes_read, self.bytes_written)

    def stop(self):
        """ Stop our thread """
        self.stopped = True
        self.join()

    def register(self, callback):
        """ Arrange to be called back when a message arrives.
            You must pass in an object which has a 'message_ready' method """
        if hasattr(callback, 'message_ready'):
            self.callback = callback
        else:
            print "Error: no method message_ready"

    def _process_reads(self):
        if not self.read_in_progress:
            header = self.remote_socket.recv(4)
            if self.debug:
                print "Got {} header bytes".format(len(header))
            if len(header) > 0 and len(header) < 4:
                extra = self.remote_socket.recv(4 - len(header))
                print "Header was short {}; extra bytes {}".format(len(header), len(extra))
                header = header + extra
            if len(header) < 4:
                print "Did not get proper header; got {}".format(len(header))
                self.remote_vanished = True
                return
            self.bytes_read += len(header)
            self.read_in_progress = True
            un = struct.unpack('!L', header)
            self.message_length = un[0]
            if self.debug:
                print "which said {} more".format(self.message_length)
            self.recv_buffer = bytearray(self.message_length)
            self.recv_view = memoryview(self.recv_buffer)
            self.to_read = self.message_length
            # We return here so we will reloop and use the
            #  select() call again to check for inbound data.
            #  otherwise, we can go right into the next recv()
            #  which can turn into a blocking read.
            return

        if self.read_in_progress:
            if self.debug:
                print "Starting to read..."
            bytes_read = self.remote_socket.recv_into(self.recv_view)
            self.bytes_read += bytes_read
            if self.debug:
                print "...read {}".format(bytes_read)
            if bytes_read == self.to_read:
                msg = zarj_unmarshall_message(self.recv_buffer)
                self.recv_queue.put(msg)
                self.read_in_progress = False
                if self.callback:
                    try:
                        self.callback.message_ready()
                    except:
                        print "Exception in callback:"
                        traceback.print_exc(file=sys.stdout)
                        print '-'*60
            else:
                self.recv_view = self.recv_view[bytes_read:]
                self.to_read = self.to_read - bytes_read
                if self.debug:
                    print "...to read now {}, recv_view size len {}, itemsize is now {}".format(self.to_read, len(self.recv_view), self.recv_view.itemsize)


    def _process_writes(self):
        bytes_written = self.remote_socket.send(self.data_to_write)
        self.bytes_written += bytes_written
        if bytes_written == len(self.data_to_write):
            self.write_in_progress = False
            self.data_to_write = False
        else:
            self.data_to_write = self.data_to_write[bytes_written:]

    def _start_write(self, data):
        self.data_to_write = data
        self.write_in_progress = True
        header = struct.pack('!L', len(data))
        self.remote_socket.sendall(header)
        if self.debug:
            print "Sending {} + {} bytes of data".format(len(header), len(data))

    def _send_and_receive(self):
        self.write_in_progress = False
        self.read_in_progress = False
        self.remote_vanished = False
        while not self.stopped and not self.remote_vanished:
            (read, _, _) = select.select([self.remote_socket.fileno()], [], [], 0.1)
            if len(read) > 0 and read[0] == self.remote_socket.fileno():
                self._process_reads()
            if self.write_in_progress:
                self._process_writes()
            elif (not self.read_in_progress) and (not self.send_queue.empty()):
                msg = self.send_queue.get()
                data = zarj_marshall_message(msg)
                self._start_write(data)

    def _be_accepting(self, sock):
        while not self.stopped:
            (ready, _, _) = select.select([sock.fileno()], [], [], 0.1)
            if len(ready) == 0 or ready[0] != sock.fileno():
                continue

            self.remote_socket, self.remote_address = sock.accept()
            self._connected = True
            self._send_and_receive()
            self._connected = False
            self.remote_socket.close()

    def _connect(self):
        self.remote_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.remote_socket.connect((self.connect_to, self.port))
        self._connected = True
        self._send_and_receive()
        self._connected = False

    def _run_listener(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', self.port))
        sock.listen(1)
        self._be_accepting(sock)
        sock.close()


#---------------------
#  The following test code can be used to start a server
#    and client.  To use, run:
#      ./sock.py
#    on the server, and
#      ./sock.py <address-of-server>
#    on the client.
#
#    Then use the menus to send and receive 'ping' messages
#---------------------
if __name__ == '__main__':

    class Ping(object):
        def __init__(self, msg, mid):
            self.msg = msg
            self.mid = mid

    class SayHi(object):
        def message_ready(self):
            print "Message for you sah."

    arg1 = None
    pid = 1
    if len(sys.argv) > 1:
        arg1 = sys.argv[1]
    zcomm = ZarjComm(2823, arg1)
    zcomm.start()
    sayhi = SayHi()
    zcomm.register(sayhi)
    while True:
        cmd = raw_input("Enter command: ")
        if cmd == 'q':
            zcomm.stop()
            break
        elif cmd == 'g':
            inbound = zcomm.pop_message()
            if inbound == None:
                print "No message"
            else:
                if isinstance(inbound, Ping):
                    print "{}: [{}]".format(inbound.mid, inbound.msg)
                else:
                    print "Message of type {}".format(type(inbound))
                    print inbound
        elif cmd == 'p':
            ping = Ping('ping', pid)
            pid = pid + 1
            print "pushing ping {}: [{}]".format(ping.mid, ping.msg)
            zcomm.push_message(ping)
            print "pushed ping"
        else:
            print "Enter q to quit, p to ping"

    zcomm.stop()
