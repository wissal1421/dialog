import rclpy
from rclpy import Node
import pyaudio
import socket
import select


class AudioServer(Node):
    def __init__(self):
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        CHUNK = 4096
        self.audio = pyaudio.PyAudio()
        self.stream = self.audio.open(format=FORMAT, channels=CHANNELS, rate=RATE,
                                      input=True, frames_per_buffer=CHUNK,
                                      stream_callback=self._callback)
        self.serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.read_list = [self.serversocket]

        self._server_name = self.get_parameter_or('/dialogflow_client/server_name',
                                            '127.0.0.1')
        self._port = self.get_parameter_or('/dialogflow_client/port', 4444)

        self.get_logger().info("DF_CLIENT: Audio Server Started!")

    def _connect(self):
        """Create a socket, listen on the server:port and wait for a connection.
        """
        self.serversocket.bind((self._server_name, self._port))
        self.get_logger().info("DF_CLIENT: Waiting for connection...")
        self.serversocket.listen(1)

    def _callback(self, in_data, frame_count, time_info, status):
        """PyAudio callback to continuously get audio data from the mic and put
        it in a buffer.
         :param in_data: Audio data received from mic.
         :return: A tuple with a signal to keep listening to audio input device
         :rtype: tuple(None, int)
        """
        for s in self.read_list[1:]:
            s.send(in_data)
        return None, pyaudio.paContinue

    def start(self):
        """Main function that attempts to create a socket and establish a
        connection with a client."""
        self._connect()
        try:
            while True:
                # select() waits until an object is readable. Here this means
                # it will wait until there is data to be read from the socket
                readable, writable, errored = select.select(self.read_list, [], [])
                for s in readable:
                    if s is self.serversocket:
                        (clientsocket, address) = self.serversocket.accept()
                        self.read_list.append(clientsocket)
                        self.get_logger().info("DF_CLIENT: Connection from {}".format(
                                address))
                    else:
                        data = s.recv(1024)
                        if not data:
                            self.read_list.remove(s)
        except KeyboardInterrupt as k:
            self.get_logger().warn("DF_CLIENT: Caught Keyboard Interrupt: {}".format(k))
        except socket.error as e:
            self.get_logger().warn("DF_CLIENT: Caught Socket Error: {}\n "
                          "Restarting...".format(e))
            self._connect()

        self.get_logger().info("DF_CLIENT: Finished recording")

    def __del__(self):
        self.serversocket.close()
        # Stop Recording
        self.stream.stop_stream()
        self.stream.close()
        self.audio.terminate()

def main(args=None):
    rclpy.init(args=args)
    a = AudioServer()
    a.start()


if __name__ == "__main__":
    main()
