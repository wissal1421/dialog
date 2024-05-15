# Snowboy
from .snowboy import snowboydecoder
# Dialogflow
from dialogflow_ros import DialogflowClient
# ROS
import rclpy
from rclpy import Node
from ament_index_python.packages import get_package_share_directory
# Python
import pyaudio
import signal
import time
import wave
import os


class HotwordDialogflow(Node):
    def __init__(self):
        super().__init__('hotword')
        self.interrupted = False
        self.detector = None

        # UMDL or PMDL file paths along with audio files
        pkg_path = get_package_share_directory('dialogflow_ros2')
        self.model_path = os.path.join(pkg_path,'jarvis.umdl')
        ding_path = os.path.join(pkg_path, 'ding.wav')
        # Setup df
        self.df_client = None
        # Setup audio output
        ding = wave.open(ding_path, 'rb')
        self.ding_data = ding.readframes(ding.getnframes())
        self.audio = pyaudio.PyAudio()
        self.stream_out = self.audio.open(
                format=self.audio.get_format_from_width(ding.getsampwidth()),
                channels=ding.getnchannels(), rate=ding.getframerate(),
                input=False, output=True)
        self.last_contexts = []
        self.get_logger().info("HOTWORD_CLIENT: Ready!")

    def _signal_handler(self, signal, frame):
        self.get_logger().warning("SIGINT Caught!")
        self.exit()

    def _interrupt_callback(self):
        return self.interrupted

    def play_ding(self):
        """Simple function to play a Ding sound."""
        self.stream_out.start_stream()
        self.stream_out.write(self.ding_data)
        time.sleep(0.2)
        self.stream_out.stop_stream()

    def _df_callback(self):
        self.get_logger().info("HOTWORD_CLIENT: Hotword detected!")
        self.play_ding()
        # self.df_client = DialogflowClient(last_contexts=self.last_contexts)
        df_msg, res = self.df_client.detect_intent_stream(return_result=True)
        self.last_contexts = res.output_contexts

    def start(self):
        # Register Ctrl-C sigint
        signal.signal(signal.SIGINT, self._signal_handler)
        # Setup snowboy
        self.detector = snowboydecoder.HotwordDetector(self.model_path,
                                                       sensitivity=[0.5, 0.5])
        self.df_client = DialogflowClient()
        self.get_logger().info("HOTWORD_CLIENT: Listening... Press Ctrl-C to stop.")
        while True:
            try:
                self.detector.start(detected_callback=self._df_callback,
                                    interrupt_check=self._interrupt_callback,
                                    sleep_time=0.03)
            except KeyboardInterrupt:
                self.exit()

    def exit(self):
        self.interrupted = True
        self._interrupt_callback()  # IDK man...
        self.stream_out.close()
        self.audio.terminate()
        self.detector.terminate()
        exit()

def main(args=None):
    rclpy.init(args=args)
    node = HotwordDialogflow()
    node.start()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
