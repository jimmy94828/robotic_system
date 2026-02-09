import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
import time
import re
from stretch.audio.speech_to_text import WhisperSpeechToText


class AudioCommandNode(Node):
    """
    Continuously records short audio chunks, transcribes with Whisper,
    and publishes recognized text to /nl_command.
    """

    def __init__(self):
        super().__init__("audio_command_node")

        # Publisher
        self.pub = self.create_publisher(String, "/manual_command", 10)

        # Whisper model
        self.get_logger().info("🔄 Loading Whisper speech-to-text model...")
        self.stt = WhisperSpeechToText()
        self.get_logger().info("✅ Whisper model ready. Listening continuously...")

        # Audio parameters
        self.sample_rate = 16000
        self.record_secs = 5.0  # each capture duration
        self.silence_delay = 1.0  # wait before next capture

        # Start the continuous listening loop
        self.create_timer(self.record_secs + self.silence_delay, self.listen_once)

    # =====================================================
    def listen_once(self):
        """Record and transcribe one short segment."""
        self.get_logger().info("🎤 Listening...")

        try:
            audio = sd.rec(
                int(self.record_secs * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype=np.float32,
            )
            sd.wait()
            waveform = np.squeeze(audio)

            text = self.stt.process_audio(waveform).strip().lower()
            text = re.sub(r'[^\w\s]', '', text)

            if text:
                msg = String()
                msg.data = text
                self.pub.publish(msg)
                self.get_logger().info(f"🗣️ Recognized (cleaned): {msg.data}")
            else:
                self.get_logger().warn("🤔 No recognizable speech.")

        except Exception as e:
            self.get_logger().error(f"❌ STT error: {e}")


def main():
    rclpy.init()
    node = AudioCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
