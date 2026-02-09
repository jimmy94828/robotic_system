# file: audio_command_node.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import sounddevice as sd
import threading
import keyboard  # pip install keyboard
from stretch.audio.speech_to_text import WhisperSpeechToText


class AudioCommandNode(Node):
    """
    Press SPACE to record a short audio command, run Stretch AI speech-to-text,
    and publish the recognized command to /nl_command.
    """

    def __init__(self):
        super().__init__("audio_command_node")

        # ---- Publisher ----
        self.pub = self.create_publisher(String, "/nl_command", 10)

        # ---- Initialize STT model ----
        self.get_logger().info("🔄 Loading Whisper speech-to-text model...")
        self.stt = WhisperSpeechToText()
        self.get_logger().info("✅ Whisper model ready. Press SPACE to talk.")


        # ---- Audio parameters ----
        self.sample_rate = 16000
        self.record_secs = 5.0

        # ---- Start hotkey thread ----
        threading.Thread(target=self._hotkey_loop, daemon=True).start()

    # ======================================================
    def _hotkey_loop(self):
        """Wait for spacebar to trigger audio recording."""
        self.get_logger().info("⌨️  Waiting for SPACE key to record...")
        while rclpy.ok():
            keyboard.wait('space')
            self._listen_once()

    # ======================================================
    def _listen_once(self):
        """Record one audio clip and transcribe it."""
        try:
            self.get_logger().info("🎤 Recording... (speak now)")
            audio = sd.rec(
                int(self.record_secs * self.sample_rate),
                samplerate=self.sample_rate,
                channels=1,
                dtype=np.float32,
            )
            sd.wait()
            waveform = np.squeeze(audio)

            self.get_logger().info("🧠 Transcribing...")
            text = self.stt.transcribe(waveform, sample_rate=self.sample_rate).strip()
            if text:
                msg = String()
                msg.data = text.lower()
                self.pub.publish(msg)
                self.get_logger().info(f"🗣️ Recognized: {msg.data}")
            else:
                self.get_logger().warn("🤔 No recognizable speech.")
        except Exception as e:
            self.get_logger().error(f"❌ Audio/STT error: {e}")


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
