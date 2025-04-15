#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import threading
import whisper
import numpy as np
import sounddevice as sd
from scipy.io.wavfile import write
import os

# Whisper settings
MODEL = 'tiny'          # Model size: tiny, base, small, medium, large
ENGLISH = True            # Use English-only model
SAMPLE_RATE = 44100       # Audio recording frequency
BLOCK_SIZE = 30           # Block size in milliseconds
THRESHOLD = 0.1           # Minimum volume threshold
VOCALS = [125, 1000]      # Frequency range for speech detection
END_BLOCKS = 40           # Silence blocks before transcription
COMMAND_PROMPT = "Robot commands: move forward, turn left, turn right, go back, stop, speed up, slow down, rotate, navigate, home"

class Transcriber(Node):
    def __init__(self, assist=None):
        super().__init__('whisper_transcriber')
        self.callback_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(String, 'whisper_transcript', 10, callback_group=self.callback_group)

        # Simplify assistant handling
        self.asst = assist if assist else type('FakeAsst', (), {'running': True, 'talking': False, 'analyze': None})()
        self.lock = threading.Lock()
        self.running = True
        self.buffer = np.zeros((0, 1))
        self.padding = 0
        self.fileready = False

        # Load model with device detection
        print("\033[96mLoading Whisper Model..\033[0m", end='', flush=True)
        device = 'cuda' if os.path.exists('/dev/nvidia0') else 'cpu'
        self.model = whisper.load_model(f'{MODEL}{".en" if ENGLISH else ""}', device=device)
        print("\033[90m Done.\033[0m")

    def callback(self, indata, frames, time, status):
        with self.lock:
            if status:
                print(status)
            if not indata.any():
                print("\033[31mNo input or device muted.\033[0m")
                return

            # Calculate frequency and volume
            freq = np.argmax(np.abs(np.fft.rfft(indata[:, 0]))) * SAMPLE_RATE / frames
            volume = np.sqrt(np.mean(indata**2))

            # Detect speech and manage buffer
            if volume > THRESHOLD and VOCALS[0] <= freq <= VOCALS[1] and not self.asst.talking:
                print('.', end='', flush=True)
                if self.padding < 1:
                    self.buffer = indata.copy()  # Start fresh instead of using prevblock
                else:
                    self.buffer = np.concatenate((self.buffer, indata))
                self.padding = END_BLOCKS
            else:
                self.padding -= 1
                if self.padding > 1:
                    self.buffer = np.concatenate((self.buffer, indata))
                elif self.padding < 1 and self.buffer.shape[0] > SAMPLE_RATE:
                    self.fileready = True
                    write('dictate.wav', SAMPLE_RATE, self.buffer)
                    self.buffer = np.zeros((0, 1))
                elif self.padding < 1 and 0 < self.buffer.shape[0] < SAMPLE_RATE:
                    self.buffer = np.zeros((0, 1))
                    print("\033[2K\033[0G", end='', flush=True)

    def process(self):
        if not self.fileready:
            return

        print("\n\033[90mTranscribing..\033[0m")
        result = self.model.transcribe(
            'dictate.wav',
            fp16=False,
            language='en' if ENGLISH else '',
            initial_prompt=COMMAND_PROMPT
        )
        
        text = result['text']
        print(f"You said: {text}")
        
        # Publish transcription
        msg = String()
        msg.data = text
        self.publisher.publish(msg)
        
        # Optional analysis
        if self.asst.analyze:
            self.asst.analyze(text)
        
        self.fileready = False

    def listen(self):
        print("\033[32mListening.. \033[37m(Ctrl+C to Quit)\033[0m")
        with sd.InputStream(
            channels=1,
            callback=self.callback,
            blocksize=int(SAMPLE_RATE * BLOCK_SIZE / 1000),
            samplerate=SAMPLE_RATE
        ):
            while self.running and self.asst.running:
                self.process()

    def destroy_node(self):
        print("\033[93mShutting down transcriber node.\033[0m")
        self.running = False
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    transcriber = Transcriber()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(transcriber)

    listen_thread = threading.Thread(target=transcriber.listen)
    listen_thread.start()

    try:
        executor.spin()
    except (KeyboardInterrupt, SystemExit):
        transcriber.running = False
        listen_thread.join()
        transcriber.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            print("\n\033[93mQuitting..\033[0m")

if __name__ == '__main__':
    main()
