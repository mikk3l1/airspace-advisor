import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import sys

from gtts import gTTS
from playsound import playsound
import pygame

sys.path.append('../advisor_pkg')


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('text_to_speech_node')

        self.tts_subscription = self.create_subscription(
            String,
            '/tts_topic',
            self.tts_callback,
            10
        )
        self.tts_subscription  # prevent unused variable warning

    tts_text = ''

    def tts_callback(self, tts_msg):
        self.get_logger().info(f'received: \n{tts_msg.data}')

        incoming_text = tts_msg.data # String coming from /tts_topic
        
        
        if self.tts_text != incoming_text:

        #     self.tts_text = incoming_text
        #     # self.tts_text.replace('error', 'fatal warning!')
        #     tts = gTTS(text=self.tts_text, lang='en') # Create a gTTS object
        #     tts.save("output.mp3") # Save the audio file
        #     playsound("output.mp3") # Play the audio file
        # else:
        #     print('not changed')


            self.tts_text = incoming_text
            x = self.tts_text
            x = x.replace('warn', 'info')
            x = x.replace('error', 'warn')
            tts = gTTS(text=x, lang='en')
            tts.save("output.mp3")

            pygame.mixer.init()

            sound1 = pygame.mixer.Sound("output.mp3")
            sound2 = pygame.mixer.Sound("notification.mp3")

            sound1.play()
            sound2.play()

            while pygame.mixer.get_busy():
                continue

            pygame.mixer.quit()
        else:
            print('not changed')

        # text = tts_msg.data # Text to be converted to speech
        # tts = gTTS(text=text, lang='en') # Create a gTTS object
        # tts.save("output.mp3") # Save the audio file
        # # playsound("output.mp3") # Play the audio file

        # with open("output.mp3", "rb") as f:
        #     print(hashlib.md5(f.read()).hexdigest())



def main(args=None):
    rclpy.init(args=args)
    
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    # test_me()
    main()