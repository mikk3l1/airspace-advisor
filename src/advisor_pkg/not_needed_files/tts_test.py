# from gtts import gTTS
# import os

# # Text to be converted to speech
# text = "Hello, how are you doing?"

# # Create a gTTS object
# tts = gTTS(text=text, lang='en')

# # Save the audio file
# tts.save("src/advisor_pkg/not_needed_files/output.mp3")

# # Play the audio file
# os.system("start output.mp3")


from gtts import gTTS
from playsound import playsound
import pygame


# Text to be converted to speech
text = "Hello, how are you doing today?"

# Create a gTTS object
tts = gTTS(text=text, lang='en')

# Save the audio file
tts.save("output.mp3")

# Play the audio file
playsound("output.mp3")

pygame.mixer.init()

sound1 = pygame.mixer.Sound("output.mp3")
sound2 = pygame.mixer.Sound("notification.mp3")


sound1.play()
sound2.play()

while pygame.mixer.get_busy():
    continue

pygame.mixer.quit()