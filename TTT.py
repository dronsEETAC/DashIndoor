#tool to talk
import threading
import time

import speech_recognition as sr
from gtts import gTTS
import os
import subprocess
import pygame as pygame

''' Esta es la clase que me permite comunicarme por voz
Por una parte es capaz de reconocer el conjunto de palabras que le indiquemos (speach to text)
Por otra parte es capaz de pronunciar la frase que le indiquemos (text to speech)
 Es importante tener en cuenta que para poder usar esta funcionalidad el ordenador dee estar conectado a internet '''
class TTT:
    def __init__(self, words):
        self.r = sr.Recognizer()
        # me guardo las palabras que tengo que reconocer
        self.words = words
        self.talking = False

    def listeningThread(self, callback):

        while self.talking:
            print ('espero')
            code, voice = self.detect()
            callback(code, voice)
            print ('has dicho ', voice)
            time.sleep(1)

    def startListening (self, callback):
        self.talking = True
        myThread= threading.Thread (target = self.listeningThread, args = [callback])
        myThread.start()


    def stopListening (self):
        print ('paro escucha')
        self.talking = False

    # detecta si la palabra pronunciada está entre las indicadas
    def detect(self):
        with sr.Microphone() as source:
            audio = self.r.listen(source, phrase_time_limit=5)
            try:
                # for testing purposes, we're just using the default API key
                # to use another API key, use `r.recognize_google(audio, key="GOOGLE_SPEECH_RECOGNITION_API_KEY")`
                # instead of `r.recognize_google(audio)`
                voice = self.r.recognize_google(audio, language="es-ES")
            except sr.UnknownValueError:
                voice = "?????"
            voice = voice.capitalize()
            print("has dicho ", voice)
        if voice in self.words:
            code = self.words.index(voice)
            # retorno la palabra pronunciada y el indice de esa palabra en el vector
            return code, voice
        elif voice == "?????":
            return -1, None
        else:
            return -2, None

    # pronuncia la frase que recibe como parámetro
    def talk (self, sentence):
        tts = gTTS(text=sentence, lang='es')
        # salvo el audio en un fichero
        tts.save("tecsify.mp3")

        # Inicializar pygame
        pygame.init()

        # Ocultar la ventana de pygame
        pygame.display.set_mode((1, 1))

        # Reproducir el archivo de audio
        pygame.mixer.music.load("tecsify.mp3")
        pygame.mixer.music.play()

        # Esperar a que termine la reproducción
        while pygame.mixer.music.get_busy():
            pygame.time.Clock().tick(10)

        # Detener pygame
        pygame.quit()
        # elimino el fichero generado
        os.remove("tecsify.mp3")

