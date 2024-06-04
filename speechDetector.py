import speech_recognition as sr



class SpeechDetector:
    def __init__(self):
        self.r = sr.Recognizer()

    def detect(self):
        with sr.Microphone() as source:
            print("habla")
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
        code = -1
        if voice == "?????":
            code = 0
        elif voice == "Conectar":
            code = 1
        elif voice == "Armar":
            code = 2
        elif voice == "Despegar":
            code = 3
        elif voice == "Izquierda":
            code = 4
        elif voice == "Derecha":
            code = 5
        elif voice == "Adelante":
            code = 6
        elif voice == "Atrás":
            code = 7
        elif voice == "Arriba":
            code = 8
        elif voice == "Abajo":
            code = 9
        elif voice == "Aterrizar":
            code = 10
        elif voice == "Retornar":
            code = 11


        return code, voice
