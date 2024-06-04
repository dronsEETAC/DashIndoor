import json
import math
import threading
import tkinter as tk
from Dron import Dron
from tkinter import messagebox
from tkinter.simpledialog import askstring
from TTT import *
from PIL import Image,ImageTk

class Conversor:
    # La conversion entre posiciones y coordenadas tiene esta dificultad: las coordenadas
    # corresponden al plano XY. Desplazamientos en la dimensión X corresponden a movimiento Este-Oeste
    # mientras que desplazamientos en la dimensión Y corresponden a movimiento en el eje norte-sur
    # En cambio la posición en la que está el dron está en formato NED, es decir posicion[0] es el desplazamiento
    # en el eje Norte-sur respecto al home y posicion[1] es desplazamiento en eje este-oeste y posicion[2] es
    # desplazamiento en el eje down-up
    def setUp(self, metrosX, metrosY, pixelsX, pixelsY):
        self.metrosX = metrosX
        self.metrosY = metrosY
        self.pixelsX = pixelsX
        self.pixelsY = pixelsY
        self.center = (pixelsX//2, pixelsY//2)
    def convertToPosition (self, x,y):
        difX = x - self.center[0]
        difY = self.center[1] -y
        posY = (difX*self.metrosX)/self.pixelsX
        posX = (difY*self.metrosY)/self.pixelsY
        return posX, posY

    def convertToCoord (self, posX, posY, initialHeading):
        x = (self.pixelsX*posY)/self.metrosX + self.center[0]
        y = (self.pixelsY*posX)/self.metrosY - self.center[1]

        # las siguientes operaciones son las necesarias para realizar el cambio de coordenadas
        # del espacio NED al espacio XY. Ese cambio de coordenadas requiere conocer el heading
        # inicial (el angulo que formaba el dron respecto al norte en el momento de la conexión)

        angle = math.radians(initialHeading)
        dx = x - self.center[0]
        dy = -y - self.center[1]
        xp = dx*math.cos(angle)+ dy*math.sin(angle) + self.center[0]
        yp = -dx*math.sin(angle) + dy*math.cos(angle) + self.center[1]
        x,y = xp, -yp
        return int(x), -int (y)


'''cuando el dron llegue al punto de destino se ejecuta esto.
simplemente elimino el circulo azul con el que marqué el destino'''
def enDestino ():
    global mapa, destination
    mapa.delete (destination)


'''Esta es la función que se ejecuta cuando clico sobre el area de vuelo con el boton derecho para
hacer que el dron se dirija a ese punto '''
def goHere (event):
    global conversor
    global dronIcon, mapa, iconSize
    global dron, area
    global menu, destination, takeOffBtn
    # muestro el menú en el punto en el que he clicado
    menu.post(event.x_root, event.y_root)

    if takeOffBtn ['bg'] == 'green':
        # solo hago caso si el dron ya ha despegado
        # obtendo las coordenadas donde he clicado y las convierto a la posición a la que tengo que ir
        posX,posY = conversor.convertToPosition(event.x, event.y)
        # llamo a la función no bloqueante (no modifico la altura)
        # ajusto la velocidad de navegación según la elegida por el usuario
        dron.setNavSpeed(float (navSpeedSldr.get()))
        dron.moveto ((posX,posY, dron.alt), blocking = False, callback = enDestino)
        # ponto en gris los limites del area de vuelo (por si estaban en rojo en ese momento)
        mapa.itemconfig(area, outline='grey', width=10)
        # dibujo un pequeño circulo azul para marcar el destino
        destination = mapa.create_oval(event.x - 5, event.y -5 , event.x + 5, event.y + 5 , fill='blue')
    else:
        # notifico el error (con voz o con message box)
        if talking:
            ttt.talk("El dron no está volando")
        else:
            messagebox.showerror(title=None, message="El dron no está volando")


'''cuando el usuario ha definido las dimensiones del area de vuelo venimos aqui para 
crear ese espacio en la interfaz gráfica'''
def crearEspacio ():
    global dimXSldr, dimYSldr, dimZSldr, alturaSldr, takeOffAltSldr
    global ventana
    global mapa, dronIcon, iconSize, arrow
    global conversor
    global menu
    global height
    global dron
    global canvasSize, area, areaSize
    global controlFrame, scenarioFrame
    global schema
    global var1, var2
    global altura

    # elimino la imagen informativa inicial
    mapa.delete (schema)
    # obtengo las dimensiones elegidas por el uruario
    dimE_O = int (dimXSldr.get())
    dimN_S = int (dimYSldr.get())
    altura = int (dimZSldr.get())

    # configuro el geofence local con esas dimensiones
    dron.setLocalGeofence(dimN_S,dimE_O,altura)

    # elimino el frame inicial con el que se elegian las dimensiones.
    scenarioFrame.grid_forget()
    # ahora muestro el frame con los botones para controlar del dron
    controlFrame.grid(row=0, column=0, padx=5, pady=3, sticky=tk.N  + tk.E + tk.W)
    # configuro el slider que me mostrará la altura del dron
    alturaSldr.config(from_=altura, to=0)
    alturaSldr.grid(row=0, column=0, padx=5,pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    # configuro el slider con el que elegire la altura de despegue (como maximo la altura del espacio de vuelo
    takeOffAltSldr.config(from_=0, to=altura)
    takeOffAltSldr.set(3) # por defecto despegará a 3 metros de altura
    takeOffAltSldr.grid(row=2, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)


    # dibujo el rectangulo que define el area de vuelo
    areaSize = canvasSize - 60
    height = (areaSize*dimN_S)//dimE_O
    area = mapa.create_rectangle(0, 0, areaSize, height, fill='DeepPink2', stipple="gray12")
    # marco el negro los límites del area de vuelo (se pondrán en rojo en caso de breach del geofence local
    mapa.itemconfig(area, outline='black', width=5)

    # este es el menú que se mostrará al hacer click en el boton derecho del raton para señalar el punto
    # al que quiero enviar el dron
    menu = tk.Menu(ventana, tearoff=0)
    menu.add_command(label="Vuela aquí")
    # aquí indico que función hay que ejecutar al clicar en el botón derecho del raton
    mapa.bind('<Button-3>', goHere)

    # este será el tamaño del icono que representa al dron en el espacio de vuelo
    iconSize = 20
    # dibujo el circulo que representará al dron, en el centro del espacio de vuelo
    # cuando el usuario se conecte ese círculo se llenará de rojo y se le añadirá la flecha que indica el heading
    homeIcon = mapa.create_oval(areaSize//2-iconSize//2, height//2 - iconSize//2, areaSize//2+iconSize//2, height//2 + iconSize//2, outline='red',  width=2)
    # creo el conversor que me convertirá las posiciones del dron a coordenadas del rectangulo que representa el espacio de vuelo, y viceversa
    conversor = Conversor()
    # doy al conversor los datos que necesitará
    conversor.setUp (dimE_O, dimN_S, areaSize, height)



''' dibujo el dron con la flecha que indica el heading'''
def dibuja_dron ():
    global height, iconSize, arrowLength
    global dronIcon, dronHeading, areaSize

    x, y = areaSize//2, height//2
    arrowLength = 40
    dronIcon = mapa.create_oval(x - iconSize // 2, y - iconSize // 2, x + iconSize // 2,
                                y + iconSize // 2, fill='red')

    # inicialmente el dron apunta hacia delante
    dronHeading= mapa.create_line(
        x, y, x, y + arrowLength,
        fill="red", arrow=tk.LAST)


''' dibuja el dron en la posición x,y del canvas'''
def mueve_dron (x,y):
    global mapa, dronIcon, iconSize, dronHeading
    # cambio las coordenadas del ovalo que representa el dron
    mapa.coords(dronIcon, x - iconSize // 2, y - iconSize // 2, x + iconSize // 2,
                                y + iconSize // 2)
    # y las coordenadas coorespondientes a la flecha que representa el heading
    x1, y1, x2, y2 = mapa.coords(dronHeading)
    dx = x - x1
    dy = y - y1
    mapa.move(dronHeading, dx, dy)

''' llamo aquí para cambiar de orientación la flecha que indica el heading del dron'''
def cambiar_orientacion(angulo):

    global mapa, arrowLength, dronHeading, initialHeading

    x, y, x2, y2 = mapa.coords(dronHeading)
    # Convertir el ángulo de grados a radianes, teniendo en cuenta el angulo inicial
    angulo_rad = math.radians(angulo -initialHeading)

    # Calcular las coordenadas del punto final de la línea
    x_final = x + arrowLength * math.sin(angulo_rad)
    y_final = y - arrowLength * math.cos(angulo_rad)
    mapa.coords(dronHeading, x, y, x_final, y_final)

''' vengo aquí cada vez que se recibe un paquete de telemetría local'''
def process_local_telemetry_info (local_telemetry_info):
    global mapa, dronIcon, alturaSldr, iconSize, headingArrow
    global connectBtn
    global talking
    global var2, dron
    global initialHeading

    # si estaba en proceso de conexión ahora con este paquete ya doy por conectado el dron
    if connectBtn['text'] == 'Conectando...':
        connectBtn['bg'] = 'green'
        connectBtn['text'] = 'Desconectar'
        connectBtn['fg'] = 'white'
        if talking:
            ttt.talk ('Ya tienes conexión con el dron')

    else:
        # recojo los datos de posición
        posX = round (local_telemetry_info['posX'],2)
        posY = round (local_telemetry_info['posY'],2)
        posZ = round (local_telemetry_info['posZ'],2)

        # señalo la atitud del dron en el slider
        alt = math.ceil(-posZ*10)/10
        alturaSldr.set (alt)
        # calculo las coordenadas de la posición actual del dron
        x, y = conversor.convertToCoord(posX, posY, initialHeading)
        # y muevo el dron a esa posición
        mueve_dron (x,y)

''' Vedremos aquí cada vez que llegue un paquete de datos de telemetría global'''
def process_telemetry_info (telemetry_info):
    global mapa, dronIcon, dronHeading, armBtn, initialHeading
    # de la telemetría global solo me interesa el heading
    heading = round(telemetry_info['heading'],2)
    # si es el primer dato de heading que recibo entonces me lo guardo para que sirva como referencia
    # porque el dibujo del dron inicialmente está orientado hacia la parte superior de la pantalla, que debe
    # representar el heading inicial del dron. Los datos de posición del dron tengo que transformarlos
    # de acuerdo con este cambio de orientación para convertirlos en las coordenadas dentro del canvas.
    if initialHeading == 0:
        initialHeading = heading
    # llamo a la función que redibuja el icono del dron según el heading que se acaba de recibir
    cambiar_orientacion(heading)

    # aqui trato el caso en el que el dron estaba armado pero se desarma porque no lo hemos despegado a tiempo
    if telemetry_info['state'] == 'connected' and armBtn['bg'] == 'green':
        mapa.itemconfig(dronIcon, fill='red')
        mapa.itemconfig(dronHeading, fill='red')
        armBtn['bg'] = 'orange',
        armBtn['text'] = 'Armar',
        armBtn['fg'] = 'black'

''' vengo aqui cuando se produce un breach del gaofence local'''
def notify_breach ():
    global lastButton
    global mapa, area
    # simplemente pongo en rojo grueso los bordes del área de vuelo
    mapa.itemconfig(area, outline='red', width=15)
    if talking:
        ttt.talk('Te sales de los límites')

'''esta es la función que se ejecuta al clicar el boton de conexión'''
def connect ():
    global dron, stepSldr, alturaSldr
    global height, arrow, mapa, dronIcon, dronHeading, connectBtn, altura
    global connectOption, comPort

    # veo si tengo que conectar o desconectar
    if connectBtn ['text'] == 'Conectar':
        # hay que conectar. Veamos qué opcion de conexion
        option = connectOption.get()
        if option == 'Simulation':
            connection_string ='tcp:127.0.0.1:5763'
            baud = 115200
        elif option == 'Directo':
            # assumo que está en marcha el mavproxy
            connection_string = comPort
            baud = 57600
        else: # Mavproxy
            # assumo que está en marcha el mavproxy
            connection_string = 'udp:127.0.0.1:14551'
            baud = 57600


        dron.connect(connection_string,baud)

        # dibujo el dron con la flecha que indica el heading
        dibuja_dron()

        # configuro el dron para que no cambie la orientación de dron
        # y le indico la altura del RTL (menor que la altura máxima del espacio de vuelo)

        parameters = json.dumps([
            {'ID': "WP_YAW_BEHAVIOR", 'Value': 0},
            {'ID': "RTL_ALT", 'Value': alturaSldr.get() - 1},
        ])
        dron.setParams(parameters)

        # valores por defecto de los sliders

        stepSldr.set (0.5)
        alturaSldr.set (0)
        navSpeedSldr.set (1.0)
        dron.setNavSpeed(1.0)

        # pido al dron que me entregue datos de telemetria local y global
        # le indico las funciones que debe ejecutar cada vez que tiene datos de cada tipo
        dron.send_local_telemetry_info(process_local_telemetry_info)
        dron.send_telemetry_info(process_telemetry_info)

        # indico que me estoy conectando
        # la conexión se completa cuando reciba los primeros datos de telemetría local

        connectBtn ['bg']='yellow'
        connectBtn ['text']='Conectando...'
        connectBtn ['fg']='black'

    else:
        # intento desconectar el dron
        if dron.disconnect():
            # borro la representación del dron
            mapa.delete (dronIcon)
            mapa.delete (dronHeading)
            connectBtn['bg'] = 'orange'
            connectBtn['text'] = 'Conectar'
            connectBtn['fg'] = 'black'

        else:
            # no se ha desconectado porque está en el aire
            if talking:
                ttt.talk("El dron esta volando")
            else:
                messagebox.showerror(title=None, message="El dron esta volando")

''' función que se ejecuto cuando le pido que arme'''
def arm ():
    global dron, mapa, dronIcon, dronHeading, armBtn
    global ttt, talking

    if dron.arm():
        # el armado ha ido bien
        mapa.itemconfig(dronIcon, fill='yellow')
        mapa.itemconfig(dronHeading, fill='yellow')
        armBtn['bg'] = 'green'
        armBtn['text'] = 'Armado'
        armBtn['fg'] = 'white'
        if talking:
            ttt.talk ("El dron ya está armado. Despega rápido")
    else:
        # no puedo armar si el dron está desconectado
        if talking:
            ttt.talk("El dron no está conectado")
        else:
            messagebox.showerror(title=None, message="El dron no está conectado")

''' se ejecuta cuando el dron ha acabado la maniobra de despegue'''
def volando ():
    global ttt, talking
    global mapa, dronIcon, dronHeading, takeOffBtn
    mapa.itemconfig(dronIcon, fill='green')
    mapa.itemconfig(dronHeading, fill='green')
    takeOffBtn ['bg']='green'
    takeOffBtn ['text']='volando'
    takeOffBtn ['fg']='white'
    if talking:
        ttt.talk ("Ya estamos en el aire")



''' función que se ejecuta al pulsar despegar'''
def takeoff ():
    global dron, mapa, dronIcon, dronHeading, takeOffAltSldr
    # despego en modo no bloqueante
    # Cuando acabe iré a la función volando
    if dron.takeOff (takeOffAltSldr.get(), blocking = False,  callback = volando):
        # el color naranja indica que está despegando
        mapa.itemconfig(dronIcon, fill='orange')
        mapa.itemconfig(dronHeading, fill='orange')
    else:
        # no ha despagado porque no está armado
        if talking:
            ttt.talk("El dron no está armado")
        else:
            messagebox.showerror(title=None, message="El dron no está armado")

''' Se ejecuta cuando el dron está en tierra despues del aterrizaje o del retorno'''
def enTierra ():
    global mapa, dronIcon, dronHeading, landBtn, armBtn, takeOffBtn, RTLBtn

    mapa.itemconfig(dronIcon, fill='red')
    mapa.itemconfig(dronHeading, fill='red')
    landBtn['bg'] = 'orange',
    landBtn['text'] = 'Aterrizar',
    landBtn['fg'] = 'black'

    RTLBtn['bg'] = 'orange',
    RTLBtn['text'] = 'RTL',
    RTLBtn['fg'] = 'black'

    armBtn['bg'] = 'orange',
    armBtn['text'] = 'Armar',
    armBtn['fg'] = 'black'

    takeOffBtn['bg'] = 'orange',
    takeOffBtn['text'] = 'Despegar',
    takeOffBtn['fg'] = 'black'
    if talking:
            ttt.talk("Ya estamos en tierra")

''' se ejecuta cuando pulso aterrizar'''
def land():
    global dron, mapa, landBtn
    # ordeno aterriar, pero no bloqueante
    # cuando acabe me voy a enTierra
    if dron.Land(blocking = False,   callback = enTierra):
        mapa.itemconfig(dronIcon, fill='orange')
        mapa.itemconfig(dronHeading, fill='orange')
        landBtn['bg'] = 'green',
        landBtn['text'] = 'Aterrizando',
        landBtn['fg'] = 'white'
    else:
        # el dron no está volando
        if talking:
            ttt.talk("El dron no está volando")
        else:
            messagebox.showerror(title=None, message="El dron no está volando")

''' se ejecuta cuando pulso retornar'''
def RTL():
    global dron, RTLBtn, mapa
    # ordeno RTL, pero no bloqueante
    # cuando acabe me voy a enTierra
    if dron.RTL(blocking = False,   callback = enTierra):
        mapa.itemconfig(dronIcon, fill='orange')
        mapa.itemconfig(dronHeading, fill='orange')
        RTLBtn['bg'] = 'green',
        RTLBtn['text'] = 'Retornando',
        RTLBtn['fg'] = 'white'
    else:
        # el dron no está volando
        if talking:
            ttt.talk("El dron no está volando")
        else:
            messagebox.showerror(title=None, message="El dron no está volando")

''' se ejecuta cuando acaba el movimiento ordenado'''
def llegada (btn):
    # devuelvo el color al botón que se pulsó para ordenar el movimiento
    btn['bg'] = 'orange'
    btn['fg'] = 'black'

''' se ejecuta cuando pulso uno de los botones de movimento'''
def move (direction, btn = None):
    # recibo la direccion y el boton pulsado
    global dron, area, mapa, takeOffBtn
    global ttt, talking
    global lastButton
    # guardo el boton pulsado (en otro momento necesitaré saber cuál fue el ultimo botón de movimiento pulsado)
    lastButton = btn
    if takeOffBtn['bg'] == 'green':
        # solo me muevo si he despagado
        # pongo el color adecuando del borde del espacio de vuelo, dependiendo de si el geofence local está activo o no
        if dron.localGeofenceEnabled:
            mapa.itemconfig(area, outline='red', width=5)
        else:
            mapa.itemconfig(area, outline='black', width=5)
        # pongo en verde el boton pulsado
        btn['bg'] = 'green'
        btn['fg'] = 'white'
        # muevo el dron
        # cuando acabe iré a llegada para que cambie el color del boton pulsado
        dron.move (direction, blocking = False,  callback = lambda: llegada(btn))

    else:
        if talking:
            ttt.talk("El dron no está volando")
        else:
            messagebox.showerror(title=None, message="El dron no está volando")


''' vengo aqui cuando se cambia el heading con el slider'''
def changeHeading (heading):
    global dron, initialHeading
    global gradesSldr
    print ('change heading ', heading, initialHeading)
    dron.changeHeading(int (heading) + initialHeading)

''' vengo aquí cuando se cambia el tamaño de los pasos con el slider '''
def setStep (step):
    global dron
    dron.setStep(float (step))

''' vengo aquí cuando se cambia la velocidad de navegación con el slider'''
def setNavSpeed (speed):
    global dron
    dron.setNavSpeed(float(speed))

''' vengo aquí cuando se clica el boton para activar/desactivar la comunicación mediante voz con la interfaz'''
def talkClick ():
    global talkBtn, ttt
    global talking
    if not talking:
        # hay que activar comunicacion por voz
        talking = True
        talkBtn['text'] = "Dime qué quieres hacer. Usa las palabras que hay en los botones"
        talkBtn['bg'] = 'green'
        talkBtn['fg'] = 'white'
        ttt.talk('Soy tu asistente para guiar el dron con tu voz')
        ttt.talk('Dime qué quieres hacer. Usa las palabras que hay en los botones')
        # empiezo a escuchar. Cada vez que reconozca una de las palabras indicadas en la función crear_ventana me envia a wordRecognzed
        ttt.startListening(wordRecognzed)

    else:
        # hay que desactivar la comunicación por voz
        talking = False
        ttt.stopListening()
        talkBtn['text'] = "Clica aquí para hablar conmigo"
        talkBtn['bg'] = 'dark orange'
        talkBtn['fg'] = 'black'
        ttt.talk('Gracias por confiar en mi. Que te vaya bien')

''' Cada vez que se reconozca una de las palabras indicadas vendremos aqui'''
def wordRecognzed (code, word):
    global ttt
    global leftBtn, rightBtn, forwardBtn, backBtn, upBtn, downBtn
    if code == 0:
        connect()
    elif code == 1:
        arm()
    elif code == 2:
        takeoff()
    elif code == 3:
        ttt.talk('Pues nos vamos a la izquierda')
        move('Left', leftBtn)
    elif code == 4:
        ttt.talk('Pues para la derecha')
        move('Right', rightBtn)
    elif code == 5:
        ttt.talk('Pues palante')
        move('Forward', forwardBtn)
    elif code == 6:
        ttt.talk('Vamos atras')
        move('Back', backBtn)
    elif code == 7:
        ttt.talk('Subimos')
        move('Up', upBtn)
    elif code == 8:
        ttt.talk('Bajamos')
        move('Down', downBtn)
    elif code == 9:
        ttt.talk('Voy a aterrizar')
        land()
    elif code == 10:
        ttt.talk('Volvemos a casa')
        RTL()
    elif code == -2:
        ttt.talk("No reconozco esa orden")


''' vengo aqui cuando se pulsa el boton de activar/desactivar el geofence local'''
def activarGeofence ():
    global dron, var1
    global mapa, area
    global geofenceEnableBtn
    if dron.localGeofenceEnabled:
        # hay que desactivar el geofence local
        dron.disableLocalGeofence()
        mapa.itemconfig(area, outline='black', width=5)
        geofenceEnableBtn['text'] = 'Activa el geofence'
        geofenceEnableBtn['bg'] = 'dark orange'
        geofenceEnableBtn['fg'] = 'black'
    else:
        # hay que activar el geofence local
        # le indico el tipo de acción que quiere el usuario en caso de breach
        dron.setLocalGeofenceBreachAction(int (var1.get()))
        # activo el geofence indicandole qué función hay que ejecutar en caso de breach
        dron.enableLocalGeofence(notify_breach)
        mapa.itemconfig(area, outline='red', width=5)
        geofenceEnableBtn['text'] = 'Desactiva el geofence'
        geofenceEnableBtn['bg'] = 'green'
        geofenceEnableBtn['fg'] = 'white'


''' Aquí creo los diferentes frames que forman parte de la interfaz de usuario'''
def crear_ventana():
    global dron
    global ttt
    global dimXSldr, dimYSldr, dimZSldr, stepSldr, alturaSldr, takeOffAltSldr, navSpeedSldr
    global mapa, mapaFrame
    global connectBtn, armBtn, takeOffBtn, landBtn, RTLBtn
    global canvasSize
    global scenarioFrame, controlFrame, connectOption
    global leftBtn, rightBtn, forwardBtn, backBtn, upBtn, downBtn, landBtn, RTLBtn
    global talkBtn, talking
    global initialHeading
    global image, bg, schema
    global var1, var2, var3
    global geofenceEnableBtn


    # creo el objeto con el que daré órdenes al dron
    dron = Dron()
    # creo el objeto que me permitirá comunicarme con la interfaz mediante la voz y le indico las palabras que debe reconocer
    words = ["Conectar", "Armar", "Despegar", "Izquierda", "Derecha", "Adelante", "Atrás", "Arriba", "Abajo",
             "Aterrizar", "Retornar"]
    ttt = TTT(words)
    talking = False


    # creo la ventana principal
    ventana = tk.Tk()
    ventana.title("Control dron interior")
    ventana.geometry("1200x850")

    # tendrá una fila y tres columnas
    ventana.rowconfigure(0, weight=1)
    ventana.columnconfigure(0, weight=1)
    ventana.columnconfigure(1, weight=1)
    ventana.columnconfigure(2, weight=1)

    #############################################################################
    # este frame contiene los elementos que permiten al usuario configurar las dimensiones del escenario
    scenarioFrame = tk.LabelFrame(ventana, text="Configuración del escenario")
    # ese frame va a la primera columna de la ventana principal
    scenarioFrame.grid(row=0, column=0, padx=5, pady=3, sticky=tk.N + tk.E + tk.W)
    # tiene 5 filas y 1 columna
    scenarioFrame.rowconfigure(0, weight=1)
    scenarioFrame.rowconfigure(1, weight=1)
    scenarioFrame.rowconfigure(2, weight=1)
    scenarioFrame.rowconfigure(3, weight=1)
    scenarioFrame.rowconfigure(4, weight=1)
    scenarioFrame.columnconfigure(0, weight=1)

    # sliders para configurar las dimensiones del espacio de vuelo
    dimXSldr = tk.Scale(scenarioFrame, label="dimension X (m)", resolution=1, from_=0, to=50, tickinterval=10,
                          orient=tk.HORIZONTAL)
    dimXSldr.grid(row=0, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    dimYSldr = tk.Scale(scenarioFrame, label="dimension Y (m)", resolution=1, from_=0, to=50, tickinterval=10,
                        orient=tk.HORIZONTAL)
    dimYSldr.grid(row=1, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    dimZSldr = tk.Scale(scenarioFrame, label="dimension Z (m)", resolution=1, from_=0, to=10, tickinterval=1,
                        orient=tk.HORIZONTAL)
    dimZSldr.grid(row=2, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)


    # estos elementos son para indicar qué hay que hacer en caso de breach del geofence local
    limitsFrame = tk.LabelFrame(scenarioFrame, text="En caso de superar los límites del área de vuelo...")
    limitsFrame.grid(row=3, column=0, columnspan=2, padx=5, pady=(20,5), sticky=tk.N + tk.S + tk.E + tk.W)

    var1 = tk.IntVar()
    checkbox1 = tk.Radiobutton(limitsFrame, text="Ignorar el comando", variable=var1, value=1).pack(anchor="w")
    checkbox2 = tk.Radiobutton(limitsFrame, text="Aterrizar", variable=var1, value=2).pack(anchor="w")
    checkbox3 = tk.Radiobutton(limitsFrame, text="Retornar al origen", variable=var1, value = 3).pack(anchor="w")
    var1.set(1) # por defecto ignoraremos el comando que provoca el breach

    crearBtn = tk.Button(scenarioFrame, text="Crear espacio", bg="dark orange", command=crearEspacio)
    crearBtn.grid(row=4, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    #############################################################################
    # Este frame contiene los botones para controlar el movimiento del dron
    # tambien ocupa la primera columna pero inicialmente no se muestra, porque se muestra el frame
    # para configurar el escenario


    controlFrame = tk.LabelFrame(ventana, text="Controles")

    controlFrame.rowconfigure(0, weight=1)
    controlFrame.rowconfigure(1, weight=1)
    controlFrame.rowconfigure(2, weight=1)
    controlFrame.rowconfigure(3, weight=1)
    controlFrame.rowconfigure(4, weight=1)
    controlFrame.rowconfigure(5, weight=1)
    controlFrame.rowconfigure(6, weight=1)
    controlFrame.rowconfigure(7, weight=1)
    controlFrame.rowconfigure(8, weight=1)
    controlFrame.rowconfigure(9, weight=1)
    controlFrame.rowconfigure(10, weight=1)

    controlFrame.columnconfigure(0, weight=1)
    controlFrame.columnconfigure(1, weight=1)

    # pequeño frame para configurar la conexión
    connectFrame = tk.Frame(controlFrame)
    connectFrame.grid(row=0, column=0,columnspan=2,  padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)
    connectFrame.rowconfigure(0, weight=1)
    connectFrame.rowconfigure(1, weight=1)
    connectFrame.rowconfigure(2, weight=1)
    connectFrame.columnconfigure(0, weight=1)
    connectFrame.columnconfigure(1, weight=1)



    connectBtn = tk.Button(connectFrame, text="Conectar", bg="dark orange", command = connect)
    connectBtn.grid(row=0, column=0, rowspan = 2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)
    # se puede elegir entre conectarse al simulador o conectarse al dron real, y en ese segundo caso
    # la conexion puede ser directa a través de la radio de telemetría o via mavproxy
    connectOption = tk.StringVar()
    connectOption.set ('Simulation') # por defecto se trabaja en simulación
    option1 = tk.Radiobutton(connectFrame, text="Simulación", variable=connectOption, value="Simulation")
    option1.grid(row=0, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.W)
    ''' se llama cuando elegimos la conexión directa a través de un puerto COM'''
    def askPort():
        global comPort
        comPort = askstring('Puerto', '¿Cual es el puerto COM?')
    option2 = tk.Radiobutton(connectFrame, text="Producción (directo)", variable=connectOption, value="Directo",
                             command = askPort)
    option2.grid(row=1, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.W )
    option2 = tk.Radiobutton(connectFrame, text="Produción (mavproxy)", variable=connectOption, value="Mavproxy")
    option2.grid(row=2, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.W)

    # botones para las diferentes operaciones típicas

    armBtn = tk.Button(controlFrame, text="Armar", bg="dark orange", command=arm)
    armBtn.grid(row=1, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    takeOffAltSldr = tk.Scale(controlFrame, label="Altura de despegue (m)", resolution=1, from_=0, to=10, tickinterval=1,
                        orient=tk.HORIZONTAL)
    # este slider solo aparece cuando se ha configurado la altura del espacio de vuelo

    takeOffBtn = tk.Button(controlFrame, text="Despegar", bg="dark orange", command=takeoff)
    takeOffBtn.grid(row=3, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)


    landBtn = tk.Button(controlFrame, text="Aterrizar", bg="dark orange", command=land)
    landBtn.grid(row=4, column=0, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    RTLBtn = tk.Button(controlFrame, text="Retornar", bg="dark orange", command=RTL)
    RTLBtn.grid(row=4, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    # con este slider el usuario elige la distancia que se recorre en cada movimiento que le ordenemos al dron
    stepSldr = tk.Scale(controlFrame, label="Step (m)", resolution=0.5, from_=0, to=10, tickinterval=1,
                        orient=tk.HORIZONTAL, command=setStep)
    stepSldr.grid(row=5, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)
    # para seleccionar la velocidad del movimiento
    navSpeedSldr = tk.Scale(controlFrame, label="Velocidad de navegación (m/s)", resolution=1, from_=0, to=10,
                            tickinterval=1,
                            orient=tk.HORIZONTAL, command=setNavSpeed)
    navSpeedSldr.grid(row=6, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    # botones para indicar dirección del movimiento
    forwardBtn = tk.Button(controlFrame, text="Adelante", bg="dark orange", command=lambda: move("Forward", forwardBtn))
    forwardBtn.grid(row=7, column=0, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    backBtn = tk.Button(controlFrame, text="Atrás", bg="dark orange", command=lambda: move("Back", backBtn))
    backBtn.grid(row=7, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    leftBtn = tk.Button(controlFrame, text="Izquierda", bg="dark orange", command=lambda: move("Left", leftBtn))
    leftBtn.grid(row=8, column=0, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    rightBtn = tk.Button(controlFrame, text="Derecha", bg="dark orange", command=lambda: move("Right", rightBtn))
    rightBtn.grid(row=8, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    upBtn = tk.Button(controlFrame, text="Arriba", bg="dark orange", command=lambda: move("Up", upBtn))
    upBtn.grid(row=9, column=0, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    downBtn = tk.Button(controlFrame, text="Abajo", bg="dark orange", command=lambda: move("Down", downBtn))
    downBtn.grid(row=9, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)
    # para cambiar el heading
    gradesSldr = tk.Scale(controlFrame, label="Cambiar el heading (grados)", resolution=5, from_=0, to=360,
                          tickinterval=90,
                          orient=tk.HORIZONTAL, command=changeHeading)
    gradesSldr.grid(row=10, column=0, columnspan=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    #############################################################################
    # este es el frame en el que mostraremos la altura a la que está el dron
    alturaFrame = tk.LabelFrame(ventana, text="Altura")
    alturaFrame.grid(row=0, column=1, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)
    alturaFrame.rowconfigure(0, weight=1)
    alturaFrame.columnconfigure(0, weight=1)

    alturaSldr = tk.Scale(alturaFrame, resolution=0.5, from_=50, to=0, tickinterval=1,
                          orient=tk.VERTICAL)

    #############################################################################
    # En este frame mostramos el espacio de vuelo
    mapaFrame = tk.LabelFrame(ventana, text="Mapa")
    mapaFrame.grid(row=0, column=2, padx=5, pady=3, sticky=tk.N + tk.S + tk.E + tk.W)

    # contiene un pequeño frame para activar el control por habla y para activar/desactivar el geofence local
    buttonFrame = tk.Frame (mapaFrame)
    buttonFrame.pack()

    talkBtn = tk.Button(buttonFrame, text="Clica aquí para hablar conmigo", bg="dark orange", command = talkClick)
    talkBtn.pack(side = tk.LEFT)

    geofenceEnableBtn = tk.Button(buttonFrame, text="Activar geofence local", bg="dark orange",
                               command=activarGeofence)
    geofenceEnableBtn.pack(side = tk.LEFT)

    # este es el canvas para mostrar el espacio de vuelo
    canvasSize = 800
    mapa = tk.Canvas(mapaFrame, height=canvasSize, width=canvasSize)

    # inicialmente muestro una imagen informativa
    image = Image.open("assets/esquema.png")
    image = image.resize((700, 450))
    bg = ImageTk.PhotoImage(image)

    schema = mapa.create_image(0, 0, image=bg, anchor="nw")
    mapa.pack()

    # en esta variable guardaré el heading del dron en el momento de la conexión.
    # necesito que inicialmente tenga un valor
    initialHeading = 0


    return ventana


if __name__ == "__main__":
    ventana = crear_ventana()
    ventana.mainloop()
