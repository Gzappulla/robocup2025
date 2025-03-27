from machine import Pin, I2C
import time
import PicoRobotics
from bno055 import BNO055 
import math

# Inizializza la scheda Kitronik
board = PicoRobotics.KitronikPicoRobotics()

# Inizializza BNO055 su I2C
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=400000)
bno = BNO055(i2c)

# Encoder
encoderA1 = Pin(0, Pin.IN, Pin.PULL_UP)  # Motore sinistro
encoderA2 = Pin(2, Pin.IN, Pin.PULL_UP)  # Motore destro

# Variabili per contare gli impulsi
countA = 0
countB = 0

# Impulsi per giro ruota (misurare o controllare il datasheet)
IMPULSI_PER_GIRO = 36  # Cambia con il valore corretto
DIAMETRO_RUOTA_CM = 7  # Cambia con il diametro corretto
CM_PER_GIRO = math.pi * DIAMETRO_RUOTA_CM
IMPULSI_PER_CM = IMPULSI_PER_GIRO / CM_PER_GIRO

# Parametri PID per la direzione
Kp_dir, Ki_dir, Kd_dir = 1.2, 0.0, 0.2

# Callback per contare gli impulsi del motore A
def callbackA(pin):
    global countA
    countA += 1

# Callback per contare gli impulsi del motore B
def callbackB(pin):
    global countB
    countB += 1

# Assegna gli interrupt agli encoder
encoderA1.irq(trigger=Pin.IRQ_RISING, handler=callbackA)
encoderA2.irq(trigger=Pin.IRQ_RISING, handler=callbackB)

# 📌 Funzione PID
def pid_control(Kp, Ki, Kd, target, actual, integral, last_error):
    error = target - actual
    integral += error
    derivative = error - last_error
    output = Kp * error + Ki * integral + Kd * derivative
    return output, integral, error

# 📌 Funzione per avanzare di X cm con PID sulla direzione
def vai_avanti(cm, velocita):
    global countA, countB
    countA, countB = 0, 0  # Reset conteggio impulsi
    integral_dir, last_error_dir = 0, 0

    # Imposta il riferimento iniziale
    angolo_iniziale = bno.euler()[0]

    # Calcola il numero di impulsi necessari per coprire la distanza
    target_impulsi = cm * IMPULSI_PER_CM

    while (countA + countB) / 2 < target_impulsi:
        angolo_attuale = bno.euler()[0]

        # Calcolo correzione PID direzione
        correzione_dir, integral_dir, last_error_dir = pid_control(Kp_dir, Ki_dir, Kd_dir, angolo_iniziale, angolo_attuale, integral_dir, last_error_dir)

        # Regolazione velocità per correggere la direzione
        speedA = max(10, min(100, velocita - correzione_dir))  # Minimo 10 per evitare fermo
        speedB = max(10, min(100, velocita + correzione_dir))

        board.motorOn(1, "f", speedA)
        board.motorOn(2, "f", speedB)

        time.sleep(0.1)  # Frequenza di aggiornamento PID

    board.motorOff(1)
    board.motorOff(2)

def ruota(gradi, velocita):
    global bno

    Kp_rot, Ki_rot, Kd_rot = 2.0, 0.0, 0.5  # Parametri PID per la rotazione
    integral_rot, last_error_rot = 0, 0

    angolo_iniziale = bno.euler()[0]
    if angolo_iniziale is None:
        angolo_iniziale = 0  # Se il sensore non risponde, assume 0

    angolo_target = (angolo_iniziale + gradi) % 360  # Normalizza l'angolo
    tolleranza = 2  # Accettiamo un errore di ±2 gradi

    while True:
        angolo_attuale = bno.euler()[0]
        if angolo_attuale is None:
            continue  # Se il sensore non risponde, riprova

        errore_angolo = angolo_target - angolo_attuale
        if abs(errore_angolo) < tolleranza:
            break  # Se siamo dentro la tolleranza, fermiamo i motori

        # Controllo PID per la rotazione
        correzione_rot, integral_rot, last_error_rot = pid_control(Kp_rot, Ki_rot, Kd_rot, angolo_target, angolo_attuale, integral_rot, last_error_rot)

        # Imposta velocità con segno corretto
        velocita_effettiva = max(10, min(100, velocita + correzione_rot))
        direzione = "f" if errore_angolo > 0 else "r"

        # Ruota in senso orario o antiorario
        board.motorOn(1, direzione, velocita_effettiva)
        board.motorOn(2, "r" if direzione == "f" else "f", velocita_effettiva)

        time.sleep(0.1)  # Frequenza di aggiornamento PID

    board.motorOff(1)
    board.motorOff(2)

vai_avanti(20, 50)  # Avanza di 50 cm a velocità 50% con PID sulla direzione

ruota(90, 10)  # Ruota di 90° a velocità 50%
# 📌 Test della funzione
vai_avanti(20, 50)  # Avanza di 50 cm a velocità 50% con PID sulla direzione

ruota(90, 10)  # Ruota di 90° a velocità 50%
# 📌 Test della funzione
vai_avanti(20, 50)  # Avanza di 50 cm a velocità 50% con PID sulla direzione
ruota(90, 10)
vai_avanti(20, 50)