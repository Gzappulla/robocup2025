import time
import pimoroni_i2c
import breakout_vl53l5cx

# Pin SDA/SCL per l'Inventor 2040
PINS_BREAKOUT_GARDEN = {"sda": 4, "scl": 5}

# Imposta una velocità I2C sicura
i2c = pimoroni_i2c.PimoroniI2C(**PINS_BREAKOUT_GARDEN, baudrate=400_000)

# Avvio del sensore
print("Starting up sensor...")
t_sta = time.ticks_ms()
sensor = breakout_vl53l5cx.VL53L5CX(i2c)
t_end = time.ticks_ms()
print("Done in {}ms...".format(t_end - t_sta))

# Configura e avvia il ranging
sensor.set_resolution(breakout_vl53l5cx.RESOLUTION_4X4)
sensor.set_ranging_frequency_hz(15)  # Imposta la frequenza di misurazione
sensor.set_integration_time_ms(20)  # Imposta il tempo di integrazione
sensor.start_ranging()

def obstacle():
    if sensor.data_ready():
        data = sensor.get_data()
        arr = [d if d < 180 else 9999 for d in data.distance]  # Limita i valori massimi
        arr = [arr[i * 4:(i + 1) * 4] for i in range(4)]  # Rimodella in matrice 4x4
        # Controlla il valore centrale
        if arr[1][1] > 1800 or arr[1][1] < 10:
            pass
        else:
            somma = sum(arr[0]) + sum(arr[1])  # Somma delle righe superiori
            somma2 = sum(arr[2]) + sum(arr[3])  # Somma delle righe inferiori
            
            if somma > somma2:
                print("Obstacle detected\n\n\n")
                return True
            else:
                return False