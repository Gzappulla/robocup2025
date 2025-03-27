from machine import Pin, I2C

# Configura I2C id 1 pin fisici sda 9 scl 10

i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=100000)

# Scansiona i dispositivi I2C connessi
devices = i2c.scan()

if devices:
    print("Dispositivi trovati:", [hex(d) for d in devices])
else:
    print("Nessun dispositivo trovato")
