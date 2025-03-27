import PicoRobotics

def reduce(func, iterable, initializer=None):
    it = iter(iterable)
    if initializer is None:
        value = next(it)
    else:
        value = initializer
    for element in it:
        value = func(value, element)
    return value


def reshape(arr, shape):
    """
    Reshape the given array to the given shape.

    :param arr: The input array.
    :param shape: The desired shape.
    :return: The reshaped array.
    """
    # Calculate the total number of elements in the input array
    total_elements = reduce(lambda x, y: x * y, shape)

    # If the input array has fewer elements than the desired shape,
    # pad the input array with zeros
    if len(arr) < total_elements:
        arr += [0] * (total_elements - len(arr))

    # If the input array has more elements than the desired shape,
    # truncate the input array
    arr = arr[:total_elements]

    # Reshape the input array to the desired shape
    new_arr = []
    for i in range(shape[0]):
        start_index = i * shape[1]
        end_index = start_index + shape[1]
        new_arr.append(arr[start_index:end_index])
        
    return new_arr


def flip(arr, axis):
    """
    Flip the given array along the given axis.

    :param arr: The input array.
    :param axis: The axis along which to flip the array.
    :return: The flipped array.
    """
    # Check that the given axis is valid
    #if axis >= len(arr.shape):
    #    raise ValueError("Invalid axis: {}".format(axis))

    # Flip the array along the given axis
    if axis == 0:
        return arr[::-1]
    elif axis == 1:
        return [row[::-1] for row in arr]
    else:
        raise ValueError("Invalid axis: {}".format(axis))



import pimoroni_i2c
import breakout_vl53l5cx
import time
#from ulab import numpy
#from motor import Motor, pico_motor_shim
from pimoroni import NORMAL_DIR, REVERSED_DIR
#from inventor import Inventor2040W, MOTOR_A, MOTOR_B

# The VL53L5CX requires a firmware blob to start up.
# Make sure you upload "vl53l5cx_firmware.bin" via Thonny to the root of your filesystem
# You can find it here: https://github.com/ST-mirror/VL53L5CX_ULD_driver/blob/no-fw/lite/en/vl53l5cx_firmware.bin

# This example attempts to track a "bright" object (such as a white business card).
# It uses reflectance to identify the target and compute the X/Y coordinates
# of its "center of mass" in the sensors view.
# We then use the object tracking to drive a little robot towards a business card onna stick!

# Motion indication only works at distances > 400mm so it's not
# really useful as a method to reject data.

# Configure your distance and brightness thresholds to suit your object
DISTANCE_THRESHOLD = 2000  # Distance in mm
REFLECTANCE_THRESHOLD = 90  # Estimated reflectance in %

PINS_BREAKOUT_GARDEN = {"sda": 6, "scl": 7}
PINS_PICO_EXPLORER = {"sda": 20, "scl": 21}

# Sensor startup time is proportional to i2c baudrate
# HOWEVER many sensors may not run at > 400KHz (400000)
i2c = pimoroni_i2c.PimoroniI2C(**PINS_BREAKOUT_GARDEN, baudrate=2_000_000)

# Speed / distance constants for the robot - modify these to change driving behaviour!
DRIVING_SPEED = 0.7    # The speed to drive the wheels at when going forward
TURNING_SPEED = 0.5    # The speed to drive the wheels at when turning
GOAL_DISTANCE = 100.0  # The distance in mm the robot will keep an object in front of it
SPEED_RANGE = 80.0     # The distance an object is from the goal that will have the robot drive at full speed

# Setup the left and right motors - we're connecting our motors via a Motor SHIM for Pico.
# Swap the directions if this is different to your setup
#left = Motor(pico_motor_shim.MOTOR_1, direction=REVERSED_DIR)
#right = Motor(pico_motor_shim.MOTOR_2, direction=NORMAL_DIR)

'''
board = Inventor2040W()
left = board.motors[MOTOR_A]
left.direction(REVERSED_DIR)
left.enable()
right = board.motors[MOTOR_B]
right.direction(NORMAL_DIR)
right.enable()
'''



board = PicoRobotics.KitronikPicoRobotics(i2c)
board.motorOn(1, "r", 10)
board.motorOn(2, "f",10)

time.sleep(1)  # Frequenza di aggiornamento PID

board.motorOff(1)
board.motorOff(2)

# Setup the VL53L5CX sensor
print("Starting up sensor...")
t_sta = time.ticks_ms()
sensor = breakout_vl53l5cx.VL53L5CX(i2c)
t_end = time.ticks_ms()
print("Done in {}ms...".format(t_end - t_sta))

# Make sure to set resolution and other settings *before* you start ranging
sensor.set_resolution(breakout_vl53l5cx.RESOLUTION_8X8)
sensor.set_ranging_frequency_hz(15)
sensor.start_ranging()

while True:
    time.sleep(1.0 / 60)
    if sensor.data_ready():
        # "data" is a namedtuple (attrtuple technically)
        # it includes average readings as "distance_avg" and "reflectance_avg"
        # plus a full 4x4 or 8x8 set of readings (as a 1d tuple) for both values.
        data = sensor.get_data()

        #reflectance = numpy.array(data.reflectance).reshape((8, 8))
        #distance = numpy.array(data.distance).reshape((8, 8))

        reflectance = reshape(list(data.reflectance),(8, 8))
        distance = reshape(list(data.distance),(8, 8))

        scalar = 0
        target_distance = 0
        n_distances = 0
        # Filter out unwanted reflectance values
        for ox in range(8):
            for oy in range(8):
                d = distance[ox][oy]
                r = reflectance[ox][oy]
                if d > DISTANCE_THRESHOLD or r < REFLECTANCE_THRESHOLD:
                    reflectance[ox][oy] = 0
                else:
                    scalar += r

        # Get a total from all the distances within our accepted target
        for ox in range(8):
            for oy in range(8):
                d = distance[ox][oy]
                r = reflectance[ox][oy]
                if r > 0:
                    target_distance += d
                    n_distances += 1

        # Average the target distance
        if n_distances > 0:
            target_distance /= n_distances
        else:
            target_distance = 0

        # Flip reflectance now we've applied distance
        # both fields are upside-down!
        #reflectance = numpy.flip(reflectance, axis=0)
        reflectance = flip(reflectance, axis=0)
        


        # Calculate the center of mass along X and Y
        x = 0
        y = 0
        #print(scalar)
        if scalar > 0:
            for ox in range(8):
                for oy in range(8):
                    y += reflectance[ox][oy] * ox
            y /= scalar
            y /= 3.5
            y -= 1.0

            for oy in range(8):
                for ox in range(8):
                    x += reflectance[ox][oy] * oy
            x /= scalar
            x /= 3.5
            x -= 1.0

            print("Object detected at x: {:.2f}, y: {:.2f}".format(x, y))

            # Our robot will try and keep the object a goal distance in front of it.
            # If the object gets closer it will reverse, if the object gets further away it will drive forward.
            scale = (target_distance - GOAL_DISTANCE) / SPEED_RANGE
            spd = max(min(scale, 1.0), -1.0) * DRIVING_SPEED

            print("Distance is {:.1f} mm. Speed is {:.2f}".format(target_distance, spd))

            #left.speed(spd - (x * TURNING_SPEED))
            #right.speed(spd + (x * TURNING_SPEED))
            board.motorOn(1, "f", (spd - (x * TURNING_SPEED)) * 20)
            board.motorOn(2, "f", (spd + (x * TURNING_SPEED)) * 20)
            
            print("left ", (spd - (x * TURNING_SPEED)))
            print("right ", (spd + (x * TURNING_SPEED)))
            
        else:
            #left.coast()
            #right.coast()
            board.motorOn(1, "r", 0)
            board.motorOn(2, "r", 0)
            
            print("left stop")
            print("right stop")
