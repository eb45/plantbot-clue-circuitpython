# SPDX-FileCopyrightText: 2021 ladyada for Adafruit Industries
# SPDX-License-Identifier: MIT

# Adafruit Service demo for Adafruit CLUE board.
# Accessible via Adafruit Bluefruit Playground app and Web Bluetooth Dashboard.

import time
from adafruit_servokit import ServoKit

import board
from digitalio import DigitalInOut
import neopixel_write
from adafruit_ble import BLERadio

from adafruit_clue import clue

from adafruit_ble_adafruit.adafruit_service import AdafruitServerAdvertisement

from adafruit_ble_adafruit.accelerometer_service import AccelerometerService
from adafruit_ble_adafruit.addressable_pixel_service import AddressablePixelService
from adafruit_ble_adafruit.barometric_pressure_service import BarometricPressureService
from adafruit_ble_adafruit.button_service import ButtonService
from adafruit_ble_adafruit.humidity_service import HumidityService
from adafruit_ble_adafruit.light_sensor_service import LightSensorService
from adafruit_ble_adafruit.temperature_service import TemperatureService
from adafruit_ble_adafruit.tone_service import ToneService
from adafruit_ble_adafruit.gyroscope_service import GyroscopeService
from drive_service import DriveService
from exploration_service import ExplorationService

print("Please connect...")

accel_svc = AccelerometerService()
accel_svc.measurement_period = 100
accel_last_update = 0

gyro_svc = GyroscopeService()
gyro_svc.measurement_period = 100
gyro_last_update = 0

# CLUE has just one board pixel. 3 RGB bytes * 1 pixel.
NEOPIXEL_BUF_LENGTH = 3 * 1
neopixel_svc = AddressablePixelService()
neopixel_buf = bytearray(NEOPIXEL_BUF_LENGTH)
# Take over NeoPixel control from clue.
clue._pixel.deinit()  # pylint: disable=protected-access
neopixel_out = DigitalInOut(board.NEOPIXEL)
neopixel_out.switch_to_output()

baro_svc = BarometricPressureService()
baro_svc.measurement_period = 100
baro_last_update = 0

button_svc = ButtonService()
button_svc.set_pressed(False, clue.button_a, clue.button_b)

humidity_svc = HumidityService()
humidity_svc.measurement_period = 100
humidity_last_update = 0

light_svc = LightSensorService()
light_svc.measurement_period = 100
light_last_update = 0

temp_svc = TemperatureService()
temp_svc.measurement_period = 100
temp_last_update = 0

tone_svc = ToneService()

drive_svc = DriveService()
kit = ServoKit(channels=16)

exploration_svc = ExplorationService()

ble = BLERadio()
# The Web Bluetooth dashboard identifies known boards by their
# advertised name, not by advertising manufacturer data.
ble.name = "CLUE"

# The Bluefruit Playground app looks in the manufacturer data
# in the advertisement. That data uses the USB PID as a unique ID.
# Adafruit CLUE USB PID:
# Arduino: 0x8071,  CircuitPython: 0x8072, app supports either
adv = AdafruitServerAdvertisement()
adv.pid = 0x8072

while True:
    # Advertise when not connected.
    ble.start_advertising(adv)
    while not ble.connected:
        pass
    ble.stop_advertising()

    while ble.connected:
        now_msecs = time.monotonic_ns() // 1000000  # pylint: disable=no-member

        if now_msecs - accel_last_update >= accel_svc.measurement_period:
            accel_svc.acceleration = clue.acceleration
            accel_last_update = now_msecs

        if now_msecs - gyro_last_update >= gyro_svc.measurement_period:
            gyro_svc.gyro = clue.gyro
            gyro_last_update = now_msecs

        if now_msecs - baro_last_update >= baro_svc.measurement_period:
            baro_svc.pressure = clue.pressure
            baro_last_update = now_msecs

        button_svc.set_pressed(False, clue.button_a, clue.button_b)

        if now_msecs - humidity_last_update >= humidity_svc.measurement_period:
            humidity_svc.humidity = clue.humidity
            humidity_last_update = now_msecs

        if now_msecs - light_last_update >= light_svc.measurement_period:
            # Return "clear" color value from color sensor.
            light_svc.light_level = clue.color[3]
            light_last_update = now_msecs

        neopixel_values = neopixel_svc.values
        if neopixel_values is not None:
            start = neopixel_values.start
            if start > NEOPIXEL_BUF_LENGTH:
                continue
            data = neopixel_values.data
            data_len = min(len(data), NEOPIXEL_BUF_LENGTH - start)
            neopixel_buf[start : start + data_len] = data[:data_len]
            if neopixel_values.write_now:
                neopixel_write.neopixel_write(neopixel_out, neopixel_buf)

        if now_msecs - temp_last_update >= temp_svc.measurement_period:
            temp_svc.temperature = clue.temperature
            temp_last_update = now_msecs

        drive = drive_svc.drive
        if drive is not None:
            f, b, l, r = drive
            if f!=0 or b!=0 or l!=0 or r!=0:
                if f==1:
                    kit.continuous_servo[0].throttle = 1
                    kit.continuous_servo[1].throttle = -1
                    time.sleep(1)
                    kit.continuous_servo[0].throttle = 0.06
                    kit.continuous_servo[1].throttle = 0.06
                if b==1:
                    kit.continuous_servo[0].throttle = -1
                    kit.continuous_servo[1].throttle = 1
                    time.sleep(1)
                    kit.continuous_servo[0].throttle = 0.06
                    kit.continuous_servo[1].throttle = 0.06
                if r==1:
                    kit.continuous_servo[2].throttle = 1
                    kit.continuous_servo[3].throttle = -1
                    time.sleep(1)
                    kit.continuous_servo[2].throttle = 0.06
                    kit.continuous_servo[3].throttle = 0.1
                if l==1:
                    kit.continuous_servo[2].throttle = -1
                    kit.continuous_servo[3].throttle = 1
                    time.sleep(1)
                    kit.continuous_servo[2].throttle = 0.06
                    kit.continuous_servo[3].throttle = 0.1
                print(f"f={f}, b={b}, l={l}, r={r}") #change this to actual servo move
        last_drive = drive

        exploration = exploration_svc.exploration
        if exploration is not None:
            go, stop = exploration
            if go==1:
                average = sum(clue.color)/len(clue.color)
                while (average<5000):
                    kit.continuous_servo[0].throttle = 1
                    kit.continuous_servo[1].throttle = -1
                    print("not enough:(")
                    average = sum(clue.color)/len(clue.color)
                print("hi light!")
                kit.continuous_servo[0].throttle = 0.06
                kit.continuous_servo[1].throttle = 0.06
            if stop==1:
                print("STOP")
                kit.continuous_servo[0].throttle = 0.06
                kit.continuous_servo[1].throttle = 0.06
        last_exploration = exploration

        tone = tone_svc.tone
        if tone is not None:
            freq, duration_msecs = tone
            if freq != 0:
                if duration_msecs != 0:
                    # Note that this blocks. Alternatively we could
                    # use now_msecs to time a tone in a non-blocking
                    # way, but then the other updates might make the
                    # tone interval less consistent.
                    clue.play_tone(freq, duration_msecs / 1000)
                else:
                    clue.stop_tone()
                    clue.start_tone(freq)
            else:
                clue.stop_tone()
        last_tone = tone
