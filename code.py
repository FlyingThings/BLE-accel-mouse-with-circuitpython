# Bluetooth mouse with Circuit Playground Bluefruit NRF52840 
# Uses accelerometer, as there is no gyro on the Circuit Playground Bluefruit
# 
# How it works:
# Measure X Y Z accelerations, apply a low-pass filter, calculate the tilt angle
# The difference of the current and previous angles (delta angle) is proportional to the mouse movement
# This is applied on roll and pitch for x and y.

# Use:
# Hold B button to enable the mouse, and press A button to left click.
# Holding and releasing the B button acts like picking up the mouse when it runs out of space on the mouse pad.

# I used many circuitpython examples, of which the ulab/numpy helped alot to implement the low-pass filter
# https://learn.adafruit.com/ulab-crunch-numbers-fast-with-circuitpython/filter-example-measuring-barometric-pressure
import time
from adafruit_hid.consumer_control import ConsumerControl
from adafruit_hid.consumer_control_code import ConsumerControlCode
from adafruit_hid.keyboard import Keyboard
from adafruit_hid.keyboard_layout_us import KeyboardLayoutUS
from adafruit_hid.keycode import Keycode
from adafruit_hid.mouse import Mouse

import adafruit_ble
from adafruit_ble.advertising import Advertisement
from adafruit_ble.advertising.standard import ProvideServicesAdvertisement
from adafruit_ble.services.standard.hid import HIDService
from adafruit_ble.services.standard.device_info import DeviceInfoService

from adafruit_ble.services.nordic import UARTService

import board
import digitalio
import adafruit_debouncer
import neopixel
import math
import adafruit_lis3dh
import busio
from ulab import numpy as np
import touchio

RX_pin = touchio.TouchIn(board.RX)
RX_touch = adafruit_debouncer.Debouncer(RX_pin)
TX_pin = touchio.TouchIn(board.TX)
TX_touch = adafruit_debouncer.Debouncer(TX_pin)

i2c = busio.I2C(board.ACCELEROMETER_SCL, board.ACCELEROMETER_SDA)
int1 = digitalio.DigitalInOut(board.ACCELEROMETER_INTERRUPT)
lis3dh = adafruit_lis3dh.LIS3DH_I2C(i2c, address=0x19, int1=int1)

pix = neopixel.NeoPixel(board.NEOPIXEL,10, brightness=0.1)
pix.fill(0)
PINK = [228,0,124]
OFF = [0,0,0]
A_pin = digitalio.DigitalInOut(board.BUTTON_A)
A_pin.pull = digitalio.Pull.DOWN
A_button = adafruit_debouncer.Debouncer(A_pin)

B_pin = digitalio.DigitalInOut(board.BUTTON_B)
B_pin.pull = digitalio.Pull.DOWN
B_button = adafruit_debouncer.Debouncer(B_pin)
# Use default HID descriptor
hid = HIDService()
device_info = DeviceInfoService(
    software_revision=adafruit_ble.__version__, manufacturer="Adafruit Industries"
)
advertisement = ProvideServicesAdvertisement(hid)
advertisement.appearance = 961
scan_response = Advertisement()
uart = UARTService()
ad_uart = ProvideServicesAdvertisement(uart)
ad_uart.complete_name = "LEFTPAW"
ble = adafruit_ble.BLERadio()
ble.name = ad_uart.complete_name
if ble.connected:
    for c in ble.connections:
        c.disconnect()

print("advertising")
ble.start_advertising(advertisement, scan_response)

k = Keyboard(hid.devices)
kl = KeyboardLayoutUS(k)
cc = ConsumerControl(hid.devices)
cc_id = 0
mouse = Mouse(hid.devices)
class Timer:
    def __init__(self,duration):
        self.duration = duration
        self.last_time = time.monotonic()
    def done(self):
        current_time = time.monotonic()
        if current_time - self.last_time > self.duration:
            self.last_time = current_time
            return True
        else:
            return False

h = [ #fast, 17 taps
    0.040074860540691516,
    0.046261964616328623,
    0.052158860963975616,
    0.057563479835469687,
    0.062284795594462629,
    0.066151921659855950,
    0.069022372535081714,
    0.070789030852233911,
    0.071385426803800764,
    0.070789030852233911,
    0.069022372535081714,
    0.066151921659855950,
    0.062284795594462629,
    0.057563479835469687,
    0.052158860963975616,
    0.046261964616328623,
    0.040074860540691516,
]

taps = np.array(h)

ax_data, ay_data, az_data = np.zeros(len(taps)), np.zeros(len(taps)), np.zeros(len(taps))

get_accel_timer = Timer(1/100)
avg_samp_timer = Timer(1)
th_angle_list = []
psi_angle_list = []
max_samples = 75
th_last = 0
psi_last = 0


SENSITIVITY = 15
DEAD_ZONE = 0.055
while True:
    while not ble.connected:
        pass
    pix[1] = [0,50,0]
    if ble.connected:
        print(f'connections:{ble.connections}')
    while ble.connected:
        B_button.update()
        A_button.update()

        if B_button.rose:
            pix[5] = PINK
            # cc.send(ConsumerControlCode.BRIGHTNESS_INCREMENT)
        if B_button.fell:
            pix[5] = OFF
        if A_button.rose:
            pix[4] = PINK
            mouse.press(mouse.LEFT_BUTTON)
        if A_button.fell:
            pix[4] = OFF
            mouse.release(mouse.LEFT_BUTTON)
        if get_accel_timer.done():
            Ax, Ay, Az = lis3dh.acceleration
            ax_data = np.roll(ax_data,1)
            ay_data = np.roll(ay_data,1)
            az_data = np.roll(az_data,1)
            ax_data[-1] = Ax
            ay_data[-1] = Ay
            az_data[-1] = Az
                
            ax_filtered = np.sum(ax_data* taps)
            ay_filtered = np.sum(ay_data* taps)
            az_filtered = np.sum(az_data* taps)
            try:
                theta_angle = math.degrees(math.atan(ax_filtered/(math.sqrt((ay_filtered**2)+(az_filtered**2)))))
                psi_angle = math.degrees(math.atan(ay_filtered/(math.sqrt((ax_filtered**2)+(az_filtered**2)))))
                phi_angle = math.degrees(math.atan((math.sqrt((Ax**2)+(ay_filtered**2)))/az_filtered))
            except ZeroDivisionError:
                pass

            th_d = theta_angle - th_last
            psi_d = psi_angle - psi_last
            th_last = theta_angle
            psi_last = psi_angle
            if abs(th_d) > DEAD_ZONE or abs(psi_d) >DEAD_ZONE:
                x = int(th_d*SENSITIVITY)
                y = int(psi_d*SENSITIVITY)
                if  B_button.value:
                    mouse.move(x,y)
        

    ble.stop_advertising()
    ble.start_advertising(advertisement)
