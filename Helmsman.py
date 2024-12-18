import sensor
import time
import math
import struct
import bluetooth
from machine import Pin
from micropython import const

# Initialize the camera
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.skip_frames(time=2000)
sensor.set_auto_gain(False)
sensor.set_auto_whitebal(False)
clock = time.clock()

# Button setup
button = Pin("P0", Pin.IN, Pin.PULL_UP)  # Replace "P0" with the GPIO pin connected to your button

# Camera calibration values
f_x = (2.8 / 3.984) * 160
f_y = (2.8 / 2.952) * 120
c_x = 160 * 0.5
c_y = 120 * 0.5

def degrees(radians):
    return (180 * radians) / math.pi

# BLE constants and UUIDs
_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_FLAG_READ = const(0x0002)
_FLAG_NOTIFY = const(0x0010)
_SERVICE_UUID = bluetooth.UUID(0x1815)
_MOTOR_SPEED_CHAR_UUID = (bluetooth.UUID(0x2A56), _FLAG_NOTIFY | _FLAG_READ)
_MOTOR_SERVICE = (_SERVICE_UUID, (_MOTOR_SPEED_CHAR_UUID,))

class BLEMotor:
    def __init__(self, ble, name="CarlaB"):
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        handles = self._ble.gatts_register_services((_MOTOR_SERVICE,))
        self._handle = handles[0][0]
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_SERVICE_UUID])
        self._advertise()

    def _irq(self, event, data):
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print("Central connected")
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            self._advertise()
            print("Central disconnected")

    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

    def notify(self, message):
        """Send a single movement instruction."""
        for conn_handle in self._connections:
            try:
                print(f"Sending: {message}")
                self._ble.gatts_notify(conn_handle, self._handle, message)
            except Exception as e:
                print(f"Error notifying: {e}")

def advertising_payload(limited_disc=False, br_edr=False, name=None, services=None):
    payload = bytearray()

    def _append(adv_type, value):
        nonlocal payload
        payload += bytes((len(value) + 1, adv_type)) + value

    flags = (0x02 if limited_disc else 0x06) + (0x00 if br_edr else 0x04)
    _append(0x01, struct.pack("B", flags))
    if name:
        _append(0x09, name.encode())
    if services:
        for uuid in services:
            uuid_bytes = bytes(uuid)
            if len(uuid_bytes) == 2:
                _append(0x03, uuid_bytes)
            elif len(uuid_bytes) == 16:
                _append(0x07, uuid_bytes)
    return payload

# Initialize BLE
ble = bluetooth.BLE()
motor_ble = BLEMotor(ble)
print("Waiting for BLE connection...")

# Wait for connection
while not motor_ble._connections:
    time.sleep(0.1)

print("Connected! Ready to send instructions.")
# Main loop
while True:
    if not button.value():  # Button is pressed
        clock.tick()
        img = sensor.snapshot()
        detected_tags = []

        # Detect and sort AprilTags
        for tag in img.find_apriltags():
            img.draw_rectangle(tag.rect, color=(255, 0, 0))
            img.draw_cross(tag.cx, tag.cy, color=(0, 255, 0))
            detected_tags.append((tag.id, tag.y_translation, tag.rotation))
        detected_tags.sort(key=lambda t: t[1], reverse=True)

        # Process detected tags into directions
        movement_instructions = []
        for tag_id, _, rotation in detected_tags:
            rotation_deg = degrees(rotation)
            if tag_id == 1 and 0 <= rotation_deg <= 180:
                movement_instructions.append("Right")
            elif tag_id == 1 and 181 <= rotation_deg <= 359:
                movement_instructions.append("Left")
            elif tag_id == 0 and 0 <= rotation_deg <= 160:
                movement_instructions.append("Up")
            elif tag_id == 0 and 161 <= rotation_deg <= 300:
                movement_instructions.append("Down")

        # Send each instruction individually
        if movement_instructions:
            for instruction in movement_instructions:
                motor_ble.notify(instruction.encode())
                print(f"Sent instruction: {instruction}")
        time.sleep(0.2)  # Debounce the button
