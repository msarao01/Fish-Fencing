import time
import bluetooth
import struct
from machine import Pin, Timer, PWM, ADC
import neopixel
import utime
from micropython import const

# Neopixel Configs
on = (10, 0, 10)
off = (0, 0, 0)
alert = (0, 255, 0)  # Green for magnetic detection
teacher_warning = (255, 0, 0)  # Red for teacher alert


# BLE event constants
_IRQ_SCAN_RESULT = const(5)
_IRQ_SCAN_COMPLETE = const(6)
_IRQ_PERIPHERAL_CONNECT = const(7)
_IRQ_PERIPHERAL_DISCONNECT = const(8)
_IRQ_GATTC_NOTIFY = const(18)

# BLE UUIDs
_SERVICE_UUID = 0x1815
_MOTOR_SPEED_CHAR_UUID = 0x2A56

MAGNETIC_FIELD_THRESHOLD = 5000  # Adjust this based on the analog sensor's range

class BLECentral:
    def __init__(self):
        # Initialize BLE
        self._ble = bluetooth.BLE()
        self._ble.active(True)
        self._ble.irq(self._irq)
        self._conn_handle = None
        self._motor_speed_handle = None
        self.shutdown_timer = None

        # Initialize motor pins
        self.dir_pin = Pin(16, Pin.OUT)
        self.step_pin = Pin(17, Pin.OUT)
        self.wheeldir_pin = Pin(2, Pin.OUT)
        self.wheelstep_pin = Pin(3, Pin.OUT)

        # Hall Effect Sensor (Analog)
        self.hall_sensor = ADC(Pin(27))  # Pin connected to the Hall sensor
        # self.hall_sensor.atten(ADC.ATTN_11DB)  # Set attenuation for full range (0-3.3V)

        # Buzzer and NeoPixel
        self.buzzer = PWM(Pin(18, Pin.OUT))  # Pin connected to the buzzer

        # Movement and timers, ADJUST FOR TIME OF MOVEMENT
        self.steps_per_revolution = 5000
        self.tim = Timer()

        # Command queue
        self.command_queue = []

        # Neopixel for status
        self.neo = neopixel.NeoPixel(Pin(28), 1)
        self.neo[0] = off
        self.neo.write()

    # Timer callbacks for motor steps
    def spoolstep(self, t):
        self.step_pin.value(not self.step_pin.value())

    def wheelstep(self, t):
        self.wheelstep_pin.value(not self.wheelstep_pin.value())

    # Motor control functions
    def rotate_spoolmotor(self, delay):
        # ADJUST FOR SPEED
        self.tim.init(freq=2000000 // delay, mode=Timer.PERIODIC, callback=self.spoolstep)

    def rotate_wheelmotor(self, delay):
        # ADJUST FOR SPEED
        self.tim.init(freq=2000000 // delay, mode=Timer.PERIODIC, callback=self.wheelstep)

    def down(self):
        print("Moving Down")
        self.wheeldir_pin.value(0)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    def up(self):
        print("Moving Up")
        self.wheeldir_pin.value(1)
        self.rotate_wheelmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    def left(self):
        print("Moving Left")
        self.dir_pin.value(1)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    def right(self):
        print("Moving Right")
        self.dir_pin.value(0)
        self.rotate_spoolmotor(1000)
        utime.sleep_ms(self.steps_per_revolution)
        self.tim.deinit()
        time.sleep(1)

    # BLE IRQ handler
    def _irq(self, event, data):
        if event == _IRQ_SCAN_RESULT:
            addr_type, addr, adv_type, rssi, adv_data = data
            if self._find_service_in_advertisement(adv_data, _SERVICE_UUID):
                print("Found Motor Controller!")
                self._ble.gap_scan(None)
                self._ble.gap_connect(addr_type, addr)

        elif event == _IRQ_PERIPHERAL_CONNECT:
            conn_handle, addr_type, addr = data
            self.neo[0] = on
            self.neo.write()
            print("Connected")
            self._conn_handle = conn_handle
            self._ble.gattc_discover_services(conn_handle)

        elif event == _IRQ_PERIPHERAL_DISCONNECT:
            self.neo[0] = off
            self.neo.write()
            print("Disconnected")
            self._conn_handle = None
            self.start_scan()

        elif event == _IRQ_GATTC_NOTIFY:
            conn_handle, value_handle, notify_data = data
            decoded_data = bytes(notify_data).decode()
            print("Received data:", decoded_data)
            self.enqueue_motor_commands(decoded_data)

    def enqueue_motor_commands(self, direction):
        try:
            self.command_queue.append(direction)
            print("Commands added to queue:", direction)
        except Exception as e:
            print("Error processing instructions:", e)
    
    # Save an action in the logs
    def log_event(self, message):
        timestamp = time.ticks_ms()
        log_entry = f"{timestamp}: {message}"
        print(log_entry)
        self.logs.append(log_entry)
        if len(self.logs) > 100:
            self.logs.pop(0)

    # Send the logs to remote client to debug
    def send_logs(self):
        if self._conn_handle:
            for log in self.logs:
                self._ble.gatts_notify(self._conn_handle, self._log_handle, log.encode())
            self.logs.clear()
    
    # Check the hall sensor and buzz/light if its there
    def check_hall_sensor(self):
        """Check the Hall effect sensor for magnetic field detection."""
        hall_value = self.hall_sensor.read_u16()  # Use read_u16() for ADC reading
        if hall_value < MAGNETIC_FIELD_THRESHOLD:
            # print(f"Magnetic field detected! Value: {hall_value}")
            self.neo[0] = alert
            self.neo.write()
            self.buzzer.freq(500)
            self.buzzer.duty_u16(1000)
            utime.sleep(0.5)  # Buzzer on for 0.5 seconds
            self.buzzer.duty_u16(0)
        else:
            self.neo[0] = off
            self.neo.write()

    def _find_service_in_advertisement(self, adv_data, service_uuid):
        i = 0
        while i < len(adv_data):
            length = adv_data[i]
            if length == 0:
                break
            ad_type = adv_data[i + 1]
            if ad_type == 0x03:
                uuid16 = struct.unpack("<H", adv_data[i + 2: i + length + 1])[0]
                if uuid16 == service_uuid:
                    return True
            i += length + 1
        return False

    def start_scan(self):
        print("Scanning for BLE devices...")
        self._ble.gap_scan(2000, 30000, 30000)
    
    def process_command(self, command):
        if command == "Warning":
            self.neo[0] = teacher_warning
            self.neo.write()
            self.buzzer.freq(100)
            self.buzzer.duty_u16(1000)
            utime.sleep(0.5)  # Buzzer on for 0.5 seconds
            self.shutdown_timer = time.ticks_ms()  # Start shutdown timer
            self.buzzer.duty_u16(0)
        elif command == "Up":
            self.up()
        elif command == "Down":
            self.down()
        elif command == "Left":
            self.left()
        elif command == "Right":
            self.right()
        elif command == "Logs":
            self.send_logs()


# Initialize BLECentral
central = BLECentral()
central.start_scan()

# Main loop for processing commands
while True:
    if central.shutdown_timer:
        # Check if 5 minutes (300,000 ms) have passed since the warning
        if time.ticks_diff(time.ticks_ms(), central.shutdown_timer) > 300000:
            print("Shutting down after teacher warning...")
            break  # Exit the loop to shut down
    if central.command_queue:
        command = central.command_queue.pop(0)
        central.process_command(command)
        central.check_hall_sensor()
    time.sleep(0.1)  # Allow other tasks to run

