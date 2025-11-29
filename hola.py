import socket, RPi.GPIO as GPIO

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

s = socket.socket()
s.bind(("0.0.0.0", 5000))
s.listen(1)

conn, addr = s.accept()

while True:
cmd = conn.recv(10).decode().strip().upper()
GPIO.output(17, cmd=="LED ON")

import socket
s = socket.socket()
s.connect(("192.168.1.10", 5000)) # Change IP to Raspberry Pi IP
while True:
s.send(input("> ").encode())

String cmd;

void setup() {
Serial.begin(9600);
pinMode(13, OUTPUT);
}

void loop() {
if (Serial.available()) {
cmd = Serial.readStringUntil('\n');
digitalWrite(13, cmd == "LED ON");
}
}

import serial

ser = serial.Serial("COM3", 9600)

while True:
ser.write((input("> ") + "\n").encode())


NEW:

int led = 13;

void setup() {
  pinMode(led, OUTPUT);
  Serial.begin(9600);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    if (c == '1') digitalWrite(led, HIGH);
    if (c == '0') digitalWrite(led, LOW);
  }
}

-----------------------------------------------------------
Save as: control_arduino.py  

import serial

ser = serial.Serial('COM3', 9600)   # Change COM port based on device manager

while True:
    ser.write(input("> ").encode())

Usage:
Type 1 → LED ON
Type 0 → LED OFF

-----------------------------------------------------------
Save as: rpi_serial_led.py  

import RPi.GPIO as GPIO
import serial

GPIO.setmode(GPIO.BCM)
GPIO.setup(17, GPIO.OUT)

ser = serial.Serial('/dev/ttyS0', 9600)

while True:
    cmd = ser.read().decode().strip()
    if cmd == "1":
        GPIO.output(17, True)
    elif cmd == "0":
        GPIO.output(17, False)


PC PYTHON SCRIPT TO CONTROL RASPBERRY PI

Save as: control_rpi.py  

import serial

ser = serial.Serial('COM4', 9600)   # Change based on your USB-Serial adapter

while True:
    ser.write(input("> ").encode())

Usage:
Type 1 → LED ON (on Raspberry Pi)
Type 0 → LED OFF


Serial Communication wiring (if using USB-serial adapter):

#RaspberryPI:

sudo apt update
sudo apt install python3-serial

Enable UART:

sudo raspi-config  
→ Interface Options → Enable Serial Port  
(DISABLE login shell, ENABLE serial hardware)

Reboot Pi:

sudo reboot
