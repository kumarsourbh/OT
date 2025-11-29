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
