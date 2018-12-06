from serial import *
import csv
import threading
from os import mkdir
import ntpath
timestamper = 0
ki = 0  # 0 - 5
kp = 0  # 0 - 5
kd = 0  # 0 - 5
recording = False

def step_backward():
    start()
    serial_port.write("p\n".encode())
    serial_port.write("-50\n".encode())
    time.sleep(0.5)
    stop()

def step_forward():
    start()
    serial_port.write("p\n".encode())
    serial_port.write("50\n".encode())
    time.sleep(0.5)
    stop()

def start():
    global recording
    recording = True
    serial_port.write("start\n".encode())

def stop():
    global recording,plotting, timestamp, setpoint, pos_encodeur, pwm_envoye, erreur_derivative, erreur_integrale
    serial_port.write("stop\n".encode())
    recording = False

def set_constante(a,b,c):
    serial_port.write("kp\n".encode())
    serial_port.write((str(a)+"\n").encode())
    serial_port.write("ki\n".encode())
    serial_port.write((str(b)+"\n").encode())
    serial_port.write("kd\n".encode())
    serial_port.write((str(c)+"\n").encode())

class MyThread(threading.Thread):
    def run(self):
        global recording,timestamp, setpoint, pos_encodeur, pwm_envoye, erreur_derivative, erreur_integrale
        while True:
            timestamper = 0
            while recording == True:
                if serial_port.isOpen():
                    data = str(serial_port.readline())
                    list_data = data.split()
                    if(list_data[0] != "ORDER"):
                        list_data[0] = timestamper
                        timestamper += 50
                        c.writerow(list_data)

if __name__ == '__main__':
    if not ntpath.isdir("../data"):
        mkdir("../data")
    serial_port = Serial(port="COM7", baudrate=9600)
    #c = csv.writer(open("MONFICHIER.csv", "wb"))
    Thread = MyThread()
    Thread.start()
    for p in range(50):
        kp = p * 0.1    # 0.0-5.0
        for i in range(40):
            ki = i * 0.05 # 0.0-2.0
            for d in range(60):
                kd = d*0.05 # 0.0-3.0
                set_constante(kp, ki, kd)
                print(kp, ki, kd)
                file = open("../data/{kpp}_{kii}_{kdd}.csv".format(kpp=kp,kii=ki,kdd=kd),'w')
                c = csv.writer(file)
                step_forward()
                set_constante(0, 0, 0)
                serial_port.write("r\n".encode())
                time.sleep(0.1)