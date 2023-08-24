import socket 
import threading

import time
import serial

HEADER = 64
PORT = 5791
#SERVER = socket.gethostbyname(socket.gethostname())
SERVER = "192.168.1.103"
ADDR = (SERVER, PORT)
FORMAT = 'utf-8'
DISCONNECT_MESSAGE = "!DISCONNECT"

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(ADDR)
count = 0

arduino = serial.Serial(port='/dev/ttyACM0', baudrate=9600, timeout=.1)
def write(x):
    arduino.write(bytes(x, 'utf-8'))
    time.sleep(0.05)
    
def read():
    data = arduino.readline()
    return data

def handle_client(conn, addr):
    global count
    print(f"[NEW CONNECTION] {addr} connected.")
    connected = True
    try: 
        while connected:
#             msg_length = conn.recv(HEADER).decode(FORMAT)
            value = read()
            conn.send(str(value).encode(FORMAT))
            print(value)
        conn.close()
    except KeyboardInterrupt:
        ser.close()

def start():
    server.listen()
    print(f"[LISTENING] Server is listening on {SERVER}")
    while True:
        conn, addr = server.accept()
        thread = threading.Thread(target=handle_client, args=(conn,addr))
        thread.start()
        print(f"[ACTIVE CONNETIONS] {threading.active_count() - 1}")

print("[STARTING] server is starting...")
start()






