# Server program

import socket

SERVER_IP = "131.230.191.195"
SERVER_PORT = 5006
BUFFER_SIZE = 1024

def get_my_IP():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.connect(("8.8.8.8", 80))
    return sock.getsockname()[0]


def main():
    IP = get_my_IP()
    print("My IP = " + str(IP))
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.bind((SERVER_IP, SERVER_PORT))
    sock.listen(1)
    message = "none"
    conn, addr = sock.accept()
    while(message != "end"):
        data = conn.recv(BUFFER_SIZE).decode("utf-8")
        conn.send(data.encode("utf-8"))
        print(data)
    print("Exiting loop")
    sock.close()
    
    return    

if __name__ == "__main__":
    main()
