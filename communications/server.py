# Master program

import socket

SERVER_IP = "131.230.194.43"
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
    sock.connect((SERVER_IP, SERVER_PORT))
    message = "empty"
    print("Entering loop")
    while(message != "end"):
        message = input("Enter your message, end to quit")
        sock.send(str(message))
        data = sock.recv(BUFFER_SIZE)
        print(data)
    print("Exiting loop")
    sock.close()
    
    return    

if __name__ == "__main__":
    main()
