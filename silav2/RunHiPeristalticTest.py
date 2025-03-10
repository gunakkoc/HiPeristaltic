from HiPeristaltic import Server
from time import sleep

server = Server()
server.start("127.0.0.1",50052)
print("Server started")
while True:
    sleep(1)
