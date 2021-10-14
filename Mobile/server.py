import tornado.ioloop
import tornado.httpserver
import tornado.web
import tornado.options
import socket
import threading

from tornado.options import define, options

define("port", type=int, default=12345, help="run on the given port")

status = "Here is the interface of cat litter box+! The data received is:\n"



def receive_packet():
    while 1:
        data, addr = sck.recvfrom(1024)
        print(data)
        global status
        status += str(data)[2:-5] + "\n"

class index_handler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html', status=status)

class request_handler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html', status=status)

settings = {"debug": True,}
urls = [(r"/", index_handler),(r"/request", request_handler),]

def web_server():
    tornado.options.parse_command_line()
    app = tornado.web.Application(urls, **settings)
    app.listen(options.port)
    tornado.ioloop.IOLoop.current().start()


if __name__ == "__main__":
    SERVER_UDP_IP = "192.168.0.136"
    SERVER_UDP_PORT = 9999
    sck = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sck.bind((SERVER_UDP_IP, SERVER_UDP_PORT))
    threading.Thread(target=receive_packet, args=()).start()
    web_server()

