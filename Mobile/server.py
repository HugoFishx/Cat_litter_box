import tornado.ioloop
import tornado.httpserver
import tornado.web
import tornado.options
import time
import re
import requests
import datetime

from tornado.options import define, options

define("port", type=int, default=12345, help="run on the given port")

status = "Here is the interface of cat litter box+!\nThe data received is:\n"




# def receive_packet():
#     while 1:
#         data, addr = sck.recvfrom(1024)
#         print(data)
#         global status
#         status += str(data)[2:-1].replace("\\r", "").replace("\\n", "\n") # weird escape

class index_handler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html', status=status)

class request_handler(tornado.web.RequestHandler):
    def get(self):
        self.render('index.html', status=status)

class forward_handler(tornado.web.RequestHandler):
    def get(self, slug):
        print(slug)
        global status
        content = slug.replace("id=", "").replace("&n=", " ").split()
        now = datetime.datetime.now()

        if content[0] == "543775105":
            content[0] = "Bunny cat"
        elif content[0] == "446665459":
            content[0] = "Minnie cat"
        elif content[0] == "2043217032":
            content[0] = "Charlie Cat"
        elif content[0] == "2055341192":
            content[0] = "Alpha Cat"
        elif content[0] == "0":
            return

        minute = int(int(content[1]) // 1000 // 60)
        second = int(int(content[1]) // 1000 % 60)
        requests.get(
            'https://script.google.com/macros/s/AKfycbyh_ZyhGZlZZ0RUHVq3ln4q32Li58JKZe4ht98wsI1aOO4NZLQt13VN4Uf4fCyshDZQmA/exec' + '?' + "id=" + content[0] + "&n=" + str(minute) + " min " + str(second) + " second")
        status += now.strftime("%Y-%m-%d %H:%M:%S") + " ID: " + content[0] + " Duration: " + str(minute) + " min " + str(second) + " second" + "\n"
        self.render('index.html', status=status)


settings = {"debug": True,}
urls = [(r"/", index_handler),(r"/request", request_handler),(r"/query/([^/]+)", forward_handler),]

def web_server():
    tornado.options.parse_command_line()
    app = tornado.web.Application(urls, **settings)
    app.listen(options.port)
    tornado.ioloop.IOLoop.current().start()


if __name__ == "__main__":
    # SERVER_UDP_IP = "192.168.0.136"
    # SERVER_UDP_PORT = 9999
    # sck = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # sck.bind((SERVER_UDP_IP, SERVER_UDP_PORT))
    # threading.Thread(target=receive_packet, args=()).start()
    web_server()

