#!/usr/bin/env python3
import rospy
import rospkg
import time
import socket
from std_msgs.msg import String

class ServerNode(object):
    def __init__(self):
        print('Booting up navigation node...')
        rospy.init_node('server', anonymous=True)
        self.server_pub = rospy.Publisher('server_data', String, queue_size=4)
        self.rate = rospy.Rate(10) # 20Hz
        self.host = ''#"127.0.1.1"
        self.port = 5555
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((self.host, self.port))
        self.sock.settimeout(0.1)
        self.sock.listen(1)

    def check_for_client(self):
        try:
            c, addr = self.sock.accept()
        except socket.timeout:
            return False

        data = c.recv(1024)
        if not data:
            c.close()
            return False

        cmd = str(data, 'utf-8')
        print("Received this: " + cmd)
        c.send("OK".encode('utf-8'))
        c.close()
        self.server_pub.publish(cmd)
        return True

    def cleanup(self):
        print("Cleaning up...")
        self.sock.close()

if __name__ == "__main__":
    server = ServerNode()
    print("starting the main loop")
    while not rospy.is_shutdown():
        try:
            if server.check_for_client():
                pass
            else:
                pass
            server.rate.sleep()
        except rospy.ROSInterruptException:
            server.cleanup()
