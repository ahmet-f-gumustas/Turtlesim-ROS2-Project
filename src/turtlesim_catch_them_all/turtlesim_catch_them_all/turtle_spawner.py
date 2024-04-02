#!/usr/bin/env python3
# Shebang satırı, scriptin doğrudan bir Unix benzeri sistemde çalıştırılabilmesi için gerekli.

# Gerekli modüllerin ve ROS 2 paketlerinin içe aktarılması.
from functools import partial
import random
import math
import rclpy
from rclpy.node import Node

from turtlesim.srv import Spawn, Kill  # Kaplumbağaları yaratmak ve öldürmek için gerekli servisler.
from my_robot_interfaces.msg import Turtle, TurtleArray  # Özel mesaj türleri.
from my_robot_interfaces.srv import CatchTurtle  # Kaplumbağa yakalama servisi.

# TurtleSpawner sınıfı, ROS 2 Node'u olarak tanımlanır.
class TurtleSpawner(Node):
    def __init__(self):
        # Sınıfın kurucusu, Node'u başlatır ve parametreleri tanımlar.
        super().__init__("turtle_spawner")
        self.declare_parameter("spawn_frequency", 1.0)
        self.declare_parameter("turtle_name_prefix", "turtle")

        # Parametrelerin değerlerini okur.
        self.spawn_frequency_ = self.get_parameter("spawn_frequency").value 
        self.turtle_name_prefix_ = self.get_parameter("turtle_name_prefix").value
        
        # İç durum değişkenlerini başlatır.
        self.turtle_counter_ = 0
        self.alive_turtles_ = []
        
        # Yaşayan kaplumbağaları yayınlamak için bir yayıncı oluşturur.
        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "alive_turtles", 10)
        
        # Yeni kaplumbağalar yaratmak için bir zamanlayıcı ayarlar.
        self.spawn_turtle_timer_ = self.create_timer(2.0/self.spawn_frequency_, self.spawn_new_turtle)
        
        # Kaplumbağa yakalama servisini tanımlar.
        self.catch_turtle_service_ = self.create_service(CatchTurtle, "catch_turtle", self.callback_catch_turtle)

    # Kaplumbağa yakalama isteği geldiğinde çağrılan geri arama fonksiyonu.
    def callback_catch_turtle(self, request, response):
        self.call_kill_server(request.name)  # Kaplumbağayı öldürme servisini çağırır.
        response.success = True
        return response

    # Yaşayan kaplumbağaları yayınlama fonksiyonu.
    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)

    # Yeni bir kaplumbağa yaratma fonksiyonu.
    def spawn_new_turtle(self):
        self.turtle_counter_ += 1
        name = self.turtle_name_prefix_ + str(self.turtle_counter_)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2*math.pi)
        self.call_spawn_server(name, x, y, theta)  # Kaplumbağa yaratma servisini çağırır.

    # Kaplumbağa yaratma servisini çağırma fonksiyonu.
    def call_spawn_server(self, turtle_name, x, y, theta):
        client = self.create_client(Spawn, "spawn")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_spawn, turtle_name=turtle_name, x=x, y=y, theta=theta))

    # Kaplumbağa yaratma servisi sonuçlandığında çağrılan geri arama fonksiyonu.
    def callback_call_spawn(self, future, turtle_name, x, y, theta):
        try:
            response = future.result()
            if response.name != "":
                self.get_logger().info(f"Turtle {response.name} is now alive")
                new_turtle = Turtle()
                new_turtle.name = response.name
                new_turtle.x = x
                new_turtle.y = y
                new_turtle.theta = theta
                self.alive_turtles_.append(new_turtle)
                self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

    # Kaplumbağayı öldürme servisini çağırma fonksiyonu.
    def call_kill_server(self, turtle_name):
        client = self.create_client(Kill, "kill")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = Kill.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill, turtle_name=turtle_name))

    # Kaplumbağayı öldürme servisi sonuçlandığında çağrılan geri arama fonksiyonu.
    def callback_call_kill(self, future, turtle_name):
        try:
            future.result()
            self.alive_turtles_ = [turtle for turtle in self.alive_turtles_ if turtle.name != turtle_name]
            self.publish_alive_turtles()
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

# Ana fonksiyon.
def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)  # Node'u etkinleştirir ve gelen iletileri işler.
    rclpy.shutdown()  # Node'u kapatır ve kaynakları serbest bırakır.

if __name__ == "__main__":
    main()


"""
Bu kod, ROS 2'de bir TurtleSpawner düğümü oluşturur. Bu düğüm, belirli aralıklarla yeni kaplumbağalar yaratır, 
yaşayan kaplumbağaların listesini yayınlar ve istek üzerine kaplumbağaları öldüren bir servis sağlar. 

Kaplumbağaların yaratılması ve öldürülmesi, turtlesim paketindeki servisler aracılığıyla gerçekleştirilir. 
Düğüm ayrıca, özel bir mesaj türü kullanarak yaşayan kaplumbağaların listesini yayınlamak için bir yayıncı barındırır.
"""