#!/usr/bin/env python3
# Shebang satırı, scriptin Unix benzeri sistemlerde doğrudan çalıştırılabilmesini sağlar.

import math
import rclpy
from functools import partial
from rclpy.node import Node

from turtlesim.msg import Pose  # Kaplumbağanın mevcut pozisyonunu almak için.
from geometry_msgs.msg import Twist  # Kaplumbağayı hareket ettirmek için.
from my_robot_interfaces.msg import Turtle, TurtleArray  # Özel mesaj türleri.
from my_robot_interfaces.srv import CatchTurtle  # Kaplumbağa yakalama servisi.

class TurtleControllerNode(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        # Parametreleri başlatma ve varsayılan değer atama.
        self.declare_parameter("catch_closest_turtle_first", True)

        # Parametrelerin değerlerini al.
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value
        self.turtle_to_catch_ = None  # Yakalanacak kaplumbağa.
        self.pose_ = None  # Kaplumbağanın mevcut pozisyonu.

        # Yayıncı ve abone oluşturma.
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        
        # Kontrol döngüsü için zamanlayıcı oluşturma.
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)

    # Kaplumbağanın mevcut pozisyonunu güncelleyen geri arama fonksiyonu.
    def callback_turtle_pose(self, msg):
        self.pose_ = msg

    # Yaşayan kaplumbağalar hakkında bilgi alan geri arama fonksiyonu.
    def callback_alive_turtles(self, msg):
        # En yakın kaplumbağayı yakalama mantığı.
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closest_turtle = None
                closest_turtle_distance = None
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closest_turtle is None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance
                self.turtle_to_catch_ = closest_turtle
            else:
                # İlk kaplumbağayı yakala.
                self.turtle_to_catch_ = msg.turtles[0]

    # Kontrol döngüsü: Kaplumbağayı hareket ettirme ve hedefe ulaşma mantığı.
    def control_loop(self):
        if self.pose_ is None or self.turtle_to_catch_ is None:
            return  # Mevcut pozisyon veya hedef bilgisi yoksa çık.

        # Hedefe olan mesafeyi hesaplama.
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        msg = Twist()  # Hareket mesajı.

        if distance > 0.5:
            # Hedefe doğru ilerleme.
            msg.linear.x = 2 * distance
            # Hedefe dönme.
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            # Açısal düzeltme.
            if diff > math.pi:
                diff -= 2*math.pi
            elif diff < -math.pi:
                diff += 2*math.pi
            msg.angular.z = 6 * diff
        else:
            # Hedefe ulaşıldı!
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None

        self.cmd_vel_publisher_.publish(msg)

    # Kaplumbağa yakalama servisini çağırma.
    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Server...")

        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_call_catch_turtle, turtle_name=turtle_name))

    # Kaplumbağa yakalama servisi sonuçlandığında çağrılan geri arama fonksiyonu.
    def callback_call_catch_turtle(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error(f"Turtle {turtle_name} could not be caught")
        except Exception as e:
            self.get_logger().error(f"Service call failed {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleControllerNode()
    rclpy.spin(node)  # Düğümü etkinleştir ve gelen iletileri işle.
    rclpy.shutdown()  # Düğümü kapat ve kaynakları serbest bırak.

if __name__ == "__main__":
    main()

"""
Bu kod parçası, bir TurtleControllerNode sınıfı tanımlar ve bu sınıf, turtlesim simülasyonunda kaplumbağaları otomatik 
olarak yakalamak için kullanılır.

Yakalama stratejisi, en yakın kaplumbağayı öncelikli olarak yakalamayı veya sıradaki ilk kaplumbağayı yakalamayı içerebilir. 
Kaplumbağanın mevcut konumunu dinler, yaşayan kaplumbağaların listesini alır ve en yakın veya ilk kaplumbağayı 
hedef olarak belirleyerek ona doğru hareket eder. 

Hedef kaplumbağa belirlenen mesafeye (bu örnekte 0.5 birim) ulaştığında, kaplumbağa yakalama servisini çağırır.
"""