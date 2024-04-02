# ROS (Robot Operating System) launch dosyası için gerekli olan modülleri içe aktar.
from launch import LaunchDescription
from launch_ros.actions import Node

# Bir launch tanımı oluşturan fonksiyon. Bu fonksiyon, ROS düğümlerini başlatmak için kullanılacak yapılandırmayı içerir.
def generate_launch_description():
    # LaunchDescription nesnesini oluştur. Bu nesne, başlatılacak düğümlerin listesini tutar.
    ld = LaunchDescription()

    # Turtlesim düğümü oluştur. Bu, ROS'un basit bir simülasyon ortamı sağlayan standart bir paketidir.
    turtlesim_node = Node(
        package="turtlesim",  # Düğümün bulunduğu paketin adı.
        executable="turtlesim_node"  # Çalıştırılacak düğümün adı.
    )

    # Turtle spawner düğümü oluştur. Bu, özel bir düğüm olup belirli aralıklarla yeni kaplumbağalar yaratır.
    turtle_spawner_node = Node(
        package="turtlesim_catch_them_all",  # Düğümün bulunduğu özel paketin adı.
        executable="turtle_spawner",  # Çalıştırılacak düğümün adı.
        parameters=[  # Düğüm için parametreler.
            {"spawn_frequency": 1.5},  # Yeni kaplumbağaların ne sıklıkta yaratılacağı (saniye cinsinden).
            {"turtle_name_prefix": "my_turtle"}  # Yaratılan kaplumbağalara verilecek isim öneki.
        ]
    )

    # Turtle controller düğümü oluştur. Bu, kaplumbağaları yakalamak için hareket ettirilen kontrolcüdür.
    turtle_controller_node = Node(
        package="turtlesim_catch_them_all",  # Düğümün bulunduğu özel paketin adı.
        executable="turtle_controller",  # Çalıştırılacak düğümün adı.
        parameters=[  # Düğüm için parametreler.
            {"catch_closest_turtle_first": True}  # En yakın kaplumbağayı ilk yakalamak için bir strateji belirler.
        ]
    )

    # Oluşturulan düğümleri launch tanımına ekle.
    ld.add_action(turtlesim_node)
    ld.add_action(turtle_spawner_node)
    ld.add_action(turtle_controller_node)

    # Hazırlanan launch tanımını döndür. Bu tanım, ROS tarafından kullanılarak belirtilen düğümleri başlatır.
    return ld