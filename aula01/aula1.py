import math
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

ERRO = 0.5


class Robot:
    """
    Classe que representa o robô.
    """

    def __init__(self, pos_x, pos_y, theta):
        self.x = pos_x
        self.y = pos_y
        self.theta = theta


robot = Robot(0.0, 0.0, 0.0)


def pose_callback(pose):
    """
    Função que é chamada sempre que uma nova mensagem de pose é recebida.
    """
    robot.x = pose.x
    robot.y = pose.y
    robot.theta = pose.theta


def controlador(x_t, y_t, theta_t, x_d, y_d):
    """
    Função que calcula a velocidade linear e angular do robô.
    """
    theta_d = math.atan2((y_d - y_t), (x_d - x_t))

    angle_diff = theta_d - theta_t
    angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

    k_theta = 1.2
    k_lin = 0.35

    ang_vel = k_theta * angle_diff
    lin_vel = k_lin * math.sqrt((x_d - x_t) ** 2 + (y_d - y_t) ** 2)
    return lin_vel, ang_vel


def move_turtle(pos_x, pos_y):
    """
    Função que move o robô para uma posição desejada.
    """
    pub = rospy.Publisher("/turtle1/cmd_vel", Twist, queue_size=10)
    rospy.Subscriber("/turtle1/pose", Pose, pose_callback)
    rate = rospy.Rate(10)  # 10 Hz
    vel = Twist()

    while not rospy.is_shutdown():
        lin_vel, ang_vel = controlador(robot.x, robot.y, robot.theta, pos_x, pos_y)

        vel.linear.x = lin_vel
        vel.linear.y = 0
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel

        distance = math.sqrt((pos_x - robot.x) ** 2 + (pos_y - robot.y) ** 2)

        if distance <= ERRO:
            rospy.loginfo("Destino alcançado: (%.2f, %.2f)", pos_x, pos_y)
            break

        pub.publish(vel)
        rate.sleep()


coordenadas = [
    (6.5, 5.5),
    (6.5, 1.5),
    (1.5, 5.5),
    (3.5, 8.5),
    (7.5, 4.5),
    (2.5, 6.5),
    (9.5, 1.5),
    (2.5, 4.5),
    (3.5, 1.5),
    (5.5, 9.5),
    (1.5, 2.5),
    (5.5, 6.5),
    (3.5, 6.5),
    (9.5, 5.5),
    (3.5, 3.5),
    (9.5, 4.5),
    (4.5, 6.5),
    (2.5, 9.5),
    (1.5, 1.5),
    (7.5, 2.5),
    (6.5, 7.5),
    (3.5, 9.5),
    (8.5, 6.5),
    (2.5, 1.5),
    (1.5, 3.5),
    (7.5, 3.5),
    (1.5, 6.5),
    (4.5, 2.5),
    (6.5, 6.5),
    (8.5, 8.5),
    (7.5, 5.5),
    (5.5, 4.5),
    (5.5, 3.5),
    (8.5, 4.5),
    (4.5, 4.5),
    (9.5, 8.5),
    (2.5, 8.5),
    (8.5, 7.5),
    (7.5, 8.5),
    (9.5, 9.5),
    (6.5, 2.5),
    (1.5, 7.5),
    (3.5, 7.5),
    (4.5, 5.5),
    (4.5, 7.5),
    (5.5, 2.5),
    (6.5, 9.5),
    (2.5, 7.5),
    (2.5, 3.5),
    (3.5, 2.5),
    (9.5, 3.5),
    (6.5, 3.5),
    (7.5, 7.5),
    (5.5, 7.5),
    (3.5, 5.5),
    (5.5, 5.5),
    (9.5, 6.5),
    (1.5, 9.5),
    (8.5, 3.5),
    (7.5, 6.5),
    (4.5, 9.5),
    (2.5, 2.5),
    (7.5, 1.5),
    (4.5, 8.5),
    (6.5, 8.5),
    (2.5, 5.5),
    (8.5, 9.5),
    (8.5, 5.5),
    (4.5, 1.5),
    (8.5, 2.5),
    (3.5, 4.5),
    (1.5, 8.5),
    (5.5, 8.5),
    (6.5, 4.5),
    (1.5, 4.5),
    (5.5, 1.5),
    (7.5, 9.5),
    (9.5, 7.5),
    (4.5, 3.5),
    (8.5, 1.5),
    (9.5, 2.5),
    (6.5, 0.5),
    (0.5, 1.5),
    (0.5, 0.5),
]

if __name__ == "__main__":
    try:
        rospy.init_node("move_turtle", anonymous=False)
        for x, y in coordenadas:
            rospy.loginfo("Indo para (%.2f, %.2f)", x, y)
            move_turtle(x, y)

    except rospy.ROSInterruptException:
        pass
