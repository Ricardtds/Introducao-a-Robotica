import math
import heapq
import logging
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
import numpy as np
from scipy.ndimage import binary_dilation


class ExplorationNavigator:
    """
    Navegador Autônomo com Lógica Anti-Ciclo para Exploração de Ambientes Desconhecidos.
    """

    def __init__(self):
        logger = logging.getLogger("ros.tf2_ros")
        logger.setLevel(logging.ERROR)

        rospy.init_node("exploration_navigator_node")

        self.goal_x = 4.0
        self.goal_y = 0.0
        self.robot_frame = "base_footprint"
        self.map_frame = "map"
        self.obstacle_threshold = 50
        self.robot_radius = 0.22

        self.map_data = None
        self.inflated_map_data = None
        self.map_info = None
        self.robot_pose = None
        self.failed_frontiers = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_pub = rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.path_pub = rospy.Publisher("/planned_path", Path, queue_size=10)
        self.map_subscriber = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.inflated_map_pub = rospy.Publisher(
            "/inflated_map", OccupancyGrid, queue_size=1, latch=True
        )

        rospy.loginfo("==> Navegador Autônomo com Lógica Anti-Ciclo Iniciado <==")
        rospy.loginfo(f"Destino final: (x={self.goal_x}, y={self.goal_y})")

    def map_callback(self, msg):
        """
        Callback para receber o mapa e inflar os obstáculos.
        Recebe o mapa do tópico /map e processa os dados para inflar os obstáculos.
        O mapa é convertido em uma matriz numpy e os obstáculos são
        inflados com base no raio do robô.
        Se o mapa já estiver inflado, ele é atualizado com os novos dados.
        Se o mapa não tiver resolução, a função retorna sem fazer nada.
        Se o mapa for válido, ele é convertido em uma matriz numpy e os obstáculos são inflados.
        A matriz inflada é então publicada no tópico /inflated_map.
        """
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape(
            (self.map_info.height, self.map_info.width)
        )
        self.inflate_map()

    def inflate_map(self):
        """
        Infla os obstáculos no mapa com base no raio do robô.
        Utiliza uma abordagem de dilatação para expandir os obstáculos no mapa.
        A função verifica se o mapa e a resolução são válidos antes de inflar os obstáculos
        e publica o mapa inflado no tópico /inflated_map.
        """
        if self.map_data is None or self.map_info.resolution == 0:
            return
        self.inflated_map_data = self.map_data.copy()
        radius_cells = int(np.ceil(self.robot_radius / self.map_info.resolution))
        obstacle_coords = np.where(self.map_data > self.obstacle_threshold)
        for y, x in zip(*obstacle_coords):
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx * dx + dy * dy <= radius_cells * radius_cells:
                        nx, ny = x + dx, y + dy
                        if (
                            0 <= nx < self.map_info.width
                            and 0 <= ny < self.map_info.height
                        ):
                            self.inflated_map_data[ny, nx] = 100
        self.publish_inflated_map()

    def publish_inflated_map(self):
        """
        Publica o mapa inflado no tópico /inflated_map.
        Cria uma mensagem OccupancyGrid com os dados do mapa inflado e publica no tópico
        /inflated_map. A mensagem inclui o timestamp atual e o frame_id do mapa.
        Se o mapa inflado não estiver disponível, a função retorna sem fazer nada.
        Se o mapa inflado estiver disponível, ele é convertido em uma lista e publicado.
        Se o mapa inflado não tiver dados, a função retorna sem fazer nada.
        Se o mapa inflado tiver dados, ele é convertido em uma lista e publicado.
        """
        if self.inflated_map_data is None:
            return
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.info = self.map_info
        msg.data = self.inflated_map_data.flatten().tolist()
        self.inflated_map_pub.publish(msg)

    def get_robot_pose(self):
        """
        Obtém a pose atual do robô no frame do mapa.
        Utiliza o tf2 para buscar a transformação entre o frame do mapa e o frame do
        robô. Se a transformação for bem-sucedida, a pose do robô é atualizada e retornada.
        Se a transformação falhar, a função retorna False.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0)
            )
            self.robot_pose = PoseStamped()
            self.robot_pose.header.frame_id = self.map_frame
            self.robot_pose.pose.position = transform.transform.translation
            self.robot_pose.pose.orientation = transform.transform.rotation
            return True
        except Exception:
            return False

    def world_to_grid(self, world_x, world_y):
        """
        Converte coordenadas do mundo para coordenadas de grade.
        Recebe as coordenadas x e y do mundo e converte para coordenadas de grade
        com base na origem do mapa e na resolução do mapa. Se o mapa não estiver disponível
        ou a resolução for zero, retorna None.
        Se o mapa não estiver disponível ou a resolução for zero, retorna None.
        Se o mapa estiver disponível e a resolução for válida, calcula as coordenadas de grade
        e as retorna como uma tupla (grid_x, grid_y).
        """
        if self.map_data is None or self.map_info.resolution == 0.0:
            return None, None
        grid_x = int(
            (world_x - self.map_info.origin.position.x) / self.map_info.resolution
        )
        grid_y = int(
            (world_y - self.map_info.origin.position.y) / self.map_info.resolution
        )
        return grid_x, grid_y

    def is_obstacle(self, grid_x, grid_y):
        """
        Verifica se uma posição na grade é um obstáculo.
        Recebe as coordenadas x e y da grade e verifica se a posição correspondente no
        mapa inflado é um obstáculo. Se o mapa inflado não estiver disponível, retorna True.
        Se as coordenadas estiverem fora dos limites do mapa, retorna True.
        Se as coordenadas estiverem dentro dos limites do mapa, verifica se a posição
        correspondente no mapa inflado é um obstáculo (valor maior que o limiar de obstáculo).
        Se o mapa inflado não estiver disponível, retorna True.
        Se as coordenadas estiverem fora dos limites do mapa, retorna True.
        Se as coordenadas estiverem dentro dos limites do mapa, verifica se a posição
        correspondente no mapa inflado é um obstáculo (valor maior que o limiar de obstáculo).
        Se a posição for um obstáculo, retorna True; caso contrário, retorna False.
        """
        if self.inflated_map_data is None:
            return True
        if 0 <= grid_x < self.map_info.width and 0 <= grid_y < self.map_info.height:
            return self.inflated_map_data[grid_y, grid_x] > self.obstacle_threshold
        return True

    def is_goal_accessible(self):
        """
        Verifica se o destino final é acessível.
        Converte as coordenadas do destino final para coordenadas de grade e verifica se
        a posição correspondente no mapa inflado é um espaço livre (valor igual a 0).
        Se as coordenadas do destino final não puderem ser convertidas para coordenadas de grade
        (por exemplo, se estiverem fora dos limites do mapa), retorna False.
        Se as coordenadas do destino final puderem ser convertidas para coordenadas de grade,
        verifica se a posição correspondente no mapa inflado é um espaço livre (valor igual a
        0). Se for um espaço livre, retorna True; caso contrário, retorna False.
        """
        goal_grid_x, goal_grid_y = self.world_to_grid(self.goal_x, self.goal_y)
        if goal_grid_x is None:
            return False
        return self.map_data[goal_grid_y, goal_grid_x] == 0

    def find_frontiers(self):
        """
        Encontra as fronteiras no mapa inflado.
        Identifica as fronteiras onde o espaço livre (valor 0) encontra espaço desconhecido
        (valor -1) no mapa inflado. Utiliza a dilatação binária para expandir o espaço livre
        e identificar as bordas onde o espaço livre encontra o espaço desconhecido.
        Retorna uma lista de coordenadas (x, y) das fronteiras encontradas.
        Se o mapa inflado não estiver disponível, retorna uma lista vazia.
        Se o mapa inflado estiver disponível, identifica as fronteiras onde o espaço livre
        (valor 0) encontra espaço desconhecido (valor -1) e retorna uma lista de coordenadas
        (x, y) das fronteiras encontradas.
        """
        if self.map_data is None:
            return []

        free_space = self.map_data == 0
        unknown_space = self.map_data == -1
        dilated_free = binary_dilation(free_space)
        edges = dilated_free & unknown_space
        frontier_coords = np.where(edges)
        return list(zip(frontier_coords[1], frontier_coords[0]))

    def choose_best_frontier(self, frontiers, robot_grid_pos):
        """
        Escolhe a melhor fronteira para explorar.
        Recebe uma lista de fronteiras e a posição do robô em coordenadas de grade
        e escolhe a fronteira mais próxima do robô que não está perto de uma fronteira falha.
        Se não houver fronteiras válidas, retorna None.
        """
        if not frontiers:
            rospy.logwarn("choose_best_frontier: Nenhuma fronteira recebida.")
            return None

        failed_frontier_exclusion_radius = int(
            1.0 / self.map_info.resolution
        )  # 1 metro

        valid_frontiers = []
        for f in frontiers:
            is_near_failed = False
            for failed_f in self.failed_frontiers:
                if (
                    math.hypot(f[0] - failed_f[0], f[1] - failed_f[1])
                    < failed_frontier_exclusion_radius
                ):
                    is_near_failed = True
                    break
            if is_near_failed:
                continue
            valid_frontiers.append(f)

        if not valid_frontiers:
            rospy.logwarn(
                "Nenhuma fronteira válida encontrada fora das zonas de falha. "
                "A limpar a lista de falhas para tentar novamente."
            )
            self.failed_frontiers.clear()
            valid_frontiers = frontiers

        return min(
            valid_frontiers,
            key=lambda f: math.hypot(
                f[0] - robot_grid_pos[0], f[1] - robot_grid_pos[1]
            ),
        )

    def astar_search(self, start_grid, goal_grid):
        """
        Implementa o algoritmo A* para encontrar o caminho mais curto entre dois pontos na grade.
        Recebe as coordenadas de grade de início e fim e retorna uma lista de coordenadas
        representando o caminho mais curto. Se não houver caminho, retorna None.
        O algoritmo A* utiliza uma fila de prioridade para explorar os nós com menor custo total
        (custo g + custo heurístico h). A função verifica os vizinhos válidos
        e atualiza os custos g e h para encontrar o caminho mais curto.
        Se não houver caminho, retorna None.
        Se houver caminho, retorna uma lista de coordenadas representando o caminho mais curto.
        """
        open_set = [(0, start_grid)]
        came_from = {}
        g_cost = {start_grid: 0}
        while open_set:
            _, current = heapq.heappop(open_set)
            if current == goal_grid:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start_grid)
                return path[::-1]
            for dx, dy in [
                (0, 1),
                (0, -1),
                (1, 0),
                (-1, 0),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ]:
                neighbor = (current[0] + dx, current[1] + dy)
                if self.is_obstacle(neighbor[0], neighbor[1]):
                    continue
                tentative_g_cost = g_cost[current] + math.hypot(dx, dy)
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g_cost
                    h_cost = math.hypot(
                        neighbor[0] - goal_grid[0], neighbor[1] - goal_grid[1]
                    )
                    heapq.heappush(open_set, (tentative_g_cost + h_cost, neighbor))
                    came_from[neighbor] = current
        return None

    def publish_path_for_rviz(self, grid_path):
        """
        Publica o caminho encontrado no formato Path para visualização no RViz.
        Recebe uma lista de coordenadas de grade e converte para coordenadas do mundo
        com base na resolução do mapa e na origem do mapa. Cria uma mensagem Path com
        as poses correspondentes e publica no tópico /planned_path.
        Se a lista de coordenadas de grade estiver vazia, não faz nada.
        """
        if not grid_path:
            return
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = rospy.Time.now()
        for grid_pos in grid_path:
            world_x = (
                grid_pos[0] * self.map_info.resolution
            ) + self.map_info.origin.position.x
            world_y = (
                grid_pos[1] * self.map_info.resolution
            ) + self.map_info.origin.position.y
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def follow_path(self, path):
        """
        Segue o caminho dado, movendo o robô através dos waypoints.
        Recebe uma lista de waypoints em coordenadas de grade e move o robô para
        cada waypoint, verificando se o robô está preso ou se o tempo limite é atingido.
        Se o caminho estiver vazio ou se o robô não puder obter sua pose, retorna False.
        Se o robô conseguir seguir o caminho até o final, retorna True.
        """
        if not path or len(path) < 2:
            return False
        rate = rospy.Rate(10)
        for waypoint_grid in path[::5] + [path[-1]]:
            waypoint_world_x = (
                waypoint_grid[0] * self.map_info.resolution
            ) + self.map_info.origin.position.x
            waypoint_world_y = (
                waypoint_grid[1] * self.map_info.resolution
            ) + self.map_info.origin.position.y
            start_time = rospy.Time.now()
            timeout = rospy.Duration(20.0)
            last_pose_time = rospy.Time.now()
            if not self.get_robot_pose():
                return False
            last_position = self.robot_pose.pose.position
            while not rospy.is_shutdown():
                if rospy.Time.now() - start_time > timeout:
                    rospy.logwarn(
                        "Tempo limite atingido para o waypoint. Abortando caminho."
                    )
                    self.vel_pub.publish(Twist())
                    return False
                if not self.get_robot_pose():
                    continue
                if rospy.Time.now() - last_pose_time > rospy.Duration(4.0):
                    current_position = self.robot_pose.pose.position
                    dist_moved = math.hypot(
                        current_position.x - last_position.x,
                        current_position.y - last_position.y,
                    )
                    if dist_moved < 0.1:
                        rospy.logwarn(
                            "Robô parece estar preso. Abortando este caminho."
                        )
                        self.vel_pub.publish(Twist())
                        return False
                    last_pose_time = rospy.Time.now()
                    last_position = current_position
                dx = waypoint_world_x - self.robot_pose.pose.position.x
                dy = waypoint_world_y - self.robot_pose.pose.position.y
                if math.hypot(dx, dy) < 0.3:
                    break
                angle_to_goal = math.atan2(dy, dx)
                _, _, current_yaw = euler_from_quaternion(
                    [
                        self.robot_pose.pose.orientation.x,
                        self.robot_pose.pose.orientation.y,
                        self.robot_pose.pose.orientation.z,
                        self.robot_pose.pose.orientation.w,
                    ]
                )
                angle_diff = (angle_to_goal - current_yaw + math.pi) % (
                    2 * math.pi
                ) - math.pi
                twist_msg = Twist()
                if abs(angle_diff) > 0.3:
                    twist_msg.linear.x = 0.0
                    twist_msg.angular.z = 0.6 * (1 if angle_diff > 0 else -1)
                else:
                    twist_msg.linear.x = 0.35
                    twist_msg.angular.z = 1.0 * angle_diff
                self.vel_pub.publish(twist_msg)
                rate.sleep()
        self.vel_pub.publish(Twist())
        return True

    def recovery_behavior(self):
        """
        Comportamento de recuperação: recua e roda 180 graus.
        Executa uma manobra de recuperação quando o robô não consegue alcançar o destino
        ou quando o caminho para a fronteira falha. O robô recua por um curto período
        e depois roda 180 graus para tentar encontrar um novo caminho.
        Se o robô não conseguir obter sua pose, a função retorna sem fazer nada.
        Se o robô conseguir obter sua pose, ele recua por 1.5 segundos e depois roda
        por 10 segundos. Após a manobra, o robô para e aguarda 1 segundo.
        """
        rospy.logwarn("A iniciar comportamento de recuperação: a recuar e a rodar.")
        twist_msg = Twist()
        rate = rospy.Rate(10)
        move_back_time = rospy.Duration(1.5)
        start_time = rospy.Time.now()
        twist_msg.linear.x = -0.15
        while (
            rospy.Time.now() - start_time < move_back_time and not rospy.is_shutdown()
        ):
            self.vel_pub.publish(twist_msg)
            rate.sleep()
        rotate_time = rospy.Duration(10.0)
        start_time = rospy.Time.now()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = 0.7
        while rospy.Time.now() - start_time < rotate_time and not rospy.is_shutdown():
            self.vel_pub.publish(twist_msg)
            rate.sleep()
        self.vel_pub.publish(Twist())
        rospy.loginfo("Comportamento de recuperação concluído.")
        rospy.sleep(1)

    def initial_maneuver(self):
        """
        Executa uma manobra inicial: recua e depois roda 180 graus.
        """
        rospy.loginfo("A executar manobra inicial: recuar e rodar 180 graus...")
        twist_msg = Twist()
        rate = rospy.Rate(10)

        # 1. Mover para trás
        move_back_time = rospy.Duration(1.5)
        start_time = rospy.Time.now()
        twist_msg.linear.x = -0.2
        while (
            rospy.Time.now() - start_time < move_back_time and not rospy.is_shutdown()
        ):
            self.vel_pub.publish(twist_msg)
            rate.sleep()

        # 2. Rodar 180 graus
        rotate_speed = 0.7  # rad/s
        angle_to_rotate = math.pi  # 180 graus em radianos
        rotate_time = rospy.Duration(angle_to_rotate / rotate_speed)
        start_time = rospy.Time.now()
        twist_msg.linear.x = 0.0
        twist_msg.angular.z = rotate_speed
        while rospy.Time.now() - start_time < rotate_time and not rospy.is_shutdown():
            self.vel_pub.publish(twist_msg)
            rate.sleep()

        self.vel_pub.publish(Twist())
        rospy.loginfo("Manobra inicial concluída.")
        rospy.sleep(1)  # Pequena pausa

    def run(self):
        """
        Inicia o processo de exploração e navegação.
        Aguarda até que o mapa e a localização inicial do robô estejam disponíveis.
        Se o mapa ou a localização inicial não estiverem disponíveis, continua aguardando.
        Após obter o mapa e a localização inicial, executa a manobra inicial.
        Em seguida, entra em um loop de exploração onde busca fronteiras e tenta navegar
        para a fronteira mais promissora. Se o destino final for inacessível,
        continua explorando até encontrar um caminho para o destino.
        Se o robô não conseguir obter sua pose, a função retorna sem fazer nada.
        Se o robô conseguir obter sua pose, ele tenta encontrar um caminho para o destino final
        usando o algoritmo A*. Se um caminho for encontrado, publica o caminho no RViz e
        segue o caminho até o destino final. Se o destino final for alcançado, publica uma
        mensagem de sucesso e para o robô.
        """
        while (
            self.map_data is None or not self.get_robot_pose()
        ) and not rospy.is_shutdown():
            rospy.loginfo_throttle(
                2, "Aguardando mapa e localização inicial do robô..."
            )
            rospy.sleep(1)

        # Executa a manobra inicial
        self.initial_maneuver()

        while not self.is_goal_accessible() and not rospy.is_shutdown():
            rospy.loginfo_throttle(5, "FASE DE EXPLORAÇÃO: Destino inacessível.")
            if not self.get_robot_pose():
                continue

            robot_grid_pos = self.world_to_grid(
                self.robot_pose.pose.position.x, self.robot_pose.pose.position.y
            )
            frontiers = self.find_frontiers()

            if frontiers:
                target_frontier = self.choose_best_frontier(frontiers, robot_grid_pos)
                if target_frontier:
                    path_to_frontier = self.astar_search(
                        robot_grid_pos, target_frontier
                    )
                    if path_to_frontier:
                        rospy.loginfo(
                            "A navegar para a fronteira mais promissora: %s",
                            str(target_frontier),
                        )
                        self.publish_path_for_rviz(path_to_frontier)
                        success = self.follow_path(path_to_frontier)
                        if not success:
                            rospy.logwarn(
                                "Falha ao navegar para a fronteira %s. Adicionando à "
                                "lista de falhas.",
                                str(target_frontier),
                            )
                            self.failed_frontiers.append(target_frontier)
                            self.recovery_behavior()
                else:
                    self.recovery_behavior()
            else:
                rospy.logwarn_throttle(
                    5, "Nenhuma fronteira encontrada. A executar recuperação."
                )
                self.recovery_behavior()

        rospy.loginfo("FASE FINAL: Navegando para o destino.")
        if not self.get_robot_pose():
            return
        start_grid = self.world_to_grid(
            self.robot_pose.pose.position.x, self.robot_pose.pose.position.y
        )
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y)
        final_path = self.astar_search(start_grid, goal_grid)
        if final_path:
            self.publish_path_for_rviz(final_path)
            self.follow_path(final_path)
            rospy.loginfo(">>> Destino Final Alcançado! <<<")
        self.vel_pub.publish(Twist())


if __name__ == "__main__":
    try:
        navigator = ExplorationNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.Publisher("/cmd_vel_mux/input/navi", Twist, queue_size=1).publish(Twist())
