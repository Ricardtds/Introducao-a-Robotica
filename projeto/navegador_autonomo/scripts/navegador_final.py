#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped, Twist
from tf.transformations import euler_from_quaternion
import math
import heapq
import logging
import numpy as np

class ExplorationNavigator:
    def __init__(self):
        logger = logging.getLogger('ros.tf2_ros')
        logger.setLevel(logging.ERROR)

        rospy.init_node('exploration_navigator_node')

        self.goal_x = 4.0
        self.goal_y = 0.0
        self.robot_frame = 'base_footprint'
        self.map_frame = 'map'
        self.obstacle_threshold = 50
        
        # ATUALIZADO: Raio de segurança ajustado para um valor mais realista
        self.robot_radius = 0.22

        self.map_data = None
        self.inflated_map_data = None
        self.map_info = None
        self.robot_pose = None
        self.failed_frontiers = []

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.path_pub = rospy.Publisher('/planned_path', Path, queue_size=10)
        self.map_subscriber = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.inflated_map_pub = rospy.Publisher('/inflated_map', OccupancyGrid, queue_size=1, latch=True)

        rospy.loginfo("==> Navegador Autônomo com Exploração por Cluster Iniciado <==")
        rospy.loginfo("Destino final: (x={}, y={})".format(self.goal_x, self.goal_y))

    def map_callback(self, msg):
        self.map_info = msg.info
        self.map_data = np.array(msg.data).reshape((self.map_info.height, self.map_info.width))
        self.inflate_map()

    def inflate_map(self):
        if self.map_data is None or self.map_info.resolution == 0: return
        self.inflated_map_data = self.map_data.copy()
        radius_cells = int(np.ceil(self.robot_radius / self.map_info.resolution))
        obstacle_coords = np.where(self.map_data > self.obstacle_threshold)
        for y, x in zip(*obstacle_coords):
            for dy in range(-radius_cells, radius_cells + 1):
                for dx in range(-radius_cells, radius_cells + 1):
                    if dx*dx + dy*dy <= radius_cells*radius_cells:
                        nx, ny = x + dx, y + dy
                        if 0 <= nx < self.map_info.width and 0 <= ny < self.map_info.height:
                            self.inflated_map_data[ny, nx] = 100
        self.publish_inflated_map()

    def publish_inflated_map(self):
        if self.inflated_map_data is None: return
        msg = OccupancyGrid()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.map_frame
        msg.info = self.map_info
        msg.data = self.inflated_map_data.flatten().tolist()
        self.inflated_map_pub.publish(msg)

    def get_robot_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.map_frame, self.robot_frame, rospy.Time(0), rospy.Duration(1.0))
            self.robot_pose = PoseStamped()
            self.robot_pose.header.frame_id = self.map_frame
            self.robot_pose.pose.position = transform.transform.translation
            self.robot_pose.pose.orientation = transform.transform.rotation
            return True
        except Exception:
            return False

    def world_to_grid(self, world_x, world_y):
        if self.map_data is None or self.map_info.resolution == 0.0: return None, None
        grid_x = int((world_x - self.map_info.origin.position.x) / self.map_info.resolution)
        grid_y = int((world_y - self.map_info.origin.position.y) / self.map_info.resolution)
        return grid_x, grid_y

    def is_obstacle(self, grid_x, grid_y):
        if self.inflated_map_data is None: return True
        if 0 <= grid_x < self.map_info.width and 0 <= grid_y < self.map_info.height:
            return self.inflated_map_data[grid_y, grid_x] > self.obstacle_threshold
        return True

    def is_goal_accessible(self):
        goal_grid_x, goal_grid_y = self.world_to_grid(self.goal_x, self.goal_y)
        if goal_grid_x is None: return False
        return self.map_data[goal_grid_y, goal_grid_x] == 0

    def find_frontiers(self):
        if self.map_data is None: return []
        
        from scipy.ndimage import binary_dilation
        free_space = (self.map_data == 0)
        unknown_space = (self.map_data == -1)
        
        dilated_free = binary_dilation(free_space)
        edges = dilated_free & unknown_space
        
        frontier_coords = np.where(edges)
        return list(zip(frontier_coords[1], frontier_coords[0])) # Retorna como (x, y)

    def cluster_and_select_frontier(self, frontiers, robot_grid_pos):
        if not frontiers: return None

        # 1. Clustering de Fronteiras
        cluster_radius = 15 # Aumentado para agrupar portas
        clusters = []
        remaining_frontiers = set(frontiers)

        while remaining_frontiers:
            new_cluster = []
            queue = [remaining_frontiers.pop()]
            new_cluster.append(queue[0])
            
            head = 0
            while head < len(queue):
                current_point = queue[head]
                head += 1
                
                neighbors_to_add = {p for p in remaining_frontiers if math.hypot(p[0] - current_point[0], p[1] - current_point[1]) < cluster_radius}
                
                for neighbor in neighbors_to_add:
                    remaining_frontiers.remove(neighbor)
                    new_cluster.append(neighbor)
                    queue.append(neighbor)
            clusters.append(new_cluster)

        # 2. Avaliação e Seleção do Melhor Cluster
        best_cluster_centroid = None
        max_score = -1

        for cluster in clusters:
            if not cluster or len(cluster) < 5: continue # Ignora clusters muito pequenos
            
            centroid_x = sum([p[0] for p in cluster]) / len(cluster)
            centroid_y = sum([p[1] for p in cluster]) / len(cluster)
            centroid = (int(centroid_x), int(centroid_y))
            size = len(cluster)

            is_near_failed = False
            for failed_f in self.failed_frontiers:
                if math.hypot(centroid[0] - failed_f[0], centroid[1] - failed_f[1]) < 20: # Raio de exclusão
                    is_near_failed = True
                    break
            if is_near_failed: continue
            
            distance = math.hypot(centroid[0] - robot_grid_pos[0], centroid[1] - robot_grid_pos[1])
            if distance < 10: continue

            # Pontuação: prioriza clusters grandes e penaliza a distância
            score = size / (distance**0.5) 
            if score > max_score:
                max_score = score
                best_cluster_centroid = centroid
        
        return best_cluster_centroid

    def astar_search(self, start_grid, goal_grid):
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
            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if self.is_obstacle(neighbor[0], neighbor[1]): continue
                tentative_g_cost = g_cost[current] + math.hypot(dx, dy)
                if neighbor not in g_cost or tentative_g_cost < g_cost[neighbor]:
                    g_cost[neighbor] = tentative_g_cost
                    h_cost = math.hypot(neighbor[0] - goal_grid[0], neighbor[1] - goal_grid[1])
                    heapq.heappush(open_set, (tentative_g_cost + h_cost, neighbor))
                    came_from[neighbor] = current
        return None

    def publish_path_for_rviz(self, grid_path):
        if not grid_path: return
        path_msg = Path()
        path_msg.header.frame_id = self.map_frame
        path_msg.header.stamp = rospy.Time.now()
        for grid_pos in grid_path:
            world_x = (grid_pos[0] * self.map_info.resolution) + self.map_info.origin.position.x
            world_y = (grid_pos[1] * self.map_info.resolution) + self.map_info.origin.position.y
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = world_x
            pose.pose.position.y = world_y
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

    def follow_path(self, path):
        if not path or len(path) < 2: return False
        rate = rospy.Rate(10)
        for waypoint_grid in path[::5] + [path[-1]]:
            waypoint_world_x = (waypoint_grid[0] * self.map_info.resolution) + self.map_info.origin.position.x
            waypoint_world_y = (waypoint_grid[1] * self.map_info.resolution) + self.map_info.origin.position.y
            start_time = rospy.Time.now()
            timeout = rospy.Duration(20.0) 
            last_pose_time = rospy.Time.now()
            if not self.get_robot_pose(): return False
            last_position = self.robot_pose.pose.position
            while not rospy.is_shutdown():
                if rospy.Time.now() - start_time > timeout:
                    rospy.logwarn("Tempo limite atingido para o waypoint. Abortando caminho.")
                    self.vel_pub.publish(Twist())
                    return False
                if not self.get_robot_pose(): continue
                if rospy.Time.now() - last_pose_time > rospy.Duration(4.0):
                    current_position = self.robot_pose.pose.position
                    dist_moved = math.hypot(current_position.x - last_position.x, current_position.y - last_position.y)
                    if dist_moved < 0.1:
                        rospy.logwarn("Robô parece estar preso. Abortando este caminho.")
                        self.vel_pub.publish(Twist())
                        return False
                    last_pose_time = rospy.Time.now()
                    last_position = current_position
                dx = waypoint_world_x - self.robot_pose.pose.position.x
                dy = waypoint_world_y - self.robot_pose.pose.position.y
                if math.hypot(dx, dy) < 0.3: break 
                angle_to_goal = math.atan2(dy, dx)
                _, _, current_yaw = euler_from_quaternion([self.robot_pose.pose.orientation.x, self.robot_pose.pose.orientation.y, self.robot_pose.pose.orientation.z, self.robot_pose.pose.orientation.w])
                angle_diff = (angle_to_goal - current_yaw + math.pi) % (2 * math.pi) - math.pi
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
        rospy.logwarn("A iniciar comportamento de recuperação: a recuar e a rodar.")
        twist_msg = Twist()
        rate = rospy.Rate(10)
        move_back_time = rospy.Duration(1.5)
        start_time = rospy.Time.now()
        twist_msg.linear.x = -0.15
        while rospy.Time.now() - start_time < move_back_time and not rospy.is_shutdown():
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

    def run(self):
        while (self.map_data is None or not self.get_robot_pose()) and not rospy.is_shutdown():
            rospy.loginfo_throttle(2, "Aguardando mapa e localização inicial do robô...")
            rospy.sleep(1)
        
        while not self.is_goal_accessible() and not rospy.is_shutdown():
            rospy.loginfo_throttle(5, "FASE DE EXPLORAÇÃO: Destino inacessível.")
            if not self.get_robot_pose(): continue
            
            robot_grid_pos = self.world_to_grid(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y)
            frontiers = self.find_frontiers()
            
            if frontiers:
                target_frontier = self.cluster_and_select_frontier(frontiers, robot_grid_pos)
                if target_frontier:
                    path_to_frontier = self.astar_search(robot_grid_pos, target_frontier)
                    if path_to_frontier:
                        rospy.loginfo("A navegar para o centro do cluster de fronteira mais promissor: %s", str(target_frontier))
                        self.publish_path_for_rviz(path_to_frontier)
                        success = self.follow_path(path_to_frontier)
                        if not success:
                            rospy.logwarn("Falha ao navegar para a fronteira %s. Adicionando à lista de falhas.", str(target_frontier))
                            self.failed_frontiers.append(target_frontier)
                            self.recovery_behavior()
                else:
                    self.recovery_behavior()
            else:
                rospy.logwarn_throttle(5, "Nenhuma fronteira encontrada. A executar recuperação.")
                self.recovery_behavior()

        rospy.loginfo("FASE FINAL: Navegando para o destino.")
        if not self.get_robot_pose(): return
        start_grid = self.world_to_grid(self.robot_pose.pose.position.x, self.robot_pose.pose.position.y)
        goal_grid = self.world_to_grid(self.goal_x, self.goal_y)
        final_path = self.astar_search(start_grid, goal_grid)
        if final_path:
            self.publish_path_for_rviz(final_path)
            self.follow_path(final_path)
            rospy.loginfo(">>> Destino Final Alcançado! <<<")
        self.vel_pub.publish(Twist())

if __name__ == '__main__':
    try:
        navigator = ExplorationNavigator()
        navigator.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=1).publish(Twist())
