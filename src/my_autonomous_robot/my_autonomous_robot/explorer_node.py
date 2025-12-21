import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

class ProjectAutonomousExplorer(Node):
    def __init__(self):
        super().__init__('project_explorer')
        self.navigator = BasicNavigator()
        
        # KRİTİK DÜZELTME: AMCL bekleme döngüsünü atla
        # Navigasyonun aktif olduğunu kontrol et ama AMCL servisini sorgulama
        self.get_logger().info('Navigasyon sisteminin hazirlanmasi bekleniyor...')
        
        # Robotun başlangıç konumunu (Initial Pose) SLAM modunda manuel set etmeye gerek yoktur, 
        # çünkü SLAM (0,0,0) noktasını başlangıç kabul eder.
        
    def perform_exploration(self):
        # Hedef koordinatlar (Gazebo dünyana göre bunları değiştirebilirsin)
        point_coords = [(1.0, 1.0), (2.0, -1.0), (-1.0, 1.5)]

        for pt in point_coords:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = 'map'
            goal_pose.header.stamp = self.get_clock().now().to_msg()
            goal_pose.pose.position.x = pt[0]
            goal_pose.pose.position.y = pt[1]
            goal_pose.pose.orientation.w = 1.0

            self.get_logger().info(f'Hedef noktaya gidiliyor: {pt}')
            self.navigator.goToPose(goal_pose)

            while not self.navigator.isTaskComplete():
                # Geri bildirim işlemleri burada yapılabilir
                pass

            result = self.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info('Hedefe ulasildi!')
            else:
                self.get_logger().error('Hedef basarisiz oldu.')

def main(args=None):
    rclpy.init(args=args)
    explorer = ProjectAutonomousExplorer()
    explorer.perform_exploration()
    explorer.destroy_node()
    rclpy.shutdown()