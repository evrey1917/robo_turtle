import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import numpy as np
import open3d as o3d


class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/depth/points',
            self.listener_callback,
            10)
        
        # Initialize Open3D Visualizer
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window(window_name='Open3D Point Cloud', width=800, height=600)
        self.pcd = o3d.geometry.PointCloud()
        self.is_geometry_added = False

    def listener_callback(self, msg):
        # Process the point cloud data here
        points_struct = point_cloud2.read_points_numpy(
            msg,
            field_names=("x", "y", "z"),
            skip_nans=True)
        
        if points_struct.size == 0:
            self.get_logger().info('No points received')
            return
        print(points_struct.shape)

        # Remove points with NaN or infinite coordinates
        valid_indices = np.isfinite(points_struct).all(axis=1)
        points_struct = points_struct[valid_indices]

        # points is now an Nx3 array of XYZ points
        self.get_logger().info(f'Received point cloud with {points_struct.shape} points.')

        # Compute the centroid
        centroid = np.mean(points_struct, axis=0)
        self.get_logger().info(f'Centroid: {centroid}')

        # Update Open3D point cloud
        self.pcd.points = o3d.utility.Vector3dVector(points_struct)

        if not self.is_geometry_added:
            self.vis.add_geometry(self.pcd)
            self.is_geometry_added = True

        self.vis.update_geometry(self.pcd)
        self.vis.poll_events()
        self.vis.update_renderer()


def main(args=None):
    rclpy.init(args=args)
    pointcloud_subscriber = PointCloudSubscriber()
    try:
        rclpy.spin(pointcloud_subscriber)
    except KeyboardInterrupt:
        pass
    pointcloud_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
