#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import message_filters
import numpy as np
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Header

# 全局的 Type 映射表，用于底层极致解包和封包
_DATATYPES = {}
_DATATYPES[PointField.INT8]    = np.dtype(np.int8)
_DATATYPES[PointField.UINT8]   = np.dtype(np.uint8)
_DATATYPES[PointField.INT16]   = np.dtype(np.int16)
_DATATYPES[PointField.UINT16]  = np.dtype(np.uint16)
_DATATYPES[PointField.INT32]   = np.dtype(np.int32)
_DATATYPES[PointField.UINT32]  = np.dtype(np.uint32)
_DATATYPES[PointField.FLOAT32] = np.dtype(np.float32)
_DATATYPES[PointField.FLOAT64] = np.dtype(np.float64)

class RealTimeLidarFusionNode(Node):
    def __init__(self):
        super().__init__('realtime_lidar_fusion_node')
        
        self.declare_parameter('base_frame', 'hesai_lidar') 
        self.declare_parameter('output_topic', '/lidar_merged/points')
        
        self.base_frame = self.get_parameter('base_frame').value
        self.output_topic = self.get_parameter('output_topic').value

        self.topic_1 = '/lidar_points'
        self.topic_2 = '/rslidar_1/rslidar_points'
        self.topic_3 = '/rslidar_2/rslidar_points'
        
        # --- 优化点 1: TF外参缓存机制 ---
        # 避免在回调循环中每帧高频索要TF树引起的大量CPU开销
        self.cached_transforms = {}

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 略微增大队列大小增强抗抖动性能
        self.sub1 = message_filters.Subscriber(self, PointCloud2, self.topic_1)
        self.sub2 = message_filters.Subscriber(self, PointCloud2, self.topic_2)
        self.sub3 = message_filters.Subscriber(self, PointCloud2, self.topic_3)
        
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.sub1, self.sub2, self.sub3], queue_size=20, slop=0.1)
        self.ts.registerCallback(self.sync_callback)
        
        self.publisher_ = self.create_publisher(PointCloud2, self.output_topic, 10)
        
        self.get_logger().info(f"雷达实时融合节点已启动！(经极速缓存和内存级结构优化版)")
        self.get_logger().info(f"融合基准统一投影至: {self.base_frame}")

    def get_transform(self, target_frame, source_frame):
        """核心函数：获取外参进行坐标系之间的仿射矩阵转移，添加了缓存以彻底消除查询惩罚"""
        cache_key = (target_frame, source_frame)
        if cache_key in self.cached_transforms:
            return self.cached_transforms[cache_key]
            
        try:
            trans = self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time())
            
            # --- 优化点 2: 指定 float32 ---
            # Numpy数组强制设定 dtype=float32 以避免点云变换中庞大的 float64 的升位强制运算，极大地降低算力消耗
            translation = np.array([
                trans.transform.translation.x,
                trans.transform.translation.y,
                trans.transform.translation.z
            ], dtype=np.float32)
            
            quat = [
                trans.transform.rotation.x,
                trans.transform.rotation.y,
                trans.transform.rotation.z,
                trans.transform.rotation.w
            ]
            # 计算旋转矩阵并转置，供后续使用点乘快速应用转换
            rotation_matrix_T = R.from_quat(quat).as_matrix().astype(np.float32).T
            
            self.cached_transforms[cache_key] = (translation, rotation_matrix_T)
            self.get_logger().info(f"成功获取并缓存外参: 从 {source_frame} 到 {target_frame}")
            return translation, rotation_matrix_T
            
        except (LookupException, ConnectivityException, ExtrapolationException) as ex:
            self.get_logger().warn(f"无法获取TF变换: 从 {source_frame} 到 {target_frame}: {ex}", throttle_duration_sec=3.0)
            return None, None

    def fast_read_cloud(self, msg):
        """--- 优化点 3: 内存直读解码点云 ---
        抛弃缓慢的原生 python 遍历装解器 (pc2_py.read_points)
        通过 numpy.frombuffer 以 C 速度将整片点云二进制流直接投射为数组，处理几十万个点仅需零点几毫秒"""
        dt_dict = {'names': [], 'formats': [], 'offsets': []}
        for field in msg.fields:
            if field.name in ('x', 'y', 'z', 'intensity'):
                dt_dict['names'].append(field.name)
                dt_dict['formats'].append(_DATATYPES[field.datatype])
                dt_dict['offsets'].append(field.offset)
        
        dt_dict['itemsize'] = msg.point_step
        np_dtype = np.dtype(dt_dict)
        
        # 将二进制块直接套用到结构体中
        cloud_data = np.frombuffer(msg.data, dtype=np_dtype)
        
        xyz = np.empty((len(cloud_data), 3), dtype=np.float32)
        xyz[:, 0] = cloud_data['x']
        xyz[:, 1] = cloud_data['y']
        xyz[:, 2] = cloud_data['z']
        
        # 处理雷达中的空点 (NaNs跳开检测)
        valid_mask = ~np.isnan(xyz[:, 0])
        xyz = xyz[valid_mask]
        
        if 'intensity' in dt_dict['names']:
            intensity = cloud_data['intensity'][valid_mask].astype(np.float32).reshape(-1, 1)
        else:
            intensity = np.zeros((len(xyz), 1), dtype=np.float32)
            
        return xyz, intensity

    def sync_callback(self, msg1, msg2, msg3):
        merged_pts = []
        
        for msg in [msg1, msg2, msg3]:
            # [步骤 1]: 极速解析
            xyz, intensity = self.fast_read_cloud(msg)
            
            if xyz.shape[0] == 0:
                continue
            
            source_frame = msg.header.frame_id

            # --- 优化点 6: 前置空间裁剪 (ROI Filter) ---
            # 提前剔除主进程不关心的点，避免无谓的坐标转换，直接砍掉海量数据
            # 性能技巧：计算距离对比时，直接对比距离的平方(x^2 + y^2)，避免使用极度消耗CPU的 np.sqrt()
            r2 = xyz[:, 0]**2 + xyz[:, 1]**2
            if source_frame == self.base_frame:  # hesai_lidar: 半径0.5~16m, z < 0.5m
                mask = (r2 > 0.25) & (r2 < 256.0) & (xyz[:, 2] < 0.5)
            else:  # rslidar: 半径0.3~5m, z > -0.5m
                mask = (r2 > 0.09) & (r2 < 25.0) & (xyz[:, 2] > -0.5)
                
            xyz = xyz[mask]
            intensity = intensity[mask]
            
            if xyz.shape[0] == 0:
                continue
            
            # [步骤 2]: 乘外参变换 (只对 xyz 变换，强度不变)
            if source_frame != self.base_frame:
                translation, rotation_T = self.get_transform(self.base_frame, source_frame)
                if translation is not None and rotation_T is not None:
                    # --- 优化点 4: 高效运算 ---
                    # 直接通过 numpy 内置矩阵点积进行运算 (避免了大量的矩阵转置开销)
                    xyz = xyz @ rotation_T + translation
            
            # [步骤 3]: 将转换后的 xyz 和强度块水平对齐
            pts_transformed = np.hstack((xyz, intensity))
            merged_pts.append(pts_transformed)
            
        if not merged_pts:
            return
        
        # [步骤 4]: 将三个雷达拼接成一组数据
        all_merged_pts = np.vstack(merged_pts)
        
        # --- 在此处增加旋转逻辑 (绕Z轴顺时针90度) ---
        # 旋转关系：x_new = y_old, y_new = -x_old
        # temp_x = all_merged_pts[:, 0].copy()
        # all_merged_pts[:, 0] = all_merged_pts[:, 1]
        # all_merged_pts[:, 1] = -temp_x
        
        # [步骤 5]: 打包发为 ROS2 消息
        self.publish_merged_cloud_fast(all_merged_pts, msg1.header.stamp)

    def publish_merged_cloud_fast(self, points_array, stamp):
        """--- 优化点 5: 内存直装发布点云 ---
        抛开原装缓慢且吃内存的 list 拆解和点位打包 (.tolist()), 
        转而分配连续内存结构进行构造, 避免Python的 for 循环。打包发布速度翻百倍"""
        msg = PointCloud2()
        msg.header.stamp = stamp
        msg.header.frame_id = self.base_frame
        msg.height = 1
        msg.width = points_array.shape[0]
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # --- 终极优化点: 点云矩阵和结构体步长完全对齐机制 ---
        # 此时 points_array 是 (N, 4) 的 float32 二维数组，物理内存表现为 x0, y0, z0, i0, x1, y1...
        # 完美对应了 msg.fields 要求！因此无需再重新分配构造 np.empty 结构体，节省极其庞大的重构开销和 4 次循环拷贝
        msg.data = np.ascontiguousarray(points_array, dtype=np.float32).tobytes()
        
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RealTimeLidarFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
