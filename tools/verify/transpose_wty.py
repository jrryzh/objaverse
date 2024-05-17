import numpy as np
import open3d as o3d

# 读取 OBJ 文件
obj_file_path = "/home/fudan248/zhangjinyu/tmp/water_bottle/affd4ebf5fd143d98c1b7c76eedafff3/0000_pcd.obj"
point_cloud = o3d.io.read_triangle_mesh(obj_file_path)

# 确保顶点法线和顶点颜色被正确处理
if not point_cloud.has_vertex_normals():
    point_cloud.compute_vertex_normals()

# 加载变换矩阵
transform_matrix_path = "/home/fudan248/zhangjinyu/tmp/water_bottle/affd4ebf5fd143d98c1b7c76eedafff3/0000_pose.txt"
transform_matrix = np.loadtxt(transform_matrix_path)

# 应用变换矩阵
point_cloud.transform(transform_matrix)

# 保存变换后的 OBJ 文件
output_file_path = "/home/fudan248/zhangjinyu/tmp/water_bottle/transformed_pcd_v1.obj"
o3d.io.write_triangle_mesh(output_file_path, point_cloud)

print(f"Transformed OBJ file saved to {output_file_path}")
