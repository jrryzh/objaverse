import numpy as np

def load_obj(file_path):
    vertices = []
    with open(file_path, 'r') as file:
        for line in file:
            if line.startswith('v '):
                parts = line.strip().split()
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
    return np.array(vertices)

def save_obj(vertices, file_path):
    with open(file_path, 'w') as file:
        for vertex in vertices:
            file.write(f"v {vertex[0]} {vertex[1]} {vertex[2]}\n")

def load_pose(file_path):
    with open(file_path, 'r') as file:
        matrix = []
        for line in file:
            matrix.append([float(x) for x in line.strip().split()])
    return np.array(matrix)

def apply_transformation(vertices, transformation_matrix):
    num_vertices = vertices.shape[0]
    homogenous_vertices = np.hstack([vertices, np.ones((num_vertices, 1))])
    transformed_vertices = homogenous_vertices.dot(transformation_matrix.T)
    return transformed_vertices[:, :3]

# Load obj file
vertices = load_obj('/home/fudan248/zhangjinyu/tmp/water_bottle/affd4ebf5fd143d98c1b7c76eedafff3/0000_pcd.obj')

# Load pose file
transformation_matrix = load_pose('/home/fudan248/zhangjinyu/tmp/water_bottle/affd4ebf5fd143d98c1b7c76eedafff3/0000_pose.txt')

# Apply transformation
transformed_vertices = apply_transformation(vertices, transformation_matrix)

# Save new obj file
save_obj(transformed_vertices, '/home/fudan248/zhangjinyu/tmp/water_bottle/transformed_pcd_v0.obj')
