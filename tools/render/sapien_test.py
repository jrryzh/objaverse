"""Camera.

Concepts:
    - Create and mount cameras
    - Render RGB images, point clouds, segmentation masks
"""

"""

    Modified by Tianyu Wang for Rendering
    2023/10/18

"""

import sapien.core as sapien
import numpy as np
from PIL import Image, ImageColor
import open3d as o3d
import math
import os
import json
import pybullet as p
from sapien.utils.viewer import Viewer
from transforms3d.euler import mat2euler
import time


def uint(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return  v/ norm


# Semi-sphere Pose Sampling
def semi_sphere_generate_samples(samples=300, distance=5):
    RTs = [] # pose transform matrix
    # golden angle in radians
    phi = math.pi * (math.sqrt(5.) - 1.)  
    for i in range(samples): # num -> samples
        # y goes from 1 to -1 -> 0 to 1
        # y = 1 - (i / float(samples - 1)) * 2  
        # y goes from 0 to 1
        z = i / float(samples - 1) if i / float(samples - 1)<1 else 1-1e-10 
        radius = math.sqrt(1 - z * z)  # radius at y

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        y = math.sin(theta) * radius
        cam_pos = np.array([x, y, z]) * distance
        # cam_pos = np.array([-2, -2, -3])
        # print(cam_pos)

        axisX = -cam_pos.copy()
        axisZ = np.array([0,0,1])
        axisY = np.cross(axisZ, axisX)
        axisZ = np.cross(axisX, axisY)

        cam_mat = np.array([uint(axisX), uint(axisY), uint(axisZ)])

        obj_RT = np.eye(4,4)
        obj_RT[:3, :3] = cam_mat.T
        obj_RT[:3, 3] = cam_pos

        RTs.append(obj_RT)

    return np.stack(RTs)

# Sphere Pose Sampling
def sphere_generate_samples(samples=300, distance=5):
    RTs = [] # pose transform matrix
    # golden angle in radians
    phi = math.pi * (math.sqrt(5.) - 1.)  
    for i in range(samples): # num -> samples
        # y goes from 1 to -1 -> 0 to 1
        # y = 1 - (i / float(samples - 1)) * 2  
        # y goes from 0 to 1
        z = 1 - (i / float(samples - 1)) * 2  # z goes from 1 to -1
        
        if z == 1:
            z = 1 - 1e-10
        elif z == -1:
            z = -1 + 1e-10

        radius = math.sqrt(1 - z * z)  # radius at y

        theta = phi * i  # golden angle increment

        x = math.cos(theta) * radius
        y = math.sin(theta) * radius
        cam_pos = np.array([x, y, z]) * distance
        # cam_pos = np.array([-2, -2, -3])
        # print(cam_pos)

        axisX = -cam_pos.copy()
        axisZ = np.array([0,0,1])
        axisY = np.cross(axisZ, axisX)
        axisZ = np.cross(axisX, axisY)

        cam_mat = np.array([uint(axisX), uint(axisY), uint(axisZ)])

        obj_RT = np.eye(4,4)
        obj_RT[:3, :3] = cam_mat.T
        obj_RT[:3, 3] = cam_pos

        RTs.append(obj_RT)

    return np.stack(RTs)

def clear_scene(engine, scene):
    # Remove all actors
    for actor in scene.get_all_actors():
        scene.remove_actor(actor)

    # Remove all articulations
    for articulation in scene.get_all_articulations():
        scene.remove_articulation(articulation)

    # Clear all lights
    scene.clear_lights()

SAMPLE = "SEMI-SPHERE"
if SAMPLE == "SPHERE":
    generate_samples = sphere_generate_samples
else:
    generate_samples = semi_sphere_generate_samples

def normalize_model(mesh):
    # Get the bounding box of the mesh
    bounding_box = mesh.get_axis_aligned_bounding_box()
    min_bound = bounding_box.min_bound
    max_bound = bounding_box.max_bound
    
    # Calculate the scale factor to normalize the model size
    scale_factor = 1.0 / np.max(max_bound - min_bound)
    
    # Scale and translate the mesh to normalize its size and move it to the origin
    mesh.scale(scale_factor, center=bounding_box.get_center())
    mesh.translate(-mesh.get_center())
    
    return mesh

def load_and_normalize_urdf(urdf_path):
    # Load the mesh from the corresponding .obj file
    obj_path = urdf_path.replace(".urdf", ".obj")
    mesh = o3d.io.read_triangle_mesh(obj_path)
    
    # Normalize the mesh
    mesh = normalize_model(mesh)
    
    return mesh

def main():
    objaverse_dir = "/home/add_disk_e/objaverse_lvis_trimesh_objs_normalized/3k_10k/"
    objaverse_urdf_lst = [os.path.join(objaverse_dir, file) for file in os.listdir(objaverse_dir) if file.endswith(".urdf")]

    successful_urdf_lst = []
    failed_urdf_lst = []
    
    for urdf_path in objaverse_urdf_lst[:5]:
        start_time = time.time()
        try:
            objaverse_id = urdf_path.split("/")[-1].replace(".urdf", "")
            instance_path = os.path.join("/home/fudan248/zhangjinyu/tmp/", objaverse_id)
            if not os.path.exists(instance_path):
                os.makedirs(instance_path)
                print(f"Creating instance: {objaverse_id}")
            else:
                print(f"Instance path already exists: {objaverse_id}")
                continue

            # Load and normalize the mesh
            mesh = load_and_normalize_urdf(urdf_path)

            # Save the normalized mesh
            normalized_obj_path = os.path.join(instance_path, "normalized.obj")
            o3d.io.write_triangle_mesh(normalized_obj_path, mesh)

            # define engine
            engine = sapien.Engine()
            renderer = sapien.SapienRenderer(offscreen_only=True)
            engine.set_renderer(renderer)

            scene = engine.create_scene()
            scene.set_timestep(1 / 10.0)

            # add light
            scene.set_ambient_light([0.5, 0.5, 0.5])
            scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
            scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
            scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
            scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)

            # loader = scene.create_urdf_loader()
            # asset = loader.load_kinematic(urdf_path)
            # assert asset, 'URDF not loaded.'

            # Add the normalized mesh to the scene
            actor_builder = scene.create_actor_builder()
            actor_builder.add_visual_from_file(normalized_obj_path)
            actor_builder.build_kinematic()

            # Generate camera poses
            sample_function = semi_sphere_generate_samples if SAMPLE == "SEMI-SPHERE" else sphere_generate_samples
            camera_poses = sample_function()

            for idx, cam_pose in enumerate(camera_poses):
                # Set camera pose
                camera = scene.add_camera(name=f"camera_{idx}", width=640, height=480, fovx=1.0, fovy=1.0, near=0.1, far=100.0)
                camera.set_pose(cam_pose)

                # Render images
                scene.step()
                scene.update_render()
                rgba = camera.get_color_rgba()
                depth = camera.get_depth()

                # Save rendered images
                Image.fromarray((rgba * 255).astype(np.uint8)).save(os.path.join(instance_path, f"rgb_{idx}.png"))
                np.save(os.path.join(instance_path, f"depth_{idx}.npy"), depth)

            successful_urdf_lst.append(urdf_path)
        except Exception as e:
            print(f"Failed to process {urdf_path}: {e}")
            failed_urdf_lst.append(urdf_path)

    print("Successful URDFs:", successful_urdf_lst)
    print("Failed URDFs:", failed_urdf_lst)

if __name__ == '__main__':
    import time
    start = time.time()
    main()
    end = time.time()
    print(f"Time total: {end-start}s")
    

# rgb image
# depth image
# Pose
# Intrinsic matrix
# partial PointCloud
# scale