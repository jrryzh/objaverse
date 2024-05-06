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
import os

# os.environ["CUDA_VISIBLE_DEVICES"] = "5,6,7" 

def uint(v):
    norm = np.linalg.norm(v)
    if norm == 0:
        return v
    return  v/ norm

def normalize_mesh(mesh):
    # 计算边界框中心
    bbox_min, bbox_max = mesh.bbox
    center = (bbox_min + bbox_max) / 2
    
    # 移动模型中心到原点
    mesh.vertices -= center
    
    # 缩放模型大小
    max_length = np.max(bbox_max - bbox_min)
    mesh.vertices /= max_length

    return mesh

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

SAMPLE = "SEMI-SPHERE"
if SAMPLE == "SPHERE":
    generate_samples = sphere_generate_samples
else:
    generate_samples = semi_sphere_generate_samples

def main():
    # dir path
    # root_path = "./render/Dispenser"
    # item_id = 1
    # category = "test"
    # root_path = os.path.join("./render_output", category)
    # instance_path = os.path.join(root_path, str(item_id))
    # if not os.path.exists(instance_path):
    #     os.makedirs(instance_path)

    # textured_objs_path = './dataset/101490/textured_objs'
    # textured_objs_path = os.path.join("./dataset", str(item_id), "textured_objs")
    # textured_objs_path = "/home/fudan248/zhangjinyu/code_repo/objaverse/sample_objs/bucket/"
    # all .obj
    normlized_objs_path = "/home/add_disk_e/objaverse_lvis_objs_normalized/"# "/home/add_disk_e/objaverse_lvis_objs_normalized/"
    root_path = "/home/fudan248/zhangjinyu/code_repo/objaverse/render_output/"
    all_files = os.listdir(normlized_objs_path)
    all_files = [f for f in all_files if f.endswith('.urdf')]
    sampled_files = all_files[:3] # sample 50 files for each instance
    
    # create engine and renderer
    engine = sapien.Engine()
    # renderer = sapien.SapienRenderer()
    renderer = sapien.SapienRenderer(offscreen_only=True)
    engine.set_renderer(renderer)

    for filename in sampled_files:
        try:
            if filename.endswith('.obj'):
                scene = engine.create_scene()
                scene.set_timestep(1 / 100.0) # 10ms            
                
                # loader = scene.create_urdf_loader()
                # loader.fix_root_link = True
                # # urdf_path = '../assets/179/mobility.urdf'
                # # urdf_path = './dataset/101490/mobility.urdf'
                # # urdf_path = os.path.join("./dataset", str(item_id), "mobility.urdf")
                # urdf_path = "/home/fudan248/zhangjinyu/code_repo/objaverse/sample_objs/bucket/bucket.urdf"
                # # load as a kinematic articulation
                # asset = loader.load_kinematic(urdf_path)
                # assert asset, 'URDF not loaded.'

                # add ground
                scene.add_ground(altitude=0)

                # add light
                scene.set_ambient_light([0.5, 0.5, 0.5]) # gray env light
                scene.add_directional_light([0, 1, -1], [0.5, 0.5, 0.5], shadow=True)
                scene.add_point_light([1, 2, 2], [1, 1, 1], shadow=True)
                scene.add_point_light([1, -2, 2], [1, 1, 1], shadow=True)
                scene.add_point_light([-1, 0, 1], [1, 1, 1], shadow=True)
                
                file_hash = filename.split('.')[0]
                file_path = os.path.join(normlized_objs_path, filename)
                instance_path = os.path.join(root_path, filename.split('.')[0])
                
                # load as a kinematic articulation
                loader = scene.create_urdf_loader()
                loader.fix_root_link = True
                urdf_path = os.path.join(normlized_objs_path, file_hash+ ".urdf")
                asset = loader.load_kinematic(urdf_path)
                assert asset, 'URDF not loaded.'
                
                # instance path
                if not os.path.exists(instance_path):
                    os.makedirs(instance_path)    
                
                # load .obj
                mesh = o3d.io.read_triangle_mesh(file_path)

                # get point cloud data
                points = np.asarray(mesh.vertices)
                
                # init min & max
                X_min, Y_min, Z_min = float('inf'), float('inf'), float('inf')
                X_max, Y_max, Z_max = -float('inf'), -float('inf'), -float('inf')

                # compute extreme point for each pc
                x_min, y_min, z_min = points.min(axis=0)
                x_max, y_max, z_max = points.max(axis=0)

                # update global extreme point
                X_min = min(X_min, x_min)
                Y_min = min(Y_min, y_min)
                Z_min = min(Z_min, z_min)
                X_max = max(X_max, x_max)
                Y_max = max(Y_max, y_max)
                Z_max = max(Z_max, z_max)

                # compute X_scale, Y_scale, Z_scale
                X_scale = X_max - X_min
                Y_scale = Y_max - Y_min
                Z_scale = Z_max - Z_min

                # save scale
                scale_path = os.path.join(instance_path, "scale.txt")
                with open(scale_path, 'w') as file:
                    file.write(f'{X_scale} {Y_scale} {Z_scale}\n')

                # scale = pcd.get_scale()
                # scale_file = os.path.join(pose_path, "scale.txt")
                # with open(scale_file, "w") as file:
                #     file.write(f"Scale X: {scale[0]}\n")
                #     file.write(f"Scale Y: {scale[1]}\n")
                #     file.write(f"Scale Z: {scale[2]}\n")

                # ---------------------------------------------------------------------------- #
                # Camera
                # ---------------------------------------------------------------------------- #
                near, far = 0.1, 100 # near plane to far plane -> depth range
                width, height = 640, 480

                # generate a set of cameras (position & pose)
                camera_poses = generate_samples(samples=10, distance=5) # 300, 5

                # one camera for each pose each render (enumerate pose)
                camera = scene.add_camera(
                    name="camera",
                    width=width,
                    height=height,
                    fovy=np.deg2rad(35),
                    near=near,
                    far=far,
                )
                camera.set_pose(sapien.Pose(p=[0, 0, 0]))
                # mount
                camera_mount_actor = scene.create_actor_builder().build_kinematic()
                camera.set_parent(parent=camera_mount_actor, keep_pose=False)

                # set intrinsic matrix
                print('Intrinsic matrix\n', camera.get_intrinsic_matrix())

                # save intrinsic matrix
                intrinsic_path = os.path.join(root_path, "intrinsic.txt")
                if not os.path.exists(intrinsic_path):
                    np.savetxt(intrinsic_path, camera.get_intrinsic_matrix())

                # print(camera_poses[-1])

                # define pose
                for i, cam_pos in enumerate(camera_poses):

                    # create pose path
                    # pose_path = os.path.join(instance_path, i)
                    # if not os.path.exists(pose_path):
                    #     os.makedirs(pose_path)

                    # set pose
                    camera_mount_actor.set_pose(sapien.Pose.from_transformation_matrix(cam_pos))

                    # pose
                    new_cam_pos = cam_pos
                    new_cam_pos[:3, 1:3] *=-1
                    # change
                    new_cam_pos[:3, 0], new_cam_pos[:3, 1] = new_cam_pos[:3, 1], new_cam_pos[:3, 0].copy()
                    new_cam_pos[:3, 1], new_cam_pos[:3, 2] = new_cam_pos[:3, 2], new_cam_pos[:3, 1].copy()

                    instance_pose = np.linalg.inv(new_cam_pos)
                    pose_path = os.path.join(instance_path, f"{str(i).zfill(4)}_pose.txt")
                    np.savetxt(pose_path, instance_pose)

                    # mount
                    # camera_mount_actor = scene.create_actor_builder().build_kinematic()
                    # camera.set_parent(parent=camera_mount_actor, keep_pose=False)

                    # forward = -cam_pos / np.linalg.norm(cam_pos)
                    # left = np.cross([0, 0, 1], forward)
                    # left = left / np.linalg.norm(left)
                    # up = np.cross(forward, left)
                    # mat44 = np.eye(4)
                    # mat44[:3, :3] = np.stack([forward, left, up], axis=1)
                    # mat44[:3, 3] = cam_pos
                    # camera_mount_actor.set_pose(sapien.Pose.from_transformation_matrix(mat44))

                    scene.step()  # make everything set
                    scene.update_render()
                    camera.take_picture()

                    rgba = camera.get_float_texture('Color')  # [H, W, 4]
                    # An alias is also provided
                    # rgba = camera.get_color_rgba()  # [H, W, 4]
                    rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
                    rgba_pil = Image.fromarray(rgba_img)
                    rgb_path = os.path.join(instance_path, f"{str(i).zfill(4)}_rgb.png")
                    rgba_pil.save(rgb_path)

                    # ---------------------------------------------------------------------------- #
                    # XYZ position in the camera space
                    # ---------------------------------------------------------------------------- #
                    # Each pixel is (x, y, z, render_depth) in camera space (OpenGL/Blender)
                    position = camera.get_float_texture('Position')  # [H, W, 4]
                    # 从位置信息中提取前3列，并且取出第4列小于1的行
                    points = position[..., :3][position[..., 3] < 1]
                    # 将提取的点的第2列和第3列取反
                    points[:, 1] = - points[:, 1]
                    points[:, 2] = - points[:, 2]
                    
                    # OpenGL/Blender: y up and -z forward
                    # points_opengl = position[..., :3][position[..., 3] < 1]
                    # points_color = rgba[position[..., 3] < 1][..., :3]
                    # # Model matrix is the transformation from OpenGL camera space to SAPIEN world space
                    # # camera.get_model_matrix() must be called after scene.update_render()!
                    # model_matrix = camera.get_model_matrix()
                    # points_world = points_opengl @ model_matrix[:3, :3].T + model_matrix[:3, 3]

                    # SAPIEN CAMERA: z up and x forward
                    # points_camera = points_opengl[..., [2, 0, 1]] * [-1, -1, 1]

                    pcd = o3d.geometry.PointCloud()
                    pcd.points = o3d.utility.Vector3dVector(points)
                    # pcd.colors = o3d.utility.Vector3dVector(points_color)

                    # No display
                    # coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()
                    # o3d.visualization.draw_geometries([pcd, coord_frame])

                    # save pc
                    # "{}_pcd.obj".format(str(id).zifill(4))
                    obj_path = os.path.join(instance_path, f"{str(i).zfill(4)}_pcd.obj")
                    # o3d.io.write_triangle_mesh(obj_path, pcd)
                    with open(obj_path, 'w') as f:
                        for point in pcd.points:
                            f.write(f"v {point[0]} {point[1]} {point[2]}\n")

                    # Depth
                    depth = -position[..., 2]
                    depth_image = (depth * 1000.0).astype(np.uint16)
                    depth_pil = Image.fromarray(depth_image)
                    depth_path = os.path.join(instance_path, f"{str(i).zfill(4)}_depth.png")
                    depth_pil.save(depth_path)

                    # ---------------------------------------------------------------------------- #
                    # Segmentation labels
                    # ---------------------------------------------------------------------------- #
                    # Each pixel is (visual_id, actor_id/link_id, 0, 0)
                    # visual_id is the unique id of each visual shape
                    seg_labels = camera.get_uint32_texture('Segmentation')  # [H, W, 4]
                    # colormap = sorted(set(ImageColor.colormap.values()))
                    # color_palette = np.array([ImageColor.getrgb(color) for color in colormap],
                    #                         dtype=np.uint8)
                    label0_image = seg_labels[..., 0].astype(np.uint8)  # mesh-level
                    label1_image = seg_labels[..., 1].astype(np.uint8)  # actor-level
                    # # Or you can use aliases below
                    # # label0_image = camera.get_visual_segmentation()
                    # # label1_image = camera.get_actor_segmentation()
                    # label0_pil = Image.fromarray(color_palette[label0_image])
                    # label0_path = os.path.join(instance_path, f"{str(i).zfill(4)}_label0.png")
                    # label0_pil.save(label0_path)

                    # label1_pil = Image.fromarray(color_palette[label1_image])
                    # label1_path = os.path.join(instance_path, f"{str(i).zfill(4)}_label1.png")
                    # label1_pil.save(label1_path)
                    # label1_pil = Image.fromarray(color_palette[label1_image])
                    # label1_path = os.path.join(instance_path, f"{str(i).zfill(4)}_label1.png")
                    # label1_pil.save(label1_path)
                    mask_image = np.where(label0_image == label0_image.max(), 0, 1).astype(np.uint8)
                    mask_image = Image.fromarray(mask_image)
                    mask_image_path = os.path.join(instance_path, f"{str(i).zfill(4)}_mask.png")
                    mask_image.save(mask_image_path)

                    # ---------------------------------------------------------------------------- #
                    # Take picture from the viewer
                    # ---------------------------------------------------------------------------- #
                    # viewer = Viewer(renderer)
                    # viewer.set_scene(scene)
                    # # We show how to set the viewer according to the pose of a camera
                    # # opengl camera -> sapien world
                    # model_matrix = camera.get_model_matrix()
                    # # sapien camera -> sapien world
                    # # You can also infer it from the camera pose
                    # model_matrix = model_matrix[:, [2, 0, 1, 3]] * np.array([-1, -1, 1, 1])
                    # # The rotation of the viewer camera is represented as [roll(x), pitch(-y), yaw(-z)]
                    # rpy = mat2euler(model_matrix[:3, :3]) * np.array([1, -1, -1])
                    # viewer.set_camera_xyz(*model_matrix[0:3, 3])
                    # viewer.set_camera_rpy(*rpy)
                    # viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
                    # while not viewer.closed:
                    #     if viewer.window.key_down('p'):  # Press 'p' to take the screenshot
                    #         rgba = viewer.window.get_float_texture('Color')
                    #         rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
                    #         rgba_pil = Image.fromarray(rgba_img)
                    #         rgba_pil.save('screenshot.png')
                    #     scene.step()
                    #     scene.update_render()
                    #     viewer.render()
        except Exception as e:
            print(f"Error: {e}")
# ======================== line ======================== #
# >>>>>>>>>>>>>>>>>>>>>>>>      <<<<<<<<<<<<<<<<<<<<<<<< #
# ======================== line ======================== #

    # define cameras
    # for i, cam_pose in enumerate(camera_poses):
    #     camera = scene.add_camera(
    #         name=f"camera_{i}",
    #         width=width,
    #         height=height,
    #         fovy=np.deg2rad(35),
    #         near=near,
    #         far=far,
    #     )

    #     # pose
    #     camera.set_pose(cam_pose)

    #     # mount

    # camera.set_pose(sapien.Pose(p=[1, 0, 0]))

    # print('Intrinsic matrix\n', camera.get_intrinsic_matrix())

    # camera_mount_actor = scene.create_actor_builder().build_kinematic()
    # camera.set_parent(parent=camera_mount_actor, keep_pose=False)

    # # Compute the camera pose by specifying forward(x), left(y) and up(z)
    # cam_pos = np.array([-2, -2, 3])
    # forward = -cam_pos / np.linalg.norm(cam_pos)
    # left = np.cross([0, 0, 1], forward)
    # left = left / np.linalg.norm(left)
    # up = np.cross(forward, left)
    # mat44 = np.eye(4)
    # mat44[:3, :3] = np.stack([forward, left, up], axis=1)
    # mat44[:3, 3] = cam_pos
    # camera_mount_actor.set_pose(sapien.Pose.from_transformation_matrix(mat44))

    # scene.step()  # make everything set
    # scene.update_render()
    # camera.take_picture()

    # ---------------------------------------------------------------------------- #
    # RGBA
    # ---------------------------------------------------------------------------- #
    # rgba = camera.get_float_texture('Color')  # [H, W, 4]
    # # An alias is also provided
    # # rgba = camera.get_color_rgba()  # [H, W, 4]
    # rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
    # rgba_pil = Image.fromarray(rgba_img)
    # rgba_pil.save('color.png')

    # # ---------------------------------------------------------------------------- #
    # # XYZ position in the camera space
    # # ---------------------------------------------------------------------------- #
    # # Each pixel is (x, y, z, render_depth) in camera space (OpenGL/Blender)
    # position = camera.get_float_texture('Position')  # [H, W, 4]

    # # OpenGL/Blender: y up and -z forward
    # points_opengl = position[..., :3][position[..., 3] < 1]
    # points_color = rgba[position[..., 3] < 1][..., :3]
    # # Model matrix is the transformation from OpenGL camera space to SAPIEN world space
    # # camera.get_model_matrix() must be called after scene.update_render()!
    # model_matrix = camera.get_model_matrix()
    # points_world = points_opengl @ model_matrix[:3, :3].T + model_matrix[:3, 3]

    # # SAPIEN CAMERA: z up and x forward
    # # points_camera = points_opengl[..., [2, 0, 1]] * [-1, -1, 1]

    # pcd = o3d.geometry.PointCloud()
    # pcd.points = o3d.utility.Vector3dVector(points_world)
    # pcd.colors = o3d.utility.Vector3dVector(points_color)
    # coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame()

    # # No display
    # # o3d.visualization.draw_geometries([pcd, coord_frame])

    # # save pc
    # o3d.io.write_point_cloud("partial_pc.obj", pcd)

    # # Depth
    # depth = -position[..., 2]
    # depth_image = (depth * 1000.0).astype(np.uint16)
    # depth_pil = Image.fromarray(depth_image)
    # depth_pil.save('depth.png')

    # # o3d bounding box - scale

    # # ---------------------------------------------------------------------------- #
    # # Segmentation labels
    # # ---------------------------------------------------------------------------- #
    # # Each pixel is (visual_id, actor_id/link_id, 0, 0)
    # # visual_id is the unique id of each visual shape
    # seg_labels = camera.get_uint32_texture('Segmentation')  # [H, W, 4]
    # colormap = sorted(set(ImageColor.colormap.values()))
    # color_palette = np.array([ImageColor.getrgb(color) for color in colormap],
    #                          dtype=np.uint8)
    # label0_image = seg_labels[..., 0].astype(np.uint8)  # mesh-level
    # label1_image = seg_labels[..., 1].astype(np.uint8)  # actor-level
    # # Or you can use aliases below
    # # label0_image = camera.get_visual_segmentation()
    # # label1_image = camera.get_actor_segmentation()
    # label0_pil = Image.fromarray(color_palette[label0_image])
    # label0_pil.save('label0.png')
    # label1_pil = Image.fromarray(color_palette[label1_image])
    # label1_pil.save('label1.png')

    # # ---------------------------------------------------------------------------- #
    # # Take picture from the viewer
    # # ---------------------------------------------------------------------------- #
    # viewer = Viewer(renderer)
    # viewer.set_scene(scene)
    # # We show how to set the viewer according to the pose of a camera
    # # opengl camera -> sapien world
    # model_matrix = camera.get_model_matrix()
    # # sapien camera -> sapien world
    # # You can also infer it from the camera pose
    # model_matrix = model_matrix[:, [2, 0, 1, 3]] * np.array([-1, -1, 1, 1])
    # # The rotation of the viewer camera is represented as [roll(x), pitch(-y), yaw(-z)]
    # rpy = mat2euler(model_matrix[:3, :3]) * np.array([1, -1, -1])
    # viewer.set_camera_xyz(*model_matrix[0:3, 3])
    # viewer.set_camera_rpy(*rpy)
    # viewer.window.set_camera_parameters(near=0.05, far=100, fovy=1)
    # while not viewer.closed:
    #     if viewer.window.key_down('p'):  # Press 'p' to take the screenshot
    #         rgba = viewer.window.get_float_texture('Color')
    #         rgba_img = (rgba * 255).clip(0, 255).astype("uint8")
    #         rgba_pil = Image.fromarray(rgba_img)
    #         rgba_pil.save('screenshot.png')
    #     scene.step()
    #     scene.update_render()
    #     viewer.render()


if __name__ == '__main__':
    import time
    start = time.time()
    main()
    end = time.time()
    print(f"Time total: {end-start}s")