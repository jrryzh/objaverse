import bpy
import os
import random
from mathutils import Vector

def clear_scene():
    # 删除所有物体
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_all(action='SELECT')
    bpy.ops.object.delete()

def normalize_object(obj):
    # 计算边界框大小和中心
    min_corner = Vector([min([v.co[i] for v in obj.data.vertices]) for i in range(3)])
    max_corner = Vector([max([v.co[i] for v in obj.data.vertices]) for i in range(3)])
    center = (max_corner + min_corner) / 2
    scale = max((max_corner - min_corner).to_tuple())

    # 移动中心到原点
    for vertex in obj.data.vertices:
        vertex.co -= center
    
    # 缩放到单位大小
    for vertex in obj.data.vertices:
        vertex.co /= scale

def process_directory(input_folder, output_folder, num_samples):
    # 获取文件夹中的所有文件名
    all_files = os.listdir(input_folder)

    # 检查文件数量是否足够进行采样
    if len(all_files) >= num_samples:
        # 随机选择文件
        sampled_files = random.sample(all_files, num_samples)
        print(sampled_files)
    else:
        print(f"文件夹中的文件数量少于所需的采样数量 {num_samples}")

    for filename in sampled_files:
        if filename.lower().endswith('.obj'):
            clear_scene()
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, filename)
            # 导入OBJ文件
            bpy.ops.import_scene.obj(filepath=input_path)
            obj = bpy.context.selected_objects[0]
            # 规范化对象
            normalize_object(obj)
            # 导出OBJ文件
            bpy.ops.export_scene.obj(filepath=output_path)
            # 删除对象以防内存溢出
            bpy.data.objects.remove(obj, do_unlink=True)
            print(f"Processed {filename}")

# 设置文件夹路径
process_directory("/home/add_disk_e/objaverse_lvis_trimesh_objs/", "/home/add_disk_e/objaverse_lvis_trimesh_objs_normalized/", 100)
# TEST
# process_directory("/home/fudan248/zhangjinyu/code_repo/objaverse/sample_input/test_objs/original", "/home/fudan248/zhangjinyu/code_repo/objaverse/sample_input/test_objs/normalized", 1)
