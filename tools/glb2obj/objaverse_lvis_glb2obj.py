import bpy
import os
import sys
import random
import json
import time

# 导入 lvis_annotations
# lvis_annotations = objaverse.load_lvis_annotations('/home/fudan248/zhangjinyu/data/objaverse/lvis_v1_train.json')
with open('/home/fudan248/zhangjinyu/data/objaverse/lvis.json', 'r') as json_file:
    lvis_annotations = json.load(json_file)
# 生成 lvis_annotations_list
lvis_annotations_list = []
for values in lvis_annotations.values():
    lvis_annotations_list.extend(values)
# 排序
lvis_annotations_list = sorted(lvis_annotations_list)[:3000]

# 从JSON文件中导入数据
with open('/home/fudan248/zhangjinyu/data/objaverse/lvis_inverted_index.json', 'r') as json_file:
    lvis_inverted_index = json.load(json_file)

glb_dir = "/home/add_disk_e/objaverse/hf-objaverse-v1/glbs"
output_folder = "/home/add_disk_e/objaverse_lvis_objs/"
start_time = time.time()
for index, glb_id in enumerate(lvis_annotations_list):
    
    glb_internal_dir = lvis_inverted_index[glb_id]
    glb_direct_dir = os.path.join(glb_dir, glb_internal_dir, glb_id+".glb")

    # # 获取所有的 GLB 文件
    # glb_files = [f for f in os.listdir(input_folder) if f.endswith('.glb')]
    # # 如果文件数量多于我们需要的采样数量，则进行随机采样
    # if sample_size != -1:   
    #     if len(glb_files) > sample_size:
    #         glb_files = random.sample(glb_files, sample_size)

    # 导入 GLB 文件
    bpy.ops.import_scene.gltf(filepath=glb_direct_dir)
    
    # 导出为 OBJ
    output_file = os.path.join(output_folder, glb_id+".obj")
    bpy.ops.export_scene.obj(filepath=output_file)
    
    # 清理场景，为下一个文件做准备
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()
    
    current_time = time.time()
    print(f"Time taken for {index+1}/{len(lvis_annotations_list)}: {current_time-start_time:.2f} seconds")

# 完成
print("Conversion complete.")
