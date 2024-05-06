import os
import sys
import random
import json
import time
import trimesh

def normalize_mesh(mesh):
    # 获取模型的边界框尺寸
    bounding_box = mesh.bounding_box_oriented
    # 计算缩放因子，使得最长的边界框维度为1
    scale_factor = 1.0 / max(bounding_box.extents)
    # 将模型缩放
    mesh.apply_scale(scale_factor)
    # 将模型的中心平移到原点
    translation = -mesh.centroid
    mesh.apply_translation(translation)
    return mesh

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
output_folder = "/home/add_disk_e/objaverse_lvis_trimesh_objs_normalized/"
start_time = time.time()
failed_index_list = [] 

for index, glb_id in enumerate(lvis_annotations_list):
    try:
        glb_internal_dir = lvis_inverted_index[glb_id]
        glb_direct_dir = os.path.join(glb_dir, glb_internal_dir, glb_id+".glb")
        obj_path = os.path.join(output_folder, glb_id + '.obj')
        # 加载glb文件
        mesh = trimesh.load(glb_direct_dir, force='mesh')

        # 导出为obj文件
        with open(obj_path, 'w') as f:
            mesh.export(f, file_type='obj')
        print(f'Converting {index+1}/3000: Converted {glb_id} to OBJ format.')
    except:
        failed_index_list.append(index)
        print(f'Converting {index+1}/3000: Failed to convert {glb_id} to OBJ format.')
        
# 保存失败的索引
with open(os.path.join(output_folder, 'failed_index_list_0_3000.txt'), 'w') as f:
    for index in failed_index_list:
        f.write(str(index)+'\n')
    
# 完成
print("Conversion complete.")
