import bpy
import os
import sys
import random

# 获取从命令行传入的文件夹路径和采样数量
input_folder = sys.argv[-3]
sample_size = int(sys.argv[-2])  # 要采样的文件数量
output_folder = sys.argv[-1]

# 清理默认场景
bpy.ops.wm.read_factory_settings(use_empty=True)

# 获取所有的 GLB 文件
glb_files = [f for f in os.listdir(input_folder) if f.endswith('.glb')]
# 如果文件数量多于我们需要的采样数量，则进行随机采样
if sample_size != -1:   
    if len(glb_files) > sample_size:
        glb_files = random.sample(glb_files, sample_size)

for filename in glb_files:
    file_path = os.path.join(input_folder, filename)
    # 导入 GLB 文件
    bpy.ops.import_scene.gltf(filepath=file_path)
    
    # 导出为 OBJ
    output_file = os.path.join(output_folder, filename.replace('.glb', '.obj'))
    bpy.ops.export_scene.obj(filepath=output_file)
    
    # 清理场景，为下一个文件做准备
    bpy.ops.object.select_all(action='DESELECT')
    bpy.ops.object.select_by_type(type='MESH')
    bpy.ops.object.delete()

# 完成
print("Conversion complete.")
