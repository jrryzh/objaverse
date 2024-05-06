import trimesh
import os

def convert_glb_to_obj(source_folder, target_folder):
    # 检查目标文件夹是否存在，不存在则创建
    if not os.path.exists(target_folder):
        os.makedirs(target_folder)

    # 遍历源文件夹中的所有文件
    all_lst = os.listdir(source_folder)
    all_lst = [filename for filename in all_lst if filename.endswith('.glb')]
    for filename in all_lst[:1000]:
        if filename.endswith('.glb'):
            # 构建完整的文件路径
            glb_path = os.path.join(source_folder, filename)
            obj_path = os.path.join(target_folder, filename[:-4] + '.obj')

            # 加载glb文件
            mesh = trimesh.load(glb_path, force='mesh')

            # 导出为obj文件
            with open(obj_path, 'w') as f:
                mesh.export(f, file_type='obj')
            print(f'Converted {filename} to OBJ format.')

# 使用示例
source_folder = '/home/add_disk_e/objaverse/hf-objaverse-v1/glbs/000-000/'
target_folder = '/home/fudan248/zhangjinyu/tmp/'
convert_glb_to_obj(source_folder, target_folder)
