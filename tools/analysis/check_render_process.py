import os

# 初始化finished变量用于统计超过10个文件的子文件夹数量
total_finished = 0
total_all = 0

# 获取当前文件夹下的所有子文件夹
subfolders = [f.path for f in os.scandir('/home/add_disk_e/objaverse_lvis_trimesh_normalized_objs_output') if f.is_dir()]

# 遍历每个子文件夹，并统计文件数量
for folder in subfolders:
    all, finished = 0, 0
    subsubfolders = [f.path for f in os.scandir(folder) if f.is_dir()]
    for subfolder in subsubfolders:
        all += 1
        total_all += 1
        file_count = len([name for name in os.listdir(subfolder) if os.path.isfile(os.path.join(subfolder, name))])
        if file_count == 502:
            finished += 1
            total_finished += 1
    print(f"{folder}: {finished}/{all}")
    

print(f"Total: {total_finished}/{total_all}")
