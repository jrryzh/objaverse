import os
import json
import time
import trimesh
from concurrent.futures import ThreadPoolExecutor

def normalize_and_convert(glb_id, glb_dir, output_folder, lvis_inverted_index):
    try:
        glb_internal_dir = lvis_inverted_index[glb_id]
        glb_direct_dir = os.path.join(glb_dir, glb_internal_dir, glb_id + ".glb")
        obj_path = os.path.join(output_folder, glb_id + '.obj')
        # 加载glb文件
        mesh = trimesh.load(glb_direct_dir, force='mesh')
        # 导出为obj文件
        with open(obj_path, 'w') as f:
            mesh.export(f, file_type='obj')
        print(f'Converted {glb_id} to OBJ format.')
        return f'Converted {glb_id} to OBJ format.'
    except Exception as e:
        print(f'Failed to convert {glb_id} to OBJ format due to {str(e)}')
        return f'Failed to convert {glb_id} to OBJ format due to {str(e)}'

def main():
    # 从JSON文件中导入数据
    with open('/home/fudan248/zhangjinyu/data/objaverse/lvis.json', 'r') as json_file:
        lvis_annotations = json.load(json_file)

    lvis_annotations_list = []
    for values in lvis_annotations.values():
        lvis_annotations_list.extend(values)
    lvis_annotations_list = sorted(lvis_annotations_list)[:3000]

    with open('/home/fudan248/zhangjinyu/data/objaverse/lvis_inverted_index.json', 'r') as json_file:
        lvis_inverted_index = json.load(json_file)

    glb_dir = "/home/add_disk_e/objaverse/hf-objaverse-v1/glbs"
    output_folder = "/home/add_disk_e/objaverse_lvis_trimesh_objs_normalized/"
    start_time = time.time()

    print("Starting conversion...")
    # 创建线程池
    with ThreadPoolExecutor(max_workers=4) as executor:
        results = list(executor.map(lambda glb_id: normalize_and_convert(glb_id, glb_dir, output_folder, lvis_inverted_index), lvis_annotations_list))

    for result in results:
        print(result)

    print("Conversion complete. Time elapsed:", time.time() - start_time)

if __name__ == "__main__":
    main()
