objaverse dataset 目录
/home/add_disk_e/objaverse/hf-objaverse-v1



当前pipeline：
1. 从 /home/add_disk_e/objaverse/hf-objaverse-v1 获取全部模型
2. glb2obj 存在 /home/add_disk_e/objaverse_lvis_trimesh_objs/
3. normobj

2、3步骤可以合并
trimesh_multithread_convert

4. obj2urdf
5. render/sapien_filter.py

先用前3k个统计结果
step 2-3 合并 3900s
step4 几乎不计时间
step5