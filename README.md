objaverse dataset 目录
/home/add_disk_e/objaverse/hf-objaverse-v1
共46207个模型


当前pipeline：
1. 从 /home/add_disk_e/objaverse/hf-objaverse-v1 获取全部模型
<!-- 2. glb2obj 存在 /home/add_disk_e/objaverse_lvis_trimesh_objs/
3. normobj -->
2-3.trimesh_multithread_convert
4. obj2urdf
5. render/sapien_filter.py

stage 1:
先用前3k个统计结果
step 2-3 合并 3900s
step4 几乎不计时间
step5

stage 2:
3k-1w 统计结果
step 2-3 合并 8198s
step4 几乎不计时间
step5


stage 3:
grasp object
obj存储路径：/home/add_disk_e/objaverse_lvis_trimesh_normalized_objs/
渲染结果存储路径:
step 2-3 合并 2300s
step5
/home/fudan248/zhangjinyu/code_repo/objaverse/tools/render/sapien_noscale_0516.py