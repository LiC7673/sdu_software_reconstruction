# if [ ! -z "$1" ]; then
#     path=$1
# fi
echo $1
PROJECT_PATH='/home/lzh/igip_project/software_reconstrution/out'
colmap_PATH=/usr/bin/colmap
echo $PROJECT_PATH/database.db 

colmap_PATH feature_extractor --database_path $PROJECT_PATH/database.db --image_path  $PROJECT_PATH/input
colmap_PATH exhaustive_matcher --database_path  $PROJECT_PATH/database.db
colmap_PATH point_triangulator --database_path $PROJECT_PATH/database.db --image_path $PROJECT_PATH/input --input_path $PROJECT_PATH--output_path $PROJECT_PATH
# # 或者
# colmap mapper --database_path  $PROJECT_PATH/database.db --image_path images --input_path $PROJECT_PATH/sparse/model --output_path  $PROJECT_PATH/sparse/model

# colmap feature_extractor --database_path $cur_img_path$/database.db --image_path $cur_img_path$
# " feature_extractor "\
#         "--database_path " + args.source_path + "/distorted/database.db \
#         --image_path " + args.source_path + "/input \
#         --ImageReader.single_camera 1 \
#         --ImageReader.camera_model " + args.camera + " \
#         --SiftExtraction.use_gpu " + str(use_gpu) 