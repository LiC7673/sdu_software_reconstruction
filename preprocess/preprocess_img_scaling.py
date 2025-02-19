import exif_process
import os
import split_img
import cv2
import numpy as np
import tool
import re 
import sqlite3
import sys
import subprocess
from PIL import Image
IS_PYTHON3 = sys.version_info[0] >= 3
def array_to_blob(array):
    if IS_PYTHON3:
        return array.tobytes()
    else:
        return np.getbuffer(array)


from read_write_model import write_images_text,write_cameras_text,write_points3D_text,write_images_text_without_p3d
def insert_camera_params(database_path, cameras):
    conn = sqlite3.connect(database_path)
    cursor = conn.cursor()
    
    # 插入相机信息

    for _,camera in cameras.items():
        camera_id=int(camera.id)
        camera_model=camera.model
        width=int(camera.width)
        height=int(camera.height)
        params=array_to_blob(camera.params)

        if camera_model == "PINHOLE":
  
            cursor.execute('''
                INSERT INTO cameras (camera_id, model, width, height, params,prior_focal_length) 
                VALUES (?, ?, ?, ?, ?, ?)
            ''', (camera_id, 1, width, height, params, False))
        elif camera_model == "SIMPLE_RADIAL":
  
            cursor.execute('''
                INSERT INTO cameras (camera_id, model, width, height, params,prior_focal_length) 
                VALUES (?, ?, ?, ?, ?, ?)
                ''', (camera_id, 0, width, height, params, False))        # 添加更多相机模型类型

    conn.commit()
    conn.close()

# 将图像位姿插入数据库
def insert_image_params(database_path, images):
    conn = sqlite3.connect(database_path)
    cursor = conn.cursor()
    
    # 插入图像信息
    for _,image in images.items():
        image_id=image.id
        qvec=image.qvec
        tvec=image.tvec
        camera_id=image.camera_id
        name=image.name
      
        cursor.execute('''
            INSERT INTO images (image_id,name, camera_id, prior_qw, prior_qx, prior_qy, prior_qz,prior_tx, prior_ty, prior_tz ) 
            VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
        ''', (image_id,name, camera_id, qvec[0], qvec[1], qvec[2], qvec[3], tvec[0], tvec[1], tvec[2]))
    
    conn.commit()
    conn.close()

def scaling_camera_params(instrinsc,exstrinsc,alpha):
    instrinsc=instrinsc*alpha
    return instrinsc,exstrinsc
def scaling_image(input_path,out_path,alpha=0.25):

    img = cv2.imread(input_path)  # 读取图片
    h, w = img.shape[:2] 
    new_width = int(w * alpha)
    new_height = int(h * alpha)
    resized_img = cv2.resize(img, (new_width, new_height), interpolation=cv2.INTER_AREA)  
    cv2.imwrite(out_path, resized_img)  # 保存
if __name__ == '__main__':
    dir_path="/home/lzh/dataSet/software_demo/new_lake/lake"

    output_directory="/home/lzh/igip_project/sdu_software_reconstruction/out/lake_low"
    output_directory_image_path=os.path.join(output_directory,"input")

    alpha=0.25
    database_path=os.path.join(output_directory,"database.db")
    if not os.path.exists(output_directory):
        os.makedirs(output_directory)
    os.makedirs(output_directory_image_path,exist_ok=True)
    camera_txt_path=os.path.join(output_directory,"cameras.txt")
    image_txt_path=os.path.join(output_directory,"images.txt")
    point3d_txt_path=os.path.join(output_directory,"points3D.txt")
    if os.path.exists(database_path):
        os.remove(database_path)

    subprocess.run(["colmap", "database_creator", "--database_path",database_path])
    print(f"colmap init {database_path}")
    with open(point3d_txt_path, "w") as file:
        pass 

    file_names=[]
    cur_imgs,iamge_name,exif_data=None,None,None
    now_imgs=None
    for root, dirs, files in os.walk(dir_path):
        for file in files:
            if file.lower().endswith(".jpg"):  # 检查文件扩展名是否为.jpg（忽略大小写）
                file_names.append(file)
    #sorted_file_names = file_names
    sorted_file_names = sorted(file_names, key=lambda x: int(re.search(r'\d+', x).group()))
    # cameras=[]
    # images=[]
    cameras={}
    images={}
    cur_camera_id=0
    cur_img_id=0
    instrinscs=[]
    exstrinscs=[]
    without_preprocess_instrinscs=[]
    without_preprocess_exstrinscs=[]
    print("numbers :",len(sorted_file_names))
    for i in range(len(sorted_file_names)):
       input_image_path = os.path.join(dir_path, sorted_file_names[i])
       instrinsc,exstrinsc=exif_process.get_camera_param(input_image_path)
       without_preprocess_exstrinscs.append(exstrinsc)
       without_preprocess_instrinscs.append(instrinsc)
    without_preprocess_exstrinscs,_=exif_process.norm_extrinsic_divide(without_preprocess_exstrinscs)
  #  print("without_preprocess_exstrinscs")
    input_image_path = os.path.join(dir_path, sorted_file_names[0])
    iamge_name = os.path.basename(input_image_path)
    img = Image.open( input_image_path)

        # 获取图片尺寸
    width, height = img.size
    for i in range(len(sorted_file_names)):
        input_image_path = os.path.join(dir_path, sorted_file_names[i])
        iamge_name = os.path.basename(input_image_path)
        # instrinsc,exstrinsc=exif_process.get_camera_param(input_image_path)
        instrinsc = without_preprocess_instrinscs[i]
        exstrinsc = without_preprocess_exstrinscs[i]
        instrinsc,exstrinsc=scaling_camera_params(instrinsc,exstrinsc,alpha)
        instrinscs.append(instrinsc)
        exstrinscs.append(exstrinsc)

        camera=exif_process.instrinsc2camera(instrinsc,cur_camera_id,width*alpha,height*alpha)
        image=exif_process.extrinsic2image(exstrinsc,cur_img_id,iamge_name,camera.id)      
        cameras[camera.id]=camera
        images[image.id]=image
        cur_camera_id+=1
        cur_img_id+=1
        scaling_image(input_image_path,os.path.join(output_directory_image_path,sorted_file_names[i]))
    # print(cameras)
    print(len(cameras))
    insert_camera_params(database_path, cameras)
    insert_image_params(database_path, images)
    write_cameras_text(cameras, camera_txt_path)
    write_images_text_without_p3d(images, image_txt_path)
    # np.save(instrinscs,os.path.join(output_directory,"ins.npy"))
    # np.save(exstrinscs,os.path.join(output_directory,"exs.npy"))
    np.savez(os.path.join(output_directory,"camera_params.npz"),instrinscs=instrinscs,exstrinscs=exstrinscs)
    # for i in range(len(cameras)):
    #     write_cameras_text(cameras[i], camera_txt_path)
    #     write_images_text(images[i], image_txt_path)
