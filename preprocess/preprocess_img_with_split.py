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
#放弃分割图片
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

if __name__ == '__main__':
    dir_path="/home/lzh/dataSet/software_demo/lake/block1/0"

    output_directory="../out"
    output_directory_image_path=os.path.join(output_directory,"input")
    w_k = 5  # 水平方向分成5块
    h_k = 3  # 垂直方向分成3块
    similarity_threshold=0.8

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
    sorted_file_names = sorted(file_names, key=lambda x: int(re.search(r'\d+', x).group()))
    # cameras=[]
    # images=[]
    cameras={}
    images={}
    cur_camera_id=0
    cur_img_id=0
    instrinscs=[]
    exstrinscs=[]
    without_split_instrinscs=[]
    without_split_exstrinscs=[]
    for i in range(len(sorted_file_names)):
       input_image_path = os.path.join(dir_path, sorted_file_names[i])
       instrinsc,exstrinsc=exif_process.get_camera_param(input_image_path)
       without_split_exstrinscs.append(exstrinsc)
       without_split_instrinscs.append(instrinsc)
    
    for i in range(len(sorted_file_names)):
        input_image_path = os.path.join(dir_path, sorted_file_names[i])
        # instrinsc,exstrinsc=exif_process.get_camera_param(input_image_path)
        instrinsc =without_split_instrinscs[i]
        exstrinsc=without_split_exstrinscs[i]
        now_imgs,iamge_name,width,height= split_img.split_img_with_unque(input_image_path,w_k, h_k)
        ins,exs=split_img.split_camera_param(instrinsc,exstrinsc,width,height,w_k,h_k)
        instrinscs.extend(ins)
        exstrinscs.extend(exs)
        if cur_imgs is None:
                 
            cur_imgs=now_imgs
            for i in range(len(ins)):
                camera=exif_process.instrinsc2camera(ins[i],cur_camera_id,width//w_k,height//h_k)
             
                img=exif_process.extrinsic2image(exs[i],cur_img_id,f"{i+1}_{iamge_name[:-4]}.jpg",camera.id)      
                # cameras.append(camera)
                # images.append(img)
                cameras[camera.id]=camera
                images[img.id]=img
                cur_camera_id+=1
                cur_img_id+=1
            split_img.save_imgs(cur_imgs, output_directory_image_path, iamge_name,exif_data)
        else:

            need_save_imgs=[]
            need_save_instrinsc,need_save_exstrinsc=[],[]
                    #比较相似度
            for j in range(len(cur_imgs)):
                if  tool.calculate_similarity(np.array(cur_imgs[j]), np.array(now_imgs[j]))<similarity_threshold:
                    need_save_imgs.append(now_imgs[j])
                    need_save_instrinsc.append(ins[j])
                    need_save_exstrinsc.append(exs[j])
                    cur_imgs[j]=now_imgs[j]
            if need_save_imgs:
                print(len(need_save_imgs))

                for i in range(len(need_save_instrinsc)):
                    camera=exif_process.instrinsc2camera(need_save_instrinsc[i],cur_camera_id,width//w_k,height//h_k)
    
                    img=exif_process.extrinsic2image(need_save_exstrinsc[i],cur_img_id,f"{i+1}_{iamge_name[:-4]}.jpg",camera.id)      
                    # cameras.append(camera)
                    # images.append(img)
                    cameras[camera.id]=camera
                    images[img.id]=img
                    cur_camera_id+=1
                    cur_img_id+=1
            
                split_img.save_imgs(need_save_imgs, output_directory_image_path, iamge_name,exif_data)
    # print(cameras)
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
