from PIL import Image
import pyexif
import os
import re
import tool
import cv2
import numpy as np
import exif_process
import copy
def split_image(input_path, output_dir, w_k, h_k):
    # 打开原始图片
    iamge_name = os.path.basename(input_path)
    img = Image.open(input_path)
    exif_data=img.getexif()

    # 获取图片尺寸
    width, height = img.size

    # 计算每个块的尺寸
    block_width = width // w_k
    block_height = height // h_k

    # 分割图片为 w_k * h_k 块
    images = []
    for i in range(h_k):
        for j in range(w_k):
            left = j * block_width
            upper = i * block_height
            right = (j + 1) * block_width
            lower = (i + 1) * block_height
            images.append(img.crop((left, upper, right, lower)))


    # 保存分割后的图片，并继承 EXIF 信息
    output_paths = []
    for i in range(len(images)):
        output_path = f"{output_dir}/{iamge_name[:-4]}_{i+1}.jpg"
        output_paths.append(output_path)
        # 保存图片时附加 EXIF 信息
        images[i].save(output_path, "JPEG", exif=exif_data)
        print(f"保存图片到: {output_path}")

def split_img_with_unque(input_path,w_k, h_k):
        # 打开原始图片
    iamge_name = os.path.basename(input_path)
    img = Image.open(input_path)


    # 获取图片尺寸
    width, height = img.size

    # 计算每个块的尺寸
    block_width = width // w_k
    block_height = height // h_k

    # 分割图片为 w_k * h_k 块
    images = []
    for i in range(h_k):
        for j in range(w_k):
            left = j * block_width
            upper = i * block_height
            right = (j + 1) * block_width
            lower = (i + 1) * block_height
            images.append(img.crop((left, upper, right, lower)))

    img = pyexif.ExifEditor(input_path) 
    exif_data= img.getDictTags()
    return images,iamge_name,width,height
def save_imgs(images, output_dir, iamge_name,exif_data=None):
    print(iamge_name,len(images))
    output_paths = []
    # print(exif_data)
    # exif_bytes = pyexif.dump(exif_data)
    for i in range(len(images)):
        output_path = f"{output_dir}/{iamge_name[:-4]}_{i+1}.jpg"
        output_paths.append(output_path)
        images[i].save(output_path, "JPEG")
        # print(f"保存图片到: {output_path}")
def split_imgs(input_image_dir_path, output_directory,w_k = 5,h_k = 3, similarity_threshold=0.8):
    file_names=os.listdir(input_image_dir_path)
    cur_imgs,iamge_name,exif_data=None,None,None
    now_imgs=None
    sorted_file_names = sorted(file_names, key=lambda x: int(re.search(r'\d+', x).group()))
    for i in range(len(sorted_file_names)):
        input_image_path = os.path.join(input_image_dir_path, sorted_file_names[i])
        if cur_imgs is None:
            cur_imgs,iamge_name,_,_=split_img_with_unque(input_image_path,w_k, h_k)
            print(len(cur_imgs))
            save_imgs(cur_imgs, output_directory, iamge_name,exif_data)
        else:
            now_imgs,iamge_name,_,_=split_img_with_unque(input_image_path,w_k, h_k)
            need_save_imgs=[]
            #比较相似度
            for j in range(len(cur_imgs)):
                if  tool.calculate_similarity(np.array(cur_imgs[j]), np.array(now_imgs[j]))<similarity_threshold:
                    need_save_imgs.append(now_imgs[j])
                    cur_imgs[j]=now_imgs[j]
            if need_save_imgs:
                print(len(need_save_imgs))
                save_imgs(need_save_imgs, output_directory, iamge_name,exif_data)

#如果仅仅是裁剪，只需要考虑内参即可
def split_camera_param(instrinsc,exstrinsc,weight,height,w_k,h_k):
    instrinscs=[]
    exstrinscs=[]
    for i in range(h_k):
        for j in range(w_k):
            #内参
            instrinsc_i = copy.deepcopy(instrinsc)
            instrinsc_i[0, 2] = instrinsc_i[0, 2] - i/w_k*weight* instrinsc_i[0, 2]
            instrinsc_i[1, 2] = instrinsc_i[1, 2] - j/h_k*height * instrinsc_i[1, 2]
            instrinscs.append(instrinsc_i)
            exstrinscs.append(copy.deepcopy(exstrinsc))
    return instrinscs,exstrinscs



if __name__ == "__main__":
    input_image_dir_path = '/home/lzh/dataSet/software_demo/lake/block1/0'
    output_directory = "../out"
    w_k = 5  # 水平方向分成5块
    h_k = 3  # 垂直方向分成3块
    similarity_threshold=0.8
    split_imgs(input_image_dir_path, output_directory,w_k, h_k,similarity_threshold)
                
      
    # split_image(input_image_path, output_directory, w_k, h_k)
# # 使用示例
# input_image_path = '/home/lzh/dataSet/DJI_0129.JPG'
# output_directory = "../out"
# w_k = 2  # 水平方向分成2块
# h_k = 2  # 垂直方向分成2块
# split_image(input_image_path, output_directory, w_k, h_k)
