import pyexif
import os
import re
import math
import numpy as np
from scipy.spatial.transform import Rotation 
import piexif
from read_write_model import Camera,Image,write_cameras_text,write_images_text,write_cameras_binary,write_images_binary
#通过exif读取相机的内外参


def compute_DJI_instrinc(tags_dict):
    #因为是大疆无人机Phantom,焦距是8mm（可变）,传感器尺寸是1
    numbers = re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["FocalLength"])
    focal_length=float(numbers[0])# 焦距单位都是mm，后面要转化为米
    focal_length=focal_length/1000.0
    #计算ccd
    focalLengthIn35mmFormat=re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["FocalLengthIn35mmFormat"])# 35mm等效焦距,用于计算cx,cy
    focalLengthIn35mmFormat=float(focalLengthIn35mmFormat[0])
    focalLengthIn35mmFormat=focalLengthIn35mmFormat/1000.0


    width=tags_dict["ImageWidth"]
    height=tags_dict["ImageHeight"]
    w_h_rate=width/height
    ccd35= 43.27/1000.0
    ccd_cur=(ccd35*focal_length)/focalLengthIn35mmFormat
    dy=ccd_cur/math.sqrt(w_h_rate**2+1)
    dx=dy*w_h_rate

    # FOV=84/180.0*math.pi
    # xxx=(math.tan(FOV/2) * focal_length) * 2
    # print("ccd,dx,dy,xxx:",ccd_cur,dx,dy,xxx)
    fx=focal_length*width/dx
    fy=focal_length*height/dy
    cx=width/2.0
    cy=height/2.0
   # print("fx,fy,cx,cy:",fx,fy,cx,cy)
    

    #这是在像素空间上的内参矩阵
    instrinc=np.array([[fx, 0, cx], 
              [0, fy, cy], 
              [0, 0, 1]])
   # focal_length=focal_length/1000.0
    return instrinc
#计算内参矩阵，输入通过pyexif读取的tags_dict
def read_DJI_instrinc(tags_dict):
    instrinc=None
    try:
        F=float(tags_dict["CalibratedFocalLength"])
        cx=float( tags_dict["CalibratedOpticalCenterX"])
        cy=float( tags_dict["CalibratedOpticalCenterY"])
        print("fx,fy,cx,cy:",F,F,cx,cy)
        instrinc=np.array([[F, 0, cx], 
                [0, F, cy], 
                [0, 0, 1]])       
    except Exception as e:
        print(e)
        print("read Calibrated params error,compute")
        try:
            instrinc=compute_DJI_instrinc(tags_dict)
        except:
            raise "compute_DJI_instrincs error"
    return instrinc



# 转换角度为弧度
def degrees_to_radians(degrees):
    return degrees * np.pi / 180.0

# 计算绕X轴（Pitch）旋转的旋转矩阵
def rotation_matrix_x(pitch):
    pitch_rad = degrees_to_radians(pitch)
    return np.array([
        [1, 0, 0],
        [0, np.cos(pitch_rad), -np.sin(pitch_rad)],
        [0, np.sin(pitch_rad), np.cos(pitch_rad)]
    ])

# 计算绕Y轴（Yaw）旋转的旋转矩阵
def rotation_matrix_y(yaw):
    yaw_rad = degrees_to_radians(yaw)
    return np.array([
        [np.cos(yaw_rad), 0, np.sin(yaw_rad)],
        [0, 1, 0],
        [-np.sin(yaw_rad), 0, np.cos(yaw_rad)]
    ])

# 计算绕Z轴（Roll）旋转的旋转矩阵
def rotation_matrix_z(roll):
    roll_rad = degrees_to_radians(roll)
    return np.array([
        [np.cos(roll_rad), -np.sin(roll_rad), 0],
        [np.sin(roll_rad), np.cos(roll_rad), 0],
        [0, 0, 1]
    ])
def calculate_rotation_matrix(yaw, pitch, roll):
    R_yaw = rotation_matrix_y(yaw)
    R_pitch = rotation_matrix_x(pitch)
    R_roll = rotation_matrix_z(roll)
    
    # 旋转矩阵是先绕Z轴（Roll），再绕X轴（Pitch），最后绕Y轴（Yaw）旋转
    R = np.dot(R_yaw, np.dot(R_pitch, R_roll))
    return R 
# 经纬度转换函数
def dms_to_decimal(degrees, minutes, seconds):
    return degrees + (minutes / 60) + (seconds / 3600)
def compute_gps_position(latitude_dms,longitude_dms,h): 
    latitude_deg = dms_to_decimal(*latitude_dms)  # 纬度
    longitude_deg = dms_to_decimal(*longitude_dms)  # 经度

    # 转换为弧度
    latitude_rad = math.radians(latitude_deg)
    longitude_rad = math.radians(longitude_deg)

    # WGS-84 椭球体参数
    a = 6378137  # 地球的长半轴（米）
    f = 1 / 298.257223563  # 扁率
    e2 = 2 * f - f**2  # 第一偏心率平方

    # 计算曲率半径 N(φ)
    N = a / math.sqrt(1 - e2 * math.sin(latitude_rad)**2)

    # 假设高度 h = 0（地面高度）
    h = 0

    # 计算笛卡尔坐标 (x, y, z)
    x = (N + h) * math.cos(latitude_rad) * math.cos(longitude_rad)
    y = (N + h) * math.cos(latitude_rad) * math.sin(longitude_rad)
    z = ((1 - e2) * N + h) * math.sin(latitude_rad)
    return x,y,z
#计算外参矩阵，输入通过pyexif读取的tags_dict,输出4*4的 np.array
def compute_DIJ_extrinsic(tags_dict):
    # pitch= re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["CameraPitch"])
    # pitch=float(pitch[0])

    # roll=  re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["CameraRoll"])
    # roll=float(roll[0])
    # yaw=re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["CameraYaw"])
    # yaw=float(yaw[0])

    pitch=float(tags_dict["CameraPitch"])
    roll=float(tags_dict["CameraRoll"])
    yaw=float(tags_dict["CameraYaw"])
 #   print("pitch,roll,yaw:",pitch,roll,yaw)
    R=calculate_rotation_matrix(yaw, pitch, roll)

    latitude_dms= re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["GPSLatitude"])
    latitude_dms= [float(i) for i in latitude_dms]
    longitude_dms= re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["GPSLongitude"])
    longitude_dms= [float(i) for i in longitude_dms]
    h= re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["GPSAltitude"])
    h=float(h[0])
    camera_position=compute_gps_position(latitude_dms,longitude_dms,h)
    extrinsic_matrix = np.zeros((4, 4))
    extrinsic_matrix[:3, :3] = R  # 旋转矩阵
    extrinsic_matrix[:3, 3] = camera_position  # 位移向量
    extrinsic_matrix[3, 3] = 1  # 最后一行最后一列是1
    return extrinsic_matrix

def calculate_quaternion(extrinsic):
    R=extrinsic[:3,:3]
    T=extrinsic[:3,3]
    r = Rotation.from_matrix(R)
    qx,qy,qz,qw = r.as_quat()
    return qx,qy,qz,qw,T[0],T[1],T[2]
def dict_camera_param(instrinc,extrinsic):
    fx,fy,cx,cy=instrinc[0,0],instrinc[1,1],instrinc[0,2],instrinc[1,2]
    qx,qy,qz,qw,t1,t2,t3=calculate_quaternion(extrinsic)

    return {"fx":fx,"fy":fy,"cx":cx,"cy":cy,"qx":qx,"qy":qy,"qz":qz,"qw":qw,"t1":t1,"t2":t2,"t3":t3}

def read_dict_camera_param(dict):
    fx=dict["fx"]
    fy=dict["fy"]
    cx=dict["cx"]
    cy=dict["cy"]
    qx=dict["qx"]
    qy=dict["qy"]
    qz=dict["qz"]
    qw=dict["qw"]
    t1=dict["t1"]
    t2=dict["t2"]
    t3=dict["t3"]
    instrinc=np.array([[fx, 0, cx], 
        [0, fy, cy], 
        [0, 0, 1]])     

    r=Rotation.from_quat(np.array([qx,qy,qz,qw])).as_matrix()
    print(r)
    exstrinc=np.array([[r[0,0], r[0,1], r[0,2], t1], 
                        [r[1,0], r[1,1], r[1,2], t2], 
                        [r[2,0], r[2,1], r[2,2], t3],
                        [0, 0, 0, 1]])
    return instrinc,exstrinc


def instrinsc2camera(instrinc,id,width,height): 
    fx,fy,cx,cy=instrinc[0,0],instrinc[1,1],instrinc[0,2],instrinc[1,2]
    model="PINHOLE"

    return  Camera(
        id=id,
        model=model,
        width=width,
        height=height,
        params=np.array((fx, fy, cx, cy))
    )

def extrinsic2image(extrinsic,id,name,camera_id):
    qx,qy,qz,qw,t1,t2,t3=calculate_quaternion(extrinsic)
    return  Image(
                id=id,
                qvec=np.array((qw,qx,qy,qz)),
                tvec=np.array((t1,t2,t3)),
                camera_id=camera_id,
                name=name,
                xys=None,
                point3D_ids=None,
    )
def get_camera_param(img_path):
    img_d= pyexif.ExifEditor(img_path) 
    tags_dict= img_d.getDictTags()
    instrinc,extrinsic=compute_DJI_instrinc(tags_dict),compute_DIJ_extrinsic(tags_dict)
    return instrinc,extrinsic


## 归一化外参矩阵
def norm_extrinsic_divide(extrinsic_matrices):
    norms = [np.linalg.norm(extrinsic[:3, 3]) for extrinsic in extrinsic_matrices]

    # 取最大范数作为全局缩放因子
    max_norm = max(norms)
    normalized_extrinsics_global = []
    for extrinsic in extrinsic_matrices:
        t = extrinsic[:3, 3]
        new_extrinsic = extrinsic.copy()
        if max_norm != 0:
            new_extrinsic[:3, 3] = t / max_norm  # 按最大范数缩放
        normalized_extrinsics_global.append(new_extrinsic)
    return normalized_extrinsics_global,max_norm

# def norm_extrinsic_max_min(extrinsic_matrices):
#     t_vectors = np.array([extrinsic[:3, 3] for extrinsic in extrinsic_matrices]) 
#     t_min = np.min(t_vectors, axis=0)
#     t_max = np.max(t_vectors, axis=0)

#     # 防止除零错误
#     t_range = t_max - t_min
#     t_range[t_range == 0] = 1  # 如果最大值等于最小值，避免除0错误

#     # 归一化所有 t
#     t_vectors_normalized = (t_vectors - t_min) / t_range

#     # 更新所有外参矩阵
#     normalized_extrinsics = []
#     for i, extrinsic in enumerate(extrinsic_matrices):
#         new_extrinsic = extrinsic.copy()
#         new_extrinsic[:3, 3] = t_vectors_normalized[i]  # 赋值归一化的 t
#         normalized_extrinsics.append(new_extrinsic)

#     return normalized_extrinsics,t_max,t_min,t_range

## 


#jpg格式
def save_vis(dir_path,out_path):
    file_names = []
    exstrinscs=[]
    instrinscs=[]
    for root, dirs, files in os.walk(dir_path):
        for file in files:
            if file.lower().endswith(".jpg") or file.lower().endswith(".jpeg"):  # 检查文件扩展名是否为.jpg（忽略大小写）
                file_names.append(file)

    for i in range(len(file_names)):
       input_image_path = os.path.join(dir_path,file_names[i])
       instrinsc,exstrinsc=get_camera_param(input_image_path)
       instrinscs.append(instrinsc)
       exstrinscs.append(exstrinsc)
    np.savez(os.path.join(out_path,"camera_params.npz"),instrinscs=instrinscs,exstrinscs=exstrinscs)
#test code
#统一单位都是米
if __name__ == "__main__":
    # path='/home/lzh/dataSet/softcollege/100_0001/input/'
   # dir_path='/home/lzh/dataSet/software_demo/new_lake/lake'
    dir_path='/home/lzh/dataSet/software_demo/lake/block1/0'
    save_vis(dir_path,'./')
    # imgs_names=os.listdir(path)
    # for img_name in imgs_names:
    #     img = pyexif.ExifEditor(path+img_name) 

    #     tags_dict= img.getDictTags()
    #     # print(type( tags_dict["FocalLength"]))
    #     print(tags_dict["FocalLength"])
    
    # img_d= pyexif.ExifEditor('/home/lzh/dataSet/DJI_0129.JPG') 
    
    # # img_d = pyexif.ExifEditor("/home/lzh/igip_project/software_reconstrution/preprocess/image_with_metadata.jpg") 
    # tags_dict= img_d.getDictTags()


    # # print(tags_dict["camrera_params"])
    # import piexif
    # from PIL import Image
    # import json
    # image_path='/home/lzh/dataSet/DJI_0129.JPG'
    # img= Image.open(image_path)

    # # 获取现有的 EXIF 数据
    # exif_dict = piexif.load(img.info["exif"]) if "exif" in img.info else piexif.load(piexif.dump({}))

    # # for key in exif_dict:
    # #     print(key, exif_dict[key])
    # my_dict=dict_camera_param(compute_DJI_instrinc(tags_dict),compute_DIJ_extrinsic(tags_dict))
    # metadata_json = json.dumps(my_dict)  # 将字典转换为 JSON 字符串
    # # 将自定义数据插入到 UserComment 字段（或者其他适当的字段）
    # if 0x9286 not in exif_dict['0th']:
    #     exif_dict['0th'][0x9286] = b''
    # exif_dict['0th'][0x9286] = metadata_json.encode('utf-8')

    # # 将修改后的 EXIF 数据保存回图像
    # exif_bytes = piexif.dump(exif_dict)
    # img.save("image_with_metadata.jpg", exif=exif_bytes)

    # # 解析 EXIF 数据为字典
    # tags_dict = exif_to_dict(exif_data)
    # tags_dict = piexif.load('/home/lzh/dataSet/DJI_0042.JPG')
    # print(tags_dict)
    # for key in tags_dict:
    #   print(key, tags_dict[key])
    # print(compute_DJI_instrinc(tags_dict))
    # print(compute_DIJ_extrinsic(tags_dict))
    # print(dict_camera_param(compute_DJI_instrinc(tags_dict),compute_DIJ_extrinsic(tags_dict)))
    # inst,exst=read_dict_camera_param(dict_camera_param(compute_DJI_instrinc(tags_dict),compute_DIJ_extrinsic(tags_dict)))
    # print(inst-compute_DJI_instrinc(tags_dict))
    # print(exst-compute_DIJ_extrinsic(tags_dict))
    # print(read_DJI_instrinc(tags_dict))
    # print(compute_DJI_instrinc(tags_dict))
    # numbers = re.findall(r"[-+]?\d*\.\d+|\d+", tags_dict["FocalLength"])
    # print(type( tags_dict["FocalLength"]))# 焦距单位都是mm，要转化
    
    # print(float(numbers[0]))

    # for key in tags_dict:
    #     print(key, tags_dict[key])