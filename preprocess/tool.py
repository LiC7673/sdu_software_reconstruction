import cv2
from skimage.metrics import structural_similarity as ssim
def calculate_similarity(frame1, frame2):
    """
    计算两帧之间的相似度。
    使用结构相似性指数 (SSIM) 进行比较。
    """
    # 转换为灰度图

    gray1 = cv2.cvtColor(frame1, cv2 .COLOR_BGR2GRAY)
    gray2 = cv2.cvtColor(frame2, cv2.COLOR_BGR2GRAY)

    # 计算 SSIM 相似度
    similarity, _ = ssim(gray1, gray2, full=True)
    return similarity