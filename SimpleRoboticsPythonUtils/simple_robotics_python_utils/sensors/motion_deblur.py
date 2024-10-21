#! /usr/bin/env python3
# import cv2
# import numpy as np
# from scipy.signal import convolve2d

# def blind_deconvolution(image, kernel_size, iterations):
#     # 初始化模糊核和清晰图像
#     kernel = np.zeros((kernel_size, kernel_size))
#     kernel[kernel_size//2, :] = 1.0 / kernel_size

#     # 盲去卷积迭代过程
#     for _ in range(iterations):
#         # 估计模糊图像
#         blurred_image = convolve2d(image, kernel, mode='same', boundary='symm', fillvalue=0)

#         # 更新模糊核
#         restored_image = convolve2d(blurred_image, np.rot90(kernel, 2), mode='same', boundary='symm', fillvalue=0)
#         #TODO Remember to remove
#         print(f'{restored_image.size*()}')
#         cv2.imshow('Restored Image', restored_image)

#     return restored_image

# # 读取模糊图像
# image = cv2.imread('/home/rico/Pictures/blurred.png', cv2.IMREAD_COLOR)

# # 转换为灰度图像
# gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# # 进行盲去卷积恢复
# kernel_size = 15  # 模糊核大小
# iterations = 10  # 迭代次数
# restored_image = blind_deconvolution(gray_image, kernel_size, iterations)

# # 显示模糊图像和恢复后的图像
# cv2.imshow('Blurred Image', gray_image)
# cv2.imshow('Restored Image', restored_image)
# cv2.waitKey(0)
# cv2.destroyAllWindows()

import cv2
import numpy as np


def motion_deblur(image, kernel_size, motion_angle):
    # 生成运动模糊核
    kernel = np.zeros((kernel_size, kernel_size))
    center = kernel_size // 2
    kernel[center, :] = 1.0 / kernel_size
    # 对模糊核进行旋转
    M = cv2.getRotationMatrix2D((center, center), -motion_angle, 1.0)
    kernel = cv2.warpAffine(kernel, M, (kernel_size, kernel_size))

    # 进行逆滤波
    restored_image = cv2.filter2D(image, -1, np.linalg.pinv(kernel))

    return restored_image


# 读取模糊图像
image = cv2.imread("/home/rico/Pictures/blurred.png", cv2.IMREAD_COLOR)

# 转换为灰度图像
gray_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# 进行边缘检测
edges = cv2.Canny(gray_image, 100, 200)

# 增强边缘信息
enhanced_edges = cv2.GaussianBlur(edges, (5, 5), 0)

# 进行逆运算恢复
restored_image = motion_deblur(enhanced_edges, 15, 45)

# 显示模糊图像、边缘图像和恢复后的图像
cv2.imshow("Blurred Image", gray_image)
cv2.imshow("Enhanced Edges", enhanced_edges)
cv2.imshow("Restored Image", restored_image)
cv2.waitKey(0)
# cv2.destroyAllWindows()
