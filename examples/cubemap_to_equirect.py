import cupy as cp
import cv2
from Tiles2EquiCU import *

# Load cubemap images
front = cv2.imread("1.png")
back = cv2.imread("2.png")
top = cv2.imread("3.png")
bottom = cv2.imread("4.png")
left = cv2.imread("5.png")
right = cv2.imread("6.png")

# Create equirectangular image
dims = 128

# Convert to cupy arrays
front_gpu = cp.asarray(front)
back_gpu = cp.asarray(back)
top_gpu = cp.asarray(top)
bottom_gpu = cp.asarray(bottom)
left_gpu = cp.asarray(left)
right_gpu = cp.asarray(right)

# Create equirectangular image
equi = cube2equi_cuda(front_gpu, back_gpu, top_gpu, bottom_gpu, left_gpu, right_gpu, dims)

cv2.imshow("equirectangular", cp.asnumpy(equi))
