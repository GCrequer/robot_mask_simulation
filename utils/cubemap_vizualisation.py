import cv2
import numpy as np

front   = cv2.imread("1.png"); cv2.putText(front, "front", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 10)
back    = cv2.imread("2.png"); cv2.putText(back, "back", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 10)
top     = cv2.imread("3.png"); cv2.putText(top, "top", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 10)
bottom  = cv2.imread("4.png"); cv2.putText(bottom, "bottom", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 10)
left    = cv2.imread("5.png"); cv2.putText(left, "left", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 10)
right   = cv2.imread("6.png"); cv2.putText(right, "right", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 5, (255, 255, 255), 10)

size = front.shape[0]
cubemap = np.zeros((size*3, size*4, 3), dtype=np.uint8)


cubemap[size:2*size, size:2*size] = front
cubemap[size:2*size, 3*size:4*size] = back
cubemap[0:size, size:2*size] = top
cubemap[2*size:3*size, size:2*size] = bottom
cubemap[size:2*size, 0:size] = left
cubemap[size:2*size, 2*size:3*size] = right

cv2.imshow("cubemap", cubemap)
cv2.waitKey(0)
cv2.destroyAllWindows()

