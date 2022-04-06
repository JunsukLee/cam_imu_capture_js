
import numpy as np
import cv2

img_row = 688
img_col = 650
img = np.zeros((img_row,img_col,1), np.uint8)
img = cv2.circle(img, (int(img_col/2), int(img_row/2)), int(img_col/1.65), 255, -1, 8, 0)

cv2.imshow("mastkImage", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
cv2.imwrite("maskImage.jpg",img)

#Mat image = Mat::zeros(480, 640, CV_8UC1 );
#circle(image, cv::Point2f(320, 240), 320, Scalar(255), -1, 8, 0);
#imwrite("mask.jpg", image);
# imshow("circle", image);
# waitKey(0);
# return 0;
# }

