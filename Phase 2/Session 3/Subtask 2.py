import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import cv2

matplotlib.use('Agg')

img = cv2.imread('shapes.jpg')
out = img.copy()

blue_lower = np.array([100, 0, 0])
blue_upper = np.array([255, 100, 100])
blue_mask = cv2.inRange(img, blue_lower, blue_upper)

red_lower = np.array([0, 0, 100])
red_upper = np.array([100, 100, 255])
red_mask = cv2.inRange(img, red_lower, red_upper)

black_lower = np.array([0, 0, 0])
black_upper = np.array([50, 50, 50])
black_mask = cv2.inRange(img, black_lower, black_upper)

out[blue_mask > 0] = [0, 0, 0]

out[red_mask > 0] = [255, 0, 0]

out[black_mask > 0] = [0, 0, 255]

img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
out_rgb = cv2.cvtColor(out, cv2.COLOR_BGR2RGB)

fig, axes = plt.subplots(1, 2)
axes[0].imshow(img_rgb)
axes[0].set_title('Original Image')
axes[0].axis('off')

axes[1].imshow(out_rgb)
axes[1].set_title('Processed Image')
axes[1].axis('off')

plt.savefig('processed_image.png')