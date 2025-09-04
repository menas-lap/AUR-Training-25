import numpy as np
import matplotlib.pyplot as plt
import cv2

def convolve(image, kernel):
    """
    Apply a convolution to an image using a given kernel.

    Your code should handle different kernel sizes - not necessarily 3x3 kernels
    """

    # Check if kernel has odd dimensions
    if kernel.shape[0] % 2 != 1 or kernel.shape[1] % 2 != 1:
        raise ValueError("Kernel must have odd number of rows and columns.")

    # Flip the kernel horizontally then vertically (180 degree rotation)
    kernel = np.flip(np.flip(kernel, 0), 1)

    # Pad the image
    pad_height = kernel.shape[0] // 2
    pad_width = kernel.shape[1] // 2
    padded_image = np.pad(image, ((pad_height, pad_height), (pad_width, pad_width)), mode='constant', constant_values=0)

    # Create output image
    output = np.zeros_like(image)

    # Perform convolution using sliding window method with nested loops (no dot product)
    for i in range(image.shape[0]):
        for j in range(image.shape[1]):
            sum_val = 0.0
            for m in range(kernel.shape[0]):
                for n in range(kernel.shape[1]):
                    sum_val += padded_image[i + m, j + n] * kernel[m, n]
            output[i, j] = sum_val

    return output


img = cv2.imread('image.jpg', cv2.IMREAD_GRAYSCALE)
fig, axes = plt.subplots(2, 2, figsize=(8, 8))

axes[0, 0].imshow(img, cmap='gray')
axes[0, 0].set_title('Original Image')
axes[0, 0].axis('off')

axes[0, 1].imshow(convolve(img, np.ones((5, 5)) / 25), cmap='gray')
axes[0, 1].set_title('Box Filter')
axes[0, 1].axis('off')

axes[1, 0].imshow(convolve(img, np.array([[-1, 0, 1], [-2, 0, 2], [-1, 0, 1]])), cmap='gray')
axes[1, 0].set_title('Horizontal Sobel Filter')
axes[1, 0].axis('off')

axes[1, 1].imshow(convolve(img, np.array([[-1, -2, -1], [0, 0, 0], [1, 2, 1]])), cmap='gray')
axes[1, 1].set_title('Vertical Sobel Filter')
axes[1, 1].axis('off')
plt.show()