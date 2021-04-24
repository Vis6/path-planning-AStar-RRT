"""
Program to compose .gif with images.
Author: Ximu Zhang
Created Date: 04/23/2021
"""

import imageio
import glob


def compose_gif(img_path, fps=10):
	file_paths = glob.glob(pathname='%s/*.jpg' % img_path)
	file_num = len(file_paths)

	gif_images = []
	for i in range(file_num):
		gif_images.append(imageio.imread('%s/%d.jpg' % (img_path, i)))
	imageio.mimsave('./gif/a_star.gif', gif_images, fps=fps)


if __name__ == '__main__':
	compose_gif(img_path='./gif/a_star', fps=50)
