"""
Program to compose .gif with images.
Author: Ximu Zhang
Created Date: 04/23/2021
"""

import imageio
import os
import numpy as np


def compose_gif(img_path):
	g = os.walk(img_path)
	gif_images = []
	img_num = []
	for root, folder, files in g:
		for file in files:
			img_num.append(int(file.split('.')[0]))
		img_num = np.array(img_num)
		sort_index = np.argsort(img_num)

		for ind in sort_index:
			gif_images.append(imageio.imread('%s/%d.jpg' % (root, ind)))
		imageio.mimsave('./gif/a_star.gif', gif_images, fps=10)


if __name__ == '__main__':
	compose_gif('./gif/a_star')
