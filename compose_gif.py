"""
Program to compose .gif with images.
Author: Ximu Zhang
Created Date: 04/23/2021
"""

import imageio
import glob


def compose_gif(img_path, duration=1.0):
	gif_name = img_path.split('/')[-1]
	file_paths = glob.glob(pathname='%s/*.jpg' % img_path)
	file_num = len(file_paths)

	frames = []
	for i in range(file_num):
		frames.append(imageio.imread('%s/%d.jpg' % (img_path, i)))
	imageio.mimsave('./gif/%s.gif' % gif_name, frames, 'GIF', duration=duration)


if __name__ == '__main__':
	make_a_star_gif = True
	make_rrt_gif = True

	if make_a_star_gif:
		compose_gif(img_path='./gif/a_star', duration=0.1)
	if make_rrt_gif:
		compose_gif(img_path='./gif/rrt', duration=0.1)
