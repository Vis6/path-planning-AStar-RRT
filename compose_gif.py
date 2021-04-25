"""
Program to compose .gif with images.
Author: Ximu Zhang
Created Date: 04/23/2021
"""

import imageio
import glob


def compose_gif(img_path, duration=0.1):
	gif_name = img_path.split('/')[-1]
	file_paths = glob.glob(pathname='%s/*.jpg' % img_path)
	file_num = len(file_paths)
	frames = []
	for i in range(file_num):
		if i % 10 == 0:  # read for every 10 frames
			frames.append(imageio.imread('%s/%d.jpg' % (img_path, i)))
		if i == file_num - 1:  # read the last frame
			frames.append(imageio.imread('%s/%d.jpg' % (img_path, i)))
	duration_list = [duration] * len(frames)
	duration_list[-1] = 3.0
	imageio.mimsave('./gif/%s.gif' % gif_name, frames, 'GIF', duration=duration_list)


if __name__ == '__main__':
	make_a_star_gif = True
	make_rrt_gif = True

	if make_a_star_gif:
		compose_gif(img_path='./gif/a_star')
	if make_rrt_gif:
		compose_gif(img_path='./gif/rrt')
