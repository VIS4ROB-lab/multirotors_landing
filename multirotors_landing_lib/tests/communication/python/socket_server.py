#!/usr/bin/python

import cv2

import opencv_socket_comm


def func_show(decoded_img):
	# Processing here (e.g. pass through neural network)
	return cv2.cvtColor(decoded_img.astype('uint8'), cv2.COLOR_BGR2GRAY)


if __name__ == "__main__":
	# Create server
	socket_id=0
	opencv_socket_comm.TcpServer(func_show, socket_id, verbose=True)
	