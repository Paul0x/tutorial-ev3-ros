##############################################################################
#   Reconhecimento e Processamento de Imagens @ Trabalho de Robotica 2019.1
#   Biblioteca de Funcoes para Processamento de Imagens
##############################################################################
#   Author: Paulo Felipe - paulof (at) ufop.edu.br
from collections import deque
from imutils.video import VideoStream
from math import atan2, cos, sin, sqrt, pi
from pathPlanningUtils import Node, astar
import argparse
import numpy as np
import argparse
import cv2
import imutils
import time

class ImageProcessingUtils():
	# Retorna o frame da camera
	def getFrame(self, videoSource):
		frame = videoSource.read()
		frame = frame[1]
		return frame

	# Transforma o frame para HSV
	def frameToHsv(self, frame):
		blurred = cv2.GaussianBlur(frame, (11, 11), 0)
		hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
		return hsv

	# Aplica a mascara pre-definida
	def applyMaskToFrame(self, hsv, lowerBound, upperBound):
		mask = cv2.inRange(hsv, lowerBound, upperBound)
		mask = cv2.erode(mask, None, iterations=2)
		mask = cv2.dilate(mask, None, iterations=2)
		return mask

	# Retorna os contornos encontrados na imagem pos aplicacao da mascara
	def getObjectsContours(self, mask):
		cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,
		cv2.CHAIN_APPROX_SIMPLE)
		return imutils.grab_contours(cnts)
	
	# Detecta a cor de determinado pixel na tela da camera
	def cvEventHandler(self, event, x, y, flags, params):
		if event == cv2.EVENT_RBUTTONDOWN:
			hsv = cv2.cvtColor(self.frame, cv2.COLOR_BGR2HSV)
			color = hsv[y,x]
			print("Cor HSV nas coord. [ %s, %s ]" % (x,y))
			print(color)
		if event == cv2.EVENT_LBUTTONDOWN:
			self.getImageMap(self.graph,x, y, 16, 3)

	# Mostra a Imagem na Tela
	def printImage(self, frame, graph):
		cv2.namedWindow("Robotica")
		cv2.setMouseCallback("Robotica", self.cvEventHandler)
		cv2.imshow("Trajetoria", graph)
		self.frame = frame
		self.graph = graph
		cv2.imshow("Robotica", frame)
		return cv2.waitKey(1) & 0xFF

	# Adiciona o espaco de seguranca no mapa
	def checkRobotSafeRange(self, map, mapImg, x, y, safeScale):
		for i in range(x-1):
			for j in range(y-1):
				for k in range(1,safeScale):
					if(map[i][j] != 0):
						continue
					# Checa pos a direita
					if(k+i < x):
						if(mapImg[j][i+k][1] > 60 and mapImg[j][i+k][0] != 255 and mapImg[j][i+k][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa pos a esquerda
					if(k-1 > 0):
						if(mapImg[j][i-k][1] > 60 and mapImg[j][i-k][0] != 255 and mapImg[j][i-k][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa pos em cima
					if(j-k > 0):
						if(mapImg[j-k][i][1] > 60 and mapImg[j-k][i][0] != 255 and mapImg[j-k][i][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa pos embaixo
					if(j+k < y):
						if(mapImg[j+k][i][1] > 60 and mapImg[j+k][i][0] != 255 and mapImg[j+k][i][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal em cima direita
					if(k+i < x and j-k > 0):
						if(mapImg[j-k][i+k][1] > 60 and mapImg[j-k][i+k][0] != 255 and mapImg[j-k][i+k][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal em cima esquerda
					if(k-1 > 0 and j-k > 0):
						if(mapImg[j-k][i-k][1] > 60 and mapImg[j-k][i-k][0] != 255 and mapImg[j-k][i-k][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal embaixo direita
					if(k+i < x and j+k < y):
						if(mapImg[j+k][i+k][1] > 60 and mapImg[j+k][i+k][0] != 255 and mapImg[j+k][i+k][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					# Checa na diagonal embaixo equerda
					if(k-i < x and j+k < y):
						if(mapImg[j+k][i-k][1] > 60 and mapImg[j+k][i-k][0] != 255 and mapImg[j+k][i-k][2] < 60):
							map[i,j] = safeScale-k
							cv2.line(mapImg,(i,j), (i,j),(0,10,0),1)
					
		return map, mapImg

	# Gera o mapa utilizado pelo pathplanning
	def getImageMap(self, img, goalX, goalY, scale, safeScale):
		x = 640/scale
		y = 480/scale
		goalX = int(goalX/scale)
		goalY = int(goalY/scale)
		map = np.zeros((x,y))
		mapImg = img.copy()
		mapImg = cv2.resize(mapImg,(x, y))
		for i in range(x-1):
			for j in range(y-1):
				if(mapImg[j][i][1] > 60 and mapImg[j][i][0] != 255):
					map[i,j] = safeScale+1
		map, mapImg = self.checkRobotSafeRange(map, mapImg, x, y, safeScale)
		if(map[goalX, goalY] != 0):
			print("Caminho obstruido")
		else:
			cv2.imshow("Mapeamento", mapImg)
			path = astar(map, (int(self.ev3.front.x/scale),int(self.ev3.front.y/scale)), (goalX,goalY), 1200)
			for coord in path:
				cv2.line(mapImg,coord,coord,(255,0,0),1)
			
			#print(path)
			self.trajetoria = True
			self.path = path
			self.goal = (goalX, goalY)
			self.mapScale = scale
			self.updateTrajetoria = True
			mapImg = cv2.resize(mapImg,(640, 480))
			cv2.imshow("Mapeamento", mapImg)
		return 1

	# Reconhecimento dos pontos do objeto
	def recognizeObject(self, contours, frame, objLabel):
		if len(contours) > 0:
			c = max(contours, key=cv2.contourArea)
			((x, y), radius) = cv2.minEnclosingCircle(c)
			M = cv2.moments(c)
			center = (int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"]))
			if radius > 4:
				#cv2.circle(frame, (int(x), int(y)), 10, (0, 0, 250), -1)
				return (x,y)
			return (-1, -1)
		return (-1, -1)

	# Imprime o contorno dos objetos
	def printWalls(self, frame, graph, contours):
		for c in contours:
			if(cv2.contourArea(c) < 1e3):
				continue
			ratio = 1
			# compute the center of the contour, then detect the name of the
			# shape using only the contour
			M = cv2.moments(c)
			cX = int((M["m10"] / M["m00"]) * ratio)
			cY = int((M["m01"] / M["m00"]) * ratio)

			# multiply the contour (x, y)-coordinates by the resize ratio,
			# then draw the contours and the name of the shape on the image
			c = c.astype("float")
			c *= ratio
			c = c.astype("int")
			#cv2.drawContours(frame, [c], -1, (0, 255, 0), 2)
			cv2.drawContours(graph, [c], -1, (0, 255, 0), 2, cv2.FILLED)
			cv2.fillPoly(graph, pts =[c], color=(0,255,0))
			#self.getOrientation(c, frame)

	# Nao faz nada
	def getOrientationOld(self, contour, frame):
		sz = len(contour)
		data_contour = np.empty((sz, 2), dtype=np.float64)
		for i in range(data_contour.shape[0]):
			data_contour[i,0] = contour[i,0,0]
			data_contour[i,1] = contour[i,0,1]
		# Perform PCA analysis
		mean = np.empty((0))
		mean, eigenvectors, eigenvalues = cv2.PCACompute2(data_contour, mean)
		# Store the center of the object
		cntr = (int(mean[0,0]), int(mean[0,1]))
		
		
		cv2.circle(frame, cntr, 3, (255, 0, 255), 2)
		p1 = (cntr[0] + 0.02 * eigenvectors[0,0] * eigenvalues[0,0], cntr[1] + 0.02 * eigenvectors[0,1] * eigenvalues[0,0])
		p2 = (cntr[0] - 0.02 * eigenvectors[1,0] * eigenvalues[1,0], cntr[1] - 0.02 * eigenvectors[1,1] * eigenvalues[1,0])
		drawAxis(frame, cntr, p1, (0, 255, 0), 1)
		drawAxis(frame, cntr, p2, (255, 255, 0), 5)
		angle = atan2(eigenvectors[0,1], eigenvectors[0,0]) # orientation in radians
		if(angle>0):
			radians = angle
		else:
			radians = 2*pi + angle
		return radians

	def printRobot(self, ev3, frame, graph, scale):
		cv2.circle(graph, (int(ev3.center.x), int(ev3.center.y)), 6*scale, (0, 0, 250), -1)
		cv2.arrowedLine(graph, (int(ev3.center.x), int(ev3.center.y)), (int(ev3.front.x), int(ev3.front.y)), (255,0,0), 3, cv2.LINE_AA)
		cv2.arrowedLine(frame, (int(ev3.center.x), int(ev3.center.y)), (int(ev3.front.x), int(ev3.front.y)), (255,0,0), 3, cv2.LINE_AA)
		
	def drawGrid(self, img, line_color=(0, 0, 0), thickness=1, type_=cv2.LINE_AA, pxstep=8):
		x = pxstep
		y = pxstep
		while x < img.shape[1]:
			cv2.line(img, (x, 0), (x, img.shape[0]), color=line_color, lineType=type_, thickness=thickness)
			x += pxstep
		while y < img.shape[0]:
			cv2.line(img, (0, y), (img.shape[1], y), color=line_color, lineType=type_, thickness=thickness)
			y += pxstep
	
	def init(self):
		self.trajetoria = False
		self.updateTrajetoria = False