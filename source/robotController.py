#!/usr/bin/env python
import rospy
import os
import json
import numpy
import math
from math import atan2, acos, cos, sin, sqrt, pi
import cv2
from geometry_msgs.msg import Twist
from ev3 import Ev3, Point
from nav_msgs.msg import Odometry
import sys, select, termios, tty


class RobotController():
        
    def crossProduct(self, vecA, vecB):
        return (vecA.x*vecB.y)-(vecA.y*vecB.x)
    
    def dotProduct(self, vecA, vecB):
        return (vecA.x*vecB.x)+(vecA.y*vecB.y)

    def getAlpha(self, ev3, goal):
        vectorEv3 = Point()
        vectorEv3.x = ev3.front.x - ev3.center.x
        vectorEv3.y = ev3.front.y - ev3.center.y
        vectorGoal = Point()
        vectorGoal.x = goal.x - ev3.center.x
        vectorGoal.y = goal.y - ev3.center.y
        #print(goal.x,goal.y, "goal pose")
        #print(ev3.front.x,ev3.front.y, "ev3 front")
        #print(ev3.center.x,ev3.center.y, "ev3 center")
        #print(vectorEv3.x,vectorEv3.y, "ev3")
        #print(vectorGoal.x,vectorGoal.y, "goal")
        return math.atan2(self.crossProduct(vectorEv3,vectorGoal), self.dotProduct(vectorEv3,vectorGoal))

    def getOrientation(self, graph, rx, ry, px, py):
        vector = Point()
        u = Point()
        rx = int(rx)
        ry = int(ry)
        px = int(px)
        py = int(py)
        vector.x = px - rx
        vector.y = py - ry
        u.x = 1
        u.y = 0
        costh = (vector.x * u.x) / sqrt(vector.x**2 + vector.y ** 2) * sqrt(u.x**2)
        angle = acos(costh)
        cv2.arrowedLine(graph, (rx,ry),(px,py),  (25,0,0), 3, cv2.LINE_AA)
        if(ry < py):
        	angle = pi + (pi - angle)
        return angle

    def initPid(self):
        self.lastAlpha = 0
        self.alphaSum = 0
    
    def pidRun(self, graph, goal, pose, ev3, lastFlag):
        # Inicializa publisher para comandar a velocidade no ROS
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1) 

        # Variaveis do controlador
        kP = 0.005
        kPa = 0.656
        kI = 0.001
        kD = 0.002 
        vMax = 0.36    

        # Erro permitido pelo controlador
        err = 30

        # Calcula distancia e diferenca do angulo entre o robo e o objetivo
        rho = math.sqrt((goal.x - pose.x)**2 + (goal.y - pose.y)**2)
        alpha = self.getAlpha(ev3, goal)

        #Print
        #print("Goals = [ %s x ] [ % s y ] " % (goal.x,goal.y))
        #print("Alpha = [ %s rad] ou [ %s graus ]" % (alpha, alpha*180/math.pi))

        # Verifica a condicao de parada
        if(abs(rho) < err):
            return True

        # Define velocidade linear
        if(lastFlag == True):
            linearVelocity = -min(kP*rho,vMax)
        else:
            linearVelocity = -vMax

        # Calcula velocidade angular
        self.alphaSum+=alpha
        angularVelocity = kPa*alpha + kI*self.alphaSum + kD*(alpha - self.lastAlpha)
        self.lastAlpha = alpha

        # Aplica velocidade no robo 
        print("PosRobo (%s,%s) | Objetivo (%s,%s) | Vel Linear = %s | Vel Angular = %s" 
        % (pose.x,pose.y,goal.x,goal.y,linearVelocity,angularVelocity))
        
        twist = Twist()
        twist.linear.x = linearVelocity 
        twist.linear.y = 0 
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = angularVelocity
        pub.publish(twist)
        return False
            








    def pdRun(self, graph, robotObjX, robotObjY, robotObjTheta, poseX, poseY, poseTheta):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)      
        kv = -0.001818
        kalpha = -0.3
        #kbetha = -1.6
        

        errorRho = 30
        errorTheta = 300
        deltaTheta = robotObjTheta - poseTheta
        rho = math.sqrt((robotObjX - poseX)**2 + (robotObjY - poseY)**2)
        
        #Printa na Tela
        #print("Rodando PD com Objetivo [ %s, %s, th: %s ] Pos Atual [ %s, %s, th: %s ] Erro: [ RHO: %s > %s e  Theta: %s > %s ] " %
        #(robotObjX,robotObjY, robotObjTheta, poseX, poseY, poseTheta, rho, errorRho, deltaTheta, errorTheta))
        if(abs(rho) > errorRho or abs(deltaTheta) > errorTheta):
            anguloGoal = self.getOrientation(graph, poseX, poseY, robotObjX, robotObjY)
            difAngulo = math.atan2((robotObjY-poseY),(robotObjX-poseX))
            
            alpha = poseTheta - anguloGoal
            beta = robotObjTheta-difAngulo
            twist = Twist()
            twist.linear.x = rho * kv 
            twist.linear.y = 0 
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = kalpha * alpha #+ kbetha * beta
            pub.publish(twist)
            print("Angulos Robo [ %s ] Goal [ %s ]" % (poseTheta, anguloGoal))
            print("Dif Alpha [ %s ]" % (alpha))
            #print("Kv = %s / %s"%(rho,kv))
            #print("Velocidades: Linear %s - Angular %s" % ((rho * kv),(kalpha * alpha + kbetha * beta)))
            return False
        else:
            return True

    def stopRobot(self):
        pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)      
        twist = Twist()
        twist.linear.x = 0
        twist.linear.y = 0 
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = 0
        pub.publish(twist)