#!/usr/bin/env python3
#  -*- coding: utf-8 -*-
# from __future__ import print_function
import math
import rospy
from std_msgs.msg import Empty, Float32MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from tf2_msgs.msg import TFMessage
import numpy as np
import time
from matplotlib import pyplot as plt
from nav_msgs.msg import Odometry

# Variables globales
xActualLider = 0
yActualLider = 0
zActualLider = 0

yawActualLider = 0

estadoGlobalLider = 0
estadoGlobalSeguidor = 0

zVelAngSeguidor = 0

# Tolerancias de error lider
tolZLider = 0.025
tolPosLider = 0.05

# Tolerancias de error seguidor
tolZSeguidor = 0.025
tolPosSeguidor = 0.05

# Ganancias lider
Kp_linealXLider = 1
Kp_linealYLider = 1
Kp_linealZLider = 1

Kp_angularZLider = 1

# Ganancias Seguidor
Kp_linealXSeguidor = 1.55
Kp_linealYSeguidor = 1.55
Kp_linealZSeguidor = 2

Kp_angularZSeguidor = 3

# Variable para imprimir solo una vez
printOnce = True

# Tiempo
inicioTiempo = 0.0

# Distancia entre dornes, (-) -> derecha, (+) -> izquierda
distEntreDrones = -1.3

# Arreglo para publicar la treyectoria a seguir
graficaTrayectoria = Float32MultiArray()

# ===========================================
# 			Control para dron Lider
# ===========================================
def controlPosicionLider(data):
	global xActualLider, yActualLider, zActualLider
	global estadoGlobalLider, printOnce
	global inicioTiempo

	velLider_msg = Twist()

	xActualLider = float(data.pose.pose.position.x)
	yActualLider = float(data.pose.pose.position.y)
	zActualLider = float(data.pose.pose.position.z)

	rotXyLider= rotarMarco(-yawActualLider, xActualLider, yActualLider)

	# -------------------------------------------------------------------------------
	# 		Despegando a 1m despues de que el dron seguidor esta a la derecha
	# -------------------------------------------------------------------------------
	if( estadoGlobalLider == 0 and estadoGlobalSeguidor == 3 ):
		if( printOnce == True ):
			print('\n---------------------------------------------')
			print('Despegando')
			printOnce = False

		ezLider = 1 - zActualLider

		if( abs(ezLider) < tolZLider ):
			print('Llegó Lider, e:', ezLider)
			velLider_msg.linear.z = 0
			estadoGlobalLider = 1
			printOnce = True
		else:
			velLider_msg.linear.z = ezLider*Kp_linealZLider
			velLider_pub.publish(velLider_msg)

	# -------------------------------------------------------------------------------
	# 					Posicionar el dron en un punto deseado
	# -------------------------------------------------------------------------------
	if( estadoGlobalLider == 1 ):
		if( printOnce == True ):
			print('\n---------------------------------------------')
			print('Siguiendo trayectoria 1 Punto')
			printOnce = False

		# Control de posicion del lider
		puntoASeguir = [4, 0]
		rotXyPunto = rotarMarco(-yawActualLider, puntoASeguir[0], puntoASeguir[1])

		exL = rotXyLider[0] - rotXyPunto[0]
		eyL = rotXyLider[1] - rotXyPunto[1]

		velLider_msg.linear.x = -exL*Kp_linealXLider
		velLider_msg.linear.y = -eyL*Kp_linealYLider

		# Control de orientacion del lider
		LemnRotTra = rotarMarco(-yawActualLider, puntoASeguir[0]-xActualLider, puntoASeguir[1]-yActualLider)
		eYawLider = math.atan2(LemnRotTra[1], LemnRotTra[0])
		velLider_msg.angular.z = eYawLider*Kp_angularZLider

		if math.sqrt(exL**2+eyL**2) <= tolPosLider:
			estadoGlobalLider = 2
			printOnce = True

	# -------------------------------------------------------------------------------
	# 						Seguimiento de trayectoria Lemniscata
	# -------------------------------------------------------------------------------
	if( estadoGlobalLider == 2 ):
		if( printOnce == True ):
			print('\n---------------------------------------------')
			print('Siguiendo trayectoria 2 - Lemniscata')
			inicioTiempo = time.time()
			printOnce = False

		tActual = time.time() - inicioTiempo

		# Trayectoria Lemniscata
		TLemn = 30
		fLemn = 1/TLemn
		wLemn = 2*math.pi*fLemn
		aLemn = 4
		hxLemn = 0
		hyLemn = 0

		xLemn = hxLemn + aLemn * np.cos(wLemn*tActual) / (1 + np.sin(wLemn*tActual)**2)
		yLemn = hyLemn + aLemn * np.sin(wLemn*tActual) * np.cos(wLemn*tActual) / (1 + np.sin(wLemn*tActual)**2)

		# Control de posicion del lider
		rotXyTrayectoria = rotarMarco(-yawActualLider, xLemn, yLemn)

		exL = rotXyLider[0] - rotXyTrayectoria[0]
		eyL = rotXyLider[1] - rotXyTrayectoria[1]

		velLider_msg.linear.x = -exL*Kp_linealXLider
		velLider_msg.linear.y = -eyL*Kp_linealYLider

		# Control de orientacion del lider
		LemnRotTra = rotarMarco(-yawActualLider, xLemn-xActualLider, yLemn-yActualLider)
		eYawLider = math.atan2(LemnRotTra[1], LemnRotTra[0])

		velLider_msg.angular.z = eYawLider*Kp_angularZLider

		# Movimiento senoidal de arriba a abajo
		velLider_msg.linear.z = math.sin(tActual)

		graficaTrayectoria.data = [xLemn, yLemn]
		trayectoria_pub.publish(graficaTrayectoria)

		if tActual >= TLemn + TLemn/4:
			print('Fin trayectoria Lemniscata')
			estadoGlobalLider = 3
			printOnce = True

	# -------------------------------------------------------------------------------
	# 						Seguimiento de trayectoria Espiral
	# -------------------------------------------------------------------------------
	if( estadoGlobalLider == 3 ):
		if( printOnce == True ):
			print('\n---------------------------------------------')
			print('Siguiendo trayectoria 3 - Espiral')
			inicioTiempo = time.time()
			printOnce = False

		tActual = time.time() - inicioTiempo

		# Trayectoria Espiral
		# https://www.geogebra.org/m/zsHgCvq7
		TEspiral = 10
		fEspiral = 1/TEspiral
		wEspiral = 2*math.pi*fEspiral
		aEspiral = 0.23
		bEspiral = 0.2
		hxEspiral = 0
		hyEspiral = 0

		xEspiral = hxEspiral + aEspiral*math.exp(bEspiral*wEspiral*tActual)*math.cos(wEspiral*tActual)
		yEspiral = hyEspiral + aEspiral*math.exp(bEspiral*wEspiral*tActual)*math.sin(wEspiral*tActual)

		# Control de posicion del lider
		rotXyPunto = rotarMarco(-yawActualLider, xEspiral, yEspiral)

		exL = rotXyLider[0] - rotXyPunto[0]
		eyL = rotXyLider[1] - rotXyPunto[1]

		velLider_msg.linear.x = -exL*Kp_linealXLider
		velLider_msg.linear.y = -eyL*Kp_linealYLider

		# Control de orientacion del lider
		EspiralRotTra = rotarMarco(-yawActualLider, xEspiral-xActualLider, yEspiral-yActualLider)
		eYawLider = math.atan2(EspiralRotTra[1], EspiralRotTra[0])

		velLider_msg.angular.z = eYawLider*Kp_angularZLider
		velLider_msg.linear.z = 0.1

		graficaTrayectoria.data = [xEspiral, yEspiral]
		trayectoria_pub.publish(graficaTrayectoria)

		if tActual >= 2*TEspiral + TEspiral/4:
			print('Fin trayectoria Espiral')
			estadoGlobalLider = 4
			printOnce = True

	# -------------------------------------------------------------------------------
	# 						Aterrizaje senoidal
	# -------------------------------------------------------------------------------
	if( estadoGlobalLider == 4 ):
		if( printOnce == True ):
			print('\n---------------------------------------------')
			print('Siguiendo trayectoria 4 - Aterrizaje senoidal')
			inicioTiempo = time.time()
			printOnce = False

		tActual = time.time() - inicioTiempo

		if( zActualLider <= 0.3 ):
			print('\n---------------------------------------------')
			print('Final de recorrido!')
			velLider_msg.linear.z = 0
			estadoGlobalLider = 5
			printOnce = True
		else:
			velLider_msg.linear.z = -0.2
			velLider_msg.linear.x = math.sin(tActual)
			velLider_pub.publish(velLider_msg)

	velLider_pub.publish(velLider_msg)

# ===========================================
# 			Control para dron Seguidor
# ===========================================
def controlPosicionSeguidor(data):
	global estadoGlobalSeguidor

	velSeguidor_msg = Twist()

	xActualSeguidor = float(data.pose.pose.position.x)
	yActualSeguidor = float(data.pose.pose.position.y)
	zActualSeguidor = float(data.pose.pose.position.z)

	# -------------------------------------------------------------------------------
	# 			Rutina para pasar al dron seguidor a la derecha del lider
	# -------------------------------------------------------------------------------

	# Ponerse un metro arriba del seguidor
	if( estadoGlobalSeguidor == 0 ):
		ezSeguidor = zActualLider + 1 - zActualSeguidor
		if( abs(ezSeguidor) < tolZSeguidor ):
			velSeguidor_msg.linear.z = 0
			estadoGlobalSeguidor = 1
		else:
			velSeguidor_msg.linear.z = ezSeguidor*Kp_linealZSeguidor

	# Moverse a un lado deseado del dron lider
	if( estadoGlobalSeguidor == 1 ):
		# Posicion del seguidor
		rotXySeguidor= rotarMarco(-yawActualLider, xActualSeguidor, yActualSeguidor)
		rotXyLider = rotarMarco(-yawActualLider, xActualLider, yActualLider)

		exL = rotXyLider[0] - rotXySeguidor[0]
		eyL = distEntreDrones + rotXyLider[1] - rotXySeguidor[1]

		velSeguidor_msg.linear.x = exL*0.45
		velSeguidor_msg.linear.y = eyL*0.45

		if math.sqrt(exL**2+eyL**2) <= tolPosSeguidor:
			estadoGlobalSeguidor = 2

		# Orientacion del seguidor
		velSeguidor_msg.angular.z = zVelAngSeguidor

	# Posicionarlo a la misma altura que el lider
	if( estadoGlobalSeguidor == 2 ):
		ezSeguidor = zActualLider - zActualSeguidor
		if( abs(ezSeguidor) < tolZSeguidor ):
			velSeguidor_msg.linear.z = 0
			estadoGlobalSeguidor = 3
		else:
			velSeguidor_msg.linear.z = ezSeguidor*Kp_linealZSeguidor

	# -------------------------------------------------------------------------------
	# 			Control general para el seguimiento del dron lider
	# -------------------------------------------------------------------------------

	if( estadoGlobalSeguidor == 3 ):
		# Control de la altura
		ezSeguidor = zActualLider - zActualSeguidor

		if( abs(ezSeguidor) < tolZSeguidor ):
			velSeguidor_msg.linear.z = 0
		else:
			velSeguidor_msg.linear.z = ezSeguidor*Kp_linealZSeguidor

		# Posicion del seguidor
		rotXySeguidor= rotarMarco(-yawActualLider, xActualSeguidor, yActualSeguidor)
		rotXyLider = rotarMarco(-yawActualLider, xActualLider, yActualLider)

		exL = rotXyLider[0] - rotXySeguidor[0]
		eyL = distEntreDrones + rotXyLider[1] - rotXySeguidor[1]

		if estadoGlobalLider != 0:
			velSeguidor_msg.linear.x = exL*Kp_linealXSeguidor
			velSeguidor_msg.linear.y = eyL*Kp_linealYSeguidor

		# Orientacion del seguidor
		velSeguidor_msg.angular.z = zVelAngSeguidor

	velSeguidor_pub.publish(velSeguidor_msg)


# ===========================================
# 	Obtencion de orientacion dron Lider
# ===========================================
def controlOrientaLider(data):
	global yawActualLider
	yawActualLider = quaterionToRads(data)

# ===========================================
# 	Control de orientación dron Seguidor
# ===========================================
def controlOrientaSeguidor(data):
	global zVelAngSeguidor

	yawActualSeguidor = quaterionToRads(data)

	# Para calculo de angulo entre los dos vectores, entre el lider  y el seguidor
	Va = np.array([math.cos(yawActualSeguidor), math.sin(yawActualSeguidor)])
	Vb = np.array([math.cos(yawActualLider), math.sin(yawActualLider)])

	productoCruz = np.cross(Va, Vb)
	productoPunto = np.dot(Va, Vb)
	eYawSeguidor = math.atan2(productoCruz, productoPunto)

	zVelAngSeguidor = (eYawSeguidor)*Kp_angularZSeguidor

# ===========================================
# 		Func rot marco de referencia
# ===========================================
def rotarMarco(angulRot, x, y):
	Rz = np.array([[math.cos(angulRot), -math.sin(angulRot)],
				[math.sin(angulRot), math.cos(angulRot)]])
	return np.matmul( Rz, np.array([[x], [y]]))

# ===========================================
# 	Func convertir de quaternion a radianes
# ===========================================
def quaterionToRads(data):
	x = data.pose.orientation.x
	y = data.pose.orientation.y
	z = data.pose.orientation.z
	w = data.pose.orientation.w

	t3 = 2.0 * (w * z + x * y)
	t4 = 1.0 - 2.0 * (y * y + z * z)
	yawZActual = math.atan2(t3, t4)
	if yawZActual < 0:
		yawZActual = 2*math.pi + yawZActual

	return yawZActual

# ===============================================================================
# 									PRINCIPAL
# ===============================================================================
if __name__ == '__main__':
	try:
		rospy.init_node('formation_flying_controller', anonymous=True)

		# Lider
		posicionLider_sub = rospy.Subscriber("/uav1/ground_truth/state", Odometry , controlPosicionLider)
		orientaLider_sub = rospy.Subscriber("/uav1/ground_truth_to_tf/pose", PoseStamped , controlOrientaLider)
		velLider_pub = rospy.Publisher("/uav1/cmd_vel", Twist , queue_size=1)

		# Seguidor
		posicionSeguidor_sub = rospy.Subscriber("/uav2/ground_truth/state", Odometry , controlPosicionSeguidor)
		orientaSeguidor_sub = rospy.Subscriber("/uav2/ground_truth_to_tf/pose", PoseStamped , controlOrientaSeguidor)
		velSeguidor_pub = rospy.Publisher("/uav2/cmd_vel", Twist , queue_size=1)

		# trayectoria
		trayectoria_pub = rospy.Publisher("/trayectory", Float32MultiArray, queue_size=1)

		rospy.spin()
	except rospy.ROSInterruptException:
		pass
