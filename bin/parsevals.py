#!/usr/bin/python

import sys
import numpy as np
import matplotlib.pyplot as plt

case = sys.argv[1]

''' encoder feedback '''
if (case == 'encoder'):
	f = open('enc_vals.txt')
	t = np.linspace(0,40,4000)
	leftEncFB = np.zeros(len(t))
	leftEncMed  = np.zeros(len(t))
	leftEncLP = np.zeros(len(t))
	leftErr = np.zeros(len(t))
	rightEncFB = np.zeros(len(t))
	rightEncMed  = np.zeros(len(t))
	rightEncLP = np.zeros(len(t))
	rightErr = np.zeros(len(t))
	motorRef = np.zeros(len(t))

	cnt = 0
	for line in f:
	    encoders = line.split(',')
	    if cnt < 4000:
	    	leftEncFB[cnt] = encoders[0]
	    	leftEncMed[cnt]  = encoders[1]
	    	leftEncLP[cnt]   = encoders[2]
	    	leftErr[cnt] = encoders[3]
	    	rightEncFB[cnt] = encoders[4]
	    	rightEncMed[cnt] = encoders[5]
	    	rightEncLP[cnt]  = encoders[6]
	    	rightErr[cnt] = encoders[7]
	    	motorRef[cnt] = encoders[8]	    	
	    cnt += 1

	fig, (ax1, ax2) = plt.subplots(2, 1, sharey=True)
	fb, = ax1.plot(t, leftEncFB, 'k-', linewidth = 1, label = 'Encoder Feedback')
	#ax1.plot(t, leftErr, 'r-', linewidth = 1)
	med, = ax1.plot(t, leftEncMed, 'r-', linewidth=1)
	low, = ax1.plot(t, leftEncLP, 'g-', linewidth=1)
	rs, = ax1.plot(t, motorRef, 'b', linewidth = 1, label = 'Reference State')

	ax2.plot(t, rightEncFB, 'k-', linewidth = 1)
	#ax2.plot(t, rightErr, 'r-', linewidth = 1)
	#ax2.plot(t, rightEncMed, 'r-', linewidth=1)
	#ax2.plot(t, rightEncLP, 'g-', linewidth=1)
	ax2.plot(t, motorRef, 'b', linewidth = 1)

	plt.legend(prop= {'size':30}, handles=[rs, fb])
	plt.xlabel('Time (s)', fontsize = 35)
	plt.ylabel('Velocity (m/s)', fontsize = 35)
	plt.suptitle('Motor PID Step Response', fontsize = 45)

	plt.show()


''' body velo feedback '''
if (case == 'velo'):
	f = open('headingstep.txt')
	t = np.linspace(0,40,4000)
	veloREF = np.zeros(len(t))
	veloFB = np.zeros(len(t))

	cnt = 0
	for line in f:
	    velos = line.split(',')
	    if cnt < 4000:
	    	veloREF[cnt] = velos[0]
	    	veloFB[cnt]  = velos[1] 
	    	veloFB[cnt] = veloFB[cnt] * -1.0
	    cnt += 1

	ref, = plt.plot(t, veloREF, 'r-', linewidth=1, label= 'Reference State')
	fb, = plt.plot(t, veloFB, 'k-', linewidth=1, label = 'Heading Feedback')
	plt.legend(prop= {'size':30}, handles=[ref, fb], loc =2)
	plt.xlabel('Time (s)', fontsize = 35)
	plt.ylabel('Heading (rad)', fontsize = 35)
	plt.title('Heading PID Step Response', fontsize = 45)
	plt.show()


''' body angle feedback '''
if (case == 'angle'):
	f = open('angle_vals.txt')
	t = np.linspace(0,40,400)	
	angleREF = np.zeros(len(t))
	angleFB = np.zeros(len(t))

	cnt = 0
	for line in f:
		angles = line.split(',')
		if cnt < 4000:
			angleREF[cnt] = angles[0]
			angleFB[cnt] = angles[1]
		cnt += 1

	plt.plot(t, angleREF, 'r-', linewidth=1)
	plt.plot(t, angleFB, 'g-', linewidth=1)
	plt.show()


''' velo pid feedback '''
if (case == 'velo_pid'):
	f = open('velo_pid_vals.txt')
	t = np.linspace(0,40,4000)	
	pterm = np.zeros(len(t))
	iterm = np.zeros(len(t))
	dterm = np.zeros(len(t))
	err = np.zeros(len(t))

	cnt = 0
	for line in f:
		pids = line.split(',')
		if cnt < 4000:
			pterm[cnt] = pids[0]
			iterm[cnt] = pids[1]
			dterm[cnt] = pids[2]
			err[cnt] = pids[3]
		cnt += 1

	plt.plot(t, pterm, 'r-', linewidth=1)
	plt.plot(t, iterm, 'g-', linewidth=1)
	plt.plot(t, dterm, 'b-', linewidth=1)
	plt.plot(t, err, 'k-', linewidth=1)
	plt.show()


''' angle pid feedback '''
if (case == 'angle_pid'):
	f = open('angle_pid_vals.txt')
	t = np.linspace(0,40,4000)	
	pterm = np.zeros(len(t))
	iterm = np.zeros(len(t))
	dterm = np.zeros(len(t))

	cnt = 0
	for line in f:
		pids = line.split(',')
		if cnt < 4000:
			pterm[cnt] = pids[0]
			iterm[cnt] = pids[1]
			dterm[cnt] = pids[2]
		cnt += 1

	plt.plot(t, pterm, 'r-', linewidth=1)
	plt.plot(t, iterm, 'g-', linewidth=1)
	plt.plot(t, dterm, 'b-', linewidth=1)
	plt.show()

''' pos pid feedback '''
if (case == 'pos_pid'):
	f = open('pos_pid_vals.txt')
	t = np.linspace(0,40,4000)	
	pterm = np.zeros(len(t))
	iterm = np.zeros(len(t))
	dterm = np.zeros(len(t))
	err = np.zeros(len(t))

	cnt = 0
	for line in f:
		pids = line.split(',')
		if cnt < 4000:
			pterm[cnt] = pids[0]
			iterm[cnt] = pids[1]
			dterm[cnt] = pids[2]
			err[cnt] = pids[3]
		cnt += 1

	plt.plot(t, pterm, 'r-', linewidth=1)
	plt.plot(t, iterm, 'g-', linewidth=1)
	plt.plot(t, dterm, 'b-', linewidth=1)
	plt.plot(t, err, 'k-', linewidth=1)
	plt.show()	

''' head pid feedback '''
if (case == 'head_pid'):
	f = open('head_pid_vals.txt')
	t = np.linspace(0,40,4000)	
	pterm = np.zeros(len(t))
	iterm = np.zeros(len(t))
	dterm = np.zeros(len(t))
	err = np.zeros(len(t))

	cnt = 0
	for line in f:
		pids = line.split(',')
		if cnt < 4000:
			pterm[cnt] = pids[0]
			iterm[cnt] = pids[1]
			dterm[cnt] = pids[2]
			err[cnt] = pids[3]
		cnt += 1

	plt.plot(t, pterm, 'r-', linewidth=1)
	plt.plot(t, iterm, 'g-', linewidth=1)
	plt.plot(t, dterm, 'b-', linewidth=1)
	plt.plot(t, err, 'k-', linewidth=1)
	plt.show()	

	''' theta feedback '''
if (case == 'theta_fb'):
	f = open('dr_theta.txt')
	t = np.linspace(0,40,4000)	
	drval = np.zeros(len(t))
	imuval = np.zeros(len(t))
	cnt = 0
	for line in f:
		cur = line.split(',')
		if cnt < 4000:
			drval[cnt] = cur[0]
			imuval[cnt] = cur[1]
		cnt += 1

	drlab, = plt.plot(t, drval, 'k-', linewidth=2, label = "Dead Reckoning")
	imulab, = plt.plot(t, imuval, 'r-', linewidth=2, label = "IMU")
	plt.legend(prop= {'size':30}, handles=[drlab, imulab])
	plt.xlabel('Time (s)', fontsize = 35)
	plt.ylabel('Angular Velocity (rad/s)', fontsize = 35)
	plt.title('Angular Velocity Comparison of IMU and Dead Reckoning Feedback', fontsize = 45)

	plt.show()