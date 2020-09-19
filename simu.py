## Calculate and draw the trajectory of the hand form initial position (Xs, Ys) to target positon (Xe, Ye) by numerical simulation
import tinyik as ti
import math as m
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm
import control
from control.matlab import *
import sympy as sp
from mpmath import *
import mpmath as mp


def get_angles(x, y):
    arm.ee = [x-l0, y, 0.]
    a1 = arm.angles[0]
    a2 = arm.angles[1]
    a3 = arm.angles[2]
    return [a1, a2, a3]

def get_position(a1, a2, a3):
    arm.angles = [a1, a2, a3]
    x = arm.ee[0] + l0
    y = arm.ee[1]
    return [x, y]

####Dimensions of average male human being: 
# l0 = 300 // l1 = 302 // l2 = 269 // l3 = 100
# joint angle range: a1->(-40, 90) // a2 -> (-10, 140) // a3 -> (-50, 60)

[l0, l1, l2, l3] = [0.3,0.302, 0.269, 0.100]
arm = ti.Actuator(["z", [l1, 0., 0.], "z", [l2, 0., 0.], "z", [l3, 0., 0.]])

# initial position
[as1, as2, as3] = np.deg2rad([76.1, 90, 9.4])
arm.angles = [as1, as2, as3]
[xs, ys] = [arm.ee[0]+l0, arm.ee[1]]
print('initial position:', [xs,ys])

# moment of inertia
mass = 1
J1 = (mass * (l1 * m.cos(as1)+ (l2 * m.cos(as1 + as2))**2 + (l1 * m.sin(as1)+ l2 * m.sin(as1 + as2))**2))
J2 = (mass * l2**2)
J3 = (mass * l3**2)

# target position
[xe, ye] = [xs, ys-0.1]
arm.ee = [xe-l0, ye, 0.]
[ae1, ae2, ae3] = arm.angles
print('target position:', [xe, ye])
print('target angles:',np.rad2deg([ae1, ae2, ae3]))

K = 0.1; B = 0.3
s = TransferFunction.s
### Transfer function ###
# for joint 1
G1 = (K/J1)/(s**2 + (B/J1)*s + (K/J1))
omega1 = (K/J1)**0.5
zeta1 = B/(2*((J1*K)**0.5))
print(omega1, zeta1)
# for joint 2
G2 = (K/J2)/(s**2 + (B/J2)*s + (K/J2))
omega2 = (K/J2)**0.5
zeta2 = B/(2*((J2*K)**0.5))
print(omega2, zeta2)
# for joint 3
G3 = (K/J3)/(s**2 + (B/J3)*s + (K/J3))
omega3 = (K/J3)**0.5
zeta3 = B/(2*((J3*K)**0.5))
print(omega3, zeta3)
# Time Dimension
n = 6.
t = np.arange(0, n, 0.01)  # 5 seconds / 0.01s step length

input1 = (ae1-as1)/s
input2 = (ae2-as2)/s
input3 = (ae3-as3)/s
output1 =  input1 * G1
output2 =  input2 * G2
output3 =  input3 * G3
t, joint1 = control.step_response(G1, t)
t, joint2 = control.step_response(G2, t)
t, joint3 = control.step_response(G3, t)
A1 = []
for a1 in joint1:
    a1 = as1 + (ae1-as1) * a1
    A1.append(a1)
A2 = []
for a2 in joint2:
    a2 = as2 + (ae2-as2) * a2
    A2.append(a2)
A3 = []
for a3 in joint3:
    a3 = as3 + (ae3-as3) * a1
    A3.append(a3)
##plt.plot(t, A1, 'r')
##plt.plot(t, A2, 'g')
##plt.plot(t, A3, 'y')
##plt.show()



#T = list(range(0, 300))
plt.ion()

x = xs;y = ys; x_all =[]; y_all=[]; xx=[];yy=[]
#[a1, a2, a3] = get_angles(x, y)
for i in range(len(t)):
    a1 = A1[i];a2 = A2[i]; a3 = A3[i]
### draw the joint and links at the initial position ###
    x0 = 0
    y0 = 0
    x1 = l0
    y1 = 0
    x2 = l1*m.cos(a1)+x1
    y2 = l1*m.sin(a1)+y1
    x3 = l2*m.cos(a1+a2)+x2
    y3 = l2*m.sin(a1+a2)+y2
    x4 = l3*m.cos(a1+a2+a3)+x3
    y4 = l3*m.sin(a1+a2+a3)+y3

    x_all = [x0, x1, x2, x3, x4]
    y_all = [y0, y1, y2, y3, y4]
    plt.xlim(-0.01, 0.40)
    plt.ylim(-0.01, 0.45)
    plt.title('A 3 Joints Robotic Arm -- K = 0.1')
    #fig = plt.figure (1)
    ax = plt.gca()
    ax.plot(x_all, y_all, 'o-g')
    plt.show()
    plt.pause(0.01)

    
    #[x, y] = get_position(a1, a2, a3)

'''    
for time in range(len(t)):
    [a1, a2, a3] = get_angles(x, y)
### draw the joint and links at the initial position ###
    x0 = 0
    y0 = 0
    x1 = l0
    y1 = 0
    x2 = l1*m.cos(a1)+x1
    y2 = l1*m.sin(a1)+y1
    x3 = l2*m.cos(a1+a2)+x2
    y3 = l2*m.sin(a1+a2)+y2
    x4 = l3*m.cos(a1+a2+a3)+x3
    y4 = l3*m.sin(a1+a2+a3)+y3

### inputs of 3 joints
    s = control.tf('s')
    input1 = (ae1-as1)/s
    input2 = (ae2-as2)/s
    input3 = (ae3-as3)/s
    output1 =  input1 * G1
    output2 =  input2 * G2
    output3 =  input3 * G3
    
### Perform Invert Laplace to OUtputs ###
    
    t, joint1 = control.step_response(G1, t)
    t, joint2 = control.step_response(G2, t)
    t, joint3 = control.step_response(G3, t)
   
    a1 = joint1[time]
    a2 = joint2[time]
    a3 = joint3[time]
   

    [x, y] = get_position(a1, a2, a3)
### renew the moment of inertia of joint 1 & transfer function
##    J1 = mass * (l1 * m.cos(as1)+ (l2 * m.cos(as1 + as2))**2 + (l1 * m.sin(as1)+ l2 * m.sin(as1 + as2))**2)
##    G1 = (K/J1)/(s**2 + (B/J1)*s + (K/J1))
#
    x_all = [x0, x1, x2, x3, x4]
    y_all = [y0, y1, y2, y3, y4]
    #plt.xlim(-0.100, 0.900)
    #  plt.ylim(-0.100, 0.700)
    plt.title('A 3 Joints Robotic Arm')
    #fig = plt.figure (1)
    ax = plt.gca()
    ax.plot(x_all, y_all, 'o-g')
    plt.show()
    plt.pause(0.01)
'''



    

    
    






##    
##plt.figure(1)
##T, yout  = control.step_response(Gs1, T, X0 = 2)
##plt.plot(T, yout)
##plt.show(block=False)
##plt.figure(2)
##yout, T = step(G2)
##plt.plot(T.T, yout.T)
##plt.show(block=False)
##plt.figure(3)
##yout, T = step(G3)
##plt.plot(T.T, yout.T)
##plt.show(block=False)

    

##
### 关节角随时间匀速变化
##
##
##[w1, w2, w3] = [(ra1[1]-ra1[0])/T[-1], 5*(ra2[1]-ra2[0])/T[-1], 10*(ra3[1]-ra3[0])/T[-1]]
##x=[];y=[];J=[]; xx=[]; yy=[]; a=[]; aa=[]
##
##
###plt.ion()
##
##for t in T:
##    plt.clf()
##    a1 = ((ra1[1]-ra1[0])/2) * m.sin(w1 *t) + ((ra1[1]+ra1[0])/2)
##    a2 = ((ra2[1]-ra2[0])/2) * m.sin(w2 *t) + ((ra2[1]+ra2[0])/2)
##    a3 = ((ra3[1]-ra3[0])/2) * m.sin(w3 *t) + ((ra3[1]+ra3[0])/2)
##    a = [m.degrees(a1), m.degrees(a2), m.degrees(a3)]
##    aa.append(a)
##    #[a1, a2, a3] = [ra1[0]+each*w1, ra2[0]+each*w2, ra3[0]+each*w3]
##    x0 = 0
##    y0 = 0
##    x1 = l0
##    y1 = 0
##    x2 = l1*m.cos(a1)+x1
##    y2 = l1*m.sin(a1)+y1
##    x3 = l2*m.cos(a1+a2)+x2
##    y3 = l2*m.sin(a1+a2)+y2
##    xe = l3*m.cos(a1+a2+a3)+x3
##    ye = l3*m.sin(a1+a2+a3)+y3
##
##    detJ = l1*l2*m.sin(a2)
##    J.append(detJ)
##    x = [x0, x1, x2, x3, xe]
##    y = [y0, y1, y2, y3, ye]
##    xx.append(x)
##    yy.append(y)
##    
##
##
##            
##    #[a1, a2, a3] = [ra1[0]+each*w1, ra2[0]+each*w2, ra3[0]+each*w3]
##    #arm.angles = [a1, a2, a3]
##    #end = arm.ee
##
####    
####    plt.xlim(-300, 1100)
####    plt.ylim(-700, 700)
####    plt.title('A 3 Joints Robotic Arm')
####    
####    #fig = plt.figure (1)
####    ax = plt.gca()
####    ax.plot(x, y, 'o-g')
####    plt.show()
####    plt.pause(0.01)
##
####print(max(J))
####for i in range(len(J)):
####    if J[i]==max(J):
####        print(i)
####        print('joints angles:',aa[i])
####        print('positions:',xx[i],yy[i])
####        break
##
####81237.99371098864
####985
####joints angles: [76.09600371373764, 90.02254494661412, 9.396190607422987]
####positions: [0, 300.0, 372.56931582618387, 111.42567576221802, 11.731927337127502] [0, 0, 293.1513165594171, 357.6881219819189, 365.50838572138196]
##
##arm.angles = np.deg2rad([76.1, 90, 9.4])
##xi = arm.ee[0]+300
##yi = arm.ee[1]
##print('initial position',[xi,yi])
##arm.ee = [xi-290, yi, 0.]
##xi = arm.ee[0]+300
##yi = arm.ee[1]
##print('final position:', [xi,yi])
##print('joint angles',np.rad2deg(arm.angles))

