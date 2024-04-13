import math 
import numpy as np

#estimated model paramaters
c0 = 158043332.734587
c1 = -3206329132.70678
c2 = 21913523335.3977
c3 = -49588278580.0942
d0 = -4.00444367272256

#initial dimensions
D_0 = 0.020   #20 mm
L_0 = 0.150   #300 mm
H_0 = 0.0018  #1.8 mm
thread_angle_rad = math.radians(28.6 + d0) # 28.6 deg + correction
L_fiber = L_0/math.cos(thread_angle_rad) # fiber length
n = (L_0*math.tan(thread_angle_rad))/(math.pi*D_0) #n

def PAM_Force(P:float, L:float) -> float:

    D = math.sqrt(pow(L_fiber,2) - pow(L,2))/(n*math.pi) # dimater of PAM(L)
    E = c3*(pow(L,3)) + c2*(pow(L,2)) + c1*(L) + c0 # E_rubber of PAM(L)

    sigma_L = E*((L-L_0)/L_0)
    sigma_PE = E*((D-D_0)/D_0)

    F_L = sigma_L*H_0*math.pi*D
    F_PE = sigma_PE*H_0*L*math.pi

    dVdL = ((pow(L_fiber,2)) - (3*(pow(L,2))))/(4*math.pi*pow(n,2))
    dDdL = -L/(math.pi*n*math.sqrt(pow(L_fiber,2) - pow(L,2)))
    F_martens = (-P*dVdL) + (F_PE*dDdL) - F_L

    if F_martens < 0:
        F_martens = 0

    return F_martens

def PAM_volume(L:float):
    return (L*(pow(L_fiber,2) - pow(L,2)))/(4*math.pi*(pow(n,2))) # volume of PAM(L)

def current_to_diameter(p_diff, current, increasing, d_max, max_current):
    if increasing:
        startup_current = -12.877*p_diff + 163.84
    else:
        startup_current = -13.973*p_diff + 158.63
    current_diff = max_current - startup_current
    diameter_per_current = d_max/current_diff
    diameter = diameter_per_current*(current - startup_current)
    if diameter < 0:
        diameter = 0
    return diameter


def mass_flow_rate_real(p, p_inlet, d, D):
    if d == 0:
        return 0
    d = d
    D = D
    C = 0.128*(d**2)*10**-8
    b = 0.41 + 0.272*math.sqrt(d/D)
    b =0.1
    rho_0 = 1.185
    T_0 = 293
    T_1 = 330
    k_1 = 1000*C*rho_0*math.sqrt(1-((0.999-b)/(1-b))**2)
    P_1 = p_inlet
    P_2 = p

    if P_2 < 0:
        return 0

    if P_2/P_1 >= 0.999: # laminar 
        return k_1*P_1*(1-(P_2/P_1))*math.sqrt(T_0/T_1)
    elif (0.999>(P_2/P_1)) and ((P_2/P_1)>b): # subsonic
        return P_1*C*rho_0*math.sqrt(T_0/T_1)*math.sqrt(1-((((P_2/P_1)-b)/(1-b))**2))
    elif (P_2/P_1) <= b: #choked
        return P_1*C*rho_0*math.sqrt(T_0/T_1)
    

def mass_flow_rate_exhaust_real(p, p_outlet, d, D):
    if d == 0:
        return 0
    d = d
    D = D
    C = 0.128*(d**2)*10**-8
    b = 0.41 + 0.272*math.sqrt(d/D)
    rho_0 = 1.185
    T_0 = 293
    T_1 = 330
    k_1 = 1000*C*rho_0*math.sqrt(1-((0.999-b)/(1-b))**2)
    P_1 = p
    P_2 = p_outlet
    
    if P_2 < 0:
        return 0

    if P_2/P_1 >= 0.999: # laminar 
        return k_1*P_1*(1-(P_2/P_1))*math.sqrt(T_0/T_1)
    elif (0.999>(P_2/P_1)) and ((P_2/P_1)>b): # subsonic
        return P_1*C*rho_0*math.sqrt(T_0/T_1)*math.sqrt(1-((((P_2/P_1)-b)/(1-b))**2))
    elif (P_2/P_1) <= b: #choked
        return P_1*C*rho_0*math.sqrt(T_0/T_1)
