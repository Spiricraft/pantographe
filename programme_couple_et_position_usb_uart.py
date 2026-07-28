import numpy as np
import matplotlib.pyplot as plt

import sympy as sp

import can
import struct

bus = can.Bus(
    interface="socketcan",
    channel="can0"
)

L1 = 0.12
L2 = 0.15
d  = 0.07


def P3(theta1, theta5):
    x2 =  d/2 + L1*np.cos(theta1)
    y2 =  L1*np.sin(theta1)
    x4 = -d/2 + L1*np.cos(theta5)
    y4 =  L1*np.sin(theta5)
    dx = x4 - x2
    dy = y4 - y2
    R  = np.sqrt(dx**2 + dy**2)
    if R > 2*L2:
        raise ValueError("Configuration impossible")
    h  = np.sqrt(max(0.0, L2**2 - (R/2)**2))
    xm = (x2 + x4) / 2
    ym = (y2 + y4) / 2
    nx = -dy / R
    ny =  dx / R
    return xm + h*nx, ym + h*ny


def inverse_kinematics(x, y, tol=1e-8):
    solutions = []
    dx1 = x - d/2
    dy1 = y
    D1  = np.sqrt(dx1**2 + dy1**2)
    if D1 > L1 + L2 or D1 < abs(L1 - L2):
        return False
    phi1   = np.arctan2(dy1, dx1)
    alpha1 = np.arccos(np.clip((L1**2 + D1**2 - L2**2) / (2*L1*D1), -1, 1))
    theta1_candidates = [phi1 + alpha1, phi1 - alpha1]
    dx5 = x + d/2
    dy5 = y
    D5  = np.sqrt(dx5**2 + dy5**2)
    if D5 > L1 + L2 or D5 < abs(L1 - L2):
        return False
    phi5   = np.arctan2(dy5, dx5)
    alpha5 = np.arccos(np.clip((L1**2 + D5**2 - L2**2) / (2*L1*D5), -1, 1))
    theta5_candidates = [phi5 + alpha5, phi5 - alpha5]
    for theta1 in theta1_candidates:
        for theta5 in theta5_candidates:
                solutions.append((theta1,theta5 ))


    def critere(sol):
        t1, t5 = sol
        x2 =  d/2 + L1*np.cos(t1)
        y2 = L1*np.sin(t1)
        x4 = -d/2 + L1*np.cos(t5)
        y4 = L1*np.sin(t5)
        cross1 = (x - d/2) * y2  - y * (x2 - d/2)
        cross5 = (x + d/2) * y4  - y * (x4 + d/2)

        return cross1 < 0 and cross5 > 0
    bonnes = [s for s in solutions if critere(s)]

    if not bonnes:
        return False
    t1_deg = np.degrees(bonnes[0][0])
    t5_deg = np.degrees(bonnes[0][1])

    if t1_deg > 127 or t1_deg < -76:      
        return False

    if t5_deg > 256 or t5_deg < 53:       
        return False
    return bonnes[0]
####==========================================================================
import struct

def angl(x, y):
    s = inverse_kinematics(x, y, tol=1e-8)
    if s is False:
        return None

    t1_deg = np.degrees(s[0])
    t2_deg = np.degrees(s[1])

    anglmot1 = status(0x141)
    anglmot2 = status(0x142)

    sens1 = testsensrota(anglmot1, t1_deg, 1)
    sens2 = testsensrota(anglmot2, t2_deg, 2)
    envoyer(0x141, t1_deg, 300, sens1)
    envoyer(0x142, t2_deg, 300, sens2)

    return True
def testsensrota(anglea, angleb, mot):
    if mot == 1:
        borne_min, borne_max = -76, 127
    elif mot == 2:
        borne_min, borne_max = 53, 256
    pas = 0.5  
    if anglea <= angleb:
        etapes = int((angleb - anglea) / pas)
        valeurs = [anglea + i * pas for i in range(etapes + 1)]
    else:
        etapes = int((anglea - angleb) / pas)
        valeurs = [anglea - i * pas for i in range(etapes + 1)]

    for i in valeurs:
        if i > borne_max or i < borne_min:
            return 0x01  

    return 0x00  
        

def status(canid):
    data = bytearray(8)
    data[0] = 0x90  # Command byte

    msg = can.Message(arbitration_id=canid, data=data, is_extended_id=False)
    bus.send(msg)

    rep = bus.recv(timeout=0.1)
    if rep is None:
        return None

    encoder = rep.data[2] | (rep.data[3] << 8)
    encoder_raw = rep.data[4] | (rep.data[5] << 8)
    encoder_offset = rep.data[6] | (rep.data[7] << 8)

    angle_deg = (encoder / 65536) * 360

    return angle_deg



def envoyer(can_id, angle_deg,vitesse_dps,sens=0x00):
    angle_deg = angle_deg % 360
    angle = int(angle_deg * 100)
    vitesse = int(vitesse_dps)

    data = bytearray(8)

    data[0] = 0xA6
    data[1] = 0x00
    data[2:4] = struct.pack("<H", vitesse)

    data[4:8] = struct.pack("<i", angle)

    msg = can.Message(
        arbitration_id=can_id,
        data=data,
        is_extended_id=False
    )

    bus.send(msg)

    return True

def envoyer_couple(can_id, couple):

    c = int(couple*100)

    data = bytearray(8)

    data[0]=0xA1
    data[1]=0
    data[2]=0
    data[3]=0
    data[4:8]=struct.pack("<i",c)

    bus.send(can.Message(
            arbitration_id=140+can_id,
            data=data,
            is_extended_id=False))





####==========================================================================


def afficher(x, y, theta1, theta5):
    x2 =  d/2 + L1*np.cos(theta1)
    y2 =  L1*np.sin(theta1)
    x4 = -d/2 + L1*np.cos(theta5)
    y4 =  L1*np.sin(theta5)

    plt.plot([d/2,  x2], [0, y2], 'ro-', lw=2)
    plt.plot([-d/2, x4], [0, y4], 'bo-', lw=2)
    plt.plot([x2, x],   [y2, y],  'r--', lw=1.5)
    plt.plot([x4, x],   [y4, y],  'b--', lw=1.5)
    plt.plot([d/2, -d/2], [0, 0], 'ks-', markersize=8)
    plt.plot(x, y, 'g*', markersize=14)
    plt.axis('equal')
    plt.grid(True)
    plt.title(f'P3 = ({x:.3f}, {y:.3f}) m')
    plt.show()


def jacobian(theta1, theta5):
    x2 =  d/2 + L1*np.cos(theta1)
    y2 =  L1*np.sin(theta1)
    x4 = -d/2 + L1*np.cos(theta5)
    y4 =  L1*np.sin(theta5)
    x3, y3 = P3(theta1, theta5)
    delta = (x3 - x2)*(y3 - y4) - (y3 - y2)*(x3 - x4)
    b1 = L1*(-(x3 - x2)*np.sin(theta1) + (y3 - y2)*np.cos(theta1))
    b2 = L1*(-(x3 - x4)*np.sin(theta5) + (y3 - y4)*np.cos(theta5))
    J = np.array([
        [ (y3 - y4)*b1 / delta, -(y3 - y2)*b2 / delta],
        [-(x3 - x4)*b1 / delta,  (x3 - x2)*b2 / delta]])
    return J


def couple(theta1, theta5, Fx, Fy):
    J   = jacobian(theta1, theta5)
    tau = J.T @ np.array([Fx, Fy])
    return tau[0], tau[1]


def couple_pos(x, y, Fx, Fy):
    theta1, theta5 = inverse_kinematics(x, y)
    return couple(theta1, theta5, Fx, Fy)





       

def espacetravail():

    X = np.arange(-0.30, 0.30, 0.01)
    Y = np.arange(-0.30,  0.30, 0.01)
    print (X)
    T=[]
    F=[]
    for x in X:
        for y in Y:
            if inverse_kinematics(x, y, tol=1e-8) == False:
                F.append([x,y])
            else:
                T.append([x,y])
    plt.plot([p[0] for p in T], [p[1] for p in T], 'g.', markersize=5, label='accessible')
    plt.plot([p[0] for p in F], [p[1] for p in F], 'r.', markersize=5, label='inaccessible')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("Espace de travail")
    plt.show()

def trajectoire(x1, y1, x2, y2, V, pas=0.005):
    
    X = np.linspace(x1, x2, int(np.sqrt((x2-x1)**2 + (y2-y1)**2) / pas))
    Y = ((y2-y1)/(x2-x1)) * (X - x1) + y1 if x1 != x2 else np.linspace(y1, y2, len(X))

    T = []
    for i in range(len(X)):
        sol = inverse_kinematics(X[i], Y[i])
        if sol is False:
            print(f"Point inaccessible : ({X[i]:.3f}, {Y[i]:.3f})")
            return False
        T.append(sol)
    plt.figure()
    plt.plot(X, Y, 'g.-', markersize=3, label='trajectoire')
    plt.plot(x1, y1, 'go', markersize=8, label='départ')
    plt.plot(x2, y2, 'rs', markersize=8, label='arrivée')
    plt.plot([-d/2, d/2], [0, 0], 'ks-', markersize=8, label='base')
    plt.axis('equal')
    plt.grid(True)
    plt.legend()
    plt.title("Trajectoire")
    plt.show()

##    for i in range(len(T)):
##        envoyer(T[i][0], T[i][1])
##        time.sleep(pas / V)

    return True


F=[]
def appartenance (x,y,Px,Py):
    Ea=[]
    Eb=[]
    for i in range (len(Px)):
        Ea.append((Py[i]-Py[i+1])/(Px[i]-Px[i+1]))
        Eb.append(Py[i]-Px[i]*((Py[i]-Py[i+1])/(Px[i]-Px[i+1])))
    x = Symbol("x")
    k=0
    for i in range (len(Px)):
        s=(y-Eb(i))/Ea(i)
        if Px[i]<s<Px[i+1] or Px[i]>s>Px[i+1]:
            k+=1
    if k%2==1:
        return True
            
    return False

def direction_correction(x, y, traj, k):
    if k < len(traj) - 1:
        tx = traj[k+1][0] - traj[k][0]
        ty = traj[k+1][1] - traj[k][1]
    else:
        tx = traj[k][0] - traj[k-1][0]
        ty = traj[k][1] - traj[k-1][1]

    norme_t = np.sqrt(tx**2 + ty**2)
    tx /= norme_t
    ty /= norme_t
    nx = traj[k][0] - x
    ny = traj[k][1] - y
    norme_n = np.sqrt(nx**2 + ny**2)
    if norme_n < 1e-10:
        return 0.0, 0.0
    nx /= norme_n
    ny /= norme_n

    return nx, ny





def asserv(traj, kp=20.0, kd=2.0, dt=0.02, seuil_sortie=0.01):
    erreur_prec = 0.0
    run = True
    try:
        while run:
            td = time.time()
            pos = lire_position()
            if pos is None:
                time.sleep(dt)
                continue
            x, y = pos
            erreur, point_proche, k = distance_avec_index(x, y, traj)
            if erreur > seuil_sortie:
                derivee = (erreur - erreur_prec) / dt
                F_norme = kp * erreur + kd * derivee
                nx, ny = direction_correction(x, y, traj, k)
                Fx = F_norme * nx
                Fy = F_norme * ny
                sol = inverse_kinematics(x, y)
                if sol is not False:
                    tau1, tau2 = couple_pos(x, y, Fx, Fy)
                    envoyer_couple(tau1, tau2)
            else:
                envoyer_couple(0.0, 0.0)

            erreur_prec = erreur
            t_ecoule = time.time() - td
            time.sleep(max(0.0, dt - t_ecoule))

    except KeyboardInterrupt:
        envoyer_couple(0.0, 0.0)
        print("fin")


def distance_avec_index(x, y, traj):
    M = np.inf
    k = 0
    for i in range(len(traj)):
        d_i = np.sqrt((x - traj[i][0])**2 + (y - traj[i][1])**2)
        if d_i < M:
            M = d_i
            k = i
    return M, traj[k], k



         
    
       

    

    
