import numpy as np
import matplotlib.pyplot as plt



import can
import struct

bus = can.Bus(
    interface="socketcan",
    channel="can0"
)

L1 = 0.12
L2 = 0.15
d  = 0.07


def cinematique_directe(theta1, theta5):
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


def cinematique_inverse(x, y):
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
    s = cinematique_inverse(x, y)
    if s is False:
        return None

    t1_deg = np.degrees(s[0])
    t2_deg = np.degrees(s[1])

    anglmot1 = status(0x141)
    anglmot2 = status(0x142)

    sens1 = testsensrota(anglmot1, t1_deg, 1)
    sens2 = testsensrota(anglmot2, t2_deg, 2)
    envoyer(0x141, t1_deg, 300, sens1)
    envoyer(0x142, t2_deg*36, 300, sens2)

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





       

def espacetravail():

    X = np.arange(-0.30, 0.30, 0.01)
    Y = np.arange(-0.30,  0.30, 0.01)
    print (X)
    T=[]
    F=[]
    for x in X:
        for y in Y:
            if cinematique_inverse(x, y) == False:
                F.append([x,y])
            else:
                T.append([x,y])
    plt.plot([p[0] for p in T], [p[1] for p in T], 'g.', markersize=5, label='accessible')

    plt.grid(True)
    plt.legend()
    plt.title("Espace de travail")
    plt.show()

