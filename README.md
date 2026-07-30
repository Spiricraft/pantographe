### Installation module can sur Raspberry (1ere installation)

Identification Raspi:
id:basile
mdp:Basile2013

-avant tout il est souvent necessaire de corriger l'horloge

`sudo date -s "2026-07-24 11:00:00"`

-et de regler un pb de connection :

`sudo nano /etc/resolv.conf`

-puis ajouter:

```
nameserver 1.1.1.1
nameserver 8.8.8.8
```

**installer Python3:**

		sudo apt install python3 python3-pip python3-venv
    
**ajouter un environnement virutel:**

		python3 -m venv venv
    
		source venv/bin/activate
    
**Les modules pythons:**

Numpy,
python-can,
matplotlib:
`pip install numpy matplotlib python-can`
    
can-utils:
`sudo apt install can-utils`

### Setup can:

1.

```sudo apt-get update
sudo apt-get upgrade
sudo reboot
```

2.
aller dans le fichier
`sudo nano /boot/firmware/config.txt`

puis ajouter dans [all]

```dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=8000000,interrupt=25
dtoverlay=spi-bcm2835
```

enfin ctrl+x
`sudo reboot`



### A chaque démarage

**Montage du protocole can:**

`sudo ip link set can0 up type can bitrate 500000`

**verification, le status doit etre up:**

`ip link show can0`

**Lancer le programme Python**

```
cd pantographe
source venv/bin/activate
python3 -i programme_couple_et_position_usb_uart.py
```

### Commandes utiles:

-candump -e can0 :permet de voir en temps réel les réponses du moteurs et la trame envoyée, il faut l'exécuter dans un autre terminal

-ip link show can0 :verification du statut can 

-cansend can0 141#A300000000000000 :envoi d'une trame sous la forme id#0000000000000000

-sudo ip link set can0 down

-sudo ip link set can0 up : monte et démonte le protocole can

### Protocole can rapidement:

envoi des trames de 8bit avec un id en tete:
00#0000000000000000
pour les moteurs l'id est sous la forme 0x140+id :moteur 1 aura l'id 0x141...
le détail du format des trames est donner dans une documentation lktech disponible sur le git

### Programme python:

Les paramètres du robot (longueur) sont a rentrer au tout début du programme 

cinematique_directe(theta1, theta5): donne la position du point 3 en fonction de l'angle des 2 moteurs, retourne un flotant


cinematique_inverse(x, y): retourne l'angle des deux moteurs sous la forme (angle1,angle5), fonction injective.

angl(x,y):execute cinematique inverse, choisi le sens de rotation des moteurs et envoie la commande aux moteurs

testsensrota(anglea, angleb, mot): choisit le sens de rotation pour repondre aux contraintes géométriques du robot (évite de tout casser)

status(canid): demande aux moteur leur position, retourne un angle en degré, id sous la forme 0x141 ou 0x142

envoyer(can_id, angle_deg,vitesse_dps,sens=0x00): envoi une consigne de d'angle unique, avec sens de rotation et vitesse, de meme can id tel que 0x141,ou 0x142

espacetravail(): donne l'espace de travail 


les autres fonctions sont moins aboutit ou moins appliquées 





