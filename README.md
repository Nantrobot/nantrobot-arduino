# Code Arduino pour les Actionneurs du NantRobot 2016

## Protocole de test des actionneurs/capteurs

### But
- fournir une démarche efficace de test des actionneurs et capteurs.
- vérifier le bon fonctionnement des capteurs et actionneurs

Implémentation du code
----------------------
- vérifier les branchements suivants :
	- servo moteur parasol : PIN 44
	- servo moteur Gripper (bout de bras) : PIN 42
	- servo moteur base du bras (indice 0) : PIN 32
	- servo moteur bras 1 (indice 1) : PIN 34
	- servo moteur bras 2 (indice 2) : : PIN 36
	- servo moteur avant bras (indice 3) : PIN 40
	- Ultrason avant : PIN 46
	- Ultrason arrière : PIN 50
	- pinces avants : PIN TX/RX (demander à Yann)
- téléverser le code `Servo_test.ino` sur l'arduino MEGA
- observer le clignotement de la LED 13 (sur arduino) à une fréquence d'allumage de 5Hz

Lancement du noeud
------------------
- lancer le noeud : `rosrun rosserial_python serial_node.py /dev/arduino_mega` (à adapter suivant le port utilisé) 

Les capteurs ultrasons
----------------------
- `rostopic list` 
- Vérifier la présence de 2 topics : `/UltrasonFront (type Int32)` et `/UltrasonBack` (type `Int32`)
- `rostopic echo /UltrasonFront` : distance en centimètre du première objet dans le cône du capteur avant à moins de 20 cm (message de type Int32) 
- `rostopic echo /UltrasonBack` : distance en centimètre du première objet dans le cône du capteur avant à moins de 20 cm (message de type Int32)
- modifier le seuil de message : `positionAlerte` (valeur en centimètre !)

Le parasol
----------
- `rostopic list` 
- Vérifier la présence du topic : `/ParasolCommand` (type `Empty`)
- `rostopic pub /ParasolCommand std_msgs/Empty` : tourne de 140° le servo moteur du parasol (dans un sens puis dans l'autre)
	- Si nécessité de modifier la valeur de départ : modifier la variable -> `initSMParasol_angle`
	- Si nécessité de modifier la valeur de finale : modifier la variable -> `endSMParasol_angle`

Les pinces avant
----------------
- `rostopic list`
- Vérifier la présence du topic : `/FrontPlierCommand` (type string) et `/FrontPlierState` (type Bool)
- `rostopic echo /FrontPlierState` : Je n'ai pas compris le rôle de ce message... Demander à Yann (message de type Int32)
	- `rostopic pub /FrontPlierCommand std_msgs/String open` : ouvre les pinces
	- `rostopic pub /FrontPlierCommand std_msgs/String close` : ferme les pinces
	- `rostopic pub /FrontPlierCommand std_msgs/String middle` : met les pinces en position intermédiaire
	- `rostopic pub /FrontPlierCommand std_msgs/String balance` : met les pinces en oscillations
	- `rostopic pub /FrontPlierCommand std_msgs/String stop` : arrete tout mouvement des pinces
 
Le uArm
-------
- `rostopic list`
- En cas de position de départ non désirée modifier les variables : `initSM0_angle`, `initSM1_angle`, `initSM2_angle`, `initSM3_angle`
- Vérifier la présence du topic : `/UarmCommand` (type Twist) et `/UarmStat` (type Twist)
- `rostopic echo /UarmState` : Position actuel du bras dans le repère bras 
- `rostopic pub /UarmCommand geometry_msgs/Twist '{linear:  {x: ?, y: ?, z: ?}, angular: {x: ?,y: ?,z: ?}}'` : 
	les 3 linears déterminent (x,y,z) et angular.x détermine théta l'angle du robot dans le repère robot 
    
	**!!!!! ATTENTION** : Y est vers l'avant  

```
| Y
|
|               ROBOT VU DU HAUT !!!
|
|_________X
```


Le gripper
----------
- `rostopic list`
- En cas de position de départ non désirée modifier les variables :  
- Vérifier la présence du topic : `/GripperCommand` (type Int16) et `/GripperState` (type Int16)
- `rostopic echo /GripperState` : Position actuel de la pince : `UNDEFINED->0`, `CLOSED->1`,`OPEN->2`
- `rostopic pub /GripperCommand std_msgs/Int16` "valeur" (valeur > 157 -> la pince se ferme // valeur < 66 -> la pince s'ouvre)