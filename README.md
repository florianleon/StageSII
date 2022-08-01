# DataFusion

## But du projet 

Des prototypes de fusion de données vont être devoir réalisé courant 2022 afin d'essayer d'améliorer le système de positionnement actuel.

Pour rappel, actuellement (janvier 2022) le projet de positionnement fin comprend deux phases :
    -   Une phase de positionnement peu précis longue distance. Cette phase est effectuée par trilatération par UWB.
    -   Une phase de positionnement fin à courte distance. Cette phase n'est pas encore effectuée est sera faite avec le capteur à effet Hall.

La fusion de donnée peut intervenir à plusieurs points permettant d'améliorer les précisions du système.

## Prototypes possibles

### Améliorer le positionnement peu précis par accéléromètre / gyroscope

L'ajout d'un accéléromètre couplé à un gyroscope permettrait - par intégration mathématiques - de déterminer la position relative du robot par rapport à son point de départ. En utilisant cette technologie en parallèle de l-UWB, de meilleures performances peuvent être possiblement obtenues.

L'accéléromètre permet de calculer le trajet effectué et le gyroscope permet de calculer les angles dans lequel s'est tourné le robot. Ensemble, une position précise de la navigation du robot peut être calculée.

Le déplacement du robot prend comme entrée une position déterminée par l'UWB. Cette position est ensuite converti en chemin à parcourir par le robot (`ex: Tourne 67° puis avance 145cm`). Cette commande est ensuite envoyée aux moteurs. En rajoutant le gyroscope ainsi que l'accéléromètre, nous pourrions mieux garantir que les commandes ont étaient respectées par les moteurs. 

Pour palier à ce manque, la méthode actuelle est de s'arrêter en chemin pour le robot, afin de refaire une mesure UWB et d'ainsi déterminer sa position actuelle. En utilisant le couple accel/gyro, nous pourrions avoir une plus grande précision sur la distance parcourue par le robot, et ainsi mieux traiter le bruit de la mesure d'UWB.

```
TODO list 1: 
    -   Etat de l'art pour la navigation par accel/gyro et fusion de données entre accéléromètre et gyroscope
    -   Algorithme pour accéder à la position du robot à partir d'un flux de données d'accélérations 
    -   Mise en place de l'algorithme en temps réel
    -   Algorithme pour obtenir l'angle parcouru par le robot à partir d'accélérations angulaires
    -   Mise en place de l'algorithme en temps réel
    -   Banc de test afin de mesurer la précision de ce nouveau système
```

```
TODO list 2: 
    -   Etat de l'art fusion de données entre accel/gyro et triangulation
    -   Choix de l'algorithme / traitement afin de fusionner au mieux les deux (l'accel/gyro peut aider à être plus précis sur la position propre du robot pour mieux estimer la distance UWB)
    -   Implémentation temps réel
    -   Banc de test
```

### Améliorer le positionnement fin avec l'accel/gyro

Après les tests de performances effectuées sur la banc de test 1, nous pourrons déterminer si ces capteurs se prouvent utile à courte portée. Dans les derniers centimètres, le cateur à effet Hall sera utilisé pour déterminer d'où provient l'aimant le plus proche. Les mouvement seront lents et de quelques centimètres mais ces capteurs peuvent nous aider à savoir si notre robot patine ou se déplace correctement.

Si le déplacement est cartographiable pendant le déplacement fin, ces données peuvent être couplé avec le capteur à effet Hall afin de déterminer bien mieux où exactement nous nous situons prés de l'aimant. Il permettrait aussi de facilement revenir sur nos pas si l'aimant devient indétectable.

```
TODO list 3: 
    -   Utiliser le banc de test pour mesurer les performances à courte navigation
    -   Etat de l'art de cartographie à partir de données de positionnement
    -   Mise en place d'un algorithme de cartographie à partir de notre capteur
    -   Implémentation temps réel
    -   Améliorer l'algorithme du capteur à effet Hall pour prendre en compte sa position plus précise par rapport au capteur
    -   Implémentation temps réel 
    -   Banc de test pour mesurer les nouvelles performances du système
```

### Améliorer la liaison UWB / effet Hall

Pour l'instant (janvier 2022), le positionnement peu précis et le positionnement fin sont disjoint. Une possibilité existe d'utiliser le capteur UWB même pendant la phase de positionnement fin afin d'améliorer encore celle-ci.

Durant la phase de positionnement fin, le robot est directement sous le capteur UWB de la voiture. Ce système où les balises UWB (robot) entourent le tag (la voiture) dont on est cherche à connaître la position est un système fréquent en TDoA et pourrait avoir de bonnes performances.

```
TODO list 4: 
    -   Mesurer les performances de l'UWB à courte portée grâce à un banc de test déjà fait. (au moins RTT, et TDoA en plus c'est mieux)
    -   Etat de l'art pour allier l'algorithme de l'effet Hall et de la traingulation/du positionnement
    -   Mise en place d'un algorithme
    -   Tests en temps réel
    -   Banc de test pour mesurer les nouvelles performances du système
```

## Données

Dans ce git, les données des bancs de tests peuvent être pushées pour qu'elles ne se perdent pas. Pousser vos données en indicant comment a été faite la mesure (stable ou en mouvement, préciser le chemin parcouru), et la fréquence de vos acquisitions.

### Accel Gyro

Pour obtenir les données de gyroscope du module GY 521, il vous faut flasher un esp32 avec le code nommé GY521_pitch_roll_yaw. Connecter VDD, GND, SCL et SCA et connecter le pin AD0 du capteur à VDD. Connecter l'esp32 au pc par usb et donner l'accès au périphérique à votre VM. Utiliser le script de cette manière sur linux :
`minicom -D /dev/ttyUSB0 -b 921600 -C data_gyro_accel_xxx.csv`.
Cette commande est à modifier si votre périphérique n'est pas reconnu comme ttyUSB0 mais avec un autre nom de port. Ce port est vérifiable est sélectionnant le périphérique sur l'IDE Arduino depuis votre VM. Le baud rate est représenté par `-b 921600`. Cette vitesse doit être identique à celle du programme Arduino flashé sur l'esp. En baissant la vitesse à 115200, ce script est alors utilisable sur Arduino Uno. 

Pour couper les données précédant un reset de l'esp32, taper cette commande :
`awk '/start/,/end/' data/data_gyro_accel_xxx.csv > data/data_gyro_accel_xxx_clean.csv̀`.
Cela supprime toutes les lignes au-dessus de celle avec écrit "start" et en-dessous de celle écrit "end". S'il n'y a pas de line avec écrit "end", cela conserve toutes les lignes après le start.