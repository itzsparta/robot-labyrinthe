# Robot labyrinthe : démarche et problèmes rencontrés

## 1. Objectif et matériel

L'objectif est de faire parcourir un labyrinthe à un robot de façon autonome, c'est-à-dire sans
lui donner à l'avance la suite des virages. Le parcours de test compte 12 virages (G G G G D D G G D D G G),
dans des couloirs de 22 cm de large.

Matériel :

- carte Cytron Maker Nano (compatible Arduino Nano, ATmega328) ;
- 3 capteurs de distance à temps de vol VL6180X (Pololu #2489), portée utile d'environ 20 cm :
  un à l'avant, un à gauche et un à droite, orientés à 90° comme demandé dans le cahier des charges ;
- 2 servomoteurs à rotation continue FS90R (roues gauche et droite) ;
- un buzzer pour signaler les décisions du robot ;
- alimentation par une batterie externe USB 5 V (Hama Supreme 5HD).

Géométrie mesurée : capteurs latéraux à 8 cm l'un de l'autre, montés à l'avant ; capteur avant décalé
de 2 cm à gauche du capteur droit ; nez à 5,5 cm de l'axe des roues, coin du nez à 8,5 cm de cet axe.

Le robot n'a ni encodeurs sur les roues ni gyroscope : il ne connaît ni la distance parcourue ni
l'angle dont il a tourné. Il ne dispose que des trois distances mesurées.

## 2. Démarche générale

Le travail a avancé par étapes, chacune dans un programme séparé (dossiers `01` à `08`) :

1. vérifier le matériel seul (capteurs, bus I2C, réglage de l'arrêt des servos) ;
2. réussir un parcours dont on connaît les virages ;
3. rendre le robot autonome ;
4. le rendre fiable, puis plus rapide.

Chaque version qui fonctionnait sur la piste a été enregistrée (git), afin de pouvoir revenir en arrière
quand une modification dégradait le comportement.

## 3. Mise en service du matériel

### 3.1 Trois capteurs identiques sur le même bus I2C

Les trois VL6180X ont la même adresse I2C au démarrage (0x29), ils ne peuvent donc pas être interrogés
ensemble. Solution : chaque capteur a sa broche d'activation CE reliée à une sortie de la carte (D2, D3, D4).
Au démarrage, on éteint les trois capteurs, puis on les allume un par un et on donne à chacun une nouvelle
adresse (0x30, 0x31, 0x32).

Point de vigilance : la broche CE ne supporte pas le 5 V. Pour allumer un capteur, on ne met donc pas la
sortie à l'état haut ; on la passe en entrée, et c'est la résistance de tirage de la carte du capteur qui
fixe le niveau.

Programmes utilisés : `01_test_cablage` (affiche les trois distances) et `02_diag_i2c` (scanne le bus
capteur par capteur en cas de panne).

### 3.2 Arrêt des servos à rotation continue

Un FS90R ne reçoit pas une position mais une vitesse : une impulsion d'environ 1500 µs l'arrête,
au-dessus il tourne dans un sens, en dessous dans l'autre. Le point d'arrêt exact diffère d'un servo à
l'autre. Il a été mesuré roues en l'air : 1485 µs pour la roue gauche (zone morte 1468-1502),
1470 µs pour la roue droite (1455-1488).

Problème : posé au sol, le robot avançait encore légèrement à l'arrêt. Solution : pour s'arrêter
vraiment, le programme coupe le signal des servos (`detach()`).

Le servo droit est monté en miroir : pour avancer, il doit tourner dans le sens opposé au gauche, ce que
le programme compense.

## 4. Premières stratégies de conduite

### 4.1 Parcours connu (`03`, puis `06` avec une liste de virages)

Le robot avance en se centrant entre les murs. Quand le capteur avant voit un mur proche, il s'arrête et
pivote du côté indiqué par la liste (par exemple `"GGGGD"`).

Sans encodeur, l'angle de pivot est estimé au temps : environ 0,29 °/ms à la vitesse de pivot, avec 40 ms
perdues au démarrage, soit des pas de 45° de 195 ms. Le robot s'arrête entre deux pas pour mesurer sans
être perturbé par le mouvement.

Résultat : parcours réussi, mais seulement parce que le robot connaissait l'ordre des virages. Ce n'était
pas l'objectif.

### 4.2 Robot autonome par suivi du mur droit (`04`, `06`)

Principe de la « main droite » : le robot longe le mur droit ; si ce mur disparaît, il tourne à droite ;
si un mur est devant, il tourne à gauche. Pour détecter une ouverture à droite, il s'arrête et fait un
pivot de contrôle (sonde).

Outils de mise au point ajoutés :

- journal des décisions enregistré en EEPROM (16 derniers événements, relus ensuite par le port série),
  pour savoir ce que le robot a décidé pendant une course sans être relié à l'ordinateur ;
- mode lent, avec bips à chaque décision, pour observer les choix du robot ;
- détection du robot coincé (recul puis nouvelle tentative) ;
- garde « robot soulevé » : si les trois capteurs ne voient rien, le robot ne décide pas, pour que la
  manipulation à la main ne remplisse pas le journal.

Résultat : un parcours complet a été réussi une fois, mais de façon irrégulière : mauvais virages,
robot qui frotte les murs ou se bloque contre un coin. Augmenter la vitesse ou réduire les pauses
aggravait le problème.

### 4.3 Conduite réactive (`07`)

Approche différente, adaptée d'un programme fourni : le robot ne s'arrête jamais.

- Rien devant (plus de 220 mm) : tout droit, à pleine vitesse.
- Mur devant : il braque vers le côté le plus dégagé (comparaison gauche/droite), d'autant plus fort
  que le mur est proche. La roue intérieure ralentit, s'arrête puis recule quand le mur est très proche.

Les virages se font ainsi en arc et non en pivot sur place, ce qui évite que le coin du nez (à 8,5 cm de
l'axe des roues) frotte le mur. Le robot était plus vif et ne frottait plus, mais il refusait souvent de
tourner à droite alors que le passage était libre.

## 5. Le problème principal : les capteurs voyaient le sol

Pendant longtemps, les mauvaises décisions ont été attribuées au programme : seuils, vitesses, durées de
pivot ont été modifiés à de nombreuses reprises, et des hypothèses ont été testées puis abandonnées
(capteur droit trop en arrière, capteur avant gêné par le mur d'en face, suivi du mur gauche au lieu du
droit). Même en mode très lent, le robot prenait de mauvaises décisions, ce qui montrait que le problème
ne venait pas de la vitesse.

Démarche qui a permis de trouver la cause : un test statique. Robot posé sur une table, moteurs à l'arrêt,
rien autour, lecture des trois capteurs en direct avec `01_test_cablage`, puis un objet placé à une
distance connue.

Constat : sans aucun obstacle, les capteurs latéraux lisaient environ 55 à 65 mm et le capteur avant
environ 130 mm. Le cône d'émission des VL6180X, montés à plat et près du sol, touchait le sol. Pour le
robot, un côté ouvert ressemblait donc à un mur proche (environ 70 mm), ce qui expliquait qu'il refuse de
tourner vers un passage libre.

Solution : les capteurs ont été inclinés légèrement vers le haut. Mesures après correction :

| Situation | Avant | Après |
|-----------|-------|-------|
| Côté ouvert | ~55-65 mm | ~170-190 mm |
| Mur latéral à 7 cm | difficile à distinguer | ~81 mm |
| Rien devant | ~130 mm | 255 (hors de portée) |

Avec les capteurs inclinés, le programme réactif `07` d'origine, sans autre modification, a réussi le
parcours complet, puis 3 tours de suite sans erreur.

Leçon retenue : quand un robot prend des décisions incohérentes, il faut d'abord vérifier ce que
mesurent réellement les capteurs dans les conditions de la piste (hauteur, sol, obstacles connus), avant
de modifier la logique de conduite.

## 6. Recherche de vitesse

Temps de référence : 16 s pour le tour complet (chronométré à la main). Une seule modification a été
testée à la fois, en comparant au temps de référence.

| Essai | Résultat | Décision |
|-------|----------|----------|
| Roue extérieure plus rapide en virage (20→30 et 30→40) | 16 s, le robot frotte sur la longue ligne droite | abandonné |
| Centrage entre les deux murs en ligne droite | ne touche plus le mur, mais plus lent | abandonné |
| Vitesse 80 au lieu de 45 quand rien n'est vu devant | 16 s, frotte | abandonné |

Pour comprendre pourquoi la vitesse ne changeait pas, un test dédié a été écrit (`08_test_vitesse`) : le
robot roule 2 s tout droit à des commandes de 30, 45, 60 et 90, et on mesure la distance parcourue.
Résultat : la distance est la même à partir de 45. Les servos sont saturés, c'est-à-dire déjà à leur
vitesse maximale à 45 (450 µs d'écart par rapport à l'arrêt).

Conséquences :

- en ligne droite, le robot roule déjà au maximum, il n'y a rien à gagner par le programme ;
- une roue saturée ne ralentit pas quand on lui retire un peu de commande, donc une correction de
  trajectoire calculée à partir d'une commande trop élevée n'a plus d'effet, d'où les frottements ;
- le centrage ralentissait le robot car les capteurs gauche et droit ne donnent pas exactement la même
  valeur à distance égale : le robot corrigeait en permanence en freinant une roue.

Pistes restantes, matérielles : alimenter les servos en 6 V par une source séparée (environ 15 % plus
rapides d'après la documentation du FS90R, masse commune avec la carte), ou des roues de plus grand
diamètre.

## 7. Résultat final

Programme retenu : `07_reactif`, version d'origine avec capteurs inclinés. Parcours de 12 virages réussi
3 fois de suite sans erreur, en 16 s, ce qui correspond à la vitesse maximale des servos alimentés en 5 V.

## 8. Bilan des difficultés

| Problème | Cause | Solution |
|----------|-------|----------|
| Capteurs à la même adresse I2C | adresse par défaut identique | activation un par un par CE, nouvelle adresse |
| Robot qui avance à l'arrêt | point d'arrêt des servos imprécis | mesure du STOP de chaque servo, `detach()` pour s'arrêter |
| Angle de virage imprécis | pas d'encodeur ni de gyroscope | pivot au temps, puis virages en arc continu |
| Coin du robot qui frotte en pivotant | nez à 8,5 cm de l'axe des roues | virages en arc (`07`) |
| Mauvaises décisions, côté libre vu comme un mur | cône des capteurs touchant le sol | capteurs inclinés vers le haut |
| Impossible de suivre les décisions en course | robot non relié au PC | journal EEPROM, bips, mode lent |
| Vitesse impossible à augmenter | servos saturés à 45 | mesuré avec `08` ; seule piste : alimentation 6 V |
