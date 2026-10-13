# Robot labyrinthe

Cytron Maker Nano + 3x VL6180X (Pololu #2489) + 2x servos FS90R.
Bibliothèques Arduino : `VL6180X` (Pololu), `Servo`. Carte : Arduino Nano, ATmega328 (nouveau bootloader).

## Sketches (dans l'ordre d'utilisation)

| Dossier | Rôle |
|---------|------|
| `01_test_cablage` | Vérifie les 3 capteurs, affiche les distances, règle le STOP des servos (moniteur série 115200) |
| `02_diag_i2c` | Scanne le bus I2C capteur par capteur (9600) — à lancer si `01` donne ECHEC |
| `03_labyrinthe_connu` | Parcours connu, suite de virages `PARCOURS`, capteurs latéraux à 90° |
| `04_labyrinthe_autonome` | Trouve les virages seul, demi-tour en cul-de-sac |
| `05_labyrinthe_rapide` | Parcours connu rapide, capteurs diagonaux à 45° |
| `_archive` | Anciens doublons |

`DEBUG 1` : affiche les distances dans le moniteur série. Dans `03`, `DEBUG 1` coupe aussi les moteurs. Passer à 0 pour rouler.

## Câblage

VL6180X : VIN->5V, GND->GND, SDA->A4, SCL->A5. CE : D2 / D3 / D4. Servos : D9 (gauche), D10 (droite). Buzzer : D8.
Débrancher la carte du reste du robot pour téléverser.

## Ligne de commande

```
arduino-cli compile --upload -p COM3 --fqbn arduino:avr:nano:cpu=atmega328 01_test_cablage
```
