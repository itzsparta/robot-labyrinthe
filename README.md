# Robot labyrinthe

Robot autonome qui parcourt un labyrinthe de 12 virages avec trois capteurs de
distance, sans encodeur ni gyroscope. Projet d'école d'ingénieurs, en Arduino (C++).

**Résultat** : parcours complet réussi 3 fois de suite sans erreur, en 16 s le
tour, soit la vitesse maximale des servos alimentés en 5 V.

**Ce que le projet montre**
- Mise en service de 3 capteurs I2C de même adresse (activation un par un, réadressage).
- Huit versions successives du programme, de la liste de virages connue à la
  conduite réactive sans arrêt.
- Un diagnostic : les mauvaises décisions ne venaient pas du code mais du cône des
  capteurs qui touchait le sol. Trouvé par un test statique, corrigé en inclinant
  les capteurs.
- Des mesures plutôt que des réglages à l'aveugle : point d'arrêt de chaque servo,
  vitesse de pivot, saturation des servos mesurée avec un test dédié.

La démarche complète, avec les problèmes rencontrés et leurs causes, est dans
[`RAPPORT_demarche.md`](RAPPORT_demarche.md).

## Matériel

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
| `06_suivi_mur_droit` | Longe le mur droit ; à chaque mur devant, prend le virage suivant de `PARCOURS` (`"GGGGD"`, validé sur la piste le 16/10). `PARCOURS = ""` : règle de la main droite |
| `07_reactif` | **Sketch de référence** (parcours complet réussi le 16/10, après remontage des capteurs). Conduite continue sans arrêt : braque vers le côté le plus dégagé, d'autant plus fort que le mur devant est proche. Pas de correction en ligne droite : un centrage essayé le 16/10 le ralentissait. 3 tours de suite sans erreur, 16 s le tour |
| `08_test_vitesse` | Roule 2 s tout droit à plusieurs vitesses : mesurer la distance pour trouver la saturation des servos |
| `_archive` | Anciens doublons |

## Valeurs mesurées

- STOP : gauche 1485 (zone morte 1468-1502), droite 1470 (1455-1488), roues en l'air.
- Pivot à `V_PIVOT = 150` : ~0,29°/ms, ~40 ms perdues au démarrage. Pas de 45° = `T_PAS = 195`.
- Capteur latéral au centre d'un couloir : ~75 mm. Côté ouvert vu depuis un virage : ~110-140 mm (pas 255).
- **Capteurs inclinés vers le haut (16/10)** : montés à plat, le cône du VL6180X touchait le sol (côtés ~55-65 mm,
  avant ~130 mm sans aucun obstacle). Un côté ouvert ressemblait alors à un mur (~70 mm) : toutes les mauvaises
  décisions venaient de là. Après inclinaison : côté ouvert ~170-190, mur à 7 cm ~81, avant libre 255.
  Contrôle avec `01_test_cablage`, robot posé sur la table, rien autour.
- Géométrie : couloir 22 cm, capteurs latéraux à 8 cm l'un de l'autre, nez à 5,5 cm de l'axe des roues
  (coin du nez à 8,5 cm de l'axe : un pivot sur place près d'un mur le fait frotter).
- Servos saturés à `V_DROIT = 45` (450 µs autour du STOP) sous 5 V (batterie USB) : 80 en ligne droite ou des
  virages plus rapides donnent le même temps (16 s) et font frotter. Confirmé par `08_test_vitesse` (2 s tout droit
  à 30/45/60/90 : même distance à partir de 45). Seule piste restante : servos sous 6 V.
- Au sol, le STOP ne suffit pas à immobiliser le robot : arrêt par `detach()` des servos.

`DEBUG 1` : affiche les distances dans le moniteur série. Dans `03`, `DEBUG 1` coupe aussi les moteurs. Passer à 0 pour rouler.

## Câblage

VL6180X : VIN->5V, GND->GND, SDA->A4, SCL->A5. CE : D2 / D3 / D4. Servos : D9 (gauche), D10 (droite). Buzzer : D8.
Débrancher la carte du reste du robot pour téléverser.

## Ligne de commande

```
arduino-cli compile --upload -p COM3 --fqbn arduino:avr:nano:cpu=atmega328 01_test_cablage
```
