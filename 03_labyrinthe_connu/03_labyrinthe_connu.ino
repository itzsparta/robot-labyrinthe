/*
 * Robot labyrinthe - parcours connu (depart fixe)
 * Cytron Maker Nano + 3x VL6180X (Pololu #2489) + 2x servos FS90R
 * Bibliotheque : "VL6180X" by Pololu
 *
 * Principe :
 *   - roule doucement en se centrant entre les murs (capteurs gauche et droite a 90 deg)
 *   - quand le capteur avant voit un mur proche : s'arrete et pivote du cote indique
 *     par PARCOURS, jusqu'a ce que l'avant voie le couloir suivant (plus de 20 cm)
 *   - apres le dernier virage de la liste : s'arrete au mur suivant (long bip)
 *
 * Cablage :
 *   VL6180X : VIN->5V  GND->GND  SDA->A4  SCL->A5
 *   CE : gauche->D2  avant->D3  droite->D4   (gauche/droite vus de derriere le robot)
 *   Servos : signal gauche->D9, droite->D10
 */

#include <Wire.h>
#include <VL6180X.h>
#include <Servo.h>

#define DEBUG 1   // 1 = affiche G / AVANT / D dans le moniteur serie (9600), les roues ne bougent pas

// ---------- Parcours : un virage par mur rencontre droit devant ----------
const char PARCOURS[] = "GGGGD";
const int NB_VIRAGES = sizeof(PARCOURS) - 1;

// ---------- Brochage ----------
enum { GAUCHE = 0, AVANT = 1, DROITE = 2 };
const uint8_t PIN_CE[3]  = {2, 3, 4};   // {gauche, avant, droite}
const uint8_t ADRESSE[3] = {0x30, 0x31, 0x32};
const uint8_t PIN_SERVO_G = 9;
const uint8_t PIN_SERVO_D = 10;
const uint8_t PIN_BUZZER  = 8;

// ---------- Reglages ----------
const int STOP_G = 1485;        // roue gauche immobile (zone morte 1468-1502, mesure)
const int STOP_D = 1470;        // roue droite immobile (mesure)
const int V_ROULE = 90;         // vitesse en ligne droite (lent, pour laisser le temps aux capteurs)
const int V_LENT  = 45;         // vitesse en arrivant pres du mur
const int V_PIVOT = 70;         // vitesse pendant les virages
const int DIST_VIRAGE   = 70;   // mm : tourne quand le mur devant est plus pres (mesure au centre des virages)
const int DIST_RALENTIR = 160;  // mm : commence a ralentir
const int MUR_VU    = 100;      // mm : un capteur lateral "voit un mur" en dessous de cette valeur
const int CENTRE    = 70;       // mm : distance a chaque mur quand le robot est centre (couloir de 22 cm)
const int TROP_PRES = 35;       // mm : trop pres d'un mur lateral -> s'ecarte franchement
const float KP = 1.0;           // force de la correction de trajectoire
const int PIVOT_MIN = 200;      // ms : duree minimum d'un virage
const int PIVOT_MAX = 1500;     // ms : securite si l'avant ne se degage jamais

VL6180X capteur[3];
Servo servoG, servoD;
int etape = 0;

// g, d : vitesse de chaque roue autour du STOP, positif = marche avant
void moteurs(int g, int d) {
  servoG.writeMicroseconds(STOP_G + g);
  servoD.writeMicroseconds(STOP_D - d);   // servo droit monte en miroir
}

int lire(int i) {
  int d = capteur[i].readRangeSingleMillimeters();
  if (capteur[i].timeoutOccurred()) return 255;
  return d;   // 255 = rien a moins de ~20 cm
}

// Pivote jusqu'a ce que le capteur avant voie loin (le couloir suivant)
void tourner(char sens) {
  moteurs(0, 0);
  delay(100);
  tone(PIN_BUZZER, sens == 'D' ? 2500 : 1200, 80);   // aigu = droite, grave = gauche

  if (sens == 'G') moteurs(-V_PIVOT, V_PIVOT);
  else             moteurs(V_PIVOT, -V_PIVOT);

  unsigned long t0 = millis();
  delay(PIVOT_MIN);
  int libre = 0;
  while (millis() - t0 < PIVOT_MAX) {
    if (lire(AVANT) >= 200) { if (++libre >= 2) break; }   // 2 mesures "loin" de suite
    else libre = 0;
  }
  moteurs(0, 0);
  delay(100);
}

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif
  Wire.begin();
  Wire.setWireTimeout(3000, true);

  // Les 3 capteurs demarrent a la meme adresse I2C : on les reveille un par un
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], OUTPUT);
    digitalWrite(PIN_CE[i], LOW);
  }
  delay(10);
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], INPUT);   // reveil par la resistance du module (jamais HIGH : CE ne supporte pas le 5 V)
    delay(10);
    capteur[i].setTimeout(100);
    capteur[i].init();
    capteur[i].configureDefault();
    capteur[i].setAddress(ADRESSE[i]);
    capteur[i].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);   // mesure plus rapide
  }

#if !DEBUG
  servoG.attach(PIN_SERVO_G);
  servoD.attach(PIN_SERVO_D);
  moteurs(0, 0);
#endif
  tone(PIN_BUZZER, 2000, 150);
  delay(2000);   // 2 s pour poser le robot au depart
}

void loop() {
  int dG  = lire(GAUCHE);
  int dAv = lire(AVANT);
  int dD  = lire(DROITE);

#if DEBUG
  Serial.print(F("G=")); Serial.print(dG);
  Serial.print(F("\tAVANT=")); Serial.print(dAv);
  Serial.print(F("\tD=")); Serial.println(dD);
  delay(150);
  return;
#endif

  // Mur devant : 2 mesures de suite pour y croire
  static uint8_t murDevant = 0;
  murDevant = (dAv < DIST_VIRAGE) ? murDevant + 1 : 0;
  if (murDevant >= 2) {
    murDevant = 0;
    if (etape >= NB_VIRAGES) {          // parcours fini : arrivee
      moteurs(0, 0);
      tone(PIN_BUZZER, 1500, 800);
      while (true) {}
    }
    tourner(PARCOURS[etape++]);
    return;
  }

  // Vitesse : ralentit en approchant du mur
  int v = map(constrain(dAv, DIST_VIRAGE, DIST_RALENTIR),
              DIST_VIRAGE, DIST_RALENTIR, V_LENT, V_ROULE);

  // Correction de trajectoire (err > 0 : tourner a droite)
  bool murG = dG < MUR_VU;
  bool murD = dD < MUR_VU;
  int err = 0;
  if (murG && murD) err = dD - dG;       // deux murs : se centre
  else if (murG)    err = CENTRE - dG;   // un seul mur a gauche : reste a 7 cm
  else if (murD)    err = dD - CENTRE;   // un seul mur a droite : reste a 7 cm
  if (dG < TROP_PRES) err = max(err, 40);
  if (dD < TROP_PRES) err = min(err, -40);

  int corr = constrain((int)(KP * err), -v / 2, v / 2);
  moteurs(v + corr, v - corr);
}
