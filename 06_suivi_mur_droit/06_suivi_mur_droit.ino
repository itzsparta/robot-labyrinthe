/*
 * Robot labyrinthe - suivi du mur droit (règle de la main droite)
 * Cytron Maker Nano + 3x VL6180X (Pololu #2489) + 2x servos FS90R
 * Bibliothèques : "VL6180X" by Pololu, "Servo"
 *
 * Principe, dans cet ordre de priorité :
 *   1. le mur droit disparaît  -> avance jusqu'au milieu de l'ouverture, quart de tour à droite
 *   2. mur devant              -> quart de tour à gauche (deux de suite = demi-tour en cul-de-sac)
 *   3. sinon                   -> avance en gardant CONSIGNE mm entre le capteur droit et le mur
 * Les virages se font par pas de ~45°, robot arrêté entre deux pas pour mesurer sans retard.
 *
 * Câblage (vérifié le 09/10) :
 *   VL6180X : VIN->5V  GND->GND  SDA->A4  SCL->A5  |  CE : gauche->D2  avant->D3  droite->D4
 *   Servos : gauche->D9, droite->D10  |  Buzzer : D8
 *
 * Bips : aigu = virage à droite, grave = virage à gauche
 */

#include <Wire.h>
#include <VL6180X.h>
#include <Servo.h>
#include <EEPROM.h>

#define DEBUG 0   // 1 = affiche G / AVANT / D et les décisions sur le moniteur série (9600)

// ---------- Brochage ----------
enum { GAUCHE = 0, AVANT = 1, DROITE = 2 };
const uint8_t PIN_CE[3]  = {2, 3, 4};
const uint8_t ADRESSE[3] = {0x30, 0x31, 0x32};
const uint8_t PIN_SERVO_G = 9;
const uint8_t PIN_SERVO_D = 10;
const uint8_t PIN_BUZZER  = 8;

// ---------- Réglages mesurés ----------
const int STOP_G = 1485;        // milieu de la zone morte roue gauche (1468-1502)
const int STOP_D = 1470;        // milieu de la zone morte roue droite (1455-1488)
const int V_PIVOT = 110;        // vitesse des roues pendant un petit pas de pivot
const int T_PAS = 90;           // un petit pas de pivot = ~15° (à 150 : 195 ms faisaient ~45° sur table, plus sur la piste)
const int PAS_MIN = 4;          // petits pas avant de regarder l'avant (~60° : en biais il se croirait libre trop tôt)
const int PAS_MAX = 12;         // ~180° au plus
const int PAS_SONDE_D = 7;      // ~105° : jusqu'où il regarde à droite face à un mur avant de renoncer
const int PAUSE_MESURE = 120;   // arrêt après un pas, avant de mesurer (ms)

// ---------- Réglages à ajuster sur la piste ----------
const int V_SUIVI = 110;         // vitesse en ligne droite (µs autour du STOP, zone morte ~17)
const int CONSIGNE = 75;         // distance capteur droit -> mur droit (mm), mesurée au centre d'un couloir
const int MUR_PERDU = 110;       // au-delà : plus de mur à droite (ouverture lue ~110-140 mm)
const int MUR_DEVANT = 45;       // en dessous : mur devant, il faut tourner à gauche (mm)
const int AVANT_LIBRE = 150;     // après un pivot, l'avant est libre au-delà de ça (mm)
const float KP = 0.4;            // correction proportionnelle à l'écart au mur (0.8 le mettait de travers)
const float KD = 0.6;            // freine les oscillations (2.0 amplifiait le bruit de mesure)
const int T_AVANCE_OUVERTURE = 450;  // ms d'avance tout droit avant de tourner dans une ouverture à droite
const int T_REPRISE = 700;           // ms d'avance tout droit après ce virage, pour retrouver le mur droit

// Parcours connu : sens du virage à chaque mur devant, dans l'ordre. Chaîne vide = règle de la main droite.
// Les capteurs ne distinguent pas l'ouverture du 5e virage (capteur droit ~100 mm, comme aux coins à gauche).
const char PARCOURS[] = "GGGGD";
const uint8_t NB_PARCOURS = sizeof(PARCOURS) - 1;
uint8_t iParcours = 0;

VL6180X capteur[3];
Servo servoG, servoD;
int ecartPrec = 0;

// ---------- Journal des virages (EEPROM, cumulatif, relu au démarrage à 9600 bauds) ----------
// Octet 0 : nombre de virages, octet 1 : démarrages depuis effacement ('c' sur le port série = effacer)
// Par virage, 17 octets : type ('G' mur devant, 'D' ouverture droite) puis les 8 dernières paires (avant, droite)
const int NB_HIST = 8, NB_EVT = 8, TAILLE_EVT = 1 + 2 * NB_HIST;
uint8_t histAv[NB_HIST], histD[NB_HIST];
uint8_t iHist = 0, nEvt = 0;

uint8_t c8(int v) { return v > 255 ? 255 : (v < 0 ? 0 : v); }

void journalMesure(int av, int d) {
  histAv[iHist] = c8(av);
  histD[iHist] = c8(d);
  iHist = (iHist + 1) % NB_HIST;
}

void journalVirage(char type) {
  if (nEvt >= NB_EVT) return;
  int a = 2 + nEvt * TAILLE_EVT;
  EEPROM.update(a, type);
  for (int k = 0; k < NB_HIST; k++) {          // de la plus ancienne à la plus récente
    int j = (iHist + k) % NB_HIST;
    EEPROM.update(a + 1 + 2 * k, histAv[j]);
    EEPROM.update(a + 2 + 2 * k, histD[j]);
  }
  EEPROM.update(0, ++nEvt);
}

void journalAfficher() {
  uint8_t n = EEPROM.read(0);
  if (n > NB_EVT) n = 0;
  Serial.print(F("LOG ")); Serial.print(n);
  Serial.print(F(" virages, demarrages : ")); Serial.println(EEPROM.read(1));
  for (uint8_t i = 0; i < n; i++) {
    int a = 2 + i * TAILLE_EVT;
    Serial.print((char)EEPROM.read(a)); Serial.print(F("  avant/droite :"));
    for (int k = 0; k < NB_HIST; k++) {
      Serial.print(' '); Serial.print(EEPROM.read(a + 1 + 2 * k));
      Serial.print('/'); Serial.print(EEPROM.read(a + 2 + 2 * k));
    }
    Serial.println();
  }
  Serial.println(F("FIN LOG"));
}

// g, d : vitesse de chaque roue autour du STOP, positif = marche avant
void moteurs(int g, int d) {
  if (!servoG.attached()) { servoG.attach(PIN_SERVO_G); servoD.attach(PIN_SERVO_D); }
  servoG.writeMicroseconds(STOP_G + g);
  servoD.writeMicroseconds(STOP_D - d);   // servo droit monté en miroir
}

// Arrêt franc : plus de signal, le servo ne bouge plus quel que soit le STOP
void arret() {
  servoG.detach();
  servoD.detach();
}

// Renvoie la distance en mm (255 = rien à moins de ~20 cm), ou -1 si le capteur ne répond pas.
// Un capteur muet ne doit JAMAIS être pris pour "pas de mur" : sinon le robot tourne en rond.
int lire(int i) {
  int d = capteur[i].readRangeSingleMillimeters();
  if (capteur[i].timeoutOccurred()) return -1;
  return d;
}

// Réveil des capteurs un par un avec une adresse unique (au démarrage, et si un capteur a redémarré
// tout seul, par exemple sur une chute de tension des servos : il repart alors à l'adresse 0x29)
void initCapteurs() {
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], OUTPUT);
    digitalWrite(PIN_CE[i], LOW);
  }
  delay(10);
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], INPUT);   // jamais HIGH : CE ne supporte pas le 5 V
    delay(10);
    capteur[i].setTimeout(100);
    capteur[i].init();
    capteur[i].configureDefault();
    capteur[i].setAddress(ADRESSE[i]);
    capteur[i].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);
  }
}

// Un pas de pivot de ~45°, puis arrêt
void pas(char sens) {
  if (sens == 'G') moteurs(-V_PIVOT, V_PIVOT);
  else             moteurs(V_PIVOT, -V_PIVOT);
  delay(T_PAS);
  arret();
  delay(PAUSE_MESURE);
}

// Pivote par petits pas vers 'sens' jusqu'à ce que l'avant soit libre (au moins pasMin pas, au plus pasMax).
// C'est le capteur qui arrête le virage, pas un temps : l'angle d'un pas varie avec le sol et la batterie.
// Renvoie le nombre de pas faits si l'avant s'est libéré, 0 sinon.
int tourner(char sens, int pasMin, int pasMax) {
  tone(PIN_BUZZER, sens == 'D' ? 2500 : 1200, 60);
  for (int k = 1; k <= pasMax; k++) {
    pas(sens);
    if (k >= pasMin && max(lire(AVANT), lire(AVANT)) >= AVANT_LIBRE) return k;
  }
  return 0;
}

void toutDroit(int ms) {
  moteurs(V_SUIVI, V_SUIVI);
  delay(ms);
}

void setup() {
  Serial.begin(9600);
  journalAfficher();
  nEvt = EEPROM.read(0);
  if (nEvt > NB_EVT) nEvt = 0;
  EEPROM.update(1, EEPROM.read(1) + 1);
  Wire.begin();
  Wire.setWireTimeout(3000, true);

  initCapteurs();

  arret();
  tone(PIN_BUZZER, 2000, 150);
  delay(2000);   // 2 s pour poser le robot, mur à sa droite
}

void loop() {
  int dAv = lire(AVANT);
  int dD  = lire(DROITE);

#if DEBUG
  Serial.print(F("AVANT=")); Serial.print(dAv);
  Serial.print(F("\tD=")); Serial.println(dD);
#endif

  journalMesure(dAv, dD);   // 0 dans le journal = timeout
  if (Serial.available() && Serial.read() == 'c') { nEvt = 0; EEPROM.update(0, 0); EEPROM.update(1, 0); }

  // Capteur muet (-1) : ne rien décider, sinon -1 passe pour "mur devant" (< MUR_DEVANT).
  // 5 fois de suite : arrêt, bip long, réinitialisation des capteurs.
  static uint8_t muet = 0;
  if (dAv < 0 || dD < 0) {
    if (++muet >= 5) {
      arret();
      tone(PIN_BUZZER, 400, 600);
      initCapteurs();
      muet = 0;
      delay(700);
    }
    return;
  }
  muet = 0;

  // 1. Plus de mur à droite (2 mesures de suite) : on prend l'ouverture
  static uint8_t perdu = 0;
  perdu = (NB_PARCOURS == 0 && dD > MUR_PERDU) ? perdu + 1 : 0;   // parcours connu : seuls les murs devant comptent
  if (perdu >= 2) {
    perdu = 0;
#if DEBUG
    Serial.println(F("-> ouverture a droite"));
#endif
    journalVirage('D');
    toutDroit(T_AVANCE_OUVERTURE);   // amène les roues au milieu de l'ouverture
    arret();
    delay(PAUSE_MESURE);
    tourner('D', PAS_MIN, PAS_MAX);
    toutDroit(T_REPRISE);            // entre dans le nouveau couloir, le mur droit réapparaît
    ecartPrec = 0;
    return;
  }

  // 2. Mur devant (3 mesures de suite) : sonde à droite, sinon pivote à gauche jusqu'à ce que l'avant soit libre
  static uint8_t devant = 0;
  devant = (dAv < MUR_DEVANT) ? devant + 1 : 0;
  if (devant >= 3) {
    devant = 0;
    arret();
    delay(PAUSE_MESURE);
    if (NB_PARCOURS > 0) {
      if (iParcours >= NB_PARCOURS) {          // parcours fini : arrêt définitif
        tone(PIN_BUZZER, 3000, 1000);
        while (true) {}
      }
      char sens = PARCOURS[iParcours++];
      journalVirage(sens);
      tourner(sens, PAS_MIN, PAS_MAX);
      ecartPrec = 0;
      return;
    }
    journalVirage('G');
#if DEBUG
    Serial.println(F("-> mur devant, gauche"));
#endif
    // Main droite : on sonde d'abord à droite avec le capteur avant (le capteur droit lit ~78 mm
    // dans certaines ouvertures, indiscernable d'un mur). Libre : on y va. Sinon on revient et on part à gauche.
    if (tourner('D', PAS_MIN, PAS_SONDE_D) == 0)
      tourner('G', PAS_SONDE_D + PAS_MIN, PAS_SONDE_D + PAS_MAX);
    ecartPrec = 0;
    return;
  }

  // 3. Ligne droite en gardant CONSIGNE mm au mur droit (ecart > 0 : trop loin, tourner à droite)
  int ecart = dD - CONSIGNE;
  int corr = (int)(KP * ecart + KD * (ecart - ecartPrec));
  ecartPrec = ecart;
  corr = constrain(corr, -V_SUIVI / 4, V_SUIVI / 4);   // braquage doux : il reste dans l'axe du couloir
  moteurs(V_SUIVI + corr, V_SUIVI - corr);
}
