/*
 * Robot labyrinthe — trouve les virages tout seul (v3, capteurs latéraux à 90°)
 * Cytron Maker Nano + 3x VL6180X (Pololu #2489) + 2x servos FS90R
 * Bibliothèque requise : "VL6180X" by Pololu
 *
 * Capteurs (2-3 cm du sol, à plat) :
 *   AVANT : centré, droit devant
 *   GAUCHE / DROITE : sur les côtés, à 90°, plutôt vers l'avant du robot
 *
 * Principe :
 *   - en ligne droite : roule et se centre entre les deux murs latéraux
 *   - mur devant : s'arrête, regarde à gauche et à droite, tourne du côté ouvert
 *       les deux ouverts -> COTE_PREFERE ; les deux fermés (cul-de-sac) -> demi-tour
 *   - si après le pivot l'avant est encore bouché, il repart de l'autre côté
 *
 * Câblage :
 *   VL6180X : VIN->5V  GND->GND  SDA->A4  SCL->A5  |  CE : gauche->D2  avant->D3  droite->D4
 *   Servos : signal gauche->D9, droite->D10, rouge -> batterie, masses communes
 */

#include <Wire.h>
#include <VL6180X.h>
#include <Servo.h>
#include <EEPROM.h>

#define DEBUG 0        // 1 = affiche distances et décisions dans le moniteur série (9600)
#define LOG 1          // 1 = enregistre les 20 premiers virages en EEPROM, relus au démarrage sur le moniteur série (9600)
#define TEST_PIVOT 0   // 1 = le robot fait juste 2 pas de pivot (doit faire 90°) toutes les 3 s, pour régler T_PAS

// ---------- Brochage ----------
enum { GAUCHE = 0, AVANT = 1, DROITE = 2 };
const uint8_t PIN_CE[3]  = {2, 3, 4};   // gauche->D2, avant->D3, droite->D4 (vérifié au test câblage)
const uint8_t ADRESSE[3] = {0x30, 0x31, 0x32};
const uint8_t PIN_SERVO_G = 9;
const uint8_t PIN_SERVO_D = 10;
const uint8_t PIN_BUZZER  = 8;   // bip aigu = droite, grave = gauche, 2 bips graves = demi-tour / correction

// ---------- Réglages ----------
const int STOP_G = 1485;         // roue gauche immobile (zone morte 1468-1502, mesure)
const int STOP_D = 1470;         // roue droite immobile (mesuré)
const int V_MAX = 90;           // vitesse en ligne droite (µs autour du STOP)
const int V_MIN = 60;            // vitesse juste avant un mur (zone morte des roues ~±17 : rester bien au-dessus)
const int V_PIVOT = 150;         // vitesse de pivot (mesure faite à l'arrêt entre les pas : elle peut être franche)
const int T_PAS = 195;           // durée d'un pas de pivot de ~45° à V_PIVOT (ms) — à régler avec TEST_PIVOT
const int PAUSE_MESURE = 120;    // arrêt après chaque pas, avant de mesurer (ms)
const int DIST_VIRAGE = 60;      // tourne sous cette distance au mur devant (mm). 30 était trop près : il calait avant d'y arriver
const int DIST_TOUT_DROIT = 100; // plus de correction latérale sous cette distance : les capteurs latéraux voient les coins
const int DIST_RALENTIR = 190;   // commence à ralentir sous cette distance (mm, portée max ~200)
const int AVANT_DEGAGE = 150;    // virage fini quand l'avant voit plus loin que ça (mm)
const int COTE_OUVERT = 100;     // au-delà, pas de mur latéral pour se centrer (mur ~75 mm, ouverture ~110-140 mm mesurés)
const int ECART_MIN = 30;        // au virage, si un côté voit 3 cm plus loin que l'autre, on va de ce côté
const int CONSIGNE_COTE = 70;    // valeur de départ, remplacée dès qu'il voit un mur
const int DIST_MUR_MIN = 40;     // ne s'approche jamais à moins de 4 cm d'un mur latéral
const char COTE_PREFERE = 'D';   // côté choisi quand les deux sont ouverts
const float KP = 1.5;            // gain de la correction de trajectoire

VL6180X capteur[3];
Servo servoG, servoD;
char mode = 'N';            // situation actuelle : 'B' 2 murs, 'G'/'D' un seul mur, 'N' aucun
int ref = CONSIGNE_COTE;    // distance à garder quand un seul mur est vu
int dernierAvant = 0;       // dernière mesure avant, pour le journal

#if LOG
// Journal : octet 0 = nombre de virages, puis 4 octets par virage :
// gauche, droite, avant (mm, max 255), code = décision + 16 * résultat
//   décision : 1 demi-tour, 2 gauche, 3 droite
//   résultat : 1 avant libéré au 1er pivot, 2 libéré au 2e pivot, 3 jamais libéré, 0 non fini
uint8_t nLog = 0;
uint8_t c8(int v) { return v > 255 ? 255 : (v < 0 ? 0 : v); }
void logDebut(int g, int d, uint8_t decision) {
  if (nLog >= 20) return;
  int a = 1 + nLog * 4;
  EEPROM.update(a, c8(g)); EEPROM.update(a + 1, c8(d));
  EEPROM.update(a + 2, c8(dernierAvant)); EEPROM.update(a + 3, decision);
  EEPROM.update(0, ++nLog);
}
void logResultat(uint8_t r) {
  if (nLog == 0 || nLog > 20) return;
  int a = 1 + (nLog - 1) * 4 + 3;
  EEPROM.update(a, EEPROM.read(a) + 16 * r);
}
void logAfficher() {
  uint8_t n = EEPROM.read(0);
  if (n > 20) n = 0;
  Serial.print(F("LOG ")); Serial.print(n); Serial.print(F(" virages, demarrages depuis effacement : "));
  Serial.println(EEPROM.read(81));
  Serial.println(F("gauche droite avant decision resultat"));
  for (uint8_t i = 0; i < n; i++) {
    int a = 1 + i * 4;
    Serial.print(EEPROM.read(a)); Serial.print('\t'); Serial.print(EEPROM.read(a + 1)); Serial.print('\t');
    Serial.print(EEPROM.read(a + 2)); Serial.print('\t');
    Serial.print(EEPROM.read(a + 3) & 15); Serial.print('\t'); Serial.println(EEPROM.read(a + 3) >> 4);
  }
  Serial.println(F("FIN LOG"));
}
#else
void logDebut(int, int, uint8_t) {}
void logResultat(uint8_t) {}
#endif

// g, d : vitesse de chaque roue en µs autour du STOP, positif = marche avant
void moteurs(int g, int d) {
  if (!servoG.attached()) { servoG.attach(PIN_SERVO_G); servoD.attach(PIN_SERVO_D); }
  servoG.writeMicroseconds(STOP_G + g);
  servoD.writeMicroseconds(STOP_D - d);   // servo droit monté en miroir
}

// Arrêt franc : plus aucun signal, le servo ne bouge plus quel que soit le réglage du STOP
// (au sol, la valeur STOP mesurée roues en l'air laissait le robot dériver)
void arret() {
  servoG.detach();
  servoD.detach();
}

int lire(int i) {
  int d = capteur[i].readRangeSingleMillimeters();
  if (capteur[i].timeoutOccurred()) return 999;
  return d;   // 255 = rien à moins de ~20 cm
}

void bip(char sens) {
  tone(PIN_BUZZER, sens == 'D' ? 2500 : 1200, 60);
}

void doubleBip() {
  tone(PIN_BUZZER, 800, 40); delay(80); tone(PIN_BUZZER, 800, 40);
}

// Un pas de pivot (~45°) puis arrêt complet : le capteur mesure robot immobile, sans retard.
void pas(char sens) {
  if (sens == 'G') moteurs(-V_PIVOT, V_PIVOT);
  else             moteurs(V_PIVOT, -V_PIVOT);
  delay(T_PAS);
  arret();
  delay(PAUSE_MESURE);
}

// Pivote par pas de ~45° vers 'sens'. Après chaque pas : arrêt et mesure de l'avant.
// Ne regarde l'avant qu'à partir de pasMin pas (à 45° l'avant voit en biais dans le couloir
// et se croirait déjà libre). Renvoie true si l'avant est libre avant pasMax pas.
bool pivoter(char sens, int pasMin, int pasMax) {
  for (int k = 1; k <= pasMax; k++) {
    pas(sens);
    if (k >= pasMin && max(lire(AVANT), lire(AVANT)) >= AVANT_DEGAGE) return true;
  }
  return false;
}

void virage() {
  arret();
  delay(PAUSE_MESURE);

  // Les capteurs latéraux servent seulement d'indice pour choisir le côté à essayer en premier :
  // au mur, le côté ouvert lit ~110-140 mm (pas 255), le côté fermé ~40-80 mm (journal du 16/10).
  int g = max(lire(GAUCHE), lire(GAUCHE));
  int d = max(lire(DROITE), lire(DROITE));
  char premier;
  if (g - d > ECART_MIN)      premier = 'G';
  else if (d - g > ECART_MIN) premier = 'D';
  else                        premier = COTE_PREFERE;
  char autre = (premier == 'G') ? 'D' : 'G';

#if DEBUG
  Serial.print(F("Mur devant  G=")); Serial.print(g);
  Serial.print(F("  D=")); Serial.print(d);
  Serial.print(F("  -> essaie ")); Serial.println(premier);
#endif
  bip(premier);
  logDebut(g, d, premier == 'G' ? 2 : 3);

  // C'est le capteur avant qui décide, robot à l'arrêt :
  // 1) quart de tour du côté le plus prometteur, l'avant est libre ? on y va
  if (pivoter(premier, 2, 2)) { logResultat(1); moteurs(V_MIN, V_MIN); return; }

  // 2) sinon demi-tour complet vers l'autre côté (180° depuis ici), l'avant est libre ? on y va
  doubleBip();
  if (pivoter(autre, 4, 4)) { logResultat(2); moteurs(V_MIN, V_MIN); return; }

  // 3) sinon cul-de-sac : encore un quart de tour, il repart par où il est venu
  logResultat(3);
  pivoter(autre, 2, 2);
  moteurs(V_MIN, V_MIN);
}

void setup() {
#if DEBUG || LOG
  Serial.begin(9600);
#endif
#if LOG
  logAfficher();        // journal des courses précédentes (cumulatif, effacé par la commande 'c')
  nLog = EEPROM.read(0);
  if (nLog > 20) nLog = 0;
  EEPROM.update(81, EEPROM.read(81) + 1);
#endif
  Wire.begin();
  Wire.setWireTimeout(3000, true);   // évite que la Nano se fige si le bus I2C bloque

  // Les 3 capteurs démarrent à la même adresse I2C (0x29) :
  // tous en veille, puis réveil un par un avec une adresse unique.
  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], OUTPUT);
    digitalWrite(PIN_CE[i], LOW);
  }
  delay(10);

  for (int i = 0; i < 3; i++) {
    pinMode(PIN_CE[i], INPUT);   // réveil par la pull-up du module. Jamais HIGH : CE n'est pas tolérante au 5 V
    delay(10);
    capteur[i].setTimeout(100);
    capteur[i].init();
    capteur[i].configureDefault();
    capteur[i].setAddress(ADRESSE[i]);
    // Mesure plus rapide quand rien n'est en face (49 ms par défaut -> 20 ms) : boucle ~2x plus rapide
    capteur[i].writeReg(VL6180X::SYSRANGE__MAX_CONVERGENCE_TIME, 20);
  }

  servoG.attach(PIN_SERVO_G);
  servoD.attach(PIN_SERVO_D);
  arret();
  delay(2000);   // 2 s pour poser le robot
}

void loop() {
#if TEST_PIVOT
  // Pose le robot au sol : 2 pas = pile un quart de tour. Trop : baisse T_PAS. Pas assez : monte T_PAS.
  pas('G');
  pas('G');
  delay(3000);
  return;
#endif

  int dG  = lire(GAUCHE);
  int dAv = lire(AVANT);
  int dD  = lire(DROITE);
  dernierAvant = dAv;
#if LOG
  if (Serial.available() && Serial.read() == 'c') { nLog = 0; EEPROM.update(0, 0); EEPROM.update(81, 0); }
#endif

#if DEBUG
  Serial.print(dG);  Serial.print('\t');
  Serial.print(dAv); Serial.print('\t');
  Serial.println(dD);
#endif

  // Mur devant : 2 mesures de suite pour y croire (filtre les fausses détections)
  static uint8_t confirm = 0;
  confirm = (dAv < DIST_VIRAGE) ? confirm + 1 : 0;
  if (confirm >= 2) {
    confirm = 0;
    virage();
    mode = 'N';   // nouveau couloir : il réapprendra sa distance au mur
    return;
  }

  // Vitesse : plein gaz, puis ralentit en approchant d'un mur
  int v = map(constrain(dAv, DIST_VIRAGE, DIST_RALENTIR),
              DIST_VIRAGE, DIST_RALENTIR, V_MIN, V_MAX);

  // Correction de trajectoire :
  //  - 2 murs : se centre entre les deux
  //  - 1 seul mur : garde la distance qu'il avait à ce mur au moment où l'autre a disparu
  //    (il roule parallèle au mur au lieu d'aller s'y coller)
  //  - aucun mur : tout droit
  bool murG = dG < COTE_OUVERT;
  bool murD = dD < COTE_OUVERT;
  char nouveau = (murG && murD) ? 'B' : murG ? 'G' : murD ? 'D' : 'N';
  if (nouveau != mode) {                         // on vient de changer de situation
    if (nouveau == 'G') ref = max(dG, DIST_MUR_MIN);
    if (nouveau == 'D') ref = max(dD, DIST_MUR_MIN);
    mode = nouveau;
  }
  int err = 0;                                   // > 0 : il faut tourner à droite
  if (mode == 'B')      err = dD - dG;
  else if (mode == 'G') err = ref - dG;
  else if (mode == 'D') err = dD - ref;

  if (dAv < DIST_TOUT_DROIT) err = 0;           // face au mur : tout droit, sinon il se tortille
  int corr = constrain((int)(KP * err), -v / 3, v / 3);
  moteurs(v + corr, v - corr);
}
