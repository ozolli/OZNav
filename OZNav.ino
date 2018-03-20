/*
 * OZNav.ino
 *
 * Emet en nmea à 38400bds les phrases suivantes :
 *
 * A 5Hz :
 * IIXDR Heel
 * IIXDR Pitch
 * IIHDM (NXP)
 * IIHDT (NXP)
 *
 * A 1Hz :
 * HEHDM (Tacktick fallback)
 * HEHDT (Tacktick fallback)
 * IIMMB
 * IIMTA
 * IIMWV T
 * IIMWV R
 * IIMWD
 * IIVWR
 * IIVHW
 * IIVDR
 * POZPE
 * POZTX
 *
 * L'accéléromètre est calibré avec OZNav_Calib_Accel
 * Le magnétomètre est calibré en 3D avec OZNav_Calib_Mag
 * puis en 2D in-situ (dans le bateau) avec OZNav_Calib_Dev
 *
 * Le Miniplex doit rediriger IIVWR et IIVHW du Tacktick vers l'Arduino pour calculer
 * le vent apparent corrigé de la gite, le vent réel corrigé et le courant.

$IIVWR,030,L,15,N,,,,*66
$IIVHW,,,63,M,05.8,N,,*5C
 */

 // La colorisation syntaxique est dans
 // C:\Users\Olivier\Documents\Arduino\libraries\elapsedMillis-master\keywords.txt

// A commenter pour sortir sur RS422
#define DEBUG

// A commenter quand Tacktick connecté
#define NO_TACKTICK

// A commenter quand GPS connecté
//#define NO_GPS

#pragma GCC optimize ("-O1")

#include <TinyGPS++.h>
#include <elapsedMillis.h>
#include "boat.h"
#include "baro.h"
#include "compas.h"
#include "polar.h"
#include "IO.h"
#include "filtre.h"

// Phrases NMEA (longueur max 82 caractères en incluant <CR><LF> selon norme)
// Tolérance 150 caractères en entrée
char nmeaOut[80] = "";
char nmeaGPS[150] = "";
char nmeaLog[200] = "";

// Compteurs de temps
// On démarre 5 compteurs 1Hz en décalé pour fluidifier le calcul et la sortie
// Le compteur 5Hz est intercalé entre les compteurs 1Hz
elapsedMillis tick_100 = 0;
elapsedMillis tick_5 = 100;
elapsedMillis tick_1_200 = 200;
elapsedMillis tick_1_400 = 400;
elapsedMillis tick_1_600 = 600;
elapsedMillis tick_1_800 = 800;
elapsedMillis tick_1_1000 = 0;
elapsedMillis tick_1_acc = 0;
elapsedMillis une_min = 0;
elapsedMillis timeVWR = 0;
elapsedMillis timeVHW = 0;
elapsedMillis timeGPS = 0;
FPB Fcog, Fset;

// Données du bateau et de son environnement. Voir boat.h pour la structure
Boat mus;

// Objets nmea GGA, RMC, VWR et VHW
TinyGPSPlus tgps;
struct {
  TinyGPSCustom awa;
  TinyGPSCustom LR;
  TinyGPSCustom aws;
  TinyGPSCustom hdg;
  TinyGPSCustom bsp;
} nmea;

void setup(void) {
  nmea.awa.begin(tgps, "IIVWR", 1);
  nmea.LR.begin(tgps, "IIVWR", 2);
  nmea.aws.begin(tgps, "IIVWR", 3);
  nmea.hdg.begin(tgps, "IIVHW", 3);
  nmea.bsp.begin(tgps, "IIVHW", 5);
  Serial.begin(38400);
  RS422.begin(38400);
  sdcard_init();
  imu_init();
  baro_init();
}

void hdg_heel_pitch() {
  // Acquisition et calcul cap, gite et tangage
  // Gite positive vers la droite
  // Tangage positif vers l'avant

  boolean log_it = false;

  int hdgr = (int)round(mus.imu.d_hdg);
  if (hdgr == 360) hdgr = 0;
  int hdtr = (int)round(mus.imu.d_hdt);
  if (hdtr == 360) hdtr = 0;

  if (tick_1_acc > 1000) {
    tick_1_acc = tick_1_acc - 1000;
    log_it = true;
  }

  sprintf(nmeaOut, "$IIXDR,A,%i,D,Heel", (int)round(mus.imu.d_roll));
  send_nmea(add_checksum(nmeaOut));
  if (log_it) {
    strcpy(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);
  }

  sprintf(nmeaOut, "$IIXDR,A,%i,D,Pitch", (int)round(mus.imu.d_pitch));
  send_nmea(add_checksum(nmeaOut));
  if (log_it) {
    strcat(nmeaLog, "\r\n");
    strcat(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);
  }

  sprintf(nmeaOut, "$IIHDM,%i,M", hdgr);
  send_nmea(add_checksum(nmeaOut));
  if (log_it) {
    strcat(nmeaLog, "\r\n");
    strcat(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);
  }

  sprintf(nmeaOut, "$IIHDT,%i,T", hdtr);
  send_nmea(add_checksum(nmeaOut));
  if (log_it) {
    strcat(nmeaLog, "\r\n");
    strcat(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);
    add_to_log(nmeaLog);
    log_it = false;
  }
}

void baro_temp() {
  // Acquisition pression atmosphérique et température air

  double T  = 0.0f;
  double p0 = 0.0f;
  baro_query(T, p0);

  sprintf(nmeaOut, "$IIXDR,P,%s,B,Barometer", d2s((p0 + mus.cal.baro) / 100000.0f, 7, 5));
  send_nmea(add_checksum(nmeaOut));
  strcpy(nmeaLog, timestamp());
  strcat(nmeaLog, nmeaOut);

  sprintf(nmeaOut, "$IIXDR,C,%s,C,TempAir", d2s(T, 3, 1));
  send_nmea(add_checksum(nmeaOut));
  strcat(nmeaLog, "\r\n");
  strcat(nmeaLog, timestamp());
  strcat(nmeaLog, nmeaOut);
  add_to_log(nmeaLog);
}

void aw() {
  // Compensation et envoi awa et aws en fonction de la gite
  // Traite $IIVWR

  // AWA
  #ifdef NO_TACKTICK
    mus.wind.d_awa = 148.0f;
  #else
    mus.wind.d_awa = atof(nmea.awa.value());
  #endif

  if (mus.wind.d_awa == 90.0f) mus.wind.d_awa += 0.001f;
  mus.wind.r_awa = mus.wind.d_awa * DEG_TO_RAD;
  mus.wind.r_awa_cor = atan(tan(mus.wind.r_awa) / cos(mus.imu.r_roll));
  if (mus.wind.r_awa >= 0.0f) {
    if (mus.wind.r_awa > M_PI / 2.0f) mus.wind.r_awa_cor += M_PI;
  } else if (mus.wind.r_awa < -M_PI / 2.0f) mus.wind.r_awa_cor -= M_PI;

  // Sens du vent : Left ou Right
  #ifdef NO_TACKTICK
    mus.wind.LR = 'R';
  #else
    mus.wind.LR = nmea.LR.value()[0];
  #endif
  
  // AWA_REL
  if (mus.wind.LR == 'L') mus.wind.d_awa_rel = 360.0 - mus.wind.r_awa_cor * RAD_TO_DEG;
  else if (mus.wind.LR == 'R') mus.wind.d_awa_rel = mus.wind.r_awa_cor * RAD_TO_DEG;

  // AWD
  if (mus.wind.LR == 'L') mus.wind.r_awd = mus.imu.r_hdg - mus.wind.r_awa_cor;
  else if (mus.wind.LR == 'R') mus.wind.r_awd = mus.imu.r_hdg + mus.wind.r_awa_cor;

  // AWS
  #ifdef NO_TACKTICK
    mus.wind.aws = 14.0f;
  #else
    mus.wind.aws = atof(nmea.aws.value());
  #endif
  
  sprintf(nmeaOut, "$IIVWR,%i,%c,%s,N,,,,", (int)round(mus.wind.r_awa_cor * RAD_TO_DEG),
                                            mus.wind.LR, d2s(mus.wind.aws, 4, 2));
  send_nmea(add_checksum(nmeaOut));
  strcpy(nmeaLog, timestamp());
  strcat(nmeaLog, nmeaOut);

  sprintf(nmeaOut, "$IIMWV,%i,R,%s,N,A", (int)round(mus.wind.d_awa_rel), d2s(mus.wind.aws, 4, 2));
  send_nmea(add_checksum(nmeaOut));
  strcat(nmeaLog, "\r\n");
  strcat(nmeaLog, timestamp());
  strcat(nmeaLog, nmeaOut);
  add_to_log(nmeaLog);
}

void bsp_tw_vmg() {
  // Calcul et compensation twa, tws et vmg en fonction de la dérive dûe au vent
  // Calcul twd et compensation en fonction du courant et envoi
  // Traite $IIVHW (BSP)
  // HDM non traité pour l'instant (nmea.hdg.value())
 
  // BSP
  #ifdef NO_TACKTICK
    mus.speedo.bsp = 6.32f;
  #else
    mus.speedo.bsp = atof(nmea.bsp.value());
  #endif

  if (isnan(mus.speedo.bsp) || mus.speedo.bsp == 0.0) mus.speedo.bsp = 0.001;

  if (mus.wind.hasVWR) {

    // Leeway linéaire de 0° pour AWA 180 jusqu'a mus.cal.leeway pour AWA 30
    // Correction AWS (dérive majorée) et BSP (dérive minorée)
    // A essayer avec mus.cal.leeway = 8
    double bsp = mus.speedo.bsp;
    if (bsp < 0.5f) bsp = 0.5f;
    double leeway = (M_PI - mus.wind.r_awa_cor) * mus.cal.leeway / 2.618f;
    leeway = leeway * sqrt(mus.wind.aws) / 4.0f;
    leeway = leeway / (sqrt(bsp) / 2.25f) * DEG_TO_RAD;

    // TWS
    mus.wind.tws = sqrt(mus.speedo.bsp * mus.speedo.bsp + mus.wind.aws * mus.wind.aws - 2.0f *
                        mus.speedo.bsp * mus.wind.aws * cos(mus.wind.r_awa_cor + leeway));

    // TWA
    mus.wind.r_twa = mus.wind.r_awa_cor + leeway + acos((mus.wind.aws * mus.wind.aws + mus.wind.tws *
                mus.wind.tws - mus.speedo.bsp * mus.speedo.bsp) / (2.0f * mus.wind.tws * mus.wind.aws));
    if (isnan(mus.wind.r_twa)) mus.wind.r_twa = 0.001f;
    mus.wind.d_twa = mus.wind.r_twa * RAD_TO_DEG;

    // TWA_REL
    if (mus.wind.LR == 'L') mus.wind.d_twa_rel = 360.0f - mus.wind.d_twa;
    else if (mus.wind.LR == 'R') mus.wind.d_twa_rel = mus.wind.d_twa;

    // VMG
    mus.speedo.vmg = mus.speedo.bsp * cos(mus.wind.r_twa);
    if (mus.speedo.vmg < 0.0f) mus.speedo.vmg = -mus.speedo.vmg;

    sprintf(nmeaOut, "$IIMWV,%i,T,%s,N,A", (int)round(mus.wind.d_twa_rel), d2s(mus.wind.tws, 4, 2));
    send_nmea(add_checksum(nmeaOut));
    strcpy(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);

    // On recrée VHW avec le mus.imu.d_hdg corrigé
    sprintf(nmeaOut, "$IIVHW,%i,T,%i,M,%s,N,,K", (int)round(mus.imu.d_hdt),
                                                 (int)round(mus.imu.d_hdg), d2s(mus.speedo.bsp, 4, 2));
    send_nmea(add_checksum(nmeaOut));
    strcat(nmeaLog, "\r\n");
    strcat(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);

    // Le calcul de TWD doit se faire en prenant en compte COG et SOG
    // pour ajouter le vecteur courant et avoir le vent fond (GWD).
    // Cela permet de comparer le vent calculé avec la prévision météo
    // et de ne pas avoir un TWD différent selon qu'on remonte ou qu'on
    // descende le courant de marée.
    // Il ne faut pas le faire pour TWA et TWS car c'est le vent surface
    // qui nous importe pour les polaires.
    // Quand on n'a pas de fix GPS on calcule le TWD surface 

    //TWD fond (GWD)   
    if (mus.gps.hasGPS) {
      double u = mus.gps.sog * sin(mus.gps.r_cog) - mus.wind.aws * sin(mus.wind.r_awd);
      double v = mus.gps.sog * cos(mus.gps.r_cog) - mus.wind.aws * cos(mus.wind.r_awd);
      mus.wind.r_twd = atan(u / v);
      if (isnan(mus.wind.r_twd)) mus.wind.r_twd = 0.0f;
      if (mus.wind.r_twd < 0.0f) mus.wind.r_twd += M_PI * 2.0f;
      else if (mus.wind.r_twd >= M_PI * 2.0f) mus.wind.r_twd -= M_PI * 2.0f;
    }

    // TWD surface
    else {
      if (mus.wind.LR == 'L') mus.wind.r_twd = mus.imu.r_hdg - mus.wind.r_twa;
      else if (mus.wind.LR == 'R') mus.wind.r_twd = mus.imu.r_hdg + mus.wind.r_twa;
    }

    int twdtr = (int)round(mus.wind.r_twd * RAD_TO_DEG + mus.cal.mag);
    if (twdtr < 0) twdtr += 360;
    else if (twdtr > 360) twdtr -= 360;
    int twdmr = (int)round(mus.wind.r_twd * RAD_TO_DEG);
    if (twdmr == 360) twdmr = 0;
    
    sprintf(nmeaOut, "$IIMWD,%i,T,%i,M,%s,N,,", twdtr, twdmr, d2s(mus.wind.tws, 4, 2));
    send_nmea(add_checksum(nmeaOut));
    strcat(nmeaLog, "\r\n");
    strcat(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);
    add_to_log(nmeaLog);
  }
}

void set_drift() {
  // Calcul et envoi du courant
  // La dérive dûe au vent n'est pas soustraite du résultat, elle est donc incluse
  // Traite $GPRMC

  double raw_sog = 0.0f;
  double raw_cog = 0.0f;
  double raw_drift = 0.0f;

  #ifdef NO_GPS
    raw_sog = mus.speedo.bsp + 0.5f;
  #else
    raw_sog = tgps.speed.knots();
  #endif

  // Lissage exponentiel simple
  mus.gps.sog = raw_sog * mus.cal.alpha_sogcog + (mus.gps.sog * (1.0 - mus.cal.alpha_sogcog));
  //Serial.print(F("\tmus.gps.sog :\t")); Serial.println(mus.gps.sog);

  // COG
  #ifdef NO_GPS
    raw_cog = mus.imu.d_hdt;
  #else
    raw_cog = tgps.course.deg();
  #endif
  
  raw_cog *= DEG_TO_RAD;
  mus.gps.r_cog = filtre(Fcog, raw_cog, mus.cal.alpha_sogcog);
  if (mus.gps.r_cog < 0.0f) mus.gps.r_cog += M_PI * 2.0f;
  else if (mus.gps.r_cog >= M_PI * 2.0f) mus.gps.r_cog -= M_PI * 2.0f;

  mus.gps.d_cog = mus.gps.r_cog * RAD_TO_DEG;

  if (mus.speedo.hasVHW) {
    double vlat = raw_sog * cos(raw_cog) - mus.speedo.bsp * cos(mus.imu.r_hdt);
    double vlong = raw_sog * sin(raw_cog) - mus.speedo.bsp * sin(mus.imu.r_hdt);

    // Drift
    raw_drift = sqrt(vlong * vlong + vlat * vlat);

    // Lissage exponentiel simple
    mus.gps.drift = raw_drift * mus.cal.alpha_setdrift + (mus.gps.drift * (1.0f - mus.cal.alpha_setdrift));

    // Set
    mus.gps.r_set = filtre(Fset, vlong, vlat, mus.cal.alpha_setdrift);

    if (mus.gps.r_set < 0.0f) mus.gps.r_set += M_PI * 2.0f;
    else if (mus.gps.r_set >= M_PI * 2.0f) mus.gps.r_set -= M_PI * 2.0f;
    mus.gps.d_set = mus.gps.r_set * RAD_TO_DEG;
    //Serial.print(F("\tmus.gps.d_set :\t")); Serial.println(mus.gps.d_set);

    int setr = (int)round(mus.gps.d_set);
    if (setr == 360) setr = 0;
    int setmr = (int)round(mus.gps.d_set + mus.cal.mag);
    if (setmr == 360) setmr = 0;

    sprintf(nmeaOut, "$IIVDR,%i,T,%i,M,%s,N", setmr, setr, d2s(mus.gps.drift, 4, 2));
    send_nmea(add_checksum(nmeaOut));
    strcpy(nmeaLog, timestamp());
    strcat(nmeaLog, nmeaOut);
    add_to_log(nmeaLog);
  }
}

int tack() {
  // Retourne le cap sur l'autre bord

  double a = 0.0f;
  if (mus.wind.LR == 'L') a = mus.imu.d_hdg - mus.wind.d_twa * 2.0f;
  else if (mus.wind.LR == 'R') a = mus.imu.d_hdg + mus.wind.d_twa * 2.0f;
  if (a < 0.0) a += 360.0f;
  else if (a >= 360.0f) a -= 360.0f;
  int r = (int)round(a);
  if (r == 360) r = 0;
  return r;
}

void polar_perf() {
  // Calcul de vitesse cible, meilleur angle et vmg correspondant

  byte btwa, ppres, pport;

  polaropt(mus, btwa, ppres, pport);
  sprintf(nmeaOut, "$POZPE,%s,%i,%i,%i,%i", d2s(polarbsp(mus), 4, 2), tack(), btwa, ppres, pport);
  send_nmea(add_checksum(nmeaOut));
  strcpy(nmeaLog, timestamp());
  strcat(nmeaLog, nmeaOut);
  add_to_log(nmeaLog);
}

void loop() {
  
  // Ces 3 lignes à supprimer quand les tests seront finis
  mus.gps.hasGPS = true;
  mus.wind.hasVWR = true;
  mus.speedo.hasVHW = true;

  // Réception et traitement des phrases nmea entrantes
  while (RS422.available() > 0) {
    char c = RS422.read();
    tgps.encode(c);

    // Ajout au log des phrases nmea entrantes avec timestamp
    if (readGPS(c, nmeaGPS, sizeof(nmeaGPS)) > 0) {
      strcpy(nmeaLog, timestamp());
      strcat(nmeaLog, nmeaGPS);
      add_to_log(nmeaLog);
    }
  }

  // 1Hz : Baro, Temp Air, vent, polaire, Set, Drift
  if (tick_1_200 > 1000) {
    tick_1_200 = tick_1_200 - 1000;

    #ifdef DEBUG
      if (timeStatus() == timeSet) Serial.println(timestamp());
    #endif

    if (nmea.bsp.isUpdated()) {
      timeVHW = 0;
      mus.speedo.hasVHW = true;
      bsp_tw_vmg();
    }

    // Ces 3 appels à supprimer quand les tests seront finis
    bsp_tw_vmg();
  }
  
  if (tick_1_400 > 1000) {
    tick_1_400 = tick_1_400 - 1000;
    if (nmea.bsp.isUpdated()) {
      if (mus.wind.hasVWR) polar_perf();
    }

    // Ces 3 appels à supprimer quand les tests seront finis
    polar_perf();
  }
  
  if (tick_1_600 > 1000) {
    tick_1_600 = tick_1_600 - 1000;
    if (nmea.awa.isUpdated()) {
      timeVWR = 0;
      mus.wind.hasVWR = true;
      if (mus.speedo.hasVHW) aw();
    }    

    // Ces 3 appels à supprimer quand les tests seront finis
    aw();
  }

  if (tick_1_800 > 1000) {
    tick_1_800 = tick_1_800 - 1000;
    baro_temp();    
  }

  if (tick_1_1000 > 1000) {
    tick_1_1000 = tick_1_1000 - 1000;
    if (tgps.speed.isUpdated()) {
      timeGPS = 0;
      mus.gps.hasGPS = true;
      set_drift();
    } 
  }

  // 5Hz : Cap, Gite, Tangage
  if (tick_5 > 200) {
    tick_5 = tick_5 - 200;
    hdg_heel_pitch();
  }

  // 100Hz : IMU
  if (tick_100 > 10) {
    tick_100 = tick_100 - 10;
    imu_query(mus);
  }

  // 1 minute : Synchro heure GPS
  // Au pire le log démarre au bout d'une minute suivant le fix GPS
  if (timeStatus() == timeNotSet) time_GPS(tgps);
  else if (une_min > 60000) {
    une_min = une_min - 60000;
    time_GPS(tgps);
  }

  // Expiration des infos après 5 secondes
  if (timeVWR > 5000) {
    timeVWR = timeVWR - 5000;
    mus.wind.hasVWR = false;
  }
  
  if (timeVHW > 5000) {
    timeVHW = timeVHW - 5000;
    mus.speedo.hasVHW = false;
  }
  
  if (timeGPS > 5000) {
    timeGPS = timeGPS - 5000;
    mus.gps.hasGPS = false;
  }
}
