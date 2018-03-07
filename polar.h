// Polaire modèle SailGrib WR

const byte polar[18][12] = {
// 00 04 06 08 10 12 14 16 20 25 30 35 (TWS)
  {00,00,00,00,00,00,00,00,00,00,00,00}, // 000 (TWA)
  {00,19,28,33,36,39,42,45,48,46,45,42}, // 036
  {00,20,30,35,38,42,45,48,50,48,47,46}, // 040
  {00,22,33,38,42,45,47,50,52,49,48,47}, // 045
  {00,24,35,40,45,48,50,52,54,53,52,51}, // 052
  {00,26,37,42,47,51,53,54,57,56,55,53}, // 060
  {00,28,38,43,48,52,54,56,58,58,57,56}, // 070
  {00,30,40,44,48,53,55,57,60,60,59,58}, // 080
  {00,31,41,45,49,54,55,57,62,62,62,61}, // 090
  {00,31,41,45,49,54,55,58,64,65,65,64}, // 100
  {00,30,41,45,49,54,56,58,64,65,67,67}, // 110
  {00,30,40,44,49,54,56,60,65,66,68,69}, // 120
  {00,30,40,44,48,53,55,59,65,66,69,71}, // 130
  {00,30,40,44,48,53,55,58,63,65,69,71}, // 140
  {00,29,39,43,47,52,54,57,62,64,68,70}, // 150
  {00,27,37,41,46,52,54,57,61,64,67,69}, // 160
  {00,25,34,39,45,52,54,57,60,63,67,69}, // 170
  {00,23,32,37,44,51,54,57,60,63,66,68}  // 180
};

// Meilleurs VMG calculés par SailGrib PolarEdit (à part TWS 00 extrapolés à vue de nez :-)

const byte vmg[12][4] = {
//{BSP près,TWA près,BSP portant,TWA portant}
  {00,58,00,145},  // 00 (TWS)
  {16,43,25,157},  // 04
  {24,43,35,157},  // 06
  {27,43,39,163},  // 08
  {30,43,44,173},  // 10
  {32,41,51,173},  // 12
  {35,39,54,180},  // 14
  {37,39,57,180},  // 16
  {40,37,60,180},  // 20
  {37,37,63,180},  // 25
  {37,37,66,173},  // 30
  {35,39,68,173}   // 35
};

const byte pwa[18] = {0, 36, 40, 45, 52, 60, 70, 80, 90, 100, 110, 120, 130, 140, 150, 160, 170, 180};
const byte pws[12] = {0, 4, 6, 8, 10, 12, 14, 16, 20, 25, 30, 35};

void polaropt(Boat mus, byte& btwa, byte& ppres, byte& pport) {
  // Retourne l'angle pour optimiser le vmg, le rendement au près et le rendement au portant

  byte s, b, ad, au;

  // OZ 2018 - On ne commence pas à pws[0] pour éviter de lire hors table (bugfix tws = 0)
  for (byte i = 1; i < 12; i++) {
    if (pws[i] >= mus.wind.tws) {
      s = i;
      break;
    }
    s = i; // au dessus de 35Kn on utilise 35Kn mais on conserve tws
  }

  if (mus.wind.d_twa <= 90.0f) b = 1;
  else b = 3;

  au = vmg[s][b];
  ad = vmg[s - 1][b];
  btwa = round(((mus.wind.tws - pws[s - 1]) / (pws[s] - pws[s - 1]) * (au - ad) + ad));

  au = vmg[s][0];
  ad = vmg[s - 1][0];
  ppres = round(mus.speedo.vmg / ((mus.wind.tws - pws[s - 1]) / (pws[s] - pws[s - 1]) * (au - ad) + ad) * 1000);

  au = vmg[s][2];
  ad = vmg[s - 1][2];
  pport = round(mus.speedo.vmg / ((mus.wind.tws - pws[s - 1]) / (pws[s] - pws[s - 1]) * (au - ad) + ad) * 1000);
}

double polarbsp(Boat mus) {
  // Retourne la vitesse cible interpolée pour tws et twa actuels

  byte no, ne, so, se, s;
  byte a = 0;
  double bspd = 0.0f;
  double bspu = 0.0f;

  for (byte i = 0; i < 10; i++) {
    if (pws[i] >= mus.wind.tws) {
      s = i;
      break;
    }
    s = i; // au dessus de 30Kn on utilise pws 30Kn mais on conserve tws
  }

  for (byte i = 0; i < 12; i++) {
    if (pws[i] >= mus.wind.tws) {
      s = i;
      break;
    }
    s = i; // au dessus de 35Kn on utilise 35Kn mais on conserve tws
  }

  for (byte i = 0; i < 18; i++) {
    if (pwa[i] >= mus.wind.d_twa) {
      a = i;
      break;
    }
  }

  // valeurs bsp encadrant tws et twa
  no = polar[a - 1][s - 1];
  ne = polar[a - 1][s];
  so = polar[a][s - 1];
  se = polar[a][s];
 
  // Calcul bsp pour pws à gauche de tws (vent inférieur)
  bspu = ((mus.wind.d_twa - pwa[a - 1]) / (pwa[a] - pwa[a - 1]) * (se - ne) + ne);

  // Calcul bsp pour pws à droite de tws (vent supérieur)
  bspd = ((mus.wind.d_twa - pwa[a - 1]) / (pwa[a] - pwa[a - 1]) * (so - no) + no);

  // Interpolation gauche/droite
  return ((mus.wind.tws - pws[s - 1]) / (pws[s] - pws[s - 1]) * (bspu - bspd) + bspd) / 10.0f;
}
