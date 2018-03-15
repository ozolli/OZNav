// Lissage exponentiel simple d'un angle
// (aussi appelé filtre passe bas)
// On transforme d'abord l'angle en vecteur pour éliminer
// le problème du passage 360° (2 Pi radians) > 0°

#ifndef filtre_h
#define filtre_h

struct FPB {
  double x = 0.0f;
  double y = 0.0f;
};

double filtre(FPB& valeur, double angle, double alpha) {
  valeur.x = sin(angle) * alpha + valeur.x * (1.0f - alpha);
  valeur.y = cos(angle) * alpha + valeur.y * (1.0f - alpha);
  return atan2(valeur.x, valeur.y);
}

double filtre(FPB& valeur, double x, double y, double alpha) {
  valeur.x = x * alpha + valeur.x * (1.0f - alpha);
  valeur.x = y * alpha + valeur.x * (1.0f - alpha);
  return atan2(valeur.x, valeur.x);
}
#endif

