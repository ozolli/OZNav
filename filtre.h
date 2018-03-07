// Lissage exponentiel simple d'un angle
// (aussi appelé filtre passe bas)
// On transforme d'abord l'angle en vecteur pour éliminer
// le problème du passage 360° (2 Pi radians) > 0°

double x_cog, y_cog, x_set, y_set = 0.0f;

double filtre_cog(double angle, double alpha) {
  x_cog = sin(angle) * alpha + x_cog * (1.0f - alpha);
  y_cog = cos(angle) * alpha + y_cog * (1.0f - alpha);
  return atan2(x_cog, y_cog);
}

double filtre_set(double x, double y, double alpha) {
  x_set = x * alpha + x_set * (1.0f - alpha);
  y_set = y * alpha + y_set * (1.0f - alpha);
  return atan2(x_set, y_set);
}
