#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp280;

void baro_init() {
    bmp280.begin();
}

void baro_query(double& T, double& p0) {
  // Acquisition et calcul pression atmosphérique et température air
  T = bmp280.readTemperature();
  p0 = bmp280.readPressure();
}
