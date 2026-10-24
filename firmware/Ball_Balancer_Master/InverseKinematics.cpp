#include "InverseKinematics.h"

double d;  
double e;  
double f;  
double g;  

double nmag, nz;  
double x, y, z;   
double mag;       
double angle;     

static double checkedAcos(double value) {
  const double tolerance = 1e-6;
  if (value < -1.0 - tolerance || value > 1.0 + tolerance) return NAN;
  if (value < -1.0) value = -1.0;
  if (value >  1.0) value =  1.0;
  return acos(value);
}

Machine::Machine(double _d, double _e, double _f, double _g) {
  d = _d;
  e = _e;
  f = _f;
  g = _g;
}

double Machine::theta(int leg, double hz, double nx, double ny) {
  
  nmag = sqrt(pow(nx, 2) + pow(ny, 2) + 1);  
  nx /= nmag;
  ny /= nmag;
  nz = 1 / nmag;

  switch (leg) {
    case A:  
      y = d + (e / 2) * (1 - (pow(nx, 2) + 3 * pow(nz, 2) + 3 * nz) / (nz + 1 - pow(nx, 2) + (pow(nx, 4) - 3 * pow(nx, 2) * pow(ny, 2)) / ((nz + 1) * (nz + 1 - pow(nx, 2)))));
      z = hz + e * ny;
      mag = sqrt(pow(y, 2) + pow(z, 2));
      angle = checkedAcos(y / mag)
            + checkedAcos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
      
    case B:  
      x = (sqrt(3) / 2) * (e * (1 - (pow(nx, 2) + sqrt(3) * nx * ny) / (nz + 1)) - d);
      y = x / sqrt(3);
      z = hz - (e / 2) * (sqrt(3) * nx + ny);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = checkedAcos((sqrt(3) * x + y) / (-2 * mag))
            + checkedAcos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;
      
    case C:  
      x = (sqrt(3) / 2) * (d - e * (1 - (pow(nx, 2) - sqrt(3) * nx * ny) / (nz + 1)));
      y = -x / sqrt(3);
      z = hz + (e / 2) * (sqrt(3) * nx - ny);
      mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
      angle = checkedAcos((sqrt(3) * x - y) / (2 * mag))
            + checkedAcos((pow(mag, 2) + pow(f, 2) - pow(g, 2)) / (2 * mag * f));
      break;

    default:
      return NAN;
  }
  return (angle * (180 / PI));  
}
