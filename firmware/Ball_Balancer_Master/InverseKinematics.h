#ifndef InverseKinematics_H
#define InverseKinematics_H

#include <Arduino.h> 

#define A 0
#define B 1
#define C 2

class Machine { 
  public:
    
    Machine(double d, double e, double f, double g);

    double theta(int leg, double hz, double nx, double ny); 
};

#endif
