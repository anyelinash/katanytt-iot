// Motores.h
#ifndef MOTORES_H
#define MOTORES_H

#include <Arduino.h>

class Motores {
public:
  Motores(int in1, int in2, int pwm);

  void detener();
  void encenderBajo();
  void encenderMedio();
  void encenderMaximo();
  void controlarPorMQTT(const String &mensaje);

private:
  int in1;
  int in2;
  int pwm;
};

#endif
