// Motores.cpp
#include "Motores.h"

Motores::Motores(int in1, int in2, int pwm) : in1(in1), in2(in2), pwm(pwm) {
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(pwm, OUTPUT);
}

void Motores::detener() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void Motores::encenderBajo() {
  detener();
  analogWrite(pwm, 100);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void Motores::encenderMedio() {
  detener();
  analogWrite(pwm, 200);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void Motores::encenderMaximo() {
  detener();
  analogWrite(pwm, 255);
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
}

void Motores::controlarPorMQTT(const String &mensaje) {
  Serial.print("Controlando motores con mensaje: ");
  Serial.println(mensaje);

  if (mensaje == "A") {
    detener();
  } else if (mensaje == "B") {
    encenderBajo();
  } else if (mensaje == "C") {
    encenderMedio();
  } else if (mensaje == "D") {
    encenderMaximo();
  }
}

