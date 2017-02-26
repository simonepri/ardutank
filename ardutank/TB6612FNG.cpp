#include "TB6612FNG.h"

// ---------------------------------------------------------------------------
// Motor costruttore
// ---------------------------------------------------------------------------

Motor::Motor(byte pin1, byte pin2, byte pin3) {
  PWM_pin = pin1;            // Memorizzo il pin PWM
  FWD_pin = pin2;            // Memorizzo il pin FORWARD
  REV_pin = pin3;            // Memorizzo il pin REVERSE
  pinMode(PWM_pin, OUTPUT);  // Programmo il pin in uscita
  pinMode(FWD_pin, OUTPUT);  // Programmo il pin in uscita
  pinMode(REV_pin, OUTPUT);  // Programmo il pin in uscita
  _rotating = false;         // Inizializzo la variabile
  _direction = INVALID;      // Inizializzo la variabile
  _speed = 0;                // Inizializzo la variabile
}

void Motor::rotate(byte direction, byte speed) {
  _rotating = true;               // Cambio lo stato della variabile che indica se il motore sta ruotando o no
  this->setDirection(direction);  // Imposto la direzione
  this->setSpeed(speed);          // Imposto la velocità
}

void Motor::stop(boolean graduate) {
  if(graduate) {
    while(_speed >= 10) {
      _speed -= 10;                // Decrementa la variabile della velocità attuale del motore
      this->setSpeed(_speed);      // Imposta la nuova velocità al motore
      delay(2);
    }
  }
  this->setSpeed(0);              // Ferma il motore del tutto
  this->setDirection(INVALID);    // Imposta la direzione come INVALID
  _rotating = false;              // Cambia lo stato della variabile per indicare che il motore non sta più ruotando
}

void Motor::setSpeed(byte speed) {
  if(!_rotating || _speed == speed) return;    // Se il motore non sta ruotando o ha già la stessa velocità return
  _speed = speed;                              // Aggiorna la variabile della velocità
  analogWrite(PWM_pin, speed);                 // Imposta la nuova velocità
}

void Motor::setDirection(byte direction) {
  if(!_rotating || _direction == direction) return;    // Se il motore non sta ruotando o ha già la stessa direzione return
  _direction = direction;                              // Aggiorna la variabile della direzione
  if(direction == FORWARD) {
    digitalWrite(FWD_pin, HIGH);             // Imposta il pin FWD a H
    digitalWrite(REV_pin, LOW);              // Imposta il pin REV a L
  }
  else if(direction == REVERSE) {
    digitalWrite(FWD_pin, LOW);              // Imposta il pin FWD a L
    digitalWrite(REV_pin, HIGH);             // Imposta il pin REV a H
  }
  else {
  digitalWrite(FWD_pin, LOW);                // Imposta il pin FWD a L
    digitalWrite(REV_pin, LOW);              // Imposta il pin REV a L
  }
}

byte Motor::getSpeed() {
  return _speed;                    // Ritorna la velocità attuale del motore
}

byte Motor::getDirection() {
  return _direction;                // Ritorna la direzione di rotazione attuale del motore
}

boolean Motor::isRotating() {
  return _rotating;                 // Ritorna vero se il motore sta ruotando, falso altrimenti
}
