#include "Robot.h"

// ---------------------------------------------------------------------------
// Robot costruttore
// ---------------------------------------------------------------------------

Robot::Robot(byte pin_sx_pwm, byte pin_sx_in2, byte pin_sx_in1, byte pin_dx_pwm, byte pin_dx_in2, byte pin_dx_in1, byte pin_sero, uint8_t pin_sonar_trig, uint8_t pin_sonar_echo) {
  obstacle_count = 0;                              // Inizializzazione variabili
  REC_dir_info[0] = DIRECTION_INVALID;             // Inizializzazione variabili
  REC_dist_info[0] = DISTANCE_INVALID;             // Inizializzazione variabili
  REC_dir_info[1] = 0;                             // Inizializzazione variabili
  REC_dist_info[1] = 0;                            // Inizializzazione variabili
  memset(REC_dist, DISTANCE_INVALID, MAX_RECORD);  // Inizializzazione variabili
  memset(REC_dir, DIRECTION_INVALID, MAX_RECORD);  // Inizializzazione variabili
  
  sonar = new HCSR04(pin_sonar_trig, pin_sonar_echo, MAX_DISTANCE);    // Creo un istanza della classe HCSR04
  servo = new Servo();                                                 // Creo un istanza della casse Servo
  servo->attach(pin_sero);
  this->angle(90);
  motor_left = new Motor(pin_sx_pwm, pin_sx_in2, pin_sx_in1);    // Creo un istanza della classe Motor
  motor_right = new Motor(pin_dx_pwm, pin_dx_in2, pin_dx_in1);   // Creo un istanza della classe Motor
}

float Robot::getDistance(int i) {
  return REC_dist[(REC_dist_info[0]-1-i) % MAX_RECORD];   // Ritorna l'(attuale - i) esima distanza misurata 
}

byte Robot::getDirection(int i) {
  return REC_dir[(REC_dir_info[0]-1-i) % MAX_RECORD];     // Ritorna l'(attuale - i) esima direzione percorsa
}

unsigned int Robot::getDistanceRepeat() {
  return REC_dist_info[1];    // Ritorna il numero di volte che è stata misurata la stessa distanza più volte
}

unsigned int Robot::getDirectionRepeat() {
  return REC_dist_info[1];    // Ritorna il numero di volte che è stata misurata la stessa direzione più volte
}

unsigned int Robot::getObstableRepeat() {
  return obstacle_count;      // Ritorna il numero di volte che ho trovato un ostacolo senza trovare uno spazio libero
}

void Robot::addDistance(float recordvalue) {
  if(abs(this->getDistance()-recordvalue) < 1.0) REC_dist_info[1]++;    // Incrementa il contatore delle misurazioni uguali consecutive se recordvalue è simile alla distanza precedente
  else REC_dist_info[1] = 0;                                 // Altrimenti azzeralo
  REC_dist[REC_dist_info[0]] = recordvalue;                  // Aggiungo recordvalue alo storico delle misurazioni
  REC_dist_info[0] = (REC_dist_info[0] + 1) % MAX_RECORD;    // Aggiorno la variabile che tiene memoria dell'indice in cui è posizionata l'ultima misurazione all'interno del vettore delle misurazioni
}

void Robot::addDirection(byte recordvalue) {
  if(this->getDirection() == recordvalue || this->getDirection(1) == recordvalue) REC_dist_info[1]++;  // Incrementa il contatore delle direzioni uguali consecutive se recordvalue è uguale alle direzioni precedenti
  else REC_dist_info[1] = 0;                                 // Altrimenti azzerolo
  REC_dir[REC_dir_info[0]] = recordvalue;                    // Aggiungo recordvalue alo storico delle direzioni
  REC_dir_info[0] = (REC_dir_info[0] + 1) % MAX_RECORD;      // Aggiorno la variabile che tiene memoria dell'indice in cui è posizionata l'ultima direzione all'interno del vettore delle direzioni
}

void Robot::move(int distance_cm, byte speed, boolean stop_graduate) {
  if(speed < MIN_SPEED || distance_cm == 0) return;
  this->addDirection(((distance_cm < 0.0) ? DIRECTION_REVERSE : DIRECTION_FORWARD));  // Aggiungi questo spostamento al vettore degli spostamenti
  float cmsec = float(speed*MAX_SPEED_CMSEC)/float(MAX_SPEED);    // Calcola i cm/sec in base alla velocità passata
  long times = abs(long(distance_cm/float(cmsec/1000.0)));        // Calcola quanti millisecondi sono necessari per percorrere la distanza passata
  motor_left->rotate(distance_cm >= 0 ? FORWARD : REVERSE, map(speed, 0, MAX_SPEED, 0, MAX_LEFT_SPEED));    // Fai ruotare il motore sinistro
  motor_right->rotate(distance_cm >= 0 ? FORWARD : REVERSE, map(speed, 0, MAX_SPEED, 0, MAX_RIGHT_SPEED));  // Fai ruotare il motore destro
  delay(times);                              // Aspetta il tempo calcolato precedentemente
  this->stop(stop_graduate);                 // Ferma i motori "lentamente"
}

void Robot::turn(int angle_grad, byte speed, boolean stop_graduate) {
  if(speed < MIN_SPEED || angle_grad == 0) return;                                // Se i parametri sono invalidi return
  this->addDirection(((angle_grad < 0.0) ? DIRECTION_RIGHT : DIRECTION_LEFT));    // Aggiungi questo spostamento al vettore degli spostamenti
  float gradsec = float(speed*MAX_SPEED_GRADSEC)/float(MAX_SPEED);    // Cacola i grad/sec in base alla velocità [DA MIGLIORARE CON ALTE VELOCITA']
  long times = abs(long(angle_grad/float(gradsec/1000.0)));           // Calcola i millisecondi necessari a spazzare l'angolo passato
  motor_left->rotate(angle_grad >= 0 ? REVERSE : FORWARD, map(speed, 0, MAX_SPEED, 0, MAX_LEFT_SPEED));    // Fai ruotare il motore sinistro
  motor_right->rotate(angle_grad >= 0 ? FORWARD : REVERSE, map(speed, 0, MAX_SPEED, 0, MAX_RIGHT_SPEED));  // Fai ruotare il motore destro
  delay(times);                              // Aspetta il tempo calcolato precedentemente
  this->stop(stop_graduate);                 // Ferma i motori "lentamente"
}

void Robot::stop(boolean graduate) {
  if(graduate) {
  byte speed_sx = motor_left->getSpeed();             // Leggo la velocità attuale del motore sx
  byte speed_dx = motor_right->getSpeed();            // Leggo la velocità attuale del motore dx
  while(speed_sx >= 10 && speed_dx >= 10) {
    speed_sx -= byte(10*MAX_LEFT_SPEED/MAX_SPEED);    // Decremento la variabile della velocità attuale del motore sx
    speed_dx -= byte(10*MAX_RIGHT_SPEED/MAX_SPEED);   // Decremento la variabile della velocità attuale del motore dx
    motor_left->setSpeed(speed_sx);          // Imposto la nuova velocità per il motore sx
    motor_right->setSpeed(speed_dx);         // Imposto la nuova velocità per il motore dx
    delay(2);
  }
  motor_left->stop(true);                    // Fermo completamente il motore sx
  motor_right->stop(true);                   //Fermo completamente il motore dx
  return;
  }
  motor_left->stop();                        // Fermo completamente il motore sx
  motor_right->stop();                       // Fermo completamente il motore dx
}

float Robot::distance(boolean useforcount) {
  float dist = sonar->ping_cm();                   // Ricevo la distanza dal sensore
  if(!useforcount) return dist;
  this->addDistance(dist);                         // Aggiungo la misurazioni al vettore delle distanze
  if(dist < OBSTACLE_DISTANCE) obstacle_count++;   // Se cè un ostacolo vicino incremento il contatore degli ostacoli
  else obstacle_count = 0;                         // Altrimenti lo azzero
  return dist;                                     // Ritorna la distanza in cm
}

void Robot::angle(byte angle) {
  servo->write(map(angle, 0, 180, 40, 130));   // Mappo il range 0-180 in un range 40-130 ed applico il valore al servo
}

int Robot::sweep(int a1, int a2,float &max_dist, boolean useforcount) {
  int max_angle = 90;                      // Inizializzo l'angolo massimo a 90
  max_dist = DISTANCE_INVALID;             // Inizializzo la distanza massima a 0
  float temp;
  for(int ang = a1; (a1 > a2) ? (ang >= a2) : (ang <= a2); (a1 > a2) ? ang -= 5 : ang += 5) {
    this->angle(ang);                      // Do la nuova posizione al servo
    delay(5);
    temp = this->distance(useforcount);   // Ricevo la distanza dal sensore
    delay(5);
    if(temp != 0 && temp > max_dist) {    // Se la nuova distanza è maggiore di quelle passate
      max_dist = temp;                    // Aggiorna con la nuova distanza
      max_angle = ang;                    // Aggiorna con la nuova angolazione
    }
    delay(5);
  }
  this->angle(90);                        // Muovo il servo alla posizione iniziale
  delay(5);
  return -90+max_angle;                   // Ritorna l'angolo di rotazione dell'ostacolo rispetto al robot
}


// Ritorna l'angolo della distanza maggiore a sinistra
int Robot::sweep_sx(float &max_dist, boolean useforcount) {
  return sweep(95, 175, max_dist, useforcount);
}

// Ritorna l'angolo della distanza maggiore a destra
int Robot::sweep_dx(float &max_dist, boolean useforcount) {
  return sweep(85, 5, max_dist, useforcount);
}
