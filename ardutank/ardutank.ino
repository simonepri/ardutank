/**
 * Ardutank
 *
 * @author        Simone Primarosa,(http://simoneprimarosa.com)
 * @link          (https://github.com/simonepri/ardu-tank)
 * @license       MIT License (https://opensource.org/licenses/MIT)
 */

#include "Robot.h"

#define  AIN1  2
#define  AIN2  4
#define  BIN1  7
#define  BIN2  8
#define  PWMA  5
#define  PWMB  6
#define  SERV  3
#define  ECHO  11
#define  TRIG  12

#define INDEBUG 1

Robot *robot;

void setup() {
  #if INDEBUG
  Serial.begin(9600);             // Inizializzo la comunicazione seriale con 9600 baud/sec se il debug è abilitato
  #endif
  randomSeed(analogRead(0));      // Passo il seed per il generatore di numeri pseudocasuali
  robot = new Robot(PWMB, BIN2, BIN1, PWMA, AIN2, AIN1, SERV, TRIG, ECHO);  // Creo un instanza della classe Robot
  return;
}

void loop() {
  float dist = robot->distance();  // Ricevo l'informazione della distanza dal sensore ultrasuoni
  #if INDEBUG
  Serial.print("Robot -> distanza: ");Serial.print(dist);Serial.println("cm");
  #endif
  
  if(robot->getObstableRepeat() > 10) {       // Se ho trovato un ostacolo per 10 volte consecutive
    robot->move(random(-20, 20), 150, true);  // Muovo il robot di una distanza random compresa tra -20cm e 20cm
    robot->turn(random(-90, 90), 100, true);  // Ruoto il robot di un angolazione random compresa tra -90° e 90°
    #if INDEBUG
    Serial.println("Robot -> sono incastrato, provo a liberarmi");
    #endif
  }
  else if((dist > OBSTACLE_DISTANCE || dist == 0) && robot->getDistanceRepeat() < 10) {  // Se sono distante dall'ostacolo e se non ho ripetuto la stessa misura più volte
    robot->move(10, 150);      // Muovo il robot avanti di 10 cm
    #if INDEBUG
    Serial.println("Robot -> tutto ok, vado avanti");
    #endif
  }
  else {
    robot->stop(true);         // Fermo il robot
    
    byte past_dir = robot->getDirection();  // Prendo la direzione dell'ultimo movimento del robot
    float dist_sx, dist_dx;
    int ang_sx = robot->sweep_sx(dist_sx);  // Eseguo uno sweep a sinistra per ricercare una zona libera
    int ang_dx = robot->sweep_dx(dist_dx);  // Eseguo uno sweep a destra per ricercare una zona libera
    
    if(dist_sx == dist_dx) {    // Se da entrambe le parti ho lo stesso spazio
      robot->move(-20, 150, true);          // Muovo il robot indietro di 20 cm
      robot->turn((past_dir == DIRECTION_LEFT) ? ang_sx : ((past_dir == DIRECTION_RIGHT) ? ang_dx : random(-90, 90)), 100, true);  // Ruoto il robot nella direzione passata se non è ne destra ne sinistra ruoto in un angolo random compreso tra -90° e 0'°
      #if INDEBUG
      Serial.print("Robot -> s = d, vado a: ");Serial.println((past_dir == DIRECTION_LEFT) ? "sinistra" : ((past_dir == DIRECTION_RIGHT) ? "destra" : "random"));
      #endif
    }
    if(dist_sx > dist_dx) {     // Se lo spazio a sinistra è maggiore di quello a destra
      robot->move(-10, 150, true);          // Muovo il robot indeitro di 10 cm
      robot->turn((past_dir == DIRECTION_LEFT || dist_sx-dist_dx > 10) ? ang_sx : ang_dx, 100, true);  // Ruoto il robot verso sinistra se prima mi sono spostato a sinistra o se la differenza tra destra e sinistra è rilevante altrimenti a destra
      #if INDEBUG
      Serial.print("Robot -> s > d, vado a: ");Serial.println((past_dir == DIRECTION_LEFT || dist_sx-dist_dx > 5) ? "sinistra" : "destra");
      #endif
    }
    else if(dist_dx > dist_sx) {
      robot->move(-10, 150, true);
      robot->turn((past_dir == DIRECTION_RIGHT || dist_dx-dist_sx > 10) ? ang_dx : ang_sx, 100, true);  // Ruoto il robot verso destra se prima mi sono spostato a destra o se la differenza tra destra e sinistra è rilevante altrimenti a sinistra
      #if INDEBUG
      Serial.print("Robot -> d > s, vado a: ");Serial.println((past_dir == DIRECTION_RIGHT || dist_dx-dist_sx > 5) ? "destra" : "sinistra");
      #endif
    }
  }
  return;
}
