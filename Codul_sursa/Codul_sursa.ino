#include "IR_remote.h"  // Include biblioteca pentru controlul telecomenzii IR
#include "keymap.h"     // Include biblioteca pentru maparea tastelor telecomenzii IR
#include <Servo.h>      // Include biblioteca pentru controlul servomotorului

// Inițializare variabile și obiecte
IRremote ir(3);                      // controlul telecomenzii IR, inițializat pe pinul 3
Servo servo_10;                      // controlul servomotorului
Servo servo_11;
volatile float V_Servo_angle = 50;   // Unghiul inițial al servomotorului
volatile float V_Servo_angle_11 = 120;
volatile char BLE_bit_temp;          // Variabilă temporară pentru stocarea unui bit primit prin Bluetooth
String BLE_value;                    // Variabilă pentru stocarea valorii primite prin Bluetooth
String BLE_value_temp;;              // Variabilă temporară pentru stocarea valorii intermediare Bluetooth
volatile int Black = 1;              // Variabilă pentru detectarea culorii negre (linie neagră)
volatile int Front_Distance;         // Variabilă pentru distanța frontală (senzor ultrasonic)
volatile int Left_Tra_Value;         // Valoare senzor de urmărire a liniei pe partea stângă
volatile int Center_Tra_Value;       // Valoare senzor de urmărire a liniei în centru
volatile int Right_Tra_Value;        // Valoare senzor de urmărire a liniei pe partea dreaptă
volatile char IR_Car_Mode = ' ';     // Mod de control al robotului (primit prin IR)
volatile boolean IR_Mode_Flag = false;  // Flag pentru indicarea modului de control IR
volatile boolean continuous_mode = false; // Flag pentru indicarea modului continuu
volatile boolean continuous_line = false;
bool ultrasonicAvoidanceMode = false;  // Mod de evitare a obstacolelor cu senzor ultrasonic
bool ultrasonicFollowMode = false;     // Mod de urmărire a persoanelor cu senzor ultrasonic
unsigned long previousMillis = 0;      // Variabilă pentru stocarea timpului anterior
const long interval = 2000;            // Interval de 2 secunde
bool servoDirection = true;            // Direcția de mișcare a servomotorului
int servoAngles[] = {15, 45, 60, 75, 90}; // Lista de unghiuri prestabilite
int currentAngleIndex = 0;             // Indexul curent în array-ul de unghiuri
bool isTurningLeft = false;            // Flag pentru indicarea virării la stânga
bool isTurningRight = false;           // Flag pentru indicarea virării la dreapta

const int LINE_TRACING_SPEED = 120;    // Viteza constantă pentru urmărirea liniei

// Declarații funcții
void STOP();
void Move_Forward(int car_speed);
void Move_Backward(int car_speed);
void Rotate_Left(int car_speed);
void Rotate_Right(int car_speed);
void Move_Left(int car_speed);
void Move_Right(int car_speed);
float checkdistance();
void Ultrasonic_Avoidance();
void Ultrasonic_Follow();
float checkLeftDistance();
float checkRightDistance();
void Infrared_Tracing();
void IR_remote_control();
void rotateServo();

void setup() {
  Serial.begin(115200);
  servo_10.attach(10); // Atașează servomotorul la pinul 11
  servo_11.attach(11);
  servo_10.write(round(V_Servo_angle)); // Setează unghiul inițial
  servo_11.write(round(V_Servo_angle_11));
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(12, OUTPUT);
  pinMode(13, INPUT);
  delay(0);
}

void loop() {
    
  // Citirea valorilor de la modulul BLE
  while (Serial.available() > 0) {
    BLE_value_temp += (char)(Serial.read());
    delay(2);
    if (!Serial.available() > 0) {
      BLE_value = BLE_value_temp;
      BLE_value_temp = "";
    }
  }

  // Verifică dacă BLE_value are o lungime validă și procesează comanda
  if (0 < String(BLE_value).length() && 4 >= String(BLE_value).length()) {
    if (4 >= String(BLE_value).length()) {
      if ('%' == String(BLE_value).charAt(0) && '#' == String(BLE_value).charAt((String(BLE_value).length() - 1))) {
        if (IR_Mode_Flag == true) {
          STOP();
          IR_Car_Mode = ' ';
          IR_Mode_Flag = false;
        }
        switch (String(BLE_value).charAt(1)) {
          case 'G': // Scade unghiul servomotorului
            V_Servo_angle += 3;
            if (V_Servo_angle > 180)
              V_Servo_angle = 180;
            servo_10.write(round(V_Servo_angle));
            BLE_value = "";
            break;
          case 'H': // Scade unghiul servomotorului
            V_Servo_angle -= 3;
            if (V_Servo_angle < 0)
              V_Servo_angle = 0;
            servo_10.write(round(V_Servo_angle));
            delay(0);
            BLE_value = "";
            break;
          case 'F': // Mișcare înainte
            Move_Forward(110);
            delay(400);
            BLE_value = "";
            break;
          case 'B': // Mișcare înapoi
            Move_Backward(110);
            delay(400);
            BLE_value = "";
            break;
          case 'L': // Rotire la stânga
            Rotate_Right(110);
            delay(250);
            BLE_value = "";
            break;
          case 'R': // Rotire la dreapta
            Rotate_Left(110);
            delay(250);
            BLE_value = "";
            break;
          case 'T': // Urmărire linie cu infraroșu
            Infrared_Tracing();
            break;
          case 'S': // Oprire
            BLE_value = "";
            STOP();
            break;
          case 'A': // Activare mod de evitare obstacole
            Ultrasonic_Avoidance();
            break;
          case 'Z': // Activare mod de urmărire a persoanelor/copiilor
            Ultrasonic_Follow();
            break;
        }
      }
    } else {
      BLE_value = "";
      STOP();
    }
  } else {
    STOP();
  }

  // Controlul cu telecomanda IR
  IR_remote_control();

  // Modul continuu
  if (continuous_mode) {
    Front_Distance = checkdistance(); // Verifică distanța frontală

    if (Front_Distance <= 10) { // Dacă obstacolul este foarte aproape
      Move_Backward(100);
      delay(200);
      if (50 >= random(1, 100)) {
        Rotate_Left(100);
      } else {
        Rotate_Right(100);
      }
      delay(500);
    } else if (Front_Distance >= 10 && Front_Distance <= 20) { // Dacă obstacolul este aproape
      STOP();
      delay(200);
      if (50 >= random(1, 100)) {
        Rotate_Left(100);
      } else {
        Rotate_Right(100);
      }
      delay(200);
    } else if (Front_Distance > 20 && Front_Distance <= 30) { // Dacă obstacolul este la distanță medie
      Move_Forward(150);
      delay(150);
    } else if (Front_Distance > 30) { // Dacă nu există obstacole în apropiere
      STOP();
    }
  }
  if(continuous_line){
     Left_Tra_Value = digitalRead(7);
  Center_Tra_Value = digitalRead(8);
  Right_Tra_Value = digitalRead(9);
  if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {
    Move_Forward(60);

  } else if (Left_Tra_Value == Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {
    Rotate_Left(255);
  } else if (Left_Tra_Value == Black && (Center_Tra_Value != Black && Right_Tra_Value != Black)) {
    Rotate_Left(255);
  } else if (Left_Tra_Value != Black && (Center_Tra_Value != Black && Right_Tra_Value == Black)) {
    Rotate_Right(255);
  } else if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value == Black)) {
    Rotate_Right(255);
  } else if (Left_Tra_Value == Black && (Center_Tra_Value == Black && Right_Tra_Value == Black)) {
    STOP();
  }
  else if (Left_Tra_Value != Black && (Center_Tra_Value != Black && Right_Tra_Value != Black)) {
    STOP();
  }
  }
}

void IR_remote_control() {
  int keyCode = ir.getIrKey(ir.getCode(), 1);

  // Setarea modului în funcție de butornul  cheii IR primite
  if (keyCode == IR_KEYCODE_UP) {
    IR_Car_Mode = 'f';
    IR_Mode_Flag = true;
    continuous_mode = false;
  } else if (keyCode == IR_KEYCODE_LEFT) {
    IR_Car_Mode = 'l';
    IR_Mode_Flag = true;
    continuous_mode = false;
  } else if (keyCode == IR_KEYCODE_DOWN) {
    IR_Car_Mode = 'b';
    IR_Mode_Flag = true;
    continuous_mode = false;
  } else if (keyCode == IR_KEYCODE_RIGHT) {
    IR_Car_Mode = 'r';
    IR_Mode_Flag = true;
    continuous_mode = false;
  } else if (keyCode == IR_KEYCODE_OK) {
    IR_Car_Mode = 's';
    IR_Mode_Flag = true;
    continuous_mode = false;
    continuous_line = false;
  } else if (keyCode == IR_KEYCODE_STAR) {
    IR_Car_Mode = '+';
    IR_Mode_Flag = true;
    continuous_mode = false;
  } else if (keyCode == IR_KEYCODE_POUND) {
    IR_Car_Mode = '-';
    IR_Mode_Flag = true;
    continuous_mode = false;
  }else if(keyCode == IR_KEYCODE_3){
    continuous_line = true;
  } else if (keyCode == IR_KEYCODE_OK) {
    // Oprirea funcției Ultrasonic_Follow() și a funcțiilor care rulează continuu
    ultrasonicFollowMode = false;
    continuous_mode = false;
  } else if (keyCode == IR_KEYCODE_2) {
    continuous_mode = true;
  }

  // Executarea acțiunilor pe baza modului setat
  switch (IR_Car_Mode) {
    case 'b': // Mișcare înapoi
      Move_Backward(160);
      delay(100);
      STOP();
      IR_Car_Mode = ' ';
      break;
    case 'f': // Mișcare înainte
      Move_Forward(160);
      delay(100);
      STOP();
      IR_Car_Mode = ' ';
      break;
    case 'l': // Rotire la stânga
      Rotate_Left(160);
      delay(100);
      STOP();
      IR_Car_Mode = ' ';
      break;
    case 'r': // Rotire la dreapta
      Rotate_Right(160);
      delay(100);
      STOP();
      IR_Car_Mode = ' ';
      break;
    case 'mr': // Mișcare la dreapta
      Move_Right(160);
      delay(100);
      STOP();
      IR_Car_Mode = ' ';
      break;
    case 'ml': // Mișcare la stânga
      Move_Left(160);
      delay(100);
      STOP();
      IR_Car_Mode = ' ';
      break;
    case 's': // Oprire
      STOP();
      IR_Car_Mode = ' ';
      break;
   case '+':
    V_Servo_angle = V_Servo_angle + 3;
    if (V_Servo_angle >= 180) {
      V_Servo_angle = 180;
    }
    servo_10.write(round(V_Servo_angle));
    delay(0);
    IR_Car_Mode = ' ';
    break;
   case '-':
    V_Servo_angle = V_Servo_angle - 3;
    if (V_Servo_angle <= 0) {
      V_Servo_angle = 0;
    }
    servo_10.write(round(V_Servo_angle));
    delay(0);
    IR_Car_Mode = ' ';
    break;
  }
  }


void Infrared_Tracing() {
  Left_Tra_Value = digitalRead(7);
  Center_Tra_Value = digitalRead(8);
  Right_Tra_Value = digitalRead(9);
  if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {
    Move_Forward(60);

  } else if (Left_Tra_Value == Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {
    Rotate_Left(60);
  } else if (Left_Tra_Value == Black && (Center_Tra_Value != Black && Right_Tra_Value != Black)) {
    Rotate_Left(60);
  } else if (Left_Tra_Value != Black && (Center_Tra_Value != Black && Right_Tra_Value == Black)) {
    Rotate_Right(60);
  } else if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value == Black)) {
    Rotate_Right(0);
  } else if (Left_Tra_Value == Black && (Center_Tra_Value == Black && Right_Tra_Value == Black)) {
    STOP();
  }
  else if (Left_Tra_Value != Black && (Center_Tra_Value != Black && Right_Tra_Value != Black)) {
    STOP();
  }
}


void Move_Forward(int car_speed = 25) {
  digitalWrite(2, HIGH);
  analogWrite(5, car_speed);
  digitalWrite(4, LOW);
  analogWrite(6, car_speed);
}

void Move_Backward(int car_speed = 255) {
  digitalWrite(2, LOW);
  analogWrite(5, car_speed);
  digitalWrite(4, HIGH);
  analogWrite(6, car_speed);
}

void Rotate_Left(int car_speed = 255) {
   digitalWrite(2, HIGH);
  analogWrite(5, car_speed);
  digitalWrite(4, HIGH);
  analogWrite(6, car_speed);
}

void Rotate_Right(int car_speed = 255) {
    digitalWrite(2, LOW);
  analogWrite(5, car_speed);
  digitalWrite(4, LOW);
  analogWrite(6, car_speed);
}
void STOP() {
  digitalWrite(2, 0);
  analogWrite(5, 0);
  digitalWrite(4, 0);
  analogWrite(6, 0);
}

float checkdistance() {
  // Trimite un impuls de 10 microsecunde pe pinul 12 pentru a activa modulul ultrasonic
  digitalWrite(12, LOW);
  delayMicroseconds(2);
  digitalWrite(12, HIGH);
  delayMicroseconds(10);
  digitalWrite(12, LOW);

  // Calculează durata timpului pentru care semnalul ultrasunete se întoarce
  unsigned long duration = pulseIn(13, HIGH);

  // Calculează distanța în centimetri pe baza duratei timpului
  float distance = (duration * 0.0343) / 2;

  // Returnează distanța calculată
  return distance;
}


void rotateServo() {
  unsigned long currentMillis = millis();

  // Verifică dacă au trecut 5 secunde
  if (currentMillis - previousMillis >= interval && !ultrasonicFollowMode && !ultrasonicAvoidanceMode) {
    previousMillis = currentMillis;

    // Schimbă unghiul servomotorului între 15, 30, 45, 60, 75 și 90 de grade
    currentAngleIndex = (currentAngleIndex + 1) % 6; // Unghiul servo-motorului trece la următorul
    servo_10.write(servoAngles[currentAngleIndex]);
    servo_11.write(servoAngles[currentAngleIndex]);
  }
}

void Ultrasonic_Avoidance() {
  // Verifică distanța frontală
  Front_Distance = checkdistance();

  // Acțiuni în funcție de distanța frontală detectată
  if (Front_Distance <= 10) { // Dacă există un obstacol foarte aproape
    Move_Backward(100);
    delay(200);
    if (random(1, 101) <= 50) { // Rotirea stânga sau dreapta în mod aleatoriu
      Rotate_Left(100);
    } else {
      Rotate_Right(100);
    }
    delay(500);
  } else if (Front_Distance >= 10 && Front_Distance <= 20) { // Dacă există un obstacol aproape
    STOP();
    delay(200);
    if (random(1, 101) <= 50) { // Rotirea stânga sau dreapta în mod aleatoriu
      Rotate_Left(100);
    } else {
      Rotate_Right(100);
    }
    delay(500);
  } else if (Front_Distance > 20) { // Dacă nu există obstacole în apropiere
    Move_Forward(100);
  }
}
void Ultrasonic_Follow() {
    Front_Distance = checkdistance(); // Verifică distanța frontală

    if (Front_Distance <= 10) { // Dacă obstacolul este foarte aproape
      Move_Backward(100);
      delay(200);
      if (50 >= random(1, 100)) {
        Rotate_Left(100);
      } else {
        Rotate_Right(100);
      }
      delay(500);
    } else if (Front_Distance >= 10 && Front_Distance <= 20) { // Dacă obstacolul este aproape
      STOP();
      delay(200);
      if (50 >= random(1, 100)) {
        Rotate_Left(100);
      } else {
        Rotate_Right(100);
      }
      delay(200);
    } else if (Front_Distance > 20 && Front_Distance <= 30) { // Dacă obstacolul este la distanță medie
      Move_Forward(150);
      delay(150);
    } else if (Front_Distance > 30) { // Dacă nu există obstacole în apropiere
      STOP();
    }
  }