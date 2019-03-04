#include <Wire.h>
#include <VL53L0X.h>


int timetostart = 1;
int cote = 1;  // SI COTE = 0, alors le coté est le gauche, si COTE = 1, le coté est le droit

float medium;
float sensorc;
float sensorc2;
float sensorc3;
float sensorc4;

VL53L0X sensor;
VL53L0X sensor2;
VL53L0X sensor3;
VL53L0X sensor4;

void setup()
{
  pinMode(5, OUTPUT); // CAPTEUR GAUCHE AVANT
  pinMode(6, OUTPUT); // CAPTEUR GAUCHE ARRIERE
 // pinMode(7, OUTPUT); // LED DE TEST
 // pinMode(8, OUTPUT); // LED DE TEST
  pinMode(9, OUTPUT); // CAPTEUR DROITE AVANT
  pinMode(10, OUTPUT);// CAPTEUR DROITE ARRIERE

// Mettre les XSHUT à 0 pour ne pas griller la carte
  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(9, LOW);
  digitalWrite(10, LOW);
  delay(5);
  Wire.begin();


  Serial.begin (9600);

  //SENSOR
  pinMode(5, INPUT);
  delay(150);
 Serial.println("Get adress of sensor 1");   /// Récupérer l'adresse du capteur 1 en I2C
  sensor.init(true);
 Serial.println("Init of the sensor 1 ");    
  delay(100);
  sensor.setAddress((uint8_t)22);


  //SENSOR 2
  pinMode(6, INPUT);
  delay(150);
  sensor2.init(true);
Serial.println("Get adress of sensor 2");  /// Récupérer l'adresse du capteur 2 en I2C
  delay(100);
  sensor2.setAddress((uint8_t)25);
Serial.println("Init of the sensor 2"); 

  //SENSOR 3
  pinMode(9, INPUT);
  delay(150);
  sensor3.init(true);
Serial.println("Get adress of sensor 3");  /// Récupérer l'adresse du capteur 3 en I2C
  delay(100);
  sensor3.setAddress((uint8_t)26);
 Serial.println("Init of the sensor 3"); 

   //SENSOR 4
  pinMode(10, INPUT);
  delay(150);
  sensor4.init(true);
Serial.println("Get adress of sensor 4");  /// Récupérer l'adresse du capteur 4 en I2C
  delay(100);
  sensor4.setAddress((uint8_t)27);
  Serial.println("Init of the sensor 4"); 
 


  sensor.setTimeout(200);
  sensor2.setTimeout(200);


}

void loop(){


sensorc = sensor.readRangeSingleMillimeters();
sensorc2 = sensor2.readRangeSingleMillimeters();


if(cote != 1) {

    Serial.print("Gauche Avant   (mm):  ");
    Serial.print( sensor.readRangeSingleMillimeters() );


   Serial.print("      "); // Pour mettre de l'espace entre les 2 affichages de distance sans toucher au serial.print "Distance 2..."
    Serial.print("Gauche Arriere   (mm): ");
    Serial.print(sensor2.readRangeSingleMillimeters());
    Serial.println(""); }

if(cote != 0) {
   Serial.print("      "); // Pour mettre de l'espace entre les 2 affichages de distance sans toucher au serial.print "Distance 2..."
    Serial.print("Droite Avant   (mm): ");
    Serial.print(sensor3.readRangeSingleMillimeters());
    Serial.print("");


   Serial.print("      "); // Pour mettre de l'espace entre les 2 affichages de distance sans toucher au serial.print "Distance 2..."
    Serial.print("Droite arriere   (mm): ");
    Serial.print(sensor4.readRangeSingleMillimeters());
    Serial.println(""); }
  




 
  delay(timetostart); 
  } // VOID LOOP
