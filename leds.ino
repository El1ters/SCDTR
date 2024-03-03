#include <Arduino.h> // Include the Arduino library
#include <SD.h> // Include the SD library
#include "pid.h" // Include the PID library

const int LED_PIN = 15;
const int DAC_RANGE = 4096;
int counter = 0;
float lux =  0;
int reference = 20;
unsigned long sampInterval = 10; // sample interval in milliseconds

pid _pid(0.001, 1, 1, 1, 0, 10, 100);

void setup() {// the setup function runs once
  Serial.begin(115200);
  analogReadResolution(12); //default is 10
  analogWriteFreq(60000); //60KHz, about max
  analogWriteRange(DAC_RANGE); //100% duty cycle
}

void loop() {// the loop function runs cyclically
  unsigned long currentTime = millis();
  static unsigned long previousTime = 0;

  float h = currentTime - previousTime;
  if( h < sampInterval ) return;

  previousTime = currentTime;
  CommandList();
  int read_adc;
  analogWrite(LED_PIN, counter); // set led PWM
  read_adc = analogRead(A0); // read analog voltage
  //format that Serial Plotter likes
  //Serial.print(0); Serial.print(" ");
  //Serial.print(DAC_RANGE); Serial.print(" ");
  //Serial.print(read_adc); Serial.print(" ");
  //Serial.print(counter); Serial.print(" ");

  if (_pid.get_feedback()){
    counter = _pid.compute_control(reference,LuxConverter(),h);
  }

}

int LuxConverter(){
  float m = -0.9;
  float b = log10(150000) + 0.8;
  float Rx = 0;
  float Vldr = 0;
  Vldr = (3.3/4096) * analogRead(A0);
  Rx = ((3.3*10000)/Vldr) - 10000;

  lux = pow(10, ((log10(Rx) - b) / m));
  Serial.print(lux); Serial.println();
  return lux;
}

void CommandList(){
  char receivedstring[64];
  byte index = 0;

  while (Serial.available() > 0) {
    // Read the incoming byte
    char incomingByte = Serial.read();
    receivedstring[index] = incomingByte;
    index++;

  }

  if (index > 0) {
    char item1;
    char item2[5];
    char item3[5];
    char item4[5];

    receivedstring[index] = '\0';

    char dummy;
    int test = sscanf(receivedstring, "%s %s %s %s %s", &item1, item2, item3, item4, &dummy);
    if ( test > 4){
      Serial.println("Invalid arguments");
      return;
    }

    switch (item1)
    {
      case 'a':
        {
          if (strcmp(item3,"1") != 0 && strcmp(item3,"0") != 0){
            Serial.println("err");
            return;
          }
          _pid.set_antiwindup(atoi(item3));
          Serial.println("ack");
          break;
        }

      case 'd':
        {
          int aux = atoi(item2);
          if (aux < 1 || aux > 3){
            Serial.println("err");
            return;
          }
          float val = atof(item3) * DAC_RANGE;
          if (val > 4096 || val < 0){
            Serial.println("err");
            return;
          }
          counter = atof(item3) * DAC_RANGE;
          Serial.println("ack");
          break;
        }
      case 'g':
        {
          if(strcmp(item2,"a") == 0){
            Serial.print("Current anti-windup: ");
            Serial.println(_pid.get_antiwindup());
          } else if(strcmp(item2,"d") == 0){
            Serial.print("Current duty cycle: ");
            Serial.println(float(counter)/DAC_RANGE);
          } else if(strcmp(item2,"k") == 0){
            Serial.print("Feedback: ");
            Serial.println(_pid.get_feedback());
          } else if(strcmp(item2,"r") == 0){
            Serial.print("Reference: ");
            Serial.println(reference);
          } else if(strcmp(item2,"l") == 0){
            Serial.print("Lux: ");
            LuxConverter();
          }
          break;
        }
      // set feedback on or off
      case 'k':
        {
          if (strcmp(item3,"1") != 0 && strcmp(item3,"0") != 0){
            Serial.println("err");
            return;
          }
          // Adicionar depois o item 2
          _pid.set_feedback(atoi(item3));
          Serial.println("ack");
          break;
        }
      case 'r':
        {
          reference = atoi(item3);
          Serial.println("ack");
          break;
        }
    }
  }
}