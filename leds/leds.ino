#include <Arduino.h> // Include the Arduino library
#include <SD.h> // Include the SD library
#include "pid.h" // Include the PID library
#include <list>

const int LED_PIN = 15;
const int DAC_RANGE = 4096;
float counter = 0; //used to control the led
int iterations = 0;
float reference = 15;
unsigned long sampInterval = 10; // sample interval in milliseconds
bool lstream = false;
bool dstream = false;
float visibility_error = 0;
float flicker_error = 0;
float duty_cycle = 0;
float duty_cycle_k1 = 0;
float duty_cycle_k2 = 0;
bool plot = false;
float aux_energy = 0;
float restart = 0;
float PMAX = 0.016165 * 3.3;
//         h,   K,  b, Ti, Td, N, Tt
pid _pid(0.01, 0.6, 1, 1, 0, 10, 0.1);

typedef struct {
  float lux;
  float duty_cycle;
  long unsigned time;
} buffer_params;

 std::list<buffer_params> buffer;

void setup() {// the setup function runs once
  Serial.begin(115200);
  analogReadResolution(12); //default is 10
  analogWriteFreq(60000); //60KHz, about max
  analogWriteRange(DAC_RANGE); //100% duty cycle
  _pid.gain = Calibration();
  restart = millis();
}

void loop() {// the loop function runs cyclically
  unsigned long currentTime = micros();
  static unsigned long previousTime = 0;

  double h = (currentTime - previousTime) * 1e-3;
  if( h < sampInterval ) return;

  previousTime = currentTime;
  CommandList();
  float lux = LuxConverter();
  float voltage = (3.3/4096) * analogRead(A0);
  float r = lux2voltage(reference);
  //Serial.println("r: " + String(r) + " y: " + String(voltage) + " h: " + String(0.01));
  if( _pid.get_active())
    counter = _pid.compute_control(r,voltage, 0.01);
  analogWrite(LED_PIN, counter * DAC_RANGE); // set led PWM
  //Serial.println(counter);

  //in case the buffer is empty
  if(buffer.empty()){
    //initialize the buffer
    buffer.push_back({lux, counter, (millis() / 1000)});
  }else{
    //if the buffer is full, pop the first element and push the new one
    if (buffer.front().time + 60 < (millis() / 1000)){
      buffer.pop_front();
      buffer.push_back({lux, counter, (millis() / 1000)});
    }else{
      //push the new element if the buffer is not full
      buffer.push_back({lux, counter, (millis() / 1000)});
    }
  }

  double jitter = sampInterval - h;

  if (lstream){
    Serial.println("s l " + String(lux) + " " + millis()); 
  }
  if (dstream){
    Serial.println("s d " + String(voltage) + " " + millis());
  }

  if (plot){
    Serial.print(0); Serial.print(" "); Serial.print(reference) + Serial.print(" "); Serial.print(lux); Serial.print(" "); Serial.print(85); Serial.println();
  }

  visibility_error += max(0,reference - lux);


  if((duty_cycle - duty_cycle_k1) * (duty_cycle_k1 - duty_cycle_k2) < 0 && iterations > 2){
    flicker_error += abs(duty_cycle - duty_cycle_k1) + abs(duty_cycle_k1 - duty_cycle_k2);
  }
  //Serial.printf("%lf %lf %lf %lf\n",duty_cycle,duty_cycle_k1, duty_cycle_k2,flicker_error);
  //Serial.println(flicker_error);
  aux_energy += duty_cycle_k1 * h;
  duty_cycle_k2 = duty_cycle_k1;
  duty_cycle_k1 = duty_cycle;
  duty_cycle = float(counter);
  iterations++;
}

float LuxConverter(){
  float m = -0.9;
  float b = log10(150000) + 0.8;
  float Rx = 0;
  float Vldr = 0;
  Vldr = (3.3/4096) * analogRead(A0);
  Rx = ((3.3*10000)/Vldr) - 10000;

  float lux = pow(10, ((log10(Rx) - b) / m));
  return lux;
}

float lux2voltage(float lux){
  float m = -0.9;
  float b = log10(150000) + 0.8;
  float Rx = 0;
  float Vldr = 0;
  Rx = pow(10, (m * log10(lux) + b));
  Vldr = (3.3*10000)/(10000 + Rx);
  return Vldr; 
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
    if ( test > 4 || test < 3 && item1 != 'E'){
      Serial.println("Invalid arguments");
      return;
    }

    switch (item1)
    {
      case 'a':
        {
          //verify if the luminaire is the main one
          if (strcmp(item2,"1") != 0){
            Serial.println("err");
            return;
          }

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
          if (strcmp(item2,"1") != 0){
            Serial.println("err");
            return;
          }

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
          counter = atof(item3);
          _pid.set_active(0);
          Serial.println("ack");
          break;
        }
      case 'g':
        {
          //verify if the luminaire is the main one
          if(test == 4 && strcmp(item4,"1") != 0){
            Serial.println("err");
            return;
          }
          if (strcmp(item3,"1") != 0 && test != 4){
            Serial.println("err");
            return;
          }

          //commands for the gets
          if(strcmp(item2,"a") == 0){
            Serial.print("a " + String(item3) + " ");
            Serial.println(_pid.get_antiwindup());
          } else if(strcmp(item2,"d") == 0){
            Serial.print("d " + String(item3) + " ");
            Serial.println(counter);
          } else if(strcmp(item2,"f") == 0) {
            Serial.print("f " + String(item3) + " ");
            Serial.printf("%lf\n",flicker_error/iterations);
          } else if(strcmp(item2,"k") == 0){
            Serial.print("k " + String(item3) + " ");
            Serial.println(_pid.get_feedback());
          } else if(strcmp(item2,"r") == 0){
            Serial.print("r " + String(item3) + " ");
            Serial.println(reference);
          } else if(strcmp(item2,"l") == 0){
            Serial.print("l " + String(item3) + " ");
            Serial.println(LuxConverter()); 
          } else if(strcmp(item2,"v") == 0){
            Serial.print("v " + String(item3) + " ");
            Serial.println(visibility_error/iterations);
          } else if(strcmp(item2,"x") == 0){
            Serial.print("x " + String(item3) + " ");
            Serial.println(LuxConverter() - _pid.gain * counter);
          } else if(strcmp(item2,"E") == 0){
            Serial.print("E " + String(item3) + " ");
            Serial.println(_pid.get_active());
          }else if(strcmp(item2,"o") == 0){
            Serial.print("o " + String(item3) + " ");
            Serial.println(_pid.get_occupancy());
          } else if(strcmp(item2,"t") == 0){
            Serial.print("t " + String(item3) + " ");
            Serial.println((millis() - restart) / 1000); //time in seconds
          } else if(strcmp(item2,"e") == 0){
            Serial.print("e " + String(item3) + " ");
            Serial.printf("%f\n",PMAX * (aux_energy /iterations));
          } else if(strcmp(item2,"p") == 0){
            Serial.print("p " + String(item3) + " ");
            Serial.printf("%f\n",PMAX * counter);
          } else if(strcmp(item2,"b") == 0){
            //print the buffer
            Serial.println("b " + String(item3) + " " + String(item4) + " ");
            for (auto it = buffer.begin(); it != buffer.end(); it++){
              if (strcmp(item3,"l") == 0){
                Serial.print(String(it->lux) + ", ");
              }
              if(strcmp(item3,"d") == 0){
                Serial.print(String(it->duty_cycle) + ", ");
              }
            }
            Serial.println();
          }
          break;
        }
      // set feedback on or off
      case 'k':
        {
          if (strcmp(item2,"1") != 0){
            Serial.println("err");
            return;
          }
          
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
          if (strcmp(item2,"1") != 0){
            Serial.println("err");
            return;
          }

          reference = atoi(item3);
          Serial.println("ack");
          break;
        }
      case 's':
        {
          if (strcmp(item3,"1") != 0){
            Serial.println("err");
            return;
          }

          if (strcmp(item2,"d") == 0)
              dstream = true;
          if (strcmp(item2,"l") == 0)
              lstream = true;
          break;
        }
      case 'S':
        {
          if (strcmp(item3,"1") != 0){
            Serial.println("err");
            return;
          }

          if (strcmp(item2,"d") == 0)
              dstream = false;
          if (strcmp(item2,"l") == 0)
              lstream = false;
          Serial.println("ack");
          break;
        }
        
      case 'o':
        {
          if (strcmp(item2,"1") != 0){
            Serial.println("err");
            return;
          }

          if (strcmp(item3,"1") != 0 && strcmp(item3,"0") != 0){
            Serial.println("err");
            return;
          }
          _pid.set_occupancy(atoi(item3));
          Serial.println("ack");
          break;
        }
      case 'p':
        {
          if (strcmp(item2,"1") != 0){
            Serial.println("err");
            return;
          }

          plot = atoi(item3);
          Serial.println("ack");
          break;
        }
      case 'E':
        {
          if (strcmp(item2,"1") != 0){
            Serial.println("err");
            return;
          }
          _pid.set_active(atoi(item3));
          Serial.println("ack");
          break;
        }
    }
  }
}

float Calibration(){
    //calculate the gain
    float y1, y2, x1, x2;
    analogWrite(LED_PIN, 0);
    delay(5000);
    Serial.println("Calibrating..."); 
    y1 = LuxConverter();
    delay(1000);
    analogWrite(LED_PIN, 4096);
    delay(5000);
    y2 = LuxConverter();
    x1 = 0;
    x2 = 1;
    Serial.println("Calibration done");
    Serial.println("Gain: " + String((y2 - y1) / (x2 - x1)));


    float gain = (y2 - y1) / (x2 - x1);
    //definining the best values for pid
    float ref_volt = lux2voltage(reference);
    float Rldr = ((3.3*10000)/ref_volt) - 10000;
    float Req = (Rldr*10000)/(Rldr+10000);
    double Ti =  pow(10, -5) * Req;
    _pid.Ti = Ti;
    // ref_volt/ref_lux;
    float h = ref_volt / reference; 
    _pid.b =  1 / (gain * h * _pid.K);
    //Serial.println("Ti: " + String(Ti) + " B: " + String(_pid.b));
    //K = 1/(G*B*H)

    return gain;
}
