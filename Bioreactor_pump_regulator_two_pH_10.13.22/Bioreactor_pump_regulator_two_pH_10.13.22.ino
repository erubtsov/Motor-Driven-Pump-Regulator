#define CLK 7                   //Setting the pins for the rotary encoder states
#define DT 8 
#include <LiquidCrystal_I2C.h>      //Including the library needed for the display

int CLK_State;                 //Control knob states needed to register rotation
int CLK_LastState;
float flow = 0;
const int refreshButtonPin = 13;     // Pin for autopilot button
int In1 = 5;                //Pins on the microcontroller that are connected to the motor driver terminals
int In2 = 6;                //^^^^
int buttonState = 0;         // variable for reading the pushbutton status
int whiteLED = 12;
int greenLED = 4;            //Setting the pins for the red and green LEDS
int redLED = 2;
float pH1;                    //Declare pH as a global variable of type float --> (precision)
float pH2;
int a_val1, a_val2; 
float volt;
float calibration_value = 21.24;
int buffer_arr1[10],temp1,buffer_arr2[10],temp2;

LiquidCrystal_I2C lcd(0x27,20,4);  // set the LCD address to 0x27 for a 16 chars and 2 line display
                                   //Digital display for pump regulator (Connected via I^2C comm protocol)

void setup() {
  // put your setup code here, to run once: 
  pinMode(refreshButtonPin, INPUT_PULLUP);
  pinMode(CLK,INPUT);         //Control knob states,
  pinMode(DT,INPUT);
  pinMode(In1, OUTPUT);           //motor driver terminals,
  pinMode(In2, OUTPUT);           //In1 MUST be connected to a PWM pin ( Symbol on microcontroller: ~ )
  pinMode(redLED, OUTPUT);             //Signal LEDs
  pinMode(greenLED, OUTPUT);
  pinMode(whiteLED, OUTPUT);
  //Serial.begin(9600);
  pinMode(a_val1, INPUT);
  pinMode(a_val2, INPUT);
  CLK_LastState = digitalRead(CLK);//Register the last state of the rotary encoder
  digitalWrite(In1, HIGH);          //Set the default direction of the motor driver that controls the pump
  digitalWrite(In2, LOW);           //by making 1 input high and 1 input low
  lcd.init();                      // initialize the lcd 
  lcd.backlight();                 //Set the backlight for the digital display
  lcd.setCursor(0,0);              //Setting the cursor so that the pH and Flow rate are displayed on different lines
  lcd.print("pH: ");                //outputting to the LCD
  lcd.setCursor(0,1);              //^^^^^
  lcd.print("Flow: ");             //^^^^^
 // attachInterrupt(digitalPinToInterrupt(DT), motor_function, CHANGE);
  
}

void loop() {
  motor_function();
  pH1_function();
  pH2_function();
  lcd_function_pH(pH1, pH2,7,0);
  indicator_function(pH1, pH2);
}

float pH1_function() {

  for(int i=0;i<10;i++) 
  { 
   buffer_arr1[i]=analogRead(A0);
   }
  for(int i=0;i<9;i++)
  {
    for(int j=i+1;j<10;j++)
    {
      if(buffer_arr1[i]>buffer_arr1[j])
      {
        temp1=buffer_arr1[i];
        buffer_arr1[i]=buffer_arr1[j];
        buffer_arr1[j]=temp1;
      }
    }
  }
  a_val1=0;
  for(int i=2;i<8;i++)
  a_val1+=buffer_arr1[i];
  float volt=(float)a_val1*5.0/1024/6; 
  pH1 = -5.70 * volt + calibration_value;

  //a_val=analogRead(A0); 
  //float volt=(float)a_val*5.0/1024.0; 
  //pH = -5.70 * volt + calibration_value;
  
  return pH1;
}
float pH2_function() {

  for(int i=0;i<10;i++) 
  { 
   buffer_arr2[i]=analogRead(A1);
   }
  for(int i=0;i<9;i++)
  {
    for(int j=i+1;j<10;j++)
    {
      if(buffer_arr2[i]>buffer_arr2[j])
      {
        temp2=buffer_arr2[i];
        buffer_arr2[i]=buffer_arr2[j];
        buffer_arr2[j]=temp2;
      }
    }
  }
  a_val2=0;
  for(int i=2;i<8;i++)
  a_val2+=buffer_arr2[i];
  float volt=(float)a_val2*5.0/1024/6; 
  pH2 = -5.70 * volt + calibration_value;

  //a_val=analogRead(A0); 
  //float volt=(float)a_val*5.0/1024.0; 
  //pH = -5.70 * volt + calibration_value;
  
  return pH2;
}

void lcd_function_pH(float val1, float val2, int pos, int line){
  lcd.setCursor(pos-2, line);                   // print the value from the sensor after "pH" on the digital display
  lcd.print(val1);
  lcd.print(",");
  //lcd.setCursor(pos+1, line);                   // print the value from the sensor after "pH" on the digital display
  lcd.print(val2); 
}
void lcd_function_flow(float val1, int pos, int line){
  lcd.setCursor(pos, line);                   // print the value from the sensor after "pH" on the digital display
  lcd.print(val1);
}

void motor_function(){  
  CLK_State = digitalRead(CLK); // Reads the "current" state of the rotary encoder
  if(flow >= 0){
    analogWrite(In1, flow);     //If the motor is driving the pump to push fluid out, the voltage levels
    analogWrite(In2, LOW);      //are set in the terminals to modulate the moto driver speed via variable "flow"
  }
  if(flow < 0){
    analogWrite(In2, -1*flow);   //If the motor is driving the pump to pull fluid in, the voltage levels
    analogWrite(In1, LOW);     //are set in the terminals to modulate the moto driver speed via variable "flow"
  }
  
  if (CLK_State != CLK_LastState){
    // If the DT state is different to the CLK state, that means the encoder is rotating clockwise
    if (digitalRead(DT) != CLK_State) {
      flow -= 5 ;          //The flow is adjusted by 2 units per control knob rotation
    } else {               //so that adjusting the flow is easier and more sensite over the whole range
      flow += 5;           //In this case, the opposite direction would require terminal "In1" to have a negative flow speed
    }
  }
  CLK_LastState = CLK_State;   //reset the last state of the control knob to the current state so that the flow value holds

  lcd_function_flow(flow,7,1); 

  if(flow >= 255){       //its desired position
    flow = 255;           //Set a ceiling value on the display, so that it accurately reflects the maximum flow rate reached
  }
  if(flow <= -255){
    flow = -255;         ////Set a floor value on the display, so that it accurately reflects the maximum flow rate reached
  }
  delay(1);
}

void indicator_function(float x, float y){
  if(x > 8){
    digitalWrite(greenLED, HIGH); //Conditional to light the green LED if the solution is basic relative to neutral
    digitalWrite(redLED, LOW);
    digitalWrite(whiteLED, LOW);
  }
  if(x > 6 && x <= 8){
    digitalWrite(greenLED, LOW); //Conditional to light the green LED if the solution is basic relative to neutral
    digitalWrite(redLED, LOW);
    digitalWrite(whiteLED, HIGH);
  }
  if(x <= 6){
    digitalWrite(redLED, HIGH);  //Conditional to light the green LED if the solution is acidic relative to neutral
    digitalWrite(greenLED, LOW);
    digitalWrite(whiteLED, LOW);
  }
}
