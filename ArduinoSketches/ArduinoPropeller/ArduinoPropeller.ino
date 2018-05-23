

/*CODE NOT COMPLETED!! WORKS FOR CONTOLLING THE PROPELLER BUT HAS NO COMUNICATION OR LOGIC*/

const int FPwm = 9; // pwm signal fór forward on pin 9
const int RPwm = 10; // pwm signal for reverse on pin 10
const int forward_enable = 13; // forward enable on pin 13
const int reverse_enable = 12; // reverse enable on pin 12
const int power = 7; // 'power on' is on pin 7



// Configure pins
void setup() {
  Serial.begin(9600);
  pinMode (FPwm, OUTPUT);
  pinMode (RPwm, OUTPUT);
  pinMode (forward_enable, OUTPUT);
  pinMode (reverse_enable, OUTPUT);
}


// Function called by Arduino on Serial input
void serialEvent(){   
  long input = Serial.parseInt();
  
  run(input);
}


/*
 * Function:  run 
 * --------------------
 * Runs the motor on the given PWM signal. 
 *
 * SPEED: % value for the motor. Negative = reverse. 
 *
 * returns: void
 */
 
void run(int SPEED) {   
  Serial.println(SPEED);
  
  
  // Converts SPEED to unsignedSPEED so it is always positive.
  
  int unsignedSPEED = SPEED > 0 ? SPEED : SPEED * -1;
  
  if (SPEED > 0)
     int unsignedSPEED = SPEED * -1; 

  else 
  
     int unsignedSPEED = SPEED; 
  
  // Converts unsignedSPEED (0-100) to PWM (0-255)
  
  int PWM = map(unsignedSPEED, 0, 100, 0, 255);
  
  // Run the motor        

if (unsignedSPEED <= 100) // Runs the motor if unsignedSPEED is under 100
{
  if (SPEED > 0) // Runs forward when SPEED is over 0
  {
        digitalWrite (power, HIGH);  // sets power on
        analogWrite (RPwm, 0);       // Sets reverse pwm to 0
        analogWrite (FPwm, PWM);     // Sets forward pwm to set value converted from unsignedSPEED
      
        digitalWrite (forward_enable, HIGH); //Enables forward
        digitalWrite (reverse_enable, HIGH); //Enables reverse(both must be enabled to run the motor)
    
        Serial.print ("Motor power is: ");
        Serial.print (unsignedSPEED); 
        Serial.println (" % forward"); 
        
   } 
  else if (SPEED < 0) // Runs reverse when SPEED is under 0
  {
        digitalWrite (power, HIGH);     
        analogWrite (FPwm, 0);
        analogWrite (RPwm, PWM); 

        digitalWrite (forward_enable, HIGH);
        digitalWrite (reverse_enable, HIGH);
        Serial.print ("Motor power is: "); 
        Serial.print (unsignedSPEED); 
        Serial.println (" % reverse"); 
   } 
   else if (SPEED == 0) // Stops the motor
  {
        analogWrite (FPwm, 0);
        analogWrite (RPwm, 0);
        digitalWrite (power, LOW);
        digitalWrite (forward_enable, LOW);
        digitalWrite (reverse_enable, LOW);
        Serial.println ("Motor stopped");   
   }
  }
  else // Stops the motor if SPEED is over 100 or under -100
  {
    analogWrite (FPwm, 0);
    analogWrite (RPwm, 0);
    digitalWrite (power, LOW);
    digitalWrite (forward_enable, LOW);
    digitalWrite (reverse_enable, LOW);
    Serial.println ("Value too high, motor stopped");   
  }

}

void loop() {
}
