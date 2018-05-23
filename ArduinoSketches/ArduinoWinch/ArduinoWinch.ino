


/*CODE NOT COMPLETED!! WORKS FOR CONTOLLING THE WINCH BUT HAS NO COMUNICATION OR LOGIC*/


const int PUL=7; //define Pulse pin
const int DIR=6; //define Direction pin
const int ENA=5; //define Enable Pin

const int DIAMETER = 84;
const float CIRCUMFERENCE = PI * DIAMETER;
const long CIRCUMFERENCE_STEPS = 640000;
const float STEPS_PER_MILLIMETER = CIRCUMFERENCE_STEPS / CIRCUMFERENCE;
const long MAX_STEPS_PER_COMMAND = 12000 * STEPS_PER_MILLIMETER;


// start position
long position = 0;

// Configure pins
void setup() {
  Serial.begin(9600);
  pinMode (PUL, OUTPUT);
  pinMode (DIR, OUTPUT);
  pinMode (ENA, OUTPUT);
}

// Function called by Arduino on Serial input
void serialEvent(){   
  long input = Serial.parseInt();
  
  // If 0 -> Reset to home 
  if (input == 0) {
    input = position * -1; 
  }
  else {
    // Calculate millimeters to steps.
    // NOTE: We're converting a float to long. Decimal precision will be lost.
    input = STEPS_PER_MILLIMETER * input;  
  }
  
  run(input);
}


/*
 * Function:  run 
 * --------------------
 * Runs the stepper motor the given amount of steps. 
 *
 * steps: the steps the motor should run. A negative long to run backwards. 
 *
 * returns: void
 */
void run(long steps) {   
  Serial.println(steps);
  
  // Direction is forward if input is positive 
  boolean forward = steps >= 0;
  
  // Make sure steps are positive. Direction is already taken care of.
  long unsignedSteps = steps >= 0 ? steps : steps * -1;
  
  // Make sure we're not moving too much on a single command 
  if (unsignedSteps > MAX_STEPS_PER_COMMAND) {
    return;
  }
  
  // Run the steps 
  for (long i=0; i < unsignedSteps; i++) {
      digitalWrite(DIR,forward ? LOW : HIGH); // Set direction 
      digitalWrite(ENA,HIGH); 
      digitalWrite(PUL,HIGH);
      delayMicroseconds(50);
      digitalWrite(PUL,LOW);
      delayMicroseconds(50);
      
      // Keep track of current position
      position = position + (forward ? 1 : -1);
  }
  digitalWrite(ENA,LOW);
  
  Serial.print("Current position: ");
  Serial.println(position);
}

void loop() {
}
