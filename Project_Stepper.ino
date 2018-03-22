#include <Stepper.h>

const int stepsPerRevolution = 200;  // number of steps per revolution
// for your motor

// initialize the stepper library on pins 8 through 11:
Stepper myStepper(stepsPerRevolution, 8, 9, 10, 11);

const float pi = 3.14159265;
int dir = 0;

void setup() 
{
  randomSeed(analogRead(0)); //Random number generator with random seed
}

void loop() 
{ 
  tremor();
}


int getRevolutions(int tremorAmplitude)
{
  float tremorFrequency = random(4, 8);//Set frequency to random number inside tremor frequency spectrum
  float tremorPeriod = 1/tremorFrequency;
  float tremorSpeed = tremorAmplitude/tremorPeriod; //Movement speed in cm/s

  /*
  * We know the speed of tremor and the size of the motor shaft - if it does only 1 revolution in 1 min,
  * then the speed would be the circumference divided by 60. However, if we multiply the circumference
  * by the number of revoutions and divide by 60 it will give us the speed C*N/t=s
  */

  float oneRevolutionLength = 2*pi*(5.8/2); //Circumference of circle of stepper motor
  int revolutions = ((int)tremorSpeed*60)/(int)oneRevolutionLength; //Number of revolutions needed in 1 min to achieve desired speed
  return revolutions;
}

void tremor()
{
  /*
 * The number of steps is calculated by approximating the motion of tremor to a circle segment that has a 
 * radius equal to the radius of a person's wrist and an arc length equal to the maxmimum possible tremor
 * and then calculating the size of the full angle using L=phi/360 * 2*pi*r
*/

  int wristRadius = 3; //Radius of the users wrist in cm
  float tremorAmplitude = random(16, 100)/100; //Arc length of tremor movement in cm
  float angle = (tremorAmplitude*360)/(2*pi*wristRadius); //Calculates the full angle of rotation
  int steps = (int)angle/(1.8*2); //Divide the angle by the step resolution to give number of steps and by 2 to give only a half-motion
  
  int revolutions = getRevolutions((int)tremorAmplitude);
  int mult = 1;
  
  myStepper.setSpeed(revolutions);
  if (dir == 1)
  {
    mult = 1;
    dir = 0;
  }
  else
  {
    mult = -1;
    dir = 1;
  }
  myStepper.step(steps*mult);
  myStepper.step(-steps*mult);
}

