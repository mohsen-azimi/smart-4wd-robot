// requires the Encoder library.
// ref: https://howtomechatronics.com/projects/arduino-mecanum-wheels-robot/

// 1) open Tools -> Manage Libraries...
// 2) install "Encoder" by Paul Stoffregen v1.4.1
#include <Encoder.h>


int wheelSpeed = 20;
int timedelay = 100;
String readString, servo1, servo2, servo3, servo4;


// --- SPD Motor ---
class SPDMotor {
  public:
  SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 );

  /// Set the PWM speed and direction pins.
  /// pwm = 0, stop (no active control)
  /// pwm = 1 to 255, proportion of CCW rotation
  /// pwm = -1 to -255, proportion of CW rotation
  void speed( int pwm );

  /// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
  void hardStop();

  /// Get the current speed.
  int getSpeed();

  /// Get the current rotation position from the encoder.
  long getEncoderPosition();

  private:
    Encoder *_encoder;
    bool _encoderReversed;
    int _motorPWM, _motorDir1, _motorDir2;

    // Current speed setting.
    int _speed;
};

SPDMotor::SPDMotor( int encoderA, int encoderB, bool encoderReversed, int motorPWM, int motorDir1, int motorDir2 ) {
  _encoder = new Encoder(encoderA, encoderB);
  _encoderReversed = encoderReversed;

  _motorPWM = motorPWM;
  pinMode( _motorPWM, OUTPUT );
  _motorDir1 = motorDir1;
  pinMode( _motorDir1, OUTPUT );
  _motorDir2 = motorDir2;
  pinMode( _motorDir2, OUTPUT );
}

/// Set the PWM speed and direction pins.
/// pwm = 0, stop (no active control)
/// pwm = 1 to 255, proportion of CCW rotation
/// pwm = -1 to -255, proportion of CW rotation
void SPDMotor::speed( int speedPWM ) {
  _speed = speedPWM;
  if( speedPWM == 0 ) {
    digitalWrite(_motorDir1,LOW);
    digitalWrite(_motorDir2,LOW);
    analogWrite( _motorPWM, 255);
  } else if( speedPWM > 0 ) {
    digitalWrite(_motorDir1, LOW );
    digitalWrite(_motorDir2, HIGH );
    analogWrite( _motorPWM, speedPWM < 255 ? speedPWM : 255);
  } else if( speedPWM < 0 ) {
    digitalWrite(_motorDir1, HIGH );
    digitalWrite(_motorDir2, LOW );
    analogWrite( _motorPWM, (-speedPWM) < 255 ? (-speedPWM): 255);
  }
}

/// Activate a SHORT BRAKE mode, which shorts the motor drive EM, clamping motion.
void SPDMotor::hardStop() {
    _speed = 0;
    digitalWrite(_motorDir1,HIGH);
    digitalWrite(_motorDir2,HIGH);
    analogWrite( _motorPWM, 0);
}

/// Get the current speed.
int SPDMotor::getSpeed() {
    return _speed;
}

/// Get the current rotation position from the encoder.
long SPDMotor::getEncoderPosition() {
  long position = _encoder->read();
  return _encoderReversed ? -position : position;
}

//                      SPDMotor( encoderA, encoderB, encoderReversed,  motorPWM,  motorDir1,  motorDir2 ) {

//SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 34, 35); // <- Encoder reversed to make +position measurement be forward.
//SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 36, 37); // <- NOTE: Motor Dir pins reversed for opposite operation
//SPDMotor *motorLR = new SPDMotor( 3, 49, true,  9, 43, 42); // <- Encoder reversed to make +position measurement be forward.
//SPDMotor *motorRR = new SPDMotor( 2, A1, false, 5, A4, A5); // <- NOTE: Motor Dir pins reversed for opposite operation

SPDMotor *motorLF = new SPDMotor(18, 31, true, 12, 34, 35); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRF = new SPDMotor(19, 38, false, 8, 36, 37); // <- NOTE: Motor Dir pins reversed for opposite operation
SPDMotor *motorLR = new SPDMotor( 3, 49, true,  9, 43, 42); // <- Encoder reversed to make +position measurement be forward.
SPDMotor *motorRR = new SPDMotor( 2, A1, false, 5, A5, A4); // <- NOTE: Motor Dir pins reversed for opposite operation



void setup(){

   /*****************************
   * SKETCH 1 SETUP: Serial
   ****************************/
   Serial.begin(9600);
   Serial.setTimeout(.1);  
}
   


void moveRobot(int LF, int LR, int RF, int RR)
{

//    if (abs(LF) > 0 || abs(LR) > 0 || abs(RF) > 0 || abs(RR) > 0) 
//    { 
      motorLF->speed(LF); motorRF->speed(RF); 
      motorLR->speed(LR); motorRR->speed(RR);
//    }
//    else 
//    {
//      motorLF->speed(LF); motorRF->speed(RF); 
//      motorLR->speed(LR); motorRR->speed(RR);
//    }
}

//
//
void loop() {

   /*****************************
   * SKETCH 1 LOOP: Serial
   ****************************/
  while (Serial.available()) {
    delay(3);  
    if (Serial.available() >0) {
      char c = Serial.read();  //gets one byte from serial buffer
      readString += c; //makes the string readString
    } 
  }

  if (readString.length() >0) {
//      Serial.println(readString); //see what was received
      
      // expect a string like 07002100 containing the two servo positions      
      servo1 = readString.substring(0, 4); //get the first four characters
      servo2 = readString.substring(4, 8); //get the next four characters 
      servo3 = readString.substring(8, 12); //get the next four characters 
      servo4 = readString.substring(12, 16); //get the next four characters 
//      servo5 = readString.substring(16, 20); //get the next four characters 
      
    
      int n1, n2, n3, n4; //declare as number  
      char carray1[6], carray2[6], carray3[6], carray4[6]; //magic needed to convert string to a number 
      
      servo1.toCharArray(carray1, sizeof(carray1));
      n1 = atoi(carray1); 
      
      servo2.toCharArray(carray2, sizeof(carray2));
      n2 = atoi(carray2); 
      
      servo3.toCharArray(carray3, sizeof(carray3));
      n3 = atoi(carray3); 
      
      servo4.toCharArray(carray4, sizeof(carray4));
      n4 = atoi(carray4);
      
//      servo5.toCharArray(carray5, sizeof(carray5));
//      n5 = atoi(carray5);

      
      Serial.print(" 1:");  //print ot serial monitor to see results
      Serial.print(n1);  //print ot serial monitor to see results
      Serial.print(", 2:");
      Serial.print(n2);      
      Serial.print(", 3:");
      Serial.print(n3);
      Serial.print(", 4:");
      Serial.print(n4);
//      Serial.print(", Duration:");
//      Serial.print(n5);
      
      moveRobot(n1, n2, n3, n4);
    readString="";
  }
   /*****************************
   * END OF SKETCH 1 LOOP: Serial
   ****************************/ 
}

     

  




 
