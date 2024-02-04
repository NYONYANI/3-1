#include <Servo.h>

Servo pan; // create servo object for pan
Servo tilt; // create servo object for tilt

String cmd; // command string
float pan_val; // pan command value
float tilt_val; // tilt command value

float p_pos = 90; // current pan position
float t_pos = 90; // current tilt position
float s1;
float s2;
void setup() {
  Serial.begin(115200); // initialize serial communication
  pan.attach(11); // attach servo 1 to pin 11
  tilt.attach(10); // attach servo 2 to pin 10
  pan.write(p_pos); // initialize pan position
  tilt.write(t_pos); // initialize tilt position
}

void loop() {
  if (Serial.available() > 0) {
    cmd = Serial.readStringUntil('\n'); // read command until new line character
    int f=cmd.indexOf("t");
    if(cmd.substring(0,1)=="p")
    {
      s1=cmd.substring(1,f).toFloat();
      s2=cmd.substring(f+1).toFloat();
      p_pos = constrain(p_pos + s1, 40, 140); // set new pan position, limited to 40-140
      t_pos = constrain(t_pos + s2, 70, 180); // set new tilt position, limited to 70-180
      pan.write(p_pos); // move pan to new position
      tilt.write(t_pos); // move tilt to new position
    }
  }
  delay(10);
}
