#include <stdint.h>
#include <Wire.h>
//from sensor_msgs.msg import Joy
//from std_msgs.msg import UInt16MultiArray, MultiArrayDimension

///////////////////////////////////////////////////////////////////////////////////////////////////
//TODO Have the CSS assign the ID via a ROS parameter.
const int robotID = 1;

int millis_since_last_shot = 0;
const int millis_cooldown = 1000;

int turret_x = 90;
int turret_y = 90;
int turret_i2c_addr = 10;
int turret_poll_period_ms = 500;

uint8_t data[4];

void cmd_turret_callback(uint8_t Msg[3]);

///////////////////////////////////////////////////////////////////////////////////////////////////
void setup(void) {
  Serial.begin(57600);
  Serial.println("Initilizing turret node.");

  //rospy.init_node('turret_driver', anonymous=True)
  //turret_pub = rospy.Publisher("turret", UInt16MultiArray, queue_size=100)

  //#TODO ros param this
  //rospy.Subscriber("/joy62", Joy, self.cmd_turret_callback, queue_size = 3)
  //rospy.Subscriber("/joy62", Joy, self.cmd_turret_callback2, queue_size = 3)
  //rate = rospy.Rate(5) #TODO implement rosparam rate

  Wire.begin();
  
  Serial.println("Started.");
}

// This section temporary, used to validate message code.
int Tick_count = 0;
const int Servo_min = 70;
const int Servo_center = 85;
const int Servo_max = 100;
int Hor_posn = Servo_center;
int Hor_dirn = 1;
int Ver_posn = Servo_center;
int Ver_dirn = 1;

///////////////////////////////////////////////////////////////////////////////////////////////////
void loop(void) {
  //if (rospy.is_shutdown()) return;

  Wire.beginTransmission(turret_i2c_addr);
  Wire.write(4);
  Wire.endTransmission();
  uint8_t ir_reading = 0;
  Wire.requestFrom(turret_i2c_addr, 1);
  if (Wire.available()) {
    ir_reading = Wire.read();
  }

  if (ir_reading != 0) {
    //message = UInt16MultiArray();
    //dim = MultiArrayDimension("RDReciverCommand", 4, 1)
  
    data[0] = 0; //#TODO need command
    data[1] = robotID; //#TODO need command
    data[2] = 0; //#TODO need target
    data[3] = ir_reading; //#TODO implement robot ID
  
    //message.data = data
    //message.layout.dim.append(dim)
  
    //turret_pub.publish(message)
    Serial.print("IR reading: 0x");
    Serial.println(ir_reading, HEX);
  }

  //// There was a feature where a button on the turrent could be pressed
  //// to stop a run-away bot. That feature may be resurrected.
  //Wire.beginTransmission(turret_i2c_addr);
  //Wire.write(5);
  //Wire.endTransmission();
  //Wire.requestFrom(turret_i2c_addr, 1);
  //uint8_t button_reading = 0;
  //if (Wire.available()) {
  //  button_reading = Wire.read();
  //}
  //
  //if (buttonReading == 1) {
  //  Serial.println("Shutdown Request Button is pressed.");
  //  //#TODO implement soft shutdown
  //}

  //rate.sleep()
  delay(turret_poll_period_ms);

  Tick_count++;
  uint8_t Msg[3];
  if ((Tick_count % 2) == 0)
  {
    Msg[0] = 1;
    Msg[1] = 0; // Must be sent, but not actually used. (as of 2016-03-31)
    Msg[2] = (uint8_t)Hor_posn;
    cmd_turret_callback(Msg);
    Serial.print("H->"); Serial.println(Hor_posn);
    Hor_posn += Hor_dirn;
    if ((Hor_posn <= Servo_min) || (Hor_posn >= Servo_max)) Hor_dirn = -Hor_dirn;
  }
  if ((Tick_count % 3) == 0)
  {
    Msg[0] = 2;
    Msg[1] = 0; // Must be sent, but not actually used. (as of 2016-03-31)
    Msg[2] = (uint8_t)Ver_posn;
    cmd_turret_callback(Msg);
    Serial.print("V->"); Serial.println(Ver_posn);
    Ver_posn += Ver_dirn;
    if ((Ver_posn <= Servo_min) || (Ver_posn >= Servo_max)) Ver_dirn = -Ver_dirn;
  }
  if ((Tick_count % 4) == 0)
  {
    Msg[0] = 3;
    Msg[1] = 0; // Not actually sent.
    Msg[2] = 0; // Not actually sent.
    cmd_turret_callback(Msg);
    Serial.println("Shoot");
  }
}

///////////////////////////////////////////////////////////////////////////////////////////////////
// Message array contains 3 bytes:
// { command, value = 1~255, reserved }
// Command => 1 = pan (h), 2 = tilt (v), 3 = fire
void cmd_turret_callback(uint8_t Msg[3]) {
  //rospy.loginfo("Turret CB %i, %i", Msg[0], Msg[1]);

  switch (Msg[0]) {
    case 1: {
      // Pan or horizontal update. Command 1 tells turret it is an h-value.
      Wire.beginTransmission(turret_i2c_addr);
      Wire.write(1);
      Wire.write(Msg[1]);
      Wire.write(Msg[2]); // Must be sent, but not actually used by the turret. (as of 2016-03-31)
      Wire.endTransmission();
      //rospy.loginfo("Pan to %i", Msg[1]);
    } break;
    case 2: {
      // Tilt or vertical update. Command 2 tells turret it is a v-value.
      Wire.beginTransmission(turret_i2c_addr);
      Wire.write(2);
      Wire.write(Msg[1]);
      Wire.write(Msg[2]); // Must be sent, but not actually used by the turret. (as of 2016-03-31)
      Wire.endTransmission();
      //rospy.loginfo("Tilt to %i", Msg[1]);
    } break;
    case 3: {
      // Only allow shooting if guard time has expired.
      // TODO Set guard time as a rosparam
      uint32_t newTime = millis();
      if (newTime - millis_since_last_shot >=  millis_cooldown) {
        millis_since_last_shot = newTime;

        // Fire update. Command 3 tells turret to fire.
        Wire.beginTransmission(turret_i2c_addr);
        Wire.write(3);
        //Wire.write(Msg[1]); // Currently, there are no variations on "shoot".
        Wire.endTransmission();
        //rospy.loginfo("Fire");
      }
    } break;
  }
}


