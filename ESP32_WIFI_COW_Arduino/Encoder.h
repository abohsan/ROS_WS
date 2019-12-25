//
//#include <ros.h>
//#include <std_msgs/String.h>
//
//ros::NodeHandle nh;
//std_msgs::String str_msg;
//ros::Publisher chatter("chatter", &str_msg);
//
//const byte right_Encoder_int = 17;
//const byte left_Encoder_int = 5;
//
//const byte right_Encoder = 28;
//const byte left_Encoder = 29;
//
//unsigned int R_encoder = 0;
//unsigned int L_encoder = 0;
//
//void right_detect() {
////  if (digitalRead(right_Encoder) == HIGH) {
//    R_encoder += 100;
////  } else {
////    R_encoder -= 100;
////  }
//  Serial.println(R_encoder);
//}
//
//void left_detect() {
////  if (digitalRead(left_Encoder) == HIGH) {
//    L_encoder += 100;
////  } else {
////    L_encoder -= 100;
////  }
//  Serial.println(L_encoder);
//}
//
//
//void encoder_setUp() {
//  pinMode(right_Encoder_int, INPUT_PULLUP);
//  pinMode(left_Encoder_int, INPUT_PULLUP);
//
//
//  attachInterrupt(digitalPinToInterrupt(right_Encoder_int), right_detect, RISING);
//  attachInterrupt(digitalPinToInterrupt(left_Encoder_int), left_detect, RISING);
//}
