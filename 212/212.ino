#define ip1 9 // 스러스터 체널2
#define ip2 11 // 서보모터 채널1 
#define ip4 6
#define ip5 4 // 변환 채널5
#define ip6 3 // 

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle  nh;
 
int ch1; 
int ch2; 

int ch5; 
int ch6; 
int val1;
int val2;
int relaypin = 24;



Servo thruster1; 
Servo thruster2; 
Servo servo;

void thruster_cb( const std_msgs::UInt16& cmd_msg){
  thruster1.write(cmd_msg.data);
  thruster2.write(3000-cmd_msg.data);//1100-1900
}

ros::Subscriber<std_msgs::UInt16> sub1("thrusterpwm", thruster_cb);

void Servo_cb( const std_msgs::UInt16& cmd_msg){
  servo.write(cmd_msg.data);
}                             //0-300

ros::Subscriber<std_msgs::UInt16> sub2("servopwm", Servo_cb);

void setup()
{ 
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  pinMode(relaypin,OUTPUT); // 릴레이를 출력으로 설정
  pinMode(ip6,INPUT_PULLUP); // 스위치(릴레이)를 입력으로 설정
  pinMode (ip1, INPUT);
  Serial.begin(57600);
  thruster1.attach(7); // 스러스터 6번핀
  thruster2.attach(8);// 스러스터2 7번핀
  thruster1.writeMicroseconds(1500); // send "stop" signal to ESC
  thruster2.writeMicroseconds(1500); 
  servo.writeMicroseconds(0); // initial state is Neutral 
  servo.attach(10); //서보모터 8번핀
  
  
}

void loop()
{
  ch5 = pulseIn(ip5,HIGH);
  {
    if(ch5 < 1450)
    {
      autonomous();
      }
    else if(ch5 > 1450)
    {
        rc();
      }
  }

}
