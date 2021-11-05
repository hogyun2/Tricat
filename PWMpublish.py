import rospy
from std_msgs.msg import Float64
from std_msgs.msg import UInt16
import time
import math

class PWM:
    def __init__(self):
        rospy.Subscriber('/V',Float64,self.v_sub)
        rospy.Subscriber('/W',Float64,self.w_sub)
        self.v=0.0
        self.w=0.0
        self.omega=0.0
        self.vel=0.0
        self.ang=0.0
        self.result=0
        self.thruster_pub=rospy.Publisher('/thrusterpwm',UInt16, queue_size=10)
        self.servo_pub=rospy.Publisher('/servopwm',UInt16, queue_size=10)

    def v_sub(self,data):
        self.v = 0.3733 * math.pow(data.data, 2) - 0.6003 * data.data + 1.843
        

    def w_sub(self,data):
        self.omega=data.data
        self.w = 1.0455 * math.pow(data.data, 2) - 0.9804 * data.data + 1.8307
        

    def PWM(self):
        result = int((self.v + self.w) * 500)

        if -0.15<self.omega<0.15:
            self.servo_pub.publish(90)

        elif self.omega>=0.15:
            self.servo_pub.publish(70)

        elif self.omega<=-0.15:
            self.servo_pub.publish(110)

        if result>1600:
            result=1600
        elif result<1400:
            result=1400
        else:
            pass

        self.thruster_pub.publish(result)
        
        
            




pwm=PWM()
def main():
    rospy.init_node("PWMpublish",anonymous=True)

    while not rospy.is_shutdown():
        
        pwm.PWM()
   
        print("omg",pwm.omega)
        time.sleep(1)
        
    



    rospy.spin()

if __name__ == '__main__':
     main()



    
    

    
