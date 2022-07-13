#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#define ROSSERIAL_ARDUINO_TCP
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>

const char*  ssid = "Ajin j";
const char*  password = "ajinj1011";

IPAddress server(192, 168, 209, 83); // IP of computer on which ROS Core is running
const uint16_t serverPort = 11411;

geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

ros::NodeHandle nh;

geometry_msgs::Quaternion orientation;
ros::Publisher imu_ori("imu_orientation",&orientation);

geometry_msgs::Vector3 angular_velocity;
ros::Publisher imu_gyr("imu_gyro",&angular_velocity);

geometry_msgs::Vector3 linear_acceleration;
ros::Publisher imu_acc("imu_accl",&linear_acceleration);

char frameid[] = "/base_link";
char child[] = "/imu_frame";

#define INTERRUPT_PIN D8 

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

volatile bool mpuInterrupt = false;
void ICACHE_RAM_ATTR dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {
  Serial.begin(115200);
  wifi_def(); 
    //Wire.begin();    
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    nh.getHardware()->setConnection(server, serverPort);
    nh.initNode();
    broadcaster.init(nh);
    nh.advertise(imu_ori);
    nh.advertise(imu_gyr);
    nh.advertise(imu_acc);
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);
    devStatus = mpu.dmpInitialize();   

    // supply your own gyro offsets here, scaled for min sensitivity
//    mpu.setXGyroOffset(220);
//    mpu.setYGyroOffset(76);
//    mpu.setZGyroOffset(-85);
//    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
    
    if (devStatus == 0) {  
//        mpu.CalibrateAccel(6);
//        mpu.CalibrateGyro(6);
//        mpu.PrintActiveOffsets();  
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
    }
}

void loop() {

    if (!dmpReady) return;
    
    while (!mpuInterrupt && fifoCount < packetSize) {}

    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        mpu.resetFIFO();
    } 
    else if (mpuIntStatus & 0x01) 
    {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        fifoCount -= packetSize;            
            
            // display quaternion values in easy matrix form: w x y z
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            mpu.dmpGetAccel(&aa, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
            mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
                        
            orientation.x = q.x;
            orientation.y = q.y;
            orientation.z = q.z;
            orientation.w = q.w;  
              
            imu_ori.publish(&orientation);
                        
            angular_velocity.x = ypr[0];
            angular_velocity.y = ypr[1];
            angular_velocity.z = ypr[2];            
            imu_gyr.publish(&angular_velocity);
            
            linear_acceleration.x = aaReal.x * 1/16384. * 9.80665;
            linear_acceleration.y = aaReal.y * 1/16384. * 9.80665;
            linear_acceleration.z = aaReal.z * 1/16384. * 9.80665;
            imu_acc.publish(&linear_acceleration);

            //Assigning values to TF header and publish the transform
            t.header.frame_id = frameid;
            t.child_frame_id = child;
            t.transform.translation.x = 0.5; 
            t.transform.rotation.x = q.x;
            t.transform.rotation.y = q.y; 
            t.transform.rotation.z = q.z; 
            t.transform.rotation.w = q.w;  
            t.header.stamp = nh.now();
            broadcaster.sendTransform(t);

            nh.spinOnce();
   
            delay(200);         
    }
}
void wifi_def() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {                             // Exit only when connected
    delay(500);
    Serial.print(".");
  }

  Serial.print("\nConnected to "); Serial.println(ssid);
  Serial.print("IP address: "); Serial.println(WiFi.localIP());
  delay(3000);
}
