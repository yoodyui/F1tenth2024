/****************************************************************
 //BP trying to combine imu (icm20948) with microros on ROS2
 //Testing with MKR zero arduino board
 ***************************************************************/
#include <micro_ros_arduino.h>
#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
//BP added
#include <micro_ros_utilities/type_utilities.h>
//#include <micro_ros_utilities/string_utilities.h>
#include <std_msgs/msg/float64.h>
//#include <std_msgs/msg/string.h>
//#include <std_msgs/msg/int32.h>

rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

//BP added
//rcl_publisher_t string_publisher;
rcl_publisher_t xvel_publisher;
rcl_publisher_t yaw_publisher;
//std_msgs__msg__String string_msg;
std_msgs__msg__Float64 xvel_msg;
std_msgs__msg__Float64 yaw_msg;

//BP complimentary filter
float vel_x = 0.0;  // Initial velocity
unsigned long previousTime;
float deltaTime;  // Time between readings
// Complementary filter constants
const float alpha = 0.98;  // Weight for accelerometer data (0.98 recommended)
const float beta = 1-alpha;   // Weight for gyroscope data (0.02 recommended)
// Constants for moving average
const int numSamples = 10;  // Number of samples to include in the moving average
float acc_xBuffer[numSamples];  // Buffer to store past acceleration values
int bufferIndex = 0;  // Index for the circular buffer
float acc_xSum = 0.0;  // Sum of acceleration values in the buffer
float vel_x_filter = 0.0; 
const float wheelRadius = 0.0525;   // Wheel Diameter in meter value
const int sensorPin = 4;             // Connect the RPM sensor to digital pin 2 //AttachInterrupt Pins -> 0,1,4,5,6,7,8,9,A1,A2 for MKR Family Board
//Encoder
const int encoderPin = 4; // Replace with the actual pin number
const int PPR = 1;        // One pulse per revolution (adjust if needed)

unsigned long prevTime;
long prevPosition = 0;
long encoderPosition = 0;
float x_vel = 0.0;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


#define QUAT_ANIMATION // Uncomment this line to output data in the correct format for ZaneL's Node.js Quaternion animation tool: https://github.com/ZaneL/quaternion_sensor_3d_nodejs

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


void setup()
{

  SERIAL_PORT.begin(115200); // Start the serial console
#ifndef QUAT_ANIMATION
  SERIAL_PORT.println(F("ICM-20948 Example"));
#endif

  delay(100);

#ifndef QUAT_ANIMATION
  while (SERIAL_PORT.available()) // Make sure the serial RX buffer is empty
    SERIAL_PORT.read();

  SERIAL_PORT.println(F("Press any key to continue..."));

  while (!SERIAL_PORT.available()) // Wait for the user to press a key (send any serial character)
    ;
#endif

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

#ifndef QUAT_ANIMATION
  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial
#endif

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

#ifndef QUAT_ANIMATION
    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
#endif
    if (myICM.status != ICM_20948_Stat_Ok)
    {
#ifndef QUAT_ANIMATION
      SERIAL_PORT.println(F("Trying again..."));
#endif
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

#ifndef QUAT_ANIMATION
  SERIAL_PORT.println(F("Device connected!"));
#endif

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. You can overwrite it if you want to e.g. to change the sample rate
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP Game Rotation Vector sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat6, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    #ifndef QUAT_ANIMATION
    SERIAL_PORT.println(F("DMP enabled!"));
    #endif
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }
  
  //BP
  // Initialize the buffer values to zero
  for (int i = 0; i < numSamples; i++) {
    acc_xBuffer[i] = 0.0;
  }
  unsigned long previousTime = millis(); // Initialize the time
  
  //BP add microROS
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(1000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create yaw publisher
  RCCHECK(rclc_publisher_init_default(
    &yaw_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "yaw")); 

  // create xvel publisher
  RCCHECK(rclc_publisher_init_default(
    &xvel_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float64),
    "xvel"));
  //end

  //Encoder
  pinMode(encoderPin, INPUT_PULLUP); // Set encoder pin as input with pull-up resistor
  prevTime = millis();              // Initialize prevTime
}

void loop()
{
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );


    if ((data.header & DMP_header_bitmap_Quat6) > 0) // We have asked for GRV data so we should receive Quat6
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat6 data is: Q1:%ld Q2:%ld Q3:%ld\r\n", data.Quat6.Data.Q1, data.Quat6.Data.Q2, data.Quat6.Data.Q3);

      // Scale to +/- 1
      double q1 = ((double)data.Quat6.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat6.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat6.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

      /*
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.println(q3, 3);
*/

      // Convert the quaternions to Euler angles (roll, pitch, yaw)
      // https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles&section=8#Source_code_2

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double q2sqr = q2 * q2;

      // roll (x-axis rotation)
      double t0 = +2.0 * (q0 * q1 + q2 * q3);
      double t1 = +1.0 - 2.0 * (q1 * q1 + q2sqr);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (q0 * q2 - q3 * q1);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (q0 * q3 + q1 * q2);
      double t4 = +1.0 - 2.0 * (q2sqr + q3 * q3);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      #ifndef QUAT_ANIMATION
      #else
      SERIAL_PORT.print(F("Roll:"));
      SERIAL_PORT.print(roll, 1);
      SERIAL_PORT.print(F(" Pitch:"));
      SERIAL_PORT.print(pitch, 1);
      SERIAL_PORT.print(F(" Yaw:"));
      SERIAL_PORT.println(yaw, 1);

      //BP publish yaw messages
      yaw_msg.data = yaw;
      //Test with fixed yaw
      //yaw_msg.data = 0;
      RCSOFTCHECK(rcl_publish(&yaw_publisher, &yaw_msg, NULL));

      
      // Output the Quaternion data in the format expected by ZaneL's Node.js Quaternion animation tool
      //SERIAL_PORT.print(F("{\"quat_w\":"));
      //SERIAL_PORT.print(q0, 3);
      //SERIAL_PORT.print(F(", \"quat_x\":"));
      //SERIAL_PORT.print(q1, 3);
      //SERIAL_PORT.print(F(", \"quat_y\":"));
      //SERIAL_PORT.print(q2, 3);
      //SERIAL_PORT.print(F(", \"quat_z\":"));
      //SERIAL_PORT.print(q3, 3);
      //SERIAL_PORT.println(F("}"));
      #endif
    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // Check for Accel
    {
      float acc_x = (float)data.Raw_Accel.Data.X; // Extract the raw accelerometer data
      float acc_y = (float)data.Raw_Accel.Data.Y;
      float acc_z = (float)data.Raw_Accel.Data.Z;
      


      // BP: Extract the raw gyro data
      float gyro_x = (float)data.Raw_Gyro.Data.X;
      float gyro_y = (float)data.Raw_Gyro.Data.Y;
      float gyro_z = (float)data.Raw_Gyro.Data.Z;

      // Read the current encoder position
      encoderPosition = readEncoder(); // Implement this function to read the encoder position

      // Calculate the time elapsed since the last reading
      unsigned long currentTime = millis();
      unsigned long deltaTime = currentTime - prevTime;

      // Calculate the change in position since the last reading
      long positionChange = encoderPosition - prevPosition;

      // Calculate linear velocity (distance traveled per second)
      x_vel = (float)positionChange / (float)PPR / (float)deltaTime * 1000.0; // Convert to mm/s (if needed)

      // Update previous position and time
      prevPosition = encoderPosition;
      prevTime = currentTime;


      /*// Calculate the time elapsed since the last reading
      unsigned long currentTime = millis();
      deltaTime = (currentTime - previousTime) / 1000.0; // Convert to seconds
      previousTime = currentTime; // Update the previous time
    
      // Integrate acceleration to get velocity in the x-axis
      vel_x += (acc_x / 16384.0) * 9.81 * deltaTime; // Scale by 16384 (sensitivity) and 9.81 m/s^2 (gravity)

      // Apply complementary filter for drift reduction
      float accelAngleX = atan2(acc_y, acc_z);

      // Add the current accelAngleX to the buffer and update the sum
      acc_xSum -= acc_xBuffer[bufferIndex]; // Subtract the oldest value
      acc_xBuffer[bufferIndex] = accelAngleX; // Store the current value
      acc_xSum += accelAngleX; // Add the new value

      // Increment the buffer index (circular buffer)
      bufferIndex = (bufferIndex + 1) % numSamples;

      // Calculate the moving average of vel_x
      float averageAcc_x = acc_xSum / numSamples;

      // Only apply the complementary filter if the accelerometer measurements are reliable
      if (abs(averageAcc_x) > 0.01) {
        // Apply the complementary filter to reduce drift
        // The filter combines the accelerometer-based velocity estimate with the gyro-based estimate
        vel_x_filter = alpha * (vel_x + averageAcc_x * deltaTime) + beta * gyro_x;

        // If you notice excessive drift, you can experiment with different values of alpha and beta
        // to fine-tune the filter's performance.
      } else {
        // If accelerometer measurements are unreliable (e.g., during rapid movement), reset the velocity.
        vel_x_filter = 0;
      }
      */
      


      //BP publish xvel messages
      xvel_msg.data = vel_x_filter;
      //Test with fixed xvel
      //xvel_msg.data = 0.05;
      RCSOFTCHECK(rcl_publish(&xvel_publisher, &xvel_msg, NULL));
      
      #ifndef QUAT_ANIMATION
      #else
      //BP wants to display Euler's angles only
      SERIAL_PORT.print(F(" Vel_x_filter:"));
      SERIAL_PORT.println(vel_x_filter);
      SERIAL_PORT.print(F(" vel_x:"));
      SERIAL_PORT.print(vel_x);
      SERIAL_PORT.print(F(" Acc_X:"));
      SERIAL_PORT.println(averageAcc_x);
      #endif
      }
    }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }
}

long readEncoder() {
  // Read the current state of the encoder pin
  int encoderState = digitalRead(encoderPin);

  // Determine if the encoder has moved
  if (encoderState != prevEncoderState) {
    // Check if the other pin is also HIGH to detect a valid pulse
    if (digitalRead(encoderPinB) == HIGH) {
      // Increment or decrement the encoder position based on direction
      if (encoderState == HIGH) {
        encoderPosition++;
      } else {
        encoderPosition--;
      }
    }
    // Update the previous encoder state
    prevEncoderState = encoderState;
  }

  return encoderPosition;
}
