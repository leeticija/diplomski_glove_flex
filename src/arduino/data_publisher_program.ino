#include <Wire.h>
#include "SparkFun_Displacement_Sensor_Arduino_Library.h"
#include <ros.h>
#include <std_msgs/Float32.h>
#include <MemoryFree.h>

ADS gloveSensor;   // Sensor at 0x12
ADS gripperSensor; // Sensor at 0x13
// ROS node & messages
ros::NodeHandle nh;
std_msgs::Float32 glove_msg;
std_msgs::Float32 gripper_msg;
ros::Publisher glove_pub("glove_data_ome", &glove_msg);
ros::Publisher gripper_pub("gripper_data", &gripper_msg);

#define WINDOW_SIZE_MEDIAN 10
int medianIndex = 0;
int medianCount = 0;
float lastValidReading = NAN;
const float OUTLIER_THRESHOLD = 20.0;  // Adjust based on your sensor range
float gloveMedianReadings[WINDOW_SIZE_MEDIAN];
float emaValue = 0;
float temp[WINDOW_SIZE_MEDIAN];
bool emaInitialized = false;
float alpha = 0.3;  // Adjust smoothing factor (0.1–0.3 typical)

float combinedOME(float newReading) {
  // --- Outlier rejection ---
  if (!isnan(lastValidReading)) {
    float diff = fabs(newReading - lastValidReading);
    if (diff > OUTLIER_THRESHOLD) {
      // Discard outlier
      return emaValue;  // Return last good filtered value
    }
  }

  lastValidReading = newReading;  // Update only if not an outlier

  // --- Median Filter ---
  gloveMedianReadings[medianIndex] = newReading;
  medianIndex = (medianIndex + 1) % WINDOW_SIZE_MEDIAN;
  if (medianCount < WINDOW_SIZE_MEDIAN) medianCount++;

  for (int i = 0; i < medianCount; i++) {
    temp[i] = gloveMedianReadings[i];
  }

  for (int i = 0; i < medianCount - 1; i++) {
    for (int j = i + 1; j < medianCount; j++) {
      if (temp[j] < temp[i]) {
        float t = temp[i];
        temp[i] = temp[j];
        temp[j] = t;
      }
    }
  }

  float median;
  if (medianCount % 2 == 1) {
    median = temp[medianCount / 2];
  } else {
    int mid = medianCount / 2;
    median = (temp[mid - 1] + temp[mid]) / 2.0;
  }

  // --- EMA ---
  if (!emaInitialized) {
    emaValue = median;
    emaInitialized = true;
  } else {
    emaValue = alpha * median + (1 - alpha) * emaValue;
  }

  return emaValue;
}

// #define WINDOW_SIZE_MEDIAN 5
// float gloveMedianReadings[WINDOW_SIZE_MEDIAN];
// int medianIndex = 0;
// int medianCount = 0;

// float emaValue = 0;
// bool emaInitialized = false;
// const float alpha = 0.1;  // Smoothing factor

// float combinedMedianEMA(float newReading) {
//   // --- Median Filter ---
//   gloveMedianReadings[medianIndex] = newReading;
//   medianIndex = (medianIndex + 1) % WINDOW_SIZE_MEDIAN;
//   if (medianCount < WINDOW_SIZE_MEDIAN) medianCount++;

//   float temp[WINDOW_SIZE_MEDIAN];
//   for (int i = 0; i < medianCount; i++) {
//     temp[i] = gloveMedianReadings[i];
//   }

//   // Simple bubble sort
//   for (int i = 0; i < medianCount - 1; i++) {
//     for (int j = i + 1; j < medianCount; j++) {
//       if (temp[j] < temp[i]) {
//         float t = temp[i];
//         temp[i] = temp[j];
//         temp[j] = t;
//       }
//     }
//   }

//   float median;
//   if (medianCount % 2 == 1) {
//     median = temp[medianCount / 2];
//   } else {
//     int mid = medianCount / 2;
//     median = (temp[mid - 1] + temp[mid]) / 2.0;
//   }

//   // --- EMA Filter ---
//   if (!emaInitialized) {
//     emaValue = median;
//     emaInitialized = true;
//   } else {
//     emaValue = alpha * median + (1 - alpha) * emaValue;
//   }

//   return emaValue;
// }


// float emaValue = 0;
// bool emaInitialized = false;
// const float alpha = 0.09;  // Smoothing factor (between 0 and 1)

// float exponentialMovingAverage(float newReading) {
//   if (!emaInitialized) {
//     emaValue = newReading;
//     emaInitialized = true;
//   } else {
//     emaValue = alpha * newReading + (1 - alpha) * emaValue;
//   }
//   return emaValue;
// }

// #define WINDOW_SIZE_MEDIAN 10  // Choose an odd number like 3, 5, 7, etc.
// float gloveMedianReadings[WINDOW_SIZE_MEDIAN];
// int medianIndex = 0;
// int medianCount = 0;

// float medianFilter(float newReading) {
//   gloveMedianReadings[medianIndex] = newReading;
//   medianIndex = (medianIndex + 1) % WINDOW_SIZE_MEDIAN;

//   if (medianCount < WINDOW_SIZE_MEDIAN) medianCount++;

//   // Copy data to temp array for sorting
//   float temp[WINDOW_SIZE_MEDIAN];
//   for (int i = 0; i < medianCount; i++) {
//     temp[i] = gloveMedianReadings[i];
//   }

//   // Sort temp array (simple bubble sort for small arrays)
//   for (int i = 0; i < medianCount - 1; i++) {
//     for (int j = i + 1; j < medianCount; j++) {
//       if (temp[j] < temp[i]) {
//         float t = temp[i];
//         temp[i] = temp[j];
//         temp[j] = t;
//       }
//     }
//   }
//   // Serial.print("Median Count: ");
//   // Serial.println(medianCount);
//   // Serial.print("New Reading: ");
//   // Serial.println(newReading);
//   // Serial.print("Median Value: ");
//   // Serial.println(temp[medianCount / 2]);

//   // Return median
//   if (medianCount % 2 == 1) {
//     return temp[medianCount / 2];
//   } else {
//     int mid = medianCount / 2;
//     return (temp[mid - 1] + temp[mid]) / 2.0;
//   }
// }

// #define WINDOW_SIZE_AVERAGE 10
// float gloveReadings[WINDOW_SIZE_AVERAGE];  // Circular buffer
// int index = 0;
// int count = 0;

// float movingAverage(float newReading) {
//   gloveReadings[index] = newReading;
//   index = (index + 1) % WINDOW_SIZE_AVERAGE;

//   if (count < WINDOW_SIZE_AVERAGE) count++;

//   float sum = 0;
//   for (int i = 0; i < count; i++) {
//     sum += gloveReadings[i];
//   }

//   return sum / count;
// }

void setup()
{
  Serial.begin(57600);
  while (!Serial);

  Wire.begin();

  nh.initNode();
  nh.advertise(glove_pub);
  nh.advertise(gripper_pub);

  // Initialize gloveSensor at 0x12
  if (!gloveSensor.begin(0x12)) {
    // Serial.println("Glove sensor at 0x12 not found. Check connection.");
    while (1);
  }

  // Initialize gripperSensor at 0x13
  if (!gripperSensor.begin(0x13)) {
    // Serial.println("Gripper sensor at 0x13 not found. Check connection.");
    while (1);
  }
  
}

void loop()
{
  gloveSensor.run();
  gripperSensor.run();
  if (gloveSensor.available() && gripperSensor.available()) {
    float gloveX = gloveSensor.getX();
    float gripperX = gripperSensor.getX();

    float gloveOME = combinedOME(gloveX);

    glove_msg.data = gloveOME;
    gripper_msg.data = gripperX;

    glove_pub.publish(&glove_msg);
    gripper_pub.publish(&gripper_msg);

    Serial.print("Glove OME: ");
    Serial.println(gloveOME);
    Serial.print("Gripper X: ");
    Serial.println(gripperX);
  }

  // if (gloveSensor.available() && gripperSensor.available()) {
  //   Serial.println("Sensors available");
  //   gloveX = gloveSensor.getX();
  //   gripperX = gripperSensor.getX();
  //   float gloveOME = combinedOME(gloveX);

  //   // sensor_data[0] = gloveOME;
  //   // sensor_data[1] = gripperX;
  //   Serial.println(gloveX);

  //   // message_pub.publish(&message);
  // }

  // else {
  //   sensor_data[0] = 0;
  //   sensor_data[1] = 0;
  //   Serial.println(sensor_data[0]);
  //   message_pub.publish(&message);
  // }

  // else {
  //   // error happened
  //   glove_msg.data = 0;
  //   gripper_msg.data = 0;
  //   glove_pub.publish(&glove_msg);
  //   gripper_pub.publish(&gripper_msg);
  // }

  nh.spinOnce();
  delay(10); // Optional pause before repeating
}
