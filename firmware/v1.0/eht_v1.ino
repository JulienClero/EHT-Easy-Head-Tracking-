#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// ======= Réseau =======
const char* ssid     = "Livebox-197A";
const char* password = "62AB114F8D346586595ED49934";
IPAddress pcIP(192, 168, 1, 21);
const uint16_t pcPort   = 4242;
WiFiUDP udp;
const uint16_t localPort = 4242;

// ======= MPU6050 =======
#define MPU_ADDR       0x68
#define PWR_MGMT_1     0x6B
#define ACCEL_XOUT_H   0x3B
#define GYRO_XOUT_H    0x43

// Variables de tracking
double headData[3];
unsigned long lastTime;

// Angles finaux en degrés
float yaw = 0, pitch = 0, roll = 0;

// Paramètres du filtre complémentaire
const float alpha = 0.99f;  // 99% gyroscope, 1% accéléromètre

// Gestion d'erreurs I2C
int i2c_error_count = 0;
const int MAX_I2C_ERRORS = 3;
bool mpu_initialized = false;

// Buffers senseurs
int16_t ax = 0, ay = 0, az = 0;
int16_t gx = 0, gy = 0, gz = 0;

// I2C write avec gestion d'erreur
bool writeByte(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  uint8_t error = Wire.endTransmission();
  
  if (error != 0) {
    i2c_error_count++;
    return false;
  }
  return true;
}

// I2C read avec timeout
bool readBytes(uint8_t addr, uint8_t reg, uint8_t len, uint8_t* buf) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    i2c_error_count++;
    return false;
  }
  
  if (Wire.requestFrom(addr, len) != len) {
    i2c_error_count++;
    return false;
  }
  
  for (uint8_t i = 0; i < len; i++) {
    buf[i] = Wire.read();
  }
  return true;
}

// Initialisation MPU6050
bool initMPU() {
  // Reset et sortie de veille
  if (!writeByte(MPU_ADDR, PWR_MGMT_1, 0x80)) return false; // Reset
  delay(50);
  if (!writeByte(MPU_ADDR, PWR_MGMT_1, 0x00)) return false; // Wake up
  delay(10);
  
  // Configuration : ±2g accelerometer, ±250°/s gyroscope
  if (!writeByte(MPU_ADDR, 0x1A, 0x03)) return false; // DLPF = 44Hz
  if (!writeByte(MPU_ADDR, 0x1B, 0x00)) return false; // Gyro ±250°/s
  if (!writeByte(MPU_ADDR, 0x1C, 0x00)) return false; // Accel ±2g
  
  return true;
}

// Réinitialisation I2C
void resetI2C() {
  Wire.end();
  delay(30);
  Wire.begin(21, 22);
  Wire.setClock(100000);
  delay(30);
  
  mpu_initialized = initMPU();
  i2c_error_count = 0;
}

// Filtre complémentaire pour pitch et roll
void updateAngles(float gx_dps, float gy_dps, float gz_dps, float ax_g, float ay_g, float az_g, float dt) {
  // Calcul des angles depuis l'accéléromètre (référence sans drift)
  float pitch_acc = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0f / PI;
  float roll_acc = atan2(ay_g, az_g) * 180.0f / PI;
  
  // Filtre complémentaire pour pitch et roll
  pitch = alpha * (pitch + gy_dps * dt) + (1.0f - alpha) * pitch_acc;
  roll = alpha * (roll + gx_dps * dt) + (1.0f - alpha) * roll_acc;
  
  // Yaw : intégration pure du gyroscope (drift accepté)
  yaw += gz_dps * dt;
  
  // Normalisation du yaw entre -180 et 180 degrés
  if (yaw > 180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;
}

void setup() {
  Serial.begin(115200);
  
  // I2C setup
  Wire.begin(21, 22);
  Wire.setClock(100000);
  
  // Initialisation MPU6050
  mpu_initialized = initMPU();
  if (!mpu_initialized) {
    Serial.println("MPU6050 init failed!");
  } else {
    Serial.println("MPU6050 OK - Filtre complémentaire");
  }

  // Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");
  udp.begin(localPort);

  lastTime = millis();
}

void loop() {
  static unsigned long lastLoop = 0;
  const unsigned long LOOP_INTERVAL_MS = 10;
  unsigned long nowLoop = millis();
  if (nowLoop - lastLoop < LOOP_INTERVAL_MS) return;
  lastLoop = nowLoop;

  // Reset I2C si trop d'erreurs
  if (i2c_error_count > MAX_I2C_ERRORS) {
    resetI2C();
    return;
  }

  // Lecture des données MPU6050
  if (!mpu_initialized) return;

  uint8_t accel_buf[6], gyro_buf[6];
  bool accel_ok = readBytes(MPU_ADDR, ACCEL_XOUT_H, 6, accel_buf);
  bool gyro_ok = readBytes(MPU_ADDR, GYRO_XOUT_H, 6, gyro_buf);

  if (!accel_ok || !gyro_ok) return;

  // Conversion des données
  ax = (accel_buf[0] << 8) | accel_buf[1];
  ay = (accel_buf[2] << 8) | accel_buf[3];
  az = (accel_buf[4] << 8) | accel_buf[5];
  
  gx = (gyro_buf[0] << 8) | gyro_buf[1];
  gy = (gyro_buf[2] << 8) | gyro_buf[3];
  gz = (gyro_buf[4] << 8) | gyro_buf[5];

  // Conversion en unités physiques
  float ax_g = ax / 16384.0f;
  float ay_g = ay / 16384.0f;
  float az_g = az / 16384.0f;
  float gx_dps = gx / 131.0f;
  float gy_dps = gy / 131.0f;
  float gz_dps = gz / 131.0f;

  // Calcul du temps écoulé
  float dt = (millis() - lastTime) / 1000.0f;
  lastTime = millis();

  // Mise à jour des angles avec filtre complémentaire
  updateAngles(gx_dps, gy_dps, gz_dps, ax_g, ay_g, az_g, dt);

  // Préparer les données pour OpenTrack
  headData[0] = yaw * 2.0;   // Yaw
  headData[1] = pitch * 3.0; // Pitch
  headData[2] = roll * 3.0;  // Roll

  // Envoi UDP
  if (udp.beginPacket(pcIP, pcPort)) {
    udp.write((uint8_t*)headData, sizeof(headData));
    udp.endPacket();
  }

  yield();
}