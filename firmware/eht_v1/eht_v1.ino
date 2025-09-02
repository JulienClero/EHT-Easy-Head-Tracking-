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

// Variables de tracking (thread-safe avec mutex)
portMUX_TYPE angleMux = portMUX_INITIALIZER_UNLOCKED;
double headData[3];
volatile float yaw = 0, pitch = 0, roll = 0;

// Paramètres du filtre complémentaire
const float alpha = 0.98f;

// Variables pour calibration automatique du drift
float drift_sum = 0.0f;
int drift_samples = 0;
float drift_correction = 0.0f;
bool calibration_complete = false;
unsigned long calibration_start = 0;
const unsigned long CALIBRATION_DURATION = 10000; // 10 secondes

// Gestion d'erreurs I2C
int i2c_error_count = 0;
const int MAX_I2C_ERRORS = 3;
bool mpu_initialized = false;

// Queue pour communication inter-core
QueueHandle_t sensorQueue;
struct SensorData {
  float ax_g, ay_g, az_g;
  float gx_dps, gy_dps, gz_dps;
  unsigned long timestamp;
};

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
  if (!writeByte(MPU_ADDR, PWR_MGMT_1, 0x80)) return false; 
  delay(50);
  if (!writeByte(MPU_ADDR, PWR_MGMT_1, 0x00)) return false; 
  delay(10);
  
  // Configuration optimisée pour la précision
  if (!writeByte(MPU_ADDR, 0x1A, 0x01)) return false; // DLPF = 188Hz
  if (!writeByte(MPU_ADDR, 0x1B, 0x00)) return false; // Gyro ±250°/s
  if (!writeByte(MPU_ADDR, 0x1C, 0x00)) return false; // Accel ±2g
  if (!writeByte(MPU_ADDR, 0x19, 0x07)) return false; // Sample rate = 125Hz
  
  return true;
}

// Réinitialisation I2C
void resetI2C() {
  Wire.end();
  delay(30);
  Wire.begin(10, 11);
  Wire.setClock(400000);
  delay(30);
  
  mpu_initialized = initMPU();
  i2c_error_count = 0;
}

// Runge-Kutta 2ème ordre pour l'intégration du yaw
float rungeKutta2_yaw(float current_yaw, float gz_dps, float dt) {
  // K1 = f(t, y) = gz_dps
  float k1 = gz_dps;
  
  // K2 = f(t + dt/2, y + k1*dt/2) 
  // Pour une vitesse angulaire constante sur dt, k2 = k1
  float k2 = gz_dps;
  
  // Intégration RK2
  return current_yaw + dt * (k1 + k2) / 2.0f;
}

// Filtre complémentaire avec Runge-Kutta pour le yaw
void updateAngles(float gx_dps, float gy_dps, float gz_dps, float ax_g, float ay_g, float az_g, float dt) {
  // Phase de calibration du drift (10 secondes)
  if (!calibration_complete) {
    drift_sum += gz_dps;
    drift_samples++;
    
    // Vérifier si la calibration est terminée
    if (millis() - calibration_start >= CALIBRATION_DURATION) {
      drift_correction = drift_sum / drift_samples;
      calibration_complete = true;
      Serial.printf("Calibration terminée - Drift correction: %.4f deg/s\n", drift_correction);
    }
    return; // Ne pas calculer les angles pendant la calibration
  }
  
  // Calcul des angles depuis l'accéléromètre
  float pitch_acc = atan2(-ax_g, sqrt(ay_g*ay_g + az_g*az_g)) * 180.0f / PI;
  float roll_acc = atan2(ay_g, az_g) * 180.0f / PI;
  
  // Filtre complémentaire pour pitch et roll
  pitch = alpha * (pitch + gy_dps * dt) + (1.0f - alpha) * pitch_acc;
  roll = alpha * (roll + gx_dps * dt) + (1.0f - alpha) * roll_acc;
  
  gz_dps = gz_dps - drift_correction;
  // Intégration Runge-Kutta 2ème ordre pour le yaw
  yaw = rungeKutta2_yaw(yaw, gz_dps, dt);
  
  // Normalisation du yaw
  if (yaw > 180.0f) yaw -= 360.0f;
  if (yaw < -180.0f) yaw += 360.0f;
}

// CORE 0: Acquisition des données senseur (haute priorité)
void sensorTask(void *pvParameters) {
  for(;;) {
    // Reset I2C si trop d'erreurs
    if (i2c_error_count > MAX_I2C_ERRORS) {
      resetI2C();
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    if (!mpu_initialized) {
      vTaskDelay(pdMS_TO_TICKS(100));
      continue;
    }

    // Lecture des données MPU6050
    uint8_t accel_buf[6], gyro_buf[6];
    bool accel_ok = readBytes(MPU_ADDR, ACCEL_XOUT_H, 6, accel_buf);
    bool gyro_ok = readBytes(MPU_ADDR, GYRO_XOUT_H, 6, gyro_buf);

    if (!accel_ok || !gyro_ok) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // Conversion des données
    int16_t raw_ax = (accel_buf[0] << 8) | accel_buf[1];
    int16_t raw_ay = (accel_buf[2] << 8) | accel_buf[3];
    int16_t raw_az = (accel_buf[4] << 8) | accel_buf[5];
    
    int16_t raw_gx = (gyro_buf[0] << 8) | gyro_buf[1];
    int16_t raw_gy = (gyro_buf[2] << 8) | gyro_buf[3];
    int16_t raw_gz = (gyro_buf[4] << 8) | gyro_buf[5];

    // Préparation des données pour le calcul
    SensorData data;
    data.ax_g = raw_ax / 16384.0f;
    data.ay_g = raw_ay / 16384.0f;
    data.az_g = raw_az / 16384.0f;
    data.gx_dps = raw_gx / 131.0f;
    data.gy_dps = raw_gy / 131.0f;
    data.gz_dps = raw_gz / 131.0f;
    data.timestamp = millis();

    // Envoi vers le core de calcul (non-bloquant)
    xQueueSend(sensorQueue, &data, 0);

    // Fréquence ~500Hz pour l'acquisition
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// CORE 1: Calcul et transmission (loop principal)
void setup() {
  Serial.begin(115200);
  
  // I2C setup
  Wire.begin(10, 11);
  Wire.setClock(400000);
  
  // Initialisation MPU6050
  mpu_initialized = initMPU();
  if (!mpu_initialized) {
    Serial.println("MPU6050 init failed!");
  } else {
    Serial.println("MPU6050 OK - RK2 Simple");
  }

  // Wi-Fi
  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  unsigned long wifiStart = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifiStart < 10000) {
    delay(500);
    Serial.print(".");
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi OK");
    udp.begin(localPort);
  } else {
    Serial.println("\nWiFi FAILED");
  }

  // Création de la queue inter-core
  sensorQueue = xQueueCreate(10, sizeof(SensorData));
  
  // Lancement de la tâche d'acquisition sur le Core 0
  xTaskCreatePinnedToCore(
    sensorTask,     // Fonction
    "SensorTask",   // Nom
    4096,          // Stack size
    NULL,          // Paramètres
    2,             // Priorité (haute)
    NULL,          // Handle
    0              // Core 0
  );

  Serial.println("Dual-core setup OK");
  
  // Démarrage de la calibration du drift
  calibration_start = millis();
  Serial.println("Calibration du drift en cours (10s)... Ne pas bouger le capteur!");
}

void loop() {
  static unsigned long lastTime = millis();
  SensorData data;
  
  // Réception des données du core d'acquisition
  if (xQueueReceive(sensorQueue, &data, pdMS_TO_TICKS(5)) == pdPASS) {
    
    // Calcul du temps écoulé
    float dt = (data.timestamp - lastTime) / 1000.0f;
    dt = constrain(dt, 0.001f, 0.1f); // Limitation pour éviter les valeurs aberrantes
    lastTime = data.timestamp;

    // Mise à jour des angles avec protection thread-safe
    portENTER_CRITICAL(&angleMux);
    updateAngles(data.gx_dps, data.gy_dps, data.gz_dps, 
                 data.ax_g, data.ay_g, data.az_g, dt);
    
    // Copie des angles pour transmission
    headData[0] = yaw * 2.0;
    headData[1] = pitch * 3.0;
    headData[2] = roll * 3.0;
    portEXIT_CRITICAL(&angleMux);

    // Debug périodique
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug > 1000) {
      if (calibration_complete) {
        Serial.printf("Yaw: %.2f Pitch: %.2f Roll: %.2f | dt: %.3f | Drift: %.4f\n", 
                      yaw, pitch, roll, dt, drift_correction);
      } else {
        unsigned long remaining = CALIBRATION_DURATION - (millis() - calibration_start);
        Serial.printf("Calibration... %lu ms restantes\n", remaining);
      }
      lastDebug = millis();
    }
  }

  // Envoi UDP seulement après calibration
  static unsigned long lastUdp = 0;
  if (calibration_complete && millis() - lastUdp > 10) {
    if (udp.beginPacket(pcIP, pcPort)) {
      udp.write((uint8_t*)headData, sizeof(headData));
      udp.endPacket();
    }
    lastUdp = millis();
  }

  // Petite pause pour ne pas surcharger
  delay(1);
}