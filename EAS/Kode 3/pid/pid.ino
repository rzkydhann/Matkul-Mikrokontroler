#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>

// --- KONFIGURASI WIFI & MQTT ---
const char* ssid = "DANERA 5G";
const char* password = "danera1234";
#define mqttServer "broker.emqx.io"
#define mqttPort 1883
#define mqttClientName "ESP32_PID_Rizky_UAS"

// --- PIN DEFINITIONS ---
const int ledPin = 2;
const int motor1Pin1 = 27;
const int motor1Pin2 = 26;
const int enable1Pin = 12; // PWM Pin
const int pin_rpm = 13;    // Encoder Input

// --- KONFIGURASI MOTOR & ENCODER ---
// PENTING: Ubah ini sesuai jumlah lubang piringan encoder anda!
const float PULSES_PER_REV = 20.0; 

// PWM Properties
const int freq = 30000;
const int pwmChannel = 0;
const int resolution = 8; // 0-255

// --- VARIABEL PID ---
double Kp = 1.5;  // Default Kp
double Ki = 0.5;  // Default Ki
double Kd = 0.05; // Default Kd

double target_rpm = 0; // Target kecepatan
double current_rpm = 0;
double error = 0, last_error = 0, integral = 0, derivative = 0;
double output_pwm = 0;

// Variabel Timing & Encoder
volatile long pulse_count = 0;
unsigned long last_time = 0;
unsigned long sample_time = 100; // Hitung PID setiap 100ms
bool motor_active = false;

// Objek WiFi & MQTT
WiFiClient espClient;
PubSubClient client(espClient);

// Interrupt Service Routine (Agar pembacaan sensor cepat)
void IRAM_ATTR isr() {
  pulse_count++;
}

void setup() {
  Serial.begin(115200);
  
  pinMode(ledPin, OUTPUT);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(pin_rpm, INPUT_PULLUP);
  
  // Setup Interrupt
  attachInterrupt(digitalPinToInterrupt(pin_rpm), isr, RISING);

  // Setup PWM Motor
  ledcSetup(pwmChannel, freq, resolution);
  ledcAttachPin(enable1Pin, pwmChannel);

  // Koneksi WiFi
  setupWifi();
  
  // Koneksi MQTT
  client.setServer(mqttServer, mqttPort);
  client.setCallback(receivedCallback);
}

void setupWifi() {
  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect(mqttClientName)) {
      Serial.println("connected");
      // Subscribe ke topik-topik kontrol
      client.subscribe("ImcRizky/command");   // '1' Start, '0' Stop
      client.subscribe("ImcRizky/setpoint");  // Input Target RPM
      client.subscribe("ImcRizky/kp");        // Input nilai Kp
      client.subscribe("ImcRizky/ki");        // Input nilai Ki
      client.subscribe("ImcRizky/kd");        // Input nilai Kd
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void receivedCallback(char* topic, byte* payload, unsigned int length) {
  // Konversi payload ke String supaya mudah dibaca
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  
  Serial.print("Message ["); Serial.print(topic); Serial.print("] "); Serial.println(message);

  // --- LOGIKA PARSING PESAN ---
  if (String(topic) == "ImcRizky/command") {
    if (message == "1") {
      motor_active = true;
      Serial.println("SYSTEM START");
    } else if (message == "0") {
      motor_active = false;
      target_rpm = 0;
      stopMotor();
      integral = 0; // Reset memori PID
      Serial.println("SYSTEM STOP");
    }
  }
  else if (String(topic) == "ImcRizky/setpoint") {
    target_rpm = message.toFloat();
  }
  // --- LIVE TUNING PID ---
  else if (String(topic) == "ImcRizky/kp") {
    Kp = message.toFloat();
  }
  else if (String(topic) == "ImcRizky/ki") {
    Ki = message.toFloat();
  }
  else if (String(topic) == "ImcRizky/kd") {
    Kd = message.toFloat();
  }
}

void driveMotor(int pwmVal) {
  // Batasi PWM 0-255 (karena resolusi 8 bit)
  if (pwmVal > 255) pwmVal = 255;
  if (pwmVal < 0) pwmVal = 0;

  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, pwmVal);
}

void stopMotor() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  ledcWrite(pwmChannel, 0);
}

void loop() {
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  // --- LOOP PID SETIAP X MILIDETIK ---
  unsigned long now = millis();
  if (now - last_time >= sample_time) {
    
    // 1. HITUNG RPM (Aman dari gangguan interrupt)
    noInterrupts(); // Stop interrupt sebentar
    long pulses = pulse_count;
    pulse_count = 0;
    interrupts();   // Nyalakan lagi
    
    // Rumus: (Pulsa / PPR) * (60000ms / delta_waktu)
    current_rpm = (pulses / PULSES_PER_REV) * (60000.0 / (now - last_time));

    // 2. LOGIKA PID
    if (motor_active) {
      error = target_rpm - current_rpm;
      
      // Integral (Penjumlahan error seiring waktu)
      integral += error * (sample_time / 1000.0);
      
      // Batasi Integral (Anti-Windup) supaya tidak 'menggila'
      if(integral > 1000) integral = 1000;
      if(integral < -1000) integral = -1000;

      // Derivative (Kecepatan perubahan error)
      derivative = (error - last_error) / (sample_time / 1000.0);

      // Rumus Utama PID
      double pid_out = (Kp * error) + (Ki * integral) + (Kd * derivative);
      
      // Simpan last error
      last_error = error;

      // Eksekusi ke Motor
      driveMotor((int)pid_out);
      output_pwm = pid_out;
      
      digitalWrite(ledPin, HIGH); // LED indikator nyala
    } else {
      stopMotor();
      digitalWrite(ledPin, LOW);
    }

    // 3. KIRIM DATA KE HP (Untuk Grafik)
    // Kirim hanya jika ada perubahan signifikan atau setiap loop
    client.publish("ImcRizky/graph/rpm", String(current_rpm).c_str());
    client.publish("ImcRizky/graph/target", String(target_rpm).c_str());

    // Debugging di Serial Monitor Arduino
    Serial.print("Tgt:"); Serial.print(target_rpm);
    Serial.print(" RPM:"); Serial.print(current_rpm);
    Serial.print(" PWM:"); Serial.print(output_pwm);
    Serial.print(" Kp:"); Serial.print(Kp);
    Serial.print(" Ki:"); Serial.println(Ki);

    last_time = now;
  }
}