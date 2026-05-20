#include <Wire.h>
#include <MPU6050.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// ============================================
// HARDWARE INITIALIZATION
// ============================================
MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ============================================
// VERY LOW SENSITIVITY SETTINGS (MAXIMUM STABILITY)
// ============================================

// Sensitivity controls (HIGHER = LESS SENSITIVE)
float motionSensitivity = 9.0;       // Near maximum (least sensitive)
float angleChangeThreshold = 5.0;    // Needs 5° change to trigger
float velocityThreshold = 6.0;       // Needs faster movement

// Heavy smoothing (very stable)
float emaAlpha = 0.08;               // Very heavy smoothing
float compFilterAlpha = 0.98;        // 98% gyro trust (very stable)

// Large hysteresis (no jitter at all)
float angleHysteresis = 2.0;         // Ignores changes under 2°

// Velocity noise gate (ignores all tiny movements)
float velocityNoiseGate = 2.0;       // Ignores slow velocity changes

// ============================================
// 3D ANGLE TRACKING VARIABLES
// ============================================
float tiltX = 0;
float tiltY = 0;
float totalTilt = 0;
float previousTotalTilt = 0;

float gravityX = 0, gravityY = 0, gravityZ = 0;
float gravityMagnitude = 0;

float currentAngle = 0;
float previousAngle = 0;
float stableAngle = 0;

float medianWindow[7] = {0};
int medianIndex = 0;
float emaAngle = 0;
float angleVelocity = 0;
float filteredVelocity = 0;

float accelBiasX = 0, accelBiasY = 0, accelBiasZ = 0;
float gyroBiasX = 0, gyroBiasY = 0, gyroBiasZ = 0;
float filteredAngle = 0;

unsigned long lastSampleTime = 0;
unsigned long sampleInterval = 40;    // 25 Hz (slower, more stable)
unsigned long lastLCDUpdate = 0;
float deltaTime = 0.04;

bool calibrated = false;
bool testInProgress = false;
bool measurementRecorded = false;
float criticalAngle = 0;
float mu = 0;
unsigned long lastDetectionTime = 0;

// ============================================
// MEDIAN FILTER
// ============================================
float medianFilter(float newValue) {
    medianWindow[medianIndex] = newValue;
    medianIndex = (medianIndex + 1) % 7;
    
    float sorted[7];
    for (int i = 0; i < 7; i++) sorted[i] = medianWindow[i];
    
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6 - i; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    return sorted[3];
}

// ============================================
// EXPONENTIAL MOVING AVERAGE
// ============================================
float exponentialMovingAverage(float newValue, float previousEMA, float alpha) {
    return alpha * newValue + (1 - alpha) * previousEMA;
}

// ============================================
// HYSTERESIS
// ============================================
float applyHysteresis(float newAngle, float previousStableAngle, float hysteresis) {
    float delta = newAngle - previousStableAngle;
    if (fabs(delta) < hysteresis) return previousStableAngle;
    return newAngle;
}

// ============================================
// CALCULATE TOTAL TILT
// ============================================
float calculateTotalTilt(int16_t ax, int16_t ay, int16_t az) {
    float realX = ax - accelBiasX;
    float realY = ay - accelBiasY;
    float realZ = az - accelBiasZ;
    
    gravityMagnitude = sqrt(realX * realX + realY * realY + realZ * realZ);
    
    if (gravityMagnitude > 0) {
        gravityX = realX / gravityMagnitude;
        gravityY = realY / gravityMagnitude;
        gravityZ = realZ / gravityMagnitude;
    }
    
    tiltX = asin(constrain(gravityX, -1, 1)) * 180 / PI;
    tiltY = asin(constrain(gravityY, -1, 1)) * 180 / PI;
    
    float total = sqrt(tiltX * tiltX + tiltY * tiltY);
    
    if (total < 0) total = 0;
    if (total > 90) total = 90;
    
    return total;
}

// ============================================
// GYROSCOPE RATE
// ============================================
void getGyroRates(int16_t gx, int16_t gy, int16_t gz, float &rateX, float &rateY, float &rateZ) {
    rateX = (gx - gyroBiasX) / 131.0;
    rateY = (gy - gyroBiasY) / 131.0;
    rateZ = (gz - gyroBiasZ) / 131.0;
}

// ============================================
// COMPLEMENTARY FILTER
// ============================================
float complementaryFilter(float accelAngle, float gyroRate, float dt, float prevAngle) {
    float gyroAngle = prevAngle + gyroRate * dt;
    return compFilterAlpha * gyroAngle + (1 - compFilterAlpha) * accelAngle;
}

// ============================================
// ADAPTIVE ALPHA (very stable)
// ============================================
float getAdaptiveAlpha() {
    float alphaMin = 0.05;   // Maximum smoothing
    float alphaMax = 0.12;   // Very slow response
    float alpha = alphaMin + (motionSensitivity - 1) * (alphaMax - alphaMin) / 9;
    return constrain(alpha, alphaMin, alphaMax);
}

// ============================================
// CALIBRATION
// ============================================
void calibrateSensors() {
    lcd.clear();
    lcd.print("CALIBRATING...");
    lcd.setCursor(0, 1);
    lcd.print("KEEP STILL!");
    Serial.println("Calibrating sensors - DO NOT MOVE");
    
    long sumAx = 0, sumAy = 0, sumAz = 0;
    long sumGx = 0, sumGy = 0, sumGz = 0;
    const int samples = 300;
    
    for (int i = 0; i < samples; i++) {
        int16_t ax, ay, az, gx, gy, gz;
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        sumAx += ax;
        sumAy += ay;
        sumAz += az;
        sumGx += gx;
        sumGy += gy;
        sumGz += gz;
        delay(10);
        
        if (i % 100 == 0) {
            lcd.setCursor(0, 1);
            lcd.print("Progress:");
            lcd.print(i / 3);
            lcd.print("%  ");
        }
    }
    
    accelBiasX = sumAx / (float)samples;
    accelBiasY = sumAy / (float)samples;
    accelBiasZ = (sumAz / (float)samples) - 16384.0;
    
    gyroBiasX = sumGx / (float)samples;
    gyroBiasY = sumGy / (float)samples;
    gyroBiasZ = sumGz / (float)samples;
    
    calibrated = true;
    
    lcd.clear();
    lcd.print("CALIBRATION DONE!");
    lcd.setCursor(0, 1);
    lcd.print("Ready");
    
    Serial.println("Calibration complete!");
    delay(2000);
    lcd.clear();
}

// ============================================
// GET STABLE ANGLE (Very Low Sensitivity)
// ============================================
float getStableAngle() {
    int16_t ax, ay, az, gx, gy, gz;
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    float rawAngle = calculateTotalTilt(ax, ay, az);
    
    float gyroRateX, gyroRateY, gyroRateZ;
    getGyroRates(gx, gy, gz, gyroRateX, gyroRateY, gyroRateZ);
    float totalGyroRate = sqrt(gyroRateX * gyroRateX + gyroRateY * gyroRateY);
    
    float medianAngle = medianFilter(rawAngle);
    float smoothedAngle = exponentialMovingAverage(medianAngle, emaAngle, getAdaptiveAlpha());
    emaAngle = smoothedAngle;
    
    float hysteresisAngle = applyHysteresis(smoothedAngle, stableAngle, angleHysteresis);
    float filtered = complementaryFilter(hysteresisAngle, totalGyroRate, deltaTime, filteredAngle);
    filteredAngle = filtered;
    
    if (filteredAngle < 0) filteredAngle = 0;
    if (filteredAngle > 90) filteredAngle = 90;
    
    angleVelocity = (filteredAngle - previousAngle) / deltaTime;
    if (fabs(angleVelocity) < velocityNoiseGate) {
        filteredVelocity = 0;
    } else {
        filteredVelocity = angleVelocity;
    }
    
    stableAngle = filteredAngle;
    previousAngle = filteredAngle;
    
    return stableAngle;
}

// ============================================
// SETUP
// ============================================
void setup() {
    Serial.begin(9600);
    Wire.begin();
    
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("FRICTION METER");
    lcd.setCursor(0, 1);
    lcd.print("Stable Mode");
    
    Serial.println("========================================");
    Serial.println("VERY STABLE FRICTION METER");
    Serial.println("Low Sensitivity - Maximum Stability");
    Serial.println("========================================");
    
    mpu.initialize();
    
    if (!mpu.testConnection()) {
        Serial.println("ERROR: MPU6050 not found!");
        lcd.clear();
        lcd.print("MPU6050 ERROR!");
        while (1);
    }
    
    calibrateSensors();
    
    lcd.clear();
    lcd.print("Sensitivity:");
    lcd.setCursor(0, 1);
    lcd.print("Low");
    delay(2000);
    
    lcd.clear();
    lcd.print("Place block");
    lcd.setCursor(0, 1);
    lcd.print("Angle: 0.0");
    
    Serial.println("Ready - Very stable readings");
    Serial.println("========================================");
    
    lastSampleTime = millis();
}

// ============================================
// MAIN LOOP
// ============================================
void loop() {
    if (!calibrated) return;
    
    unsigned long currentTime = millis();
    deltaTime = (currentTime - lastSampleTime) / 1000.0;
    
    if (deltaTime >= 0.03 && (currentTime - lastSampleTime) >= sampleInterval) {
        lastSampleTime = currentTime;
        
        float angle = getStableAngle();
        
        Serial.print("LIVE:");
        Serial.println(angle, 2);
        
        if (currentTime - lastLCDUpdate >= 200) {
            lastLCDUpdate = currentTime;
            lcd.setCursor(0, 0);
            
            if (testInProgress) {
                lcd.print("TEST:");
            } else {
                lcd.print("Angle:");
            }
            lcd.print(angle, 1);
            lcd.print(" deg ");
            
            lcd.setCursor(0, 1);
            if (fabs(filteredVelocity) < 0.5) {
                lcd.print("Stable      ");
            } else {
                lcd.print("Moving      ");
            }
        }
        
        if (!testInProgress && angle > 4.0) {
            testInProgress = true;
            measurementRecorded = false;
            lcd.clear();
            lcd.print("TESTING...");
            lcd.setCursor(0, 1);
            lcd.print("Tilt slowly");
            Serial.println("========================================");
            Serial.println("TEST STARTED - Tilt board");
            Serial.println("========================================");
        }
        
        if (testInProgress && !measurementRecorded) {
            bool velocityTrigger = fabs(filteredVelocity) > velocityThreshold;
            bool isMoving = fabs(filteredVelocity) > 0.8;
            
            if ((velocityTrigger && isMoving) || 
                (angle > 6 && previousAngle > 0 && angle > previousAngle + 1.5)) {
                
                measurementRecorded = true;
                criticalAngle = angle;
                mu = tan(criticalAngle * PI / 180.0);
                lastDetectionTime = currentTime;
                
                Serial.println("========================================");
                Serial.print("ANGLE:");
                Serial.println(criticalAngle, 2);
                Serial.print("MU:");
                Serial.println(mu, 4);
                Serial.print("SLIDE DETECTED! Angle: ");
                Serial.print(criticalAngle, 1);
                Serial.print("°, μ = ");
                Serial.println(mu, 4);
                Serial.println("========================================");
                
                lcd.clear();
                lcd.setCursor(0, 0);
                lcd.print("SLIDE!");
                lcd.setCursor(0, 1);
                lcd.print("mu=");
                lcd.print(mu, 4);
                
                delay(3000);
                
                lcd.clear();
                lcd.print("Place block");
                lcd.setCursor(0, 1);
                lcd.print("Angle: 0.0");
                testInProgress = false;
            }
        }
        
        if (!testInProgress && (currentTime - lastDetectionTime) > 1000) {
            lcd.setCursor(0, 1);
            lcd.print("Place block    ");
        }
    }
    
    delay(5);
}