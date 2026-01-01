#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>

// Pin definitions
#define LCD_CS    10
#define LCD_DC    9
#define LCD_RST   8

#define ENCODER_X_ADDR  0x36
#define ENCODER_Y_ADDR  0x37
#define SS_SWITCH 24
#define SS_NEOPIXEL 6

// QPD analog inputs
#define QPD_A     14  // Top right (A0)
#define QPD_B     15  // Bottom right (A1)
#define QPD_C     16  // Bottom left (A2)
#define QPD_D     17  // Top left (A3)

// System modes
enum SystemMode {
    DISABLED,
    MANUAL,
    AUTO
};

SystemMode currentMode = DISABLED;

// Initialize display
Adafruit_ST7789 tft = Adafruit_ST7789(LCD_CS, LCD_DC, LCD_RST);

// Initialize encoders
Adafruit_seesaw encoderX;
Adafruit_seesaw encoderY;

// Initialize NeoPixels
seesaw_NeoPixel pixelX = seesaw_NeoPixel(1, SS_NEOPIXEL, NEO_GRB + NEO_KHZ800);
seesaw_NeoPixel pixelY = seesaw_NeoPixel(1, SS_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// Encoder state
int32_t encoderX_position = 0;
int32_t encoderY_position = 0;

// Button state with debouncing
bool lastButtonX = false;
bool lastButtonY = false;
unsigned long lastButtonXTime = 0;
unsigned long lastButtonYTime = 0;
const unsigned long DEBOUNCE_MS = 50;

// QPD state
int qpd_a, qpd_b, qpd_c, qpd_d;
int totalPower;
int errorX, errorY;

// Display update timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_MS = 100;  // 10Hz

// LED Colors
const uint32_t COLOR_DISABLED = 0x100000;
const uint32_t COLOR_MANUAL   = 0x001010;
const uint32_t COLOR_AUTO     = 0x001000;
const uint32_t COLOR_OFF      = 0x000000;

void readQPD() {
    qpd_a = analogRead(QPD_A);  // Top right
    qpd_b = analogRead(QPD_B);  // Bottom right
    qpd_c = analogRead(QPD_C);  // Bottom left
    qpd_d = analogRead(QPD_D);  // Top left
    
    // Calculate errors
    errorX = (qpd_a + qpd_b) - (qpd_c + qpd_d);  // Right - Left
    errorY = (qpd_a + qpd_d) - (qpd_b + qpd_c);  // Top - Bottom
    
    // Total power check
    totalPower = qpd_a + qpd_b + qpd_c + qpd_d;
}

void updateModeLEDs() {
    uint32_t color = COLOR_OFF;
    
    switch(currentMode) {
        case DISABLED:
            color = COLOR_DISABLED;
            break;
        case MANUAL:
            color = COLOR_MANUAL;
            break;
        case AUTO:
            color = COLOR_AUTO;
            break;
    }
    
    pixelX.setPixelColor(0, color);
    pixelX.show();
    
    pixelY.setPixelColor(0, color);
    pixelY.show();
}

void handleModeButton() {
    switch(currentMode) {
        case DISABLED:
            currentMode = MANUAL;
            Serial.println("Mode: MANUAL");
            break;
        case MANUAL:
            currentMode = AUTO;
            Serial.println("Mode: AUTO");
            break;
        case AUTO:
            currentMode = DISABLED;
            Serial.println("Mode: DISABLED");
            break;
    }
    
    updateModeLEDs();
}

void updateDisplay() {
    tft.fillScreen(ST77XX_BLACK);
    
    // Display mode
    tft.setCursor(10, 10);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Mode: ");
    
    switch(currentMode) {
        case DISABLED: 
            tft.setTextColor(ST77XX_RED);
            tft.print("DISABLED"); 
            break;
        case MANUAL: 
            tft.setTextColor(ST77XX_CYAN);
            tft.print("MANUAL"); 
            break;
        case AUTO: 
            tft.setTextColor(ST77XX_GREEN);
            tft.print("AUTO"); 
            break;
    }
    
    // QPD values (small text)
    tft.setTextSize(1);
    tft.setCursor(10, 40);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("QPD: A:"); tft.print(qpd_a);
    tft.print(" B:"); tft.print(qpd_b);
    tft.print(" C:"); tft.print(qpd_c);
    tft.print(" D:"); tft.println(qpd_d);
    
    // Errors
    tft.setCursor(10, 55);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Error X:"); tft.print(errorX);
    tft.print("  Y:"); tft.println(errorY);
    
    tft.setCursor(10, 70);
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Total Power: "); tft.println(totalPower);
    
    // Encoder positions
    tft.setCursor(10, 90);
    tft.setTextColor(ST77XX_CYAN);
    tft.setTextSize(1);
    tft.print("Enc X: ");
    tft.print(encoderX_position);
    
    tft.setCursor(10, 105);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Enc Y: ");
    tft.print(encoderY_position);
    
    // Simple crosshair showing QPD error
    int centerX = 240;
    int centerY = 160;
    
    // Map error to screen position (scale as needed)
    int dotX = centerX + (errorX / 4);
    int dotY = centerY + (errorY / 4);
    
    // Draw crosshair
    tft.drawFastHLine(centerX - 20, centerY, 40, ST77XX_RED);
    tft.drawFastVLine(centerX, centerY - 20, 40, ST77XX_RED);
    
    // Draw error position dot
    tft.fillCircle(dotX, dotY, 3, ST77XX_WHITE);
}

void setup() {
    Serial.begin(115200);
    delay(1000);
    Serial.println("Dr. Strangelove - Full Integration Test");
    
    // Initialize display
    tft.init(240, 320);
    tft.setRotation(1);
    tft.fillScreen(ST77XX_BLACK);
    
    tft.setCursor(10, 10);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.println("Initializing...");
    
    // Set analog resolution
    analogReadResolution(10);  // 0-1023 range
    
    Wire.begin();
    
    // Initialize encoder X
    if (!encoderX.begin(ENCODER_X_ADDR)) {
        Serial.println("ERROR: Encoder X not found!");
    } else {
        Serial.println("Encoder X initialized");
        encoderX.pinMode(SS_SWITCH, INPUT_PULLUP);
        encoderX_position = encoderX.getEncoderPosition();
        
        pixelX.begin(ENCODER_X_ADDR);
        pixelX.setBrightness(20);
        pixelX.show();
    }
    
    // Initialize encoder Y  
    if (!encoderY.begin(ENCODER_Y_ADDR)) {
        Serial.println("ERROR: Encoder Y not found!");
    } else {
        Serial.println("Encoder Y initialized");
        encoderY.pinMode(SS_SWITCH, INPUT_PULLUP);
        encoderY_position = encoderY.getEncoderPosition();
        
        pixelY.begin(ENCODER_Y_ADDR);
        pixelY.setBrightness(20);
        pixelY.show();
    }
    
    delay(1000);
    updateModeLEDs();
    
    // Initial QPD read
    readQPD();
    updateDisplay();
}

void loop() {
    unsigned long now = millis();
    
    // Always read QPD
    readQPD();
    
    // Read encoder positions
    int32_t newX = encoderX.getEncoderPosition();
    int32_t newY = encoderY.getEncoderPosition();
    
    // Read buttons
    bool buttonX = encoderX.digitalRead(SS_SWITCH);
    bool buttonY = encoderY.digitalRead(SS_SWITCH);
    
    // Button X with debounce - mode switching
    if (buttonX == 1 && lastButtonX == 0 && (now - lastButtonXTime > DEBOUNCE_MS)) {
        handleModeButton();
        lastButtonXTime = now;
    }
    lastButtonX = buttonX;
    
    // Button Y with debounce
    if (buttonY == 1 && lastButtonY == 0 && (now - lastButtonYTime > DEBOUNCE_MS)) {
        Serial.println("Button Y pressed");
        lastButtonYTime = now;
    }
    lastButtonY = buttonY;
    
    // Check if encoders moved
    bool encoderMoved = (newX != encoderX_position || newY != encoderY_position);
    if (encoderMoved) {
        encoderX_position = newX;
        encoderY_position = newY;
    }
    
    // Update display at fixed rate or when something changes
    if ((now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) || encoderMoved) {
        updateDisplay();
        lastDisplayUpdate = now;
        
        // Serial output for debugging
        Serial.print("QPD A:"); Serial.print(qpd_a);
        Serial.print(" B:"); Serial.print(qpd_b);
        Serial.print(" C:"); Serial.print(qpd_c);
        Serial.print(" D:"); Serial.print(qpd_d);
        Serial.print(" | ErrX:"); Serial.print(errorX);
        Serial.print(" ErrY:"); Serial.print(errorY);
        Serial.print(" | Total:"); Serial.println(totalPower);
    }
}