#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_seesaw.h>
#include <seesaw_neopixel.h>
#include <AccelStepper.h>

// Pin definitions
#define LCD_CS    10
#define LCD_DC    8
#define LCD_RST   9

#define ENCODER_X_ADDR  0x36
#define ENCODER_Y_ADDR  0x37
#define SS_SWITCH 24
#define SS_NEOPIXEL 6

// Motor pins
#define DIR_X     0
#define STEP_X    1
#define DIR_Y     2
#define STEP_Y    3
#define MOTOR_EN  4

// QPD analog inputs
#define QPD_A     14  // Top right (A0)     A
#define QPD_B     15  // Bottom right (A1)  B
#define QPD_C     16  // Bottom left (A2)   C
#define QPD_D     17  // Top left (A3)      D

// QPD display on LEFT side of screen
#define QPD_DISPLAY_X      10      
#define QPD_DISPLAY_Y      30      // Move up - start below mode text
#define QPD_DISPLAY_SIZE   200     // 200x200 fits in 240 height
#define QPD_CENTER_X       (QPD_DISPLAY_X + QPD_DISPLAY_SIZE/2)
#define QPD_CENTER_Y       (QPD_DISPLAY_Y + QPD_DISPLAY_SIZE/2)


// System modes
enum SystemMode {
    DISABLED,
    MANUAL,
    AUTO
};

SystemMode currentMode = DISABLED;

// Initialize steppers
AccelStepper stepperX(AccelStepper::DRIVER, STEP_X, DIR_X);
AccelStepper stepperY(AccelStepper::DRIVER, STEP_Y, DIR_Y);

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

GFXcanvas16 qpd_canvas(QPD_DISPLAY_SIZE, QPD_DISPLAY_SIZE);
SystemMode lastMode = DISABLED;
bool displayInitialized = false;


const int STEPS_PER_ENCODER_CLICK = 100;  // Adjust for desired sensitivity


void initializeQPDCanvas() {

    // Draw static QPD elements on canvas (only once)
    qpd_canvas.fillScreen(ST77XX_BLACK);
    
    // Border
    qpd_canvas.drawRect(0, 0, QPD_DISPLAY_SIZE, QPD_DISPLAY_SIZE, ST77XX_WHITE);
    
    // Quadrant lines (gray)
    uint16_t gray = tft.color565(128, 128, 128);  // Use tft, not qpd_canvas
    qpd_canvas.drawFastVLine(QPD_DISPLAY_SIZE/2, 0, QPD_DISPLAY_SIZE, gray);
    qpd_canvas.drawFastHLine(0, QPD_DISPLAY_SIZE/2, QPD_DISPLAY_SIZE, gray);
    
    // Quadrant labels
    qpd_canvas.setTextSize(1);
    qpd_canvas.setTextColor(gray);
    qpd_canvas.setCursor(5, 5);
    qpd_canvas.print("D");
    qpd_canvas.setCursor(QPD_DISPLAY_SIZE - 15, 5);
    qpd_canvas.print("A");
    qpd_canvas.setCursor(5, QPD_DISPLAY_SIZE - 15);
    qpd_canvas.print("C");
    qpd_canvas.setCursor(QPD_DISPLAY_SIZE - 15, QPD_DISPLAY_SIZE - 15);
    qpd_canvas.print("B");
}

void updateQPDCanvas() {

    // Blank just the center area (preserve border)
    qpd_canvas.fillRect(1, 1, QPD_DISPLAY_SIZE-2, QPD_DISPLAY_SIZE-2, ST77XX_BLACK);
    
    // Redraw quadrant lines
    uint16_t gray = tft.color565(128, 128, 128);  // Use tft, not qpd_canvas
    qpd_canvas.drawFastVLine(QPD_DISPLAY_SIZE/2, 1, QPD_DISPLAY_SIZE-2, gray);
    qpd_canvas.drawFastHLine(1, QPD_DISPLAY_SIZE/2, QPD_DISPLAY_SIZE-2, gray);
    
    // Redraw labels
    qpd_canvas.setTextSize(1);
    qpd_canvas.setTextColor(gray);
    qpd_canvas.setCursor(5, 5);
    qpd_canvas.print("D");
    qpd_canvas.setCursor(QPD_DISPLAY_SIZE - 15, 5);
    qpd_canvas.print("A");
    qpd_canvas.setCursor(5, QPD_DISPLAY_SIZE - 15);
    qpd_canvas.print("C");
    qpd_canvas.setCursor(QPD_DISPLAY_SIZE - 15, QPD_DISPLAY_SIZE - 15);
    qpd_canvas.print("B");
    
    // Calculate dot position (relative to canvas, not screen)
    int dotX = (QPD_DISPLAY_SIZE/2) + (errorX / 8);
    int dotY = (QPD_DISPLAY_SIZE/2) - (errorY / 8);
    
    // Clamp to canvas area
    dotX = constrain(dotX, 5, QPD_DISPLAY_SIZE - 5);
    dotY = constrain(dotY, 5, QPD_DISPLAY_SIZE - 5);
    
    // Draw dot
    qpd_canvas.fillCircle(dotX, dotY, 4, ST77XX_RED);
    
    // Blit entire canvas to screen in one operation
    tft.drawRGBBitmap(QPD_DISPLAY_X, QPD_DISPLAY_Y,
                      qpd_canvas.getBuffer(),
                      qpd_canvas.width(),
                      qpd_canvas.height());
}

void initializeDisplay() {

    tft.fillScreen(ST77XX_BLACK);
    
    // Mode at very top
    tft.setTextSize(2);
    tft.setCursor(10, 5);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Mode: ");
    
    // QPD visualization box border - just the outline, canvas handles the rest
    // (Actually, don't draw this either - the canvas draws its own border)
    
    // Data/text area - RIGHT SIDE (starting at X=220)
    tft.setTextSize(1);
    //tft.setCursor(220, 30);
    //tft.setTextColor(ST77XX_CYAN);
    //tft.print("QPD:");
    
    tft.setCursor(220, 60);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Err X:");
    
    tft.setCursor(220, 75);
    tft.print("Err Y:");
    
    tft.setCursor(220, 100);
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Power:");
    
    tft.setCursor(220, 130);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("Enc X:");
    
    tft.setCursor(220, 145);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Enc Y:");
    
    displayInitialized = true;
}


void updateDisplayValues() {

    tft.setTextWrap(false); 
    
    // Update mode ONLY if it changed
    if (currentMode != lastMode) {
        tft.fillRect(70, 5, 130, 16, ST77XX_BLACK);
        tft.setCursor(70, 5);
        tft.setTextSize(2);
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
        lastMode = currentMode;
    }
    
    // Update QPD raw values - REWRITE to prevent overflow
    tft.fillRect(220, 40, 95, 20, ST77XX_BLACK);  // Taller erase (20 instead of 10)
    tft.setTextSize(1);
    tft.setTextColor(ST77XX_CYAN);
    
    // Put all QPD values on separate lines to prevent wrapping
    tft.setCursor(220, 40);
    tft.print("A:"); tft.print(qpd_a);
    tft.print(" B:"); tft.print(qpd_b);
    
    tft.setCursor(220, 50);
    tft.print("C:"); tft.print(qpd_c);
    tft.print(" D:"); tft.print(qpd_d);
    // Update errors
    tft.fillRect(265, 60, 50, 10, ST77XX_BLACK);
    tft.setCursor(265, 60);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print(errorX);
    
    tft.fillRect(265, 75, 50, 10, ST77XX_BLACK);
    tft.setCursor(265, 75);
    tft.print(errorY);
    
    // Update total power
    tft.fillRect(220, 110, 95, 10, ST77XX_BLACK);
    tft.setCursor(220, 110);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(totalPower);
    
    // Update encoder positions
    tft.fillRect(265, 130, 50, 10, ST77XX_BLACK);
    tft.setCursor(265, 130);
    tft.setTextColor(ST77XX_CYAN);
    tft.print(encoderX_position);
    
    tft.fillRect(265, 145, 50, 10, ST77XX_BLACK);
    tft.setCursor(265, 145);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print(encoderY_position);
    
    // Update QPD display (canvas approach)
    updateQPDCanvas();
}

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
            digitalWrite(MOTOR_EN, LOW);  // Enable motors (active LOW)
            break;
        case MANUAL:
            currentMode = AUTO;
            Serial.println("Mode: AUTO");
            // Motors stay enabled
            break;
        case AUTO:
            currentMode = DISABLED;
            Serial.println("Mode: DISABLED");
            digitalWrite(MOTOR_EN, HIGH);  // Disable motors
            break;
    }
    
    updateModeLEDs();
}


void setup() {

    Serial.begin(115200);
    delay(1000);
    Serial.println("Dr. Strangelove - Motor Control");
    
    // Configure motor enable pin
    pinMode(MOTOR_EN, OUTPUT);
    digitalWrite(MOTOR_EN, HIGH);  // Disable motors initially (TMC2209 enable is active LOW)
    
    // Configure steppers
    stepperX.setMaxSpeed(1000);     // Steps per second
    stepperX.setAcceleration(500);  // Steps per second^2
    
    stepperY.setMaxSpeed(1000);
    stepperY.setAcceleration(500);

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
    initializeDisplay();
    initializeQPDCanvas();

}

void loop() {
    unsigned long now = millis();
    
      // DEBUG: Check motor enable state
    static unsigned long lastDebug = 0;
    if (now - lastDebug > 2000) {
        Serial.print("Mode: ");
        Serial.print(currentMode);
        Serial.print(" | EN pin: ");
        Serial.print(digitalRead(MOTOR_EN));
        Serial.print(" | StepperX distToGo: ");
        Serial.println(stepperX.distanceToGo());
        lastDebug = now;
    }

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
    
    // Check if encoders moved and update motors
    bool encoderMoved = (newX != encoderX_position || newY != encoderY_position);
    if (encoderMoved) {
        // Calculate deltas
        int32_t deltaX = newX - encoderX_position;
        int32_t deltaY = newY - encoderY_position;
        
        if (currentMode == MANUAL) {
            if (deltaX != 0) {
                stepperX.move(deltaX * STEPS_PER_ENCODER_CLICK);
                Serial.print("X move: ");
                Serial.println(deltaX * STEPS_PER_ENCODER_CLICK);
            }
            
            if (deltaY != 0) {
                stepperY.move(deltaY * STEPS_PER_ENCODER_CLICK);
                Serial.print("Y move: ");
                Serial.println(deltaY * STEPS_PER_ENCODER_CLICK);
            }
        }
        
        // Update stored positions
        encoderX_position = newX;
        encoderY_position = newY;
    }
    
    // Always run steppers (must be called every loop)
    stepperX.run();
    stepperY.run();
    
    // Update display at fixed rate or when something changes
    if ((now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) || encoderMoved) {
        if (!displayInitialized) {
            initializeDisplay();
        }
        updateDisplayValues();
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