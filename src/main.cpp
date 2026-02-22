/**
 * @file main.cpp
 * @brief Dr. Strangelove - Motor Control System
 * 
 * This Arduino sketch implements a motor control system with the following features:
 * - Two stepper motors controlled via AccelStepper library
 * - LCD display (ST7789) for system status and QPD visualization
 * - Two rotary encoders with push buttons for manual control and mode switching
 * - QPD (Quadrant Photodiode) sensors for position feedback and error calculation
 * - NeoPixel LEDs for system mode indication
 * - Three operating modes: DISABLED, MANUAL, and AUTO
 * 
 * System modes:
 * - DISABLED: Motors disabled, LEDs show red
 * - MANUAL: Motors enabled, encoder input controls stepper position
 * - AUTO: Motors enabled, QPD feedback controls stepper position (not fully implemented)
 * 
 * Hardware connections:
 * - LCD: CS=10, DC=8, RST=9
 * - Encoders: X=0x36, Y=0x37 (seesaw I2C)
 * - Stepper motors: X=DIR=0, STEP=1, Y=DIR=2, STEP=3, EN=4
 * - QPD sensors: A=14, B=15, C=16, D=17
 * - NeoPixels: SS_NEOPIXEL=6
 * 
 * @author Anthony Heath
 * @date 2/20/2026
 * @version 1.0
 */

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
    CALIBRATE,  // ADD THIS LINE
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

int targetOffsetX = 0;  // Target offset for AUTO mode
int targetOffsetY = 0;

// Low-pass filtered QPD values
float filtered_a = 0, filtered_b = 0, filtered_c = 0, filtered_d = 0;
const float QPD_FILTER_ALPHA = 0.05;  // 0-1, lower = more filtering

// Display update timing
unsigned long lastDisplayUpdate = 0;
const unsigned long DISPLAY_UPDATE_MS = 100;  // 10Hz

unsigned long lastAutoUpdate = 0;  // Timing for AUTO mode control loop

// LED Colors
const uint32_t COLOR_DISABLED = 0x100000;
const uint32_t COLOR_MANUAL   = 0x001010;
const uint32_t COLOR_CALIBRATE = 0x101000;
const uint32_t COLOR_AUTO     = 0x001000;
const uint32_t COLOR_OFF      = 0x000000;

GFXcanvas16 qpd_canvas(QPD_DISPLAY_SIZE, QPD_DISPLAY_SIZE);
SystemMode lastMode = DISABLED;
bool displayInitialized = false;


const int STEPS_PER_ENCODER_CLICK = 100;  // Adjust for desired sensitivity

// Gain values for AUTO mode control
float Kp_x = 1.0;  // Proportional gain for X axis
float Kp_y = 1.0;  // Proportional gain for Y axis
const float GAIN_INCREMENT = 0.1;  // How much gain changes per encoder click

// AUTO mode control parameters
const int ERROR_DEADBAND = 10;      // Ignore errors smaller than this
const int MAX_STEPS_PER_CYCLE = 50; // Maximum correction per update
const int CONVERGED_THRESHOLD = 15; // Error threshold for "converged" status
const unsigned long AUTO_UPDATE_MS = 100; // Control loop rate (10Hz)

/**
 * @brief Initializes the QPD canvas by drawing static elements
 * 
 * This function sets up the initial display layout for the QPD (Quadrant Position Display)
 * by drawing static elements including:
 * - A black background fill
 * - A white border around the display area
 * - Gray vertical and horizontal lines dividing the display into quadrants
 * - Quadrant labels (A, B, C, D) positioned in the corners
 * 
 * The static elements are drawn only once during initialization.
 * 
 * @note This function uses qpd_canvas for drawing operations and tft for color conversion
 */
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
    
    // Draw target position as a green crosshair
    int targetX = (QPD_DISPLAY_SIZE/2) + (targetOffsetX / 8);
    int targetY = (QPD_DISPLAY_SIZE/2) - (targetOffsetY / 8);
    targetX = constrain(targetX, 5, QPD_DISPLAY_SIZE - 5);
    targetY = constrain(targetY, 5, QPD_DISPLAY_SIZE - 5);

    // Draw green crosshair at target
    uint16_t green = tft.color565(0, 255, 0);  // CHANGED: tft not qpd_canvas
    qpd_canvas.drawFastHLine(targetX - 6, targetY, 13, green);
    qpd_canvas.drawFastVLine(targetX, targetY - 6, 13, green);

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
    
    tft.setCursor(220, 170);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.print("Kp_x:");

    tft.setCursor(220, 185);
    tft.print("Kp_y:");

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
            case CALIBRATE:                          // ADD THESE 3 LINES
                tft.setTextColor(ST77XX_YELLOW);
                tft.print("CALIBRATE");
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
    
    // ADD THESE LINES:
    // Update gain values
    tft.fillRect(265, 170, 50, 10, ST77XX_BLACK);
    tft.setCursor(265, 170);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.print(Kp_x, 1);  // Show 1 decimal place

    tft.fillRect(265, 185, 50, 10, ST77XX_BLACK);
    tft.setCursor(265, 185);
    tft.print(Kp_y, 1);  // Show 1 decimal place

    // Show convergence status in AUTO mode
    if (currentMode == AUTO) {
         bool converged = (abs(errorX - targetOffsetX) < CONVERGED_THRESHOLD && 
                      abs(errorY - targetOffsetY) < CONVERGED_THRESHOLD);
        tft.fillRect(220, 210, 95, 10, ST77XX_BLACK);
        tft.setCursor(220, 210);
        if (converged) {
            tft.setTextColor(ST77XX_GREEN);
            tft.print("CONVERGED");
        } else {
            tft.setTextColor(ST77XX_RED);
            tft.print("CORRECTING");
        }
    } else {
        // Clear convergence status when not in AUTO
        tft.fillRect(220, 210, 95, 10, ST77XX_BLACK);
    }

    // Update QPD display (canvas approach)
    updateQPDCanvas();
}

void readQPD() {

    // Read and filter
    filtered_a = QPD_FILTER_ALPHA * analogRead(QPD_A) + (1.0 - QPD_FILTER_ALPHA) * filtered_a;
    filtered_b = QPD_FILTER_ALPHA * analogRead(QPD_B) + (1.0 - QPD_FILTER_ALPHA) * filtered_b;
    filtered_c = QPD_FILTER_ALPHA * analogRead(QPD_C) + (1.0 - QPD_FILTER_ALPHA) * filtered_c;
    filtered_d = QPD_FILTER_ALPHA * analogRead(QPD_D) + (1.0 - QPD_FILTER_ALPHA) * filtered_d;
    
    qpd_a = (int)filtered_a;
    qpd_b = (int)filtered_b;
    qpd_c = (int)filtered_c;
    qpd_d = (int)filtered_d;
    
    // Calculate errors
    errorX = (qpd_a + qpd_b) - (qpd_c + qpd_d);  // Right - Left
    errorY = (qpd_a + qpd_d) - (qpd_b + qpd_c);  // Top - Bottom
    
    // Total power check
    totalPower = qpd_a + qpd_b + qpd_c + qpd_d;
}

void runAutoControl() {
    // Calculate error relative to target offset
    int relativeErrorX = errorX - targetOffsetX;
    int relativeErrorY = errorY - targetOffsetY;
    
    // Apply deadband - ignore small errors
    int activeErrorX = (abs(relativeErrorX) > ERROR_DEADBAND) ? relativeErrorX : 0;
    int activeErrorY = (abs(relativeErrorY) > ERROR_DEADBAND) ? relativeErrorY : 0;
    
    if (activeErrorX != 0) {
        // Calculate correction steps
        int stepsX = (int)(Kp_x * activeErrorX);
        // Limit maximum correction
        stepsX = constrain(stepsX, -MAX_STEPS_PER_CYCLE, MAX_STEPS_PER_CYCLE);
        stepperX.move(stepsX);  // Positive for X
        
        Serial.print("AUTO X: error=");
        Serial.print(relativeErrorX);  // CHANGED: use relativeErrorX
        Serial.print(" steps=");
        Serial.println(stepsX);
    }
    
    if (activeErrorY != 0) {
        // Calculate correction steps
        int stepsY = (int)(Kp_y * activeErrorY);
        // Limit maximum correction
        stepsY = constrain(stepsY, -MAX_STEPS_PER_CYCLE, MAX_STEPS_PER_CYCLE);
        stepperY.move(-stepsY);  // Negative for Y
        
        Serial.print("AUTO Y: error=");
        Serial.print(relativeErrorY);  // CHANGED: use relativeErrorY
        Serial.print(" steps=");
        Serial.println(-stepsY);
    }
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
        case CALIBRATE:
            color = COLOR_CALIBRATE;
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
            currentMode = CALIBRATE;
            Serial.println("Mode: CALIBRATE");
            digitalWrite(MOTOR_EN, HIGH);  // Disable motors in calibrate
            break;
        case CALIBRATE:
            currentMode = AUTO;
            Serial.println("Mode: AUTO");
            digitalWrite(MOTOR_EN, LOW);  // Enable motors for AUTO
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
    stepperX.setMaxSpeed(10000);     // Steps per second
    stepperX.setAcceleration(10000);  // Steps per second^2
    
    stepperY.setMaxSpeed(10000);
    stepperY.setAcceleration(10000);

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
    //analogReadAveraging(16); 

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
        
        // ADD: Read and discard first button state (may be invalid)
        encoderX.digitalRead(SS_SWITCH);
        delay(10);
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
        
        // ADD: Read and discard first button state (may be invalid)
        encoderY.digitalRead(SS_SWITCH);
        delay(10);
    }

    // NOW read the actual stable button states
    lastButtonX = encoderX.digitalRead(SS_SWITCH);
    lastButtonY = encoderY.digitalRead(SS_SWITCH);
    lastButtonXTime = millis();
    lastButtonYTime = millis();

    delay(1000);
    updateModeLEDs();
    
    // Initial QPD read
    readQPD();
    initializeDisplay();
    initializeQPDCanvas();

    // Force initial mode display update
    lastMode = AUTO;  // Different from currentMode to trigger update
    updateDisplayValues();


}

void loop() {

    unsigned long now = millis();
    
       // First loop debug
    static bool firstLoop = true;
    if (firstLoop) {
        Serial.print("=== FIRST LOOP === Mode: ");
        Serial.print(currentMode);
        Serial.print(" | EN: ");
        Serial.print(digitalRead(MOTOR_EN));
        Serial.print(" | lastMode: ");
        Serial.println(lastMode);
        firstLoop = false;
    }
    
    // DEBUG: Check motor enable state
    static unsigned long lastDebug = 0;

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
        // Toggle target on/off
        if (targetOffsetX == 0 && targetOffsetY == 0) {
            // Currently at center, set new target
            targetOffsetX = errorX;
            targetOffsetY = errorY;
            Serial.print("Target set: X=");
            Serial.print(targetOffsetX);
            Serial.print(" Y=");
            Serial.println(targetOffsetY);
        } else {
            // Target is set, clear it back to center
            targetOffsetX = 0;
            targetOffsetY = 0;
            Serial.println("Target cleared (back to center)");
        }
        lastButtonYTime = now;
    }
    lastButtonY = buttonY;
    
    // Check if encoders moved and update motors OR gains
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
        else if (currentMode == CALIBRATE) {
            // In CALIBRATE mode, encoders adjust gains
            if (deltaX != 0) {
                Kp_x += deltaX * GAIN_INCREMENT;
                Kp_x = constrain(Kp_x, 0.0, 10.0);  // Limit 0-10
                Serial.print("Kp_x: ");
                Serial.println(Kp_x);
            }
            
            if (deltaY != 0) {
                Kp_y += deltaY * GAIN_INCREMENT;
                Kp_y = constrain(Kp_y, 0.0, 10.0);  // Limit 0-10
                Serial.print("Kp_y: ");
                Serial.println(Kp_y);
            }
        }
        
        // Update stored positions
        encoderX_position = newX;
        encoderY_position = newY;
    }
    
    // Run AUTO mode control loop
    if (currentMode == AUTO) {
        if (now - lastAutoUpdate >= AUTO_UPDATE_MS) {
            runAutoControl();
            lastAutoUpdate = now;
        }
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

