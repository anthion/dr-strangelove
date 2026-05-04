/**
 * @file main.cpp
 * @brief Dr. Strangelove - Motor Control System
 *
 * Target hardware: Teensy 4.0 (ARM Cortex-M7 @ 600 MHz)
 * Toolchain:       PlatformIO / Arduino core for Teensy
 *
 * This sketch implements a closed-loop laser auto-alignment system
 * with the following features:
 * - Two stepper motors (TMC2209 drivers) controlled via AccelStepper library
 * - LCD display (ST7789) for system status and QPD visualization
 * - Two rotary encoders with push buttons for manual control, gain tuning,
 *   and mode switching
 * - QPD (Quadrant Photodiode) sensors for position feedback and error
 *   calculation, with low-pass filtering
 * - NeoPixel LEDs (one per encoder board) for system mode indication
 * - Four operating modes: DISABLED, MANUAL, CALIBRATE, AUTO
 *
 * System modes:
 * - DISABLED:  Motors disabled, LEDs show red
 * - MANUAL:    Motors enabled, encoder input directly controls stepper position
 * - CALIBRATE: Motors disabled, encoder input adjusts Kp_x / Kp_y gains
 * - AUTO:      Motors enabled, proportional QPD feedback drives steppers
 *              toward target offset (set/cleared via encoder Y button)
 *
 * Controls:
 * - Encoder X button: cycle mode (DISABLED -> MANUAL -> CALIBRATE -> AUTO -> DISABLED)
 * - Encoder Y button: in any mode, capture current QPD error as AUTO target;
 *                     press again to clear target back to center
 * - Encoder X/Y rotation: move steppers (MANUAL) or adjust Kp_x/Kp_y (CALIBRATE)
 *
 * Hardware connections (Teensy 4.0 pin numbers):
 * - LCD:           CS=10, DC=8, RST=9 (SPI)
 * - Encoders:      I2C addresses X=0x36, Y=0x37 (Adafruit seesaw, default Wire)
 * - Stepper X:     DIR=0,  STEP=1
 * - Stepper Y:     DIR=2,  STEP=3
 * - Motor enable:  EN=4 (active LOW, shared)
 * - QPD sensors:   A=A0(14), B=A1(15), C=A2(16), D=A3(17)
 *                  ADC configured for 10-bit resolution (0-1023)
 *                  Layout: A=top-right, B=bottom-right, C=bottom-left, D=top-left
 * - NeoPixels:     seesaw pin 6 on each encoder board
 *
 * @author Anthony Heath
 * @date <update on commit>
 * @version <bump as appropriate>
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

// QPD dot and crosshair geometry
#define QPD_DOT_RADIUS         4
#define QPD_DOT_MARGIN         (QPD_DOT_RADIUS + 1)
#define QPD_CROSSHAIR_HALF     6
#define QPD_CROSSHAIR_LEN      (2 * QPD_CROSSHAIR_HALF + 1)

// Scaling: QPD error counts -> canvas pixels
#define QPD_ERROR_TO_PIXEL_DIV 8

// LCD layout: data column on right side of screen
// Labels at DATA_LABEL_X, values aligned at DATA_VALUE_X.
// Power is the exception — its value uses the full column width
// because the digit count can exceed the narrow value field.
#define DATA_LABEL_X       220
#define DATA_VALUE_X       265
#define DATA_VALUE_W       50    // value field width for erase rect
#define DATA_FIELD_W       95    // full column width (label + value)
#define DATA_LINE_H        10    // row height for size-1 text
#define MODE_LABEL_X       10    // "Mode:" label at top
#define MODE_LABEL_Y       5
#define MODE_VALUE_X       70    // mode value drawn after "Mode: "
#define MODE_VALUE_W       130   // erase width for mode value
#define MODE_VALUE_H       16    // erase height for size-2 text

// LCD layout: row Y coordinates for the right-side data column
#define ROW_QPD_RAW_1      40    // "A: B:" line
#define ROW_QPD_RAW_2      50    // "C: D:" line
#define ROW_ERR_X          60
#define ROW_ERR_Y          75
#define ROW_POWER_LABEL    100
#define ROW_POWER_VALUE    110
#define ROW_ENC_X          130
#define ROW_ENC_Y          145
#define ROW_KP_X           170
#define ROW_KP_Y           185
#define ROW_CONVERGED      210

// System modes
enum SystemMode {
    DISABLED,
    MANUAL,
    CALIBRATE,
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

// Debug gates (uncomment to enable)
//#define DEBUG_FIRST_LOOP   // Print mode/EN/lastMode on first loop iteration

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
float Kp_x = 0.07;  // Proportional gain for X axis
float Kp_y = 0.07;  // Proportional gain for Y axis
const float GAIN_INCREMENT = 0.01;  // How much gain changes per encoder click

// AUTO mode control parameters
const int ERROR_DEADBAND = 10;      // Ignore errors smaller than this
const int MAX_STEPS_PER_CYCLE = 50; // Maximum correction per update
const int CONVERGED_THRESHOLD = 15; // Error threshold for "converged" status
const unsigned long AUTO_UPDATE_MS = 100; // Control loop rate (10Hz)

// Stepper motion profile (default — FINE/COARSE refactor will add per-mode profiles)
const float STEPPER_MAX_SPEED = 10000.0;  // steps/sec
const float STEPPER_ACCEL     = 10000.0;  // steps/sec^2

// NeoPixel brightness (0–255)
const uint8_t NEOPIXEL_BRIGHTNESS = 20;

/**
 * @brief Helper: draws the static QPD frame (border, crosshair, corner labels).
 *
 * Used by both initializeQPDCanvas() and updateQPDCanvas() so the layout is
 * defined in one place. Corner labels match the physical QPD layout
 * documented in the file header (D=top-left, A=top-right,
 * C=bottom-left, B=bottom-right).
 */
static void drawQPDStaticElements() {
    uint16_t gray = tft.color565(128, 128, 128);

    // Border
    qpd_canvas.drawRect(0, 0, QPD_DISPLAY_SIZE, QPD_DISPLAY_SIZE, ST77XX_WHITE);

    // Quadrant crosshair
    qpd_canvas.drawFastVLine(QPD_DISPLAY_SIZE / 2, 0, QPD_DISPLAY_SIZE, gray);
    qpd_canvas.drawFastHLine(0, QPD_DISPLAY_SIZE / 2, QPD_DISPLAY_SIZE, gray);

    // Corner labels
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

/**
 * @brief One-time setup of the QPD canvas with background, border, and labels.
 *
 * Draws the static visual frame for the QPD (Quadrant Photodiode) display:
 * - Black background fill
 * - White outer border
 * - Gray crosshair dividing the canvas into four quadrants
 * - Quadrant labels (A, B, C, D) in the corners, matching the physical
 *   QPD layout documented in the file header (D=top-left, A=top-right,
 *   C=bottom-left, B=bottom-right)
 *
 * Called once from setup() after tft.init(). Note that updateQPDCanvas()
 * redraws the border, crosshair, and labels every frame as part of its
 * blank-and-redraw cycle, so this function is effectively a first-frame
 * priming step rather than a true one-time draw.
 *
 * @note Uses tft.color565() purely as a color-packing utility (via the
 *       shared helper); no drawing is performed on tft. Requires tft to
 *       be initialized first.
 */
void initializeQPDCanvas() {
    qpd_canvas.fillScreen(ST77XX_BLACK);
    drawQPDStaticElements();
}

/**
 * @brief Redraws the QPD canvas for the current frame and blits it to the LCD.
 *
 * Per-frame cycle:
 * - Blanks the canvas interior to black
 * - Redraws the static frame (border, crosshair, corner labels)
 * - Plots the current QPD error as a red dot
 * - Plots the AUTO-mode target offset as a green crosshair
 * - Blits the entire canvas to the LCD in a single SPI transfer
 *
 * Error-to-pixel mapping uses QPD_ERROR_TO_PIXEL_DIV. The Y axis is
 * inverted (positive errorY -> upward on screen) because the QPD "top"
 * should appear above center, but screen Y grows downward.
 *
 * All drawing happens to qpd_canvas (in RAM); only the final
 * drawRGBBitmap() generates SPI traffic, which dominates the cost
 * of this function.
 *
 * @note The dot position is clamped to stay fully inside the canvas, but
 *       the target crosshair arms (length QPD_CROSSHAIR_LEN, half
 *       QPD_CROSSHAIR_HALF) can be clipped near the edges since the
 *       clamp margin is smaller than the crosshair half-length. This is
 *       cosmetic; drawFast*Line clips silently.
 */
void updateQPDCanvas() {
    // Blank just the interior (border will be redrawn below)
    qpd_canvas.fillRect(1, 1, QPD_DISPLAY_SIZE - 2, QPD_DISPLAY_SIZE - 2, ST77XX_BLACK);

    // Redraw static frame
    drawQPDStaticElements();

    // Current QPD error as red dot
    int dotX = (QPD_DISPLAY_SIZE / 2) + (errorX / QPD_ERROR_TO_PIXEL_DIV);
    int dotY = (QPD_DISPLAY_SIZE / 2) - (errorY / QPD_ERROR_TO_PIXEL_DIV);
    dotX = constrain(dotX, QPD_DOT_MARGIN, QPD_DISPLAY_SIZE - QPD_DOT_MARGIN);
    dotY = constrain(dotY, QPD_DOT_MARGIN, QPD_DISPLAY_SIZE - QPD_DOT_MARGIN);
    qpd_canvas.fillCircle(dotX, dotY, QPD_DOT_RADIUS, ST77XX_RED);

    // AUTO-mode target offset as green crosshair
    int targetX = (QPD_DISPLAY_SIZE / 2) + (targetOffsetX / QPD_ERROR_TO_PIXEL_DIV);
    int targetY = (QPD_DISPLAY_SIZE / 2) - (targetOffsetY / QPD_ERROR_TO_PIXEL_DIV);
    targetX = constrain(targetX, QPD_DOT_MARGIN, QPD_DISPLAY_SIZE - QPD_DOT_MARGIN);
    targetY = constrain(targetY, QPD_DOT_MARGIN, QPD_DISPLAY_SIZE - QPD_DOT_MARGIN);
    qpd_canvas.drawFastHLine(targetX - QPD_CROSSHAIR_HALF, targetY, QPD_CROSSHAIR_LEN, ST77XX_GREEN);
    qpd_canvas.drawFastVLine(targetX, targetY - QPD_CROSSHAIR_HALF, QPD_CROSSHAIR_LEN, ST77XX_GREEN);

    // Single SPI transfer to LCD
    tft.drawRGBBitmap(QPD_DISPLAY_X, QPD_DISPLAY_Y,
                      qpd_canvas.getBuffer(),
                      qpd_canvas.width(),
                      qpd_canvas.height());
}

/**
 * @brief One-time setup of the static LCD layout (labels and frame).
 *
 * Draws the elements that don't change at runtime:
 * - Black screen background
 * - "Mode:" label at top-left in size-2 white text (the mode value
 *   itself is filled in later by updateDisplayValues())
 * - Right-side data column with color-coded labels for QPD errors,
 *   total power, encoder positions, and proportional gains
 *
 * Numeric values are not drawn here — updateDisplayValues() fills
 * those in next to each label on every display refresh.
 *
 * Called once from setup() after tft.init(). Sets displayInitialized
 * so loop() can verify the static layout is in place before refreshing
 * values. The QPD canvas (left side of screen) is initialized
 * separately by initializeQPDCanvas().
 */
void initializeDisplay() {
    tft.fillScreen(ST77XX_BLACK);

    // "Mode:" label at top
    tft.setTextSize(2);
    tft.setCursor(MODE_LABEL_X, MODE_LABEL_Y);
    tft.setTextColor(ST77XX_WHITE);
    tft.print("Mode: ");

    // Right-side data column labels (size-1 text, color-coded by group)
    tft.setTextSize(1);

    tft.setCursor(DATA_LABEL_X, ROW_ERR_X);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Err X:");

    tft.setCursor(DATA_LABEL_X, ROW_ERR_Y);
    tft.print("Err Y:");

    tft.setCursor(DATA_LABEL_X, ROW_POWER_LABEL);
    tft.setTextColor(ST77XX_GREEN);
    tft.print("Power:");

    tft.setCursor(DATA_LABEL_X, ROW_ENC_X);
    tft.setTextColor(ST77XX_CYAN);
    tft.print("Enc X:");

    tft.setCursor(DATA_LABEL_X, ROW_ENC_Y);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print("Enc Y:");

    tft.setCursor(DATA_LABEL_X, ROW_KP_X);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.print("Kp_x:");

    tft.setCursor(DATA_LABEL_X, ROW_KP_Y);
    tft.print("Kp_y:");

    displayInitialized = true;
}

/**
 * @brief Refreshes all dynamic content on the LCD.
 *
 * Called from loop() at DISPLAY_UPDATE_MS rate or when an encoder moves.
 * Companion to initializeDisplay(): static labels live there, dynamic
 * values live here. Each value field is erased then redrawn (no
 * double-buffering for these — small fields, 10 Hz, no visible flicker
 * in practice).
 *
 * Field layout (right-side data column):
 * - Mode value at top (size-2, color-coded by mode), only redrawn on
 *   mode change to avoid flicker
 * - QPD raw values A/B/C/D on two lines (no static label; values are
 *   self-labeling)
 * - Err X / Err Y, total Power, Enc X / Enc Y, Kp_x / Kp_y values
 *   aligned at DATA_VALUE_X
 * - Power and convergence-status fields use the full column width
 *   (DATA_FIELD_W) instead of DATA_VALUE_W because their content is wider
 * - Convergence status (AUTO mode only): "CONVERGED" or "CORRECTING"
 *   based on whether QPD error is within CONVERGED_THRESHOLD of target
 *
 * Also calls updateQPDCanvas() to refresh the left-side QPD visualization.
 * The two are currently locked to the same update rate.
 *
 * @note Text size is set to 2 in the mode block and 1 thereafter. If the
 *       order of blocks below is ever changed, size must be set explicitly
 *       per block to keep state from leaking.
 */
void updateDisplayValues() {
    tft.setTextWrap(false);

    // Mode value — only redraw on change
    if (currentMode != lastMode) {
        tft.fillRect(MODE_VALUE_X, MODE_LABEL_Y, MODE_VALUE_W, MODE_VALUE_H, ST77XX_BLACK);
        tft.setCursor(MODE_VALUE_X, MODE_LABEL_Y);
        tft.setTextSize(2);
        switch (currentMode) {
            case DISABLED:
                tft.setTextColor(ST77XX_RED);
                tft.print("DISABLED");
                break;
            case MANUAL:
                tft.setTextColor(ST77XX_CYAN);
                tft.print("MANUAL");
                break;
            case CALIBRATE:
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

    // All remaining fields are size-1
    tft.setTextSize(1);

    // QPD raw values: two self-labeled lines (A:val B:val / C:val D:val)
    tft.fillRect(DATA_LABEL_X, ROW_QPD_RAW_1, DATA_FIELD_W, 2 * DATA_LINE_H, ST77XX_BLACK);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(DATA_LABEL_X, ROW_QPD_RAW_1);
    tft.print("A:"); tft.print(qpd_a);
    tft.print(" B:"); tft.print(qpd_b);
    tft.setCursor(DATA_LABEL_X, ROW_QPD_RAW_2);
    tft.print("C:"); tft.print(qpd_c);
    tft.print(" D:"); tft.print(qpd_d);

    // Err X
    tft.fillRect(DATA_VALUE_X, ROW_ERR_X, DATA_VALUE_W, DATA_LINE_H, ST77XX_BLACK);
    tft.setCursor(DATA_VALUE_X, ROW_ERR_X);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print(errorX);

    // Err Y
    tft.fillRect(DATA_VALUE_X, ROW_ERR_Y, DATA_VALUE_W, DATA_LINE_H, ST77XX_BLACK);
    tft.setCursor(DATA_VALUE_X, ROW_ERR_Y);
    tft.print(errorY);

    // Total power (full-column field; value can exceed narrow column)
    tft.fillRect(DATA_LABEL_X, ROW_POWER_VALUE, DATA_FIELD_W, DATA_LINE_H, ST77XX_BLACK);
    tft.setCursor(DATA_LABEL_X, ROW_POWER_VALUE);
    tft.setTextColor(ST77XX_GREEN);
    tft.print(totalPower);

    // Encoder X
    tft.fillRect(DATA_VALUE_X, ROW_ENC_X, DATA_VALUE_W, DATA_LINE_H, ST77XX_BLACK);
    tft.setCursor(DATA_VALUE_X, ROW_ENC_X);
    tft.setTextColor(ST77XX_CYAN);
    tft.print(encoderX_position);

    // Encoder Y
    tft.fillRect(DATA_VALUE_X, ROW_ENC_Y, DATA_VALUE_W, DATA_LINE_H, ST77XX_BLACK);
    tft.setCursor(DATA_VALUE_X, ROW_ENC_Y);
    tft.setTextColor(ST77XX_YELLOW);
    tft.print(encoderY_position);

    // Kp_x (2-decimal)
    tft.fillRect(DATA_VALUE_X, ROW_KP_X, DATA_VALUE_W, DATA_LINE_H, ST77XX_BLACK);
    tft.setCursor(DATA_VALUE_X, ROW_KP_X);
    tft.setTextColor(ST77XX_MAGENTA);
    tft.print(Kp_x, 2);

    // Kp_y (2-decimal)
    tft.fillRect(DATA_VALUE_X, ROW_KP_Y, DATA_VALUE_W, DATA_LINE_H, ST77XX_BLACK);
    tft.setCursor(DATA_VALUE_X, ROW_KP_Y);
    tft.print(Kp_y, 2);

    // Convergence status — AUTO mode only
    tft.fillRect(DATA_LABEL_X, ROW_CONVERGED, DATA_FIELD_W, DATA_LINE_H, ST77XX_BLACK);
    if (currentMode == AUTO) {
        bool converged = (abs(errorX - targetOffsetX) < CONVERGED_THRESHOLD &&
                          abs(errorY - targetOffsetY) < CONVERGED_THRESHOLD);
        tft.setCursor(DATA_LABEL_X, ROW_CONVERGED);
        if (converged) {
            tft.setTextColor(ST77XX_GREEN);
            tft.print("CONVERGED");
        } else {
            tft.setTextColor(ST77XX_RED);
            tft.print("CORRECTING");
        }

    }

    // Refresh QPD visualization (left side)
    updateQPDCanvas();
}

/**
 * @brief Reads all four QPD channels, applies low-pass filtering, and
 *        computes X/Y error and total power.
 *
 * Called from loop() every iteration (no rate limiting). Each channel:
 * - analogRead() at the resolution set in setup() (10-bit, 0–1023)
 * - Single-pole IIR low-pass filter:
 *     filtered = α·new + (1-α)·filtered, with α = QPD_FILTER_ALPHA
 *   The float filter state retains sub-LSB precision; the int copies
 *   in qpd_a/b/c/d are for downstream use and display.
 *
 * Computed quantities:
 * - errorX = (A + B) - (C + D)  → right minus left (positive = beam right)
 * - errorY = (A + D) - (B + C)  → top minus bottom (positive = beam up)
 * - totalPower = A + B + C + D  → sum across all quadrants
 *
 * @note errorX and errorY are currently in raw ADC counts, so their
 *       magnitude scales with totalPower. This causes the same physical
 *       beam offset to read as different error values at different beam
 *       intensities, and is the reason different sensors currently need
 *       different Kp values. Normalization (divide by totalPower) is
 *       planned but not yet implemented — see project notes.
 *
 * @note Filter time constant is fixed in samples (~1/α ≈ 20 samples),
 *       not in wall-clock time. Effective cutoff frequency therefore
 *       depends on loop rate, which varies with display/SPI activity.
 *       Acceptable for 10 Hz drift correction; would need rework for
 *       a tighter control loop.
 */
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

/**
 * @brief Proportional control step for AUTO mode.
 *
 * Called from loop() at AUTO_UPDATE_MS rate (10 Hz) when currentMode is
 * AUTO. Drives both stepper axes toward the target offset using a simple
 * P controller with deadband and per-cycle step limit.
 *
 * Per axis:
 * - Relative error = measured error - target offset, so the controller
 *   tracks the user-defined target rather than QPD center
 * - Errors smaller than ERROR_DEADBAND are ignored (no correction)
 * - Step command = Kp * relativeError, clamped to ±MAX_STEPS_PER_CYCLE
 * - Issued via stepperX/Y.move(), which is a relative move that adds
 *   to any pending motion (correct for a control loop: each cycle
 *   re-evaluates from the current measured error)
 *
 * Sign conventions:
 * - X: positive relativeError → positive step command (no inversion)
 * - Y: positive relativeError → negative step command. errorY is
 *   already "top - bottom" at the QPD layer; this second inversion
 *   maps that to the mechanical Y axis direction. Adjust the sign
 *   here if the Y stepper drives the beam the wrong way after a
 *   hardware change.
 *
 * @note No totalPower threshold check — if the beam walks off the QPD,
 *       totalPower drops and (because errors are not yet normalized)
 *       errorX/Y shrink with it. Authority fades smoothly to zero
 *       rather than failing cleanly. Adding a minimum-power cutout
 *       is on the second-pass list along with error normalization.
 *
 * @note Effective deadband is max(ERROR_DEADBAND, 1/Kp) due to integer
 *       truncation of the step command. With Kp=0.07, errors below ~14
 *       counts truncate to zero steps regardless of ERROR_DEADBAND.
 *
 * @note Pure P control — no I or D term. Acceptable for thermal-drift
 *       correction at 10 Hz; would need extension for faster tracking
 *       or zero-steady-state requirements.
 */
void runAutoControl() {
    // Relative errors: track the user-defined target, not QPD center
    int relativeErrorX = errorX - targetOffsetX;
    int relativeErrorY = errorY - targetOffsetY;

    // Deadband: ignore small errors
    int activeErrorX = (abs(relativeErrorX) > ERROR_DEADBAND) ? relativeErrorX : 0;
    int activeErrorY = (abs(relativeErrorY) > ERROR_DEADBAND) ? relativeErrorY : 0;

    if (activeErrorX != 0) {

        int stepsX = (int)(Kp_x * activeErrorX);
        
        stepsX = constrain(stepsX, -MAX_STEPS_PER_CYCLE, MAX_STEPS_PER_CYCLE);
        stepperX.move(stepsX);

        Serial.print("AUTO X: error=");
        Serial.print(relativeErrorX);
        Serial.print(" steps=");
        Serial.println(stepsX);
    }

    if (activeErrorY != 0) {

        int stepsY = (int)(Kp_y * activeErrorY);
        
        stepsY = constrain(stepsY, -MAX_STEPS_PER_CYCLE, MAX_STEPS_PER_CYCLE);
        stepperY.move(-stepsY);  // Y inverted: see "Sign conventions" in doc comment

        Serial.print("AUTO Y: error=");
        Serial.print(relativeErrorY);
        Serial.print(" steps=");
        Serial.println(-stepsY);
    }
}

/**
 * @brief Updates both encoder NeoPixels to reflect the current system mode.
 *
 * Called after any mode change (and once at end of setup()) to keep the
 * indicator LEDs in sync with currentMode. Both pixels show the same
 * color since mode is system-wide:
 * - DISABLED  → red     (COLOR_DISABLED)
 * - MANUAL    → cyan    (COLOR_MANUAL)
 * - CALIBRATE → yellow  (COLOR_CALIBRATE)
 * - AUTO      → green   (COLOR_AUTO)
 *
 * @note Each show() call is an I2C transaction to the seesaw board.
 *       Cheap but not free; this is why the function is only called
 *       on mode transitions, not in the main loop.
 */
void updateModeLEDs() {
    uint32_t color = COLOR_OFF;

    switch (currentMode) {
        case DISABLED:  color = COLOR_DISABLED;  break;
        case MANUAL:    color = COLOR_MANUAL;    break;
        case CALIBRATE: color = COLOR_CALIBRATE; break;
        case AUTO:      color = COLOR_AUTO;      break;
    }

    pixelX.setPixelColor(0, color);
    pixelX.show();

    pixelY.setPixelColor(0, color);
    pixelY.show();
}

/**
 * @brief Cycles to the next operating mode and applies its motor-enable state.
 *
 * Called from loop() on a debounced press of the encoder X button.
 * Mode cycle: DISABLED → MANUAL → CALIBRATE → AUTO → DISABLED.
 *
 * Per-mode motor enable (TMC2209 EN pin is active LOW):
 * - DISABLED:  EN HIGH (motors disabled)
 * - MANUAL:    EN LOW  (motors enabled — encoders drive steppers)
 * - CALIBRATE: EN HIGH (motors disabled — encoders adjust Kp_x/Kp_y)
 * - AUTO:      EN LOW  (motors enabled — QPD feedback drives steppers)
 *
 * Updates the encoder NeoPixels at the end so indicator color always
 * matches the new mode.
 *
 * @note Mode cycling is one-directional. To return to MANUAL from AUTO,
 *       the cycle must pass through DISABLED.
 *
 * @note Exiting AUTO does not clear targetOffsetX/Y, pending stepper
 *       moves, or lastAutoUpdate. Re-entering AUTO resumes with the
 *       previous target. If a clean-slate-on-entry behavior is ever
 *       wanted, add the reset here.
 *
 * @note When FINE/COARSE adjustment is added to MANUAL, applying the
 *       chosen speed/acceleration profile belongs in the MANUAL case
 *       below.
 */
void handleModeButton() {
    switch (currentMode) {
        case DISABLED:
            currentMode = MANUAL;
            digitalWrite(MOTOR_EN, LOW);
            Serial.println("Mode: MANUAL");

            break;
        case MANUAL:
            currentMode = CALIBRATE;
            digitalWrite(MOTOR_EN, HIGH);
            Serial.println("Mode: CALIBRATE");

            break;
        case CALIBRATE:
            currentMode = AUTO;
            digitalWrite(MOTOR_EN, LOW);
            Serial.println("Mode: AUTO");

            break;
        case AUTO:
            currentMode = DISABLED;
            digitalWrite(MOTOR_EN, HIGH);
            Serial.println("Mode: DISABLED");

            break;
    }

    updateModeLEDs();
}


/**
 * @brief One-time hardware and state initialization.
 *
 * Boot sequence:
 * - Serial console
 * - Motor enable pin (steppers held disabled until MANUAL or AUTO is entered)
 * - Stepper motion profile (default speed/acceleration)
 * - LCD init and "Initializing..." banner
 * - ADC resolution
 * - I2C bus
 * - Both encoders (with NeoPixels) — failure is logged but non-fatal
 * - Initial button state captured after seesaw boards have settled
 * - Initial QPD read, full LCD layout draw, first display refresh
 *
 * @note Encoder init failure is non-fatal: an error is printed to Serial
 *       but setup() continues. Subsequent reads against a missing encoder
 *       will return stale or zero values silently. An on-screen warning
 *       on init failure is on the second-pass list.
 *
 * @note Button polarity (press vs. release detection in loop()) has not
 *       been bench-verified against the seesaw INPUT_PULLUP semantics.
 *       The system works in practice; flagged for future verification
 *       if button behavior ever feels wrong.
 */
void setup() {
    Serial.begin(115200);
    delay(1000);  // Let USB serial enumerate before first print
    Serial.println("Dr. Strangelove - Motor Control");

    // Motor enable: hold disabled at boot (TMC2209 EN is active LOW)
    pinMode(MOTOR_EN, OUTPUT);
    digitalWrite(MOTOR_EN, HIGH);

    // Stepper motion profile
    stepperX.setMaxSpeed(STEPPER_MAX_SPEED);
    stepperX.setAcceleration(STEPPER_ACCEL);
    stepperY.setMaxSpeed(STEPPER_MAX_SPEED);
    stepperY.setAcceleration(STEPPER_ACCEL);

    // LCD
    tft.init(240, 320);
    tft.setRotation(3);
    tft.fillScreen(ST77XX_BLACK);
    tft.setCursor(10, 10);
    tft.setTextColor(ST77XX_WHITE);
    tft.setTextSize(2);
    tft.println("Initializing...");

    // ADC: 10-bit (0–1023)
    analogReadResolution(10);

    // I2C
    Wire.begin();

    // Encoder X (with NeoPixel)
    if (!encoderX.begin(ENCODER_X_ADDR)) {
        Serial.println("ERROR: Encoder X not found!");
    } else {
        Serial.println("Encoder X initialized");
        encoderX.pinMode(SS_SWITCH, INPUT_PULLUP);
        encoderX_position = encoderX.getEncoderPosition();

        pixelX.begin(ENCODER_X_ADDR);
        pixelX.setBrightness(NEOPIXEL_BRIGHTNESS);
        pixelX.show();

        // Discard first switch read — initial state can be invalid
        // immediately after seesaw init
        encoderX.digitalRead(SS_SWITCH);
        delay(10);
    }

    // Encoder Y (with NeoPixel)
    if (!encoderY.begin(ENCODER_Y_ADDR)) {
        Serial.println("ERROR: Encoder Y not found!");
    } else {
        Serial.println("Encoder Y initialized");
        encoderY.pinMode(SS_SWITCH, INPUT_PULLUP);
        encoderY_position = encoderY.getEncoderPosition();

        pixelY.begin(ENCODER_Y_ADDR);
        pixelY.setBrightness(NEOPIXEL_BRIGHTNESS);
        pixelY.show();

        // Discard first switch read — initial state can be invalid
        // immediately after seesaw init
        encoderY.digitalRead(SS_SWITCH);
        delay(10);
    }

    // Capture stable initial button state (after seesaw discard-reads above)
    lastButtonX = encoderX.digitalRead(SS_SWITCH);
    lastButtonY = encoderY.digitalRead(SS_SWITCH);
    lastButtonXTime = millis();
    lastButtonYTime = millis();

    // Let seesaw boards settle fully before first frame
    delay(1000);
    updateModeLEDs();

    // Initial QPD read so first display frame has real values
    readQPD();
    initializeDisplay();
    initializeQPDCanvas();

    // First-frame mode display: trigger updateDisplayValues() to draw the
    // mode label by ensuring lastMode != currentMode
    lastMode = AUTO;
    updateDisplayValues();

}

/**
 * @brief Main runtime loop — runs continuously after setup() returns.
 *
 * Each iteration:
 * - Reads QPD (with low-pass filter applied in readQPD())
 * - Polls both encoders (position and switch)
 * - Handles debounced button presses:
 *     X button → cycle mode via handleModeButton()
 *     Y button → toggle AUTO target offset between captured and zero
 * - Routes encoder rotation by mode:
 *     MANUAL    → step the corresponding stepper
 *     CALIBRATE → adjust the corresponding Kp gain
 *     (DISABLED/AUTO ignore rotation but still track position so a
 *      mode change doesn't produce a spurious delta)
 * - Runs runAutoControl() at AUTO_UPDATE_MS rate when in AUTO mode
 * - Calls stepperX.run() / stepperY.run() every iteration (required
 *   by AccelStepper to advance step pulses; loop rate must exceed
 *   the step rate set by STEPPER_MAX_SPEED)
 * - Refreshes the LCD at DISPLAY_UPDATE_MS rate or immediately on
 *   encoder motion
 *
 * @note Encoder rotation in DISABLED/AUTO updates encoderX_position /
 *       encoderY_position but produces no action, so re-entering
 *       MANUAL/CALIBRATE doesn't generate a spurious delta from
 *       turns made while inactive.
 *
 * @note Display refresh is driven by either the 10 Hz tick or encoder
 *       motion. Burst encoder activity therefore pushes refresh rate
 *       above 10 Hz; the SPI blit self-throttles since each update
 *       takes a few ms.
 *
 * @note Serial output is verbose (per-cycle QPD dump, per-correction
 *       AUTO prints, per-delta encoder prints). Fine on USB serial;
 *       gate behind DEBUG_* defines if it ever drowns out other logs.
 */
void loop() {
    unsigned long now = millis();

#ifdef DEBUG_FIRST_LOOP
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
#endif

    // Always read QPD
    readQPD();

    // Read encoder positions
    int32_t newX = encoderX.getEncoderPosition();
    int32_t newY = encoderY.getEncoderPosition();

    // Read buttons
    bool buttonX = encoderX.digitalRead(SS_SWITCH);
    bool buttonY = encoderY.digitalRead(SS_SWITCH);

    // Button X with debounce — cycles mode
    if (buttonX == 1 && lastButtonX == 0 && (now - lastButtonXTime > DEBOUNCE_MS)) {
        handleModeButton();
        lastButtonXTime = now;
    }
    lastButtonX = buttonX;

    // Button Y with debounce — toggles AUTO target offset
    if (buttonY == 1 && lastButtonY == 0 && (now - lastButtonYTime > DEBOUNCE_MS)) {
        if (targetOffsetX == 0 && targetOffsetY == 0) {
            // No target set — capture current error as new target
            targetOffsetX = errorX;
            targetOffsetY = errorY;
            Serial.print("Target set: X=");
            Serial.print(targetOffsetX);
            Serial.print(" Y=");
            Serial.println(targetOffsetY);
        } else {
            // Target is set — clear back to center
            targetOffsetX = 0;
            targetOffsetY = 0;
            Serial.println("Target cleared (back to center)");
        }
        lastButtonYTime = now;
    }
    lastButtonY = buttonY;

    // Encoder motion — route by mode
    bool encoderMoved = (newX != encoderX_position || newY != encoderY_position);
    if (encoderMoved) {
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
            // Encoders adjust gains, not motor position
            if (deltaX != 0) {
                Kp_x += deltaX * GAIN_INCREMENT;
                Kp_x = constrain(Kp_x, 0.0, 10.0);
                Serial.print("Kp_x: ");
                Serial.println(Kp_x);
            }
            if (deltaY != 0) {
                Kp_y += deltaY * GAIN_INCREMENT;
                Kp_y = constrain(Kp_y, 0.0, 10.0);
                Serial.print("Kp_y: ");
                Serial.println(Kp_y);
            }
        }

        // Always update stored positions, even in DISABLED/AUTO
        encoderX_position = newX;
        encoderY_position = newY;
    }

    // AUTO control loop — proportional update at AUTO_UPDATE_MS rate
    if (currentMode == AUTO) {
        if (now - lastAutoUpdate >= AUTO_UPDATE_MS) {
            runAutoControl();
            lastAutoUpdate = now;
        }
    }

    // Required: advance step pulses every iteration
    stepperX.run();
    stepperY.run();

    // Display refresh at fixed rate or on encoder motion
    if ((now - lastDisplayUpdate >= DISPLAY_UPDATE_MS) || encoderMoved) {
        if (!displayInitialized) {
            initializeDisplay();
        }
        updateDisplayValues();
        lastDisplayUpdate = now;

        // Periodic Serial dump
        Serial.print("QPD A:"); Serial.print(qpd_a);
        Serial.print(" B:"); Serial.print(qpd_b);
        Serial.print(" C:"); Serial.print(qpd_c);
        Serial.print(" D:"); Serial.print(qpd_d);
        Serial.print(" | ErrX:"); Serial.print(errorX);
        Serial.print(" ErrY:"); Serial.print(errorY);
        Serial.print(" | Total:"); Serial.println(totalPower);
    }
}

