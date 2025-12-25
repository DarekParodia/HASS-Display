// LED fade in/out on GPIO 21 for ESP32 using LEDC PWM
#include <Arduino.h>
#include <ArduinoHA.h>
#include <U8g2lib.h>
#include <WiFiManager.h>
#include <LittleFS.h>
#include <time.h>
#include <sstream>
#include <iomanip>
#include <string>
#include <AccelStepper.h>

#define LCD_CLOCK            1
#define LCD_DATA             0
#define LCD_CS               4
#define LCD_RS               2
#define LCD_RSE              3
#define LCD_BACKLIGHT        21

#define BUTTON1_PIN          20
#define BUTTON2_PIN          10
#define BUTTON3_PIN          5 // Use GPIO 5 (safe pin, no flash/boot conflicts)

#define FONT_BIG             u8g2_font_t0_40_tf // 23 pixels high
#define FONT_MEDIUM          u8g2_font_t0_22_tf // 13 pixels high
#define FONT_SMALL           u8g2_font_t0_13_tf // 9 pixels high
#define FONT_TINY            u8g2_font_tiny5_tf // 7 pixels high

#define FONT_PRIMARY_DATA    u8g2_font_luRS18_tn // 18 pixels high
#define FONT_SECONDARY_DATA  FONT_MEDIUM

#define DEVICE_NAME          "HASS-Display"

#define DATA_PRIMARY_TOPIC   "GreenThing/27B529/CO/temperature"
#define DATA_SECONDARY_TOPIC "GreenThing/27B529/CWU/temperature"
#define DATA3_TOPIC          "wled/62fad8/temperature"
#define DATA4_TOPIC          "wled/b47157/temperature"

#define BACKLIGHT_TIME       15000 // ms

// NTP Configuration
#define NTP_SERVER           "pool.ntp.org"
#define GMT_OFFSET_SEC       3600 // GMT+1 (adjust for your timezone)
#define DAYLIGHT_OFFSET_SEC  3600 // Daylight saving time offset

#define EN_PIN               6
#define STEP_PIN             7
#define STEPPER_MICROSTEPS   16
#define STEPS_PER_REV        (200 * STEPPER_MICROSTEPS) // 200 full steps per revolution

enum ActivityState {
    ACTIVITY_LOW,
    ACTIVITY_HIGH,
    ACTIVITY_STEPPER
};

struct settings {
        char    mqtt_server[64]     = "";
        int     mqtt_port           = 1883;
        char    mqtt_user[64]       = "";
        char    mqtt_password[64]   = "";

        uint8_t LCD_CONTRAST_VAL    = 128;
        uint8_t LCD_BACKLIGHT_VAL   = 128;

        int     StepperSpeed        = 10;
        int     StepperAccel        = 20;
        int     GramsFeededToday    = 0;
        float   RotationsPerFeeding = 1.0f;
        float   GramsPerFeeding     = 1.0f;
        float   MaxGramsPerDay      = 100.0f;

        void    saveToFS() {
            File file = LittleFS.open("/settings.bin", "w");

            if(file) {
                file.write((uint8_t *) this, sizeof(settings));
                file.close();
                // Serial.println("Settings saved to filesystem.");
            } else {
                // Serial.println("Failed to open settings file for writing.");
            }
        }

        void loadFromFS() {
            File file = LittleFS.open("/settings.bin", "r");

            if(file) {
                if(file.size() == sizeof(settings)) {
                    file.read((uint8_t *) this, sizeof(settings));
                    // Serial.println("Settings loaded from filesystem.");
                } else {
                    // read only mqtt settings if size mismatch
                    file.read((uint8_t *) this, sizeof(char) * 64 * 3 + sizeof(int));

                    // Serial.println("Settings file size mismatch. Using default settings.");
                }
                file.close();
            } else {
                // Serial.println("Settings file not found. Using default settings.");
                this->saveToFS(); // Save default settings
            }
        }
};

WiFiManagerParameter              *mqtt_server_param;
WiFiManagerParameter              *mqtt_port_param;
WiFiManagerParameter              *mqtt_user_param;
WiFiManagerParameter              *mqtt_password_param;
WiFiManager                        wifiManager;

WiFiClient                         client;
HADevice                           device(DEVICE_NAME);
HAMqtt                             mqtt(client, device);
AccelStepper                       stepper(AccelStepper::DRIVER, STEP_PIN, 8);

hw_timer_t                        *stepperTimer = NULL;
portMUX_TYPE                       stepperMux   = portMUX_INITIALIZER_UNLOCKED;

settings                           config       = {};

HALight                            backlight("backlight", HALight::BrightnessFeature);
HANumber                           contrast("contrast", HABaseDeviceType::PrecisionP0);
HANumber                           stepperSpeed("stepper_speed", HABaseDeviceType::PrecisionP0);
HANumber                           stepperAccel("stepper_accel", HABaseDeviceType::PrecisionP0);
HANumber                           rotationsPerFeeding("rotations_per_feeding", HABaseDeviceType::PrecisionP2);
HANumber                           gramsPerFeeding("grams_per_feeding", HABaseDeviceType::PrecisionP2);
HANumber                           maxGramsPerDay("max_grams_per_day", HABaseDeviceType::PrecisionP2);

HASensorNumber                     gramsFedTodaySensor("grams_fed_today");

HAButton                           feedNowButton("feed_now");

HADeviceTrigger                    trigger1short(HADeviceTrigger::ButtonShortPressType, "btn1");
HADeviceTrigger                    trigger1long(HADeviceTrigger::ButtonLongPressType, "btn1");
bool                               triggered1long  = false;
bool                               triggered1short = false;

HADeviceTrigger                    trigger2short(HADeviceTrigger::ButtonShortPressType, "btn2");
HADeviceTrigger                    trigger2long(HADeviceTrigger::ButtonLongPressType, "btn2");
bool                               triggered2long          = false;
bool                               triggered2short         = false;

long                               button1PressinTime      = 0;
long                               button2PressinTime      = 0;
volatile unsigned long             button1LastDebounce     = 0;
volatile unsigned long             button2LastDebounce     = 0;
const unsigned long                BUTTON_DEBOUNCE_TIME    = 50; // ms

const long                         BUTTON_LONGPRESS_TIME   = 500; // ms

ActivityState                      currentActivityState    = ACTIVITY_HIGH;

const float                        DELTA_TIME_DIVIDER      = 1000.0f * 60.0f; // to convert ms to seconds

float                              PrimaryData             = 0.0f;
float                              SecondaryData           = 0.0f;
float                              PrimaryDelta            = 0.0f;
float                              SecondaryDelta          = 0.0f;
float                              PrimaryDeltaThreshold   = 0.15f;
float                              SecondaryDeltaThreshold = 0.15f;
float                              lastPrimaryData         = 0.0f;
float                              lastSecondaryData       = 0.0f;
long                               lastPrimaryDataTime     = 0;
long                               lastSecondaryDataTime   = 0;

float                              Data3                   = 0.0f;
float                              Data4                   = 0.0f;

int                                lastDay                 = -1; // Track last known day for new day detection

U8G2_ST7565_NHD_C12864_F_4W_SW_SPI u8g2(U8G2_R0,
/* clock=*/LCD_CLOCK,
/* data=*/LCD_DATA,
/* cs=*/LCD_CS,
/* dc=*/LCD_RS,
/* reset=*/LCD_RSE);


// functions
void           setBacklight(uint8_t brightness);
void           setContrast(uint8_t contrast);

void           onLCDStateCommand(bool state, HALight *sender);
void           onLCDBrightnessCommand(uint8_t brightness, HALight *sender);

void           onContrastCommand(HANumeric value, HANumber *sender);
void           onStepperSpeedCommand(HANumeric value, HANumber *sender);
void           onStepperAccelCommand(HANumeric value, HANumber *sender);
void           onRotationsPerFeedingCommand(HANumeric value, HANumber *sender);
void           onGramsPerFeedingCommand(HANumeric value, HANumber *sender);
void           onMaxGramsPerDayCommand(HANumeric value, HANumber *sender);
void           onFeedNowCommand(HAButton *sender);
void           onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length);
void           render();
void           feedNow();
void           stepperLoop();
void IRAM_ATTR onStepperTimer();
void           drawTextWithSpacing(int x, int y, const char *text, int spacing);
void IRAM_ATTR buttonISR();
void           setupNTP();
void           checkNewDay();
//
volatile unsigned long lastButtonInterruptTime = 0;
const unsigned long    BUTTON_DEBOUNCE_MS      = 200;

void IRAM_ATTR         buttonISR() {
    unsigned long currentTime = millis();
    if((currentTime - lastButtonInterruptTime) > BUTTON_DEBOUNCE_MS) {
        currentActivityState    = ACTIVITY_HIGH;
        lastButtonInterruptTime = currentTime;
        setBacklight(config.LCD_BACKLIGHT_VAL);
    }
}

void IRAM_ATTR usageButton1ISR() {
    unsigned long currentTime = millis();
    if((currentTime - button1LastDebounce) < BUTTON_DEBOUNCE_TIME) return;
    button1LastDebounce = currentTime;

    int state           = digitalRead(BUTTON2_PIN);
    if(state == HIGH) {
        // RISING - button pressed
        button1PressinTime = currentTime;
        buttonISR();
    } else {
        // FALLING - button released
        if((currentTime - button1PressinTime) >= BUTTON_LONGPRESS_TIME)
            triggered1long = true;
        else
            triggered1short = true;
    }
}

void IRAM_ATTR usageButton2ISR() {
    unsigned long currentTime = millis();
    if((currentTime - button2LastDebounce) < BUTTON_DEBOUNCE_TIME) return;
    button2LastDebounce = currentTime;

    int state           = digitalRead(BUTTON3_PIN);
    if(state == HIGH) {
        // RISING - button pressed
        button2PressinTime = currentTime;
        buttonISR();
    } else {
        // FALLING - button released
        if((currentTime - button2PressinTime) >= BUTTON_LONGPRESS_TIME)
            triggered2long = true;
        else
            triggered2short = true;
    }
}

void serviceCheck() {
    // Check WiFi connection
    if(WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi disconnected. Attempting to reconnect...");
        WiFi.reconnect();
        delay(500);
    }

    // Check MQTT connection
    if(!mqtt.isConnected()) {
        Serial.println("MQTT disconnected. Attempting to reconnect...");
        mqtt.begin(config.mqtt_server, config.mqtt_user, config.mqtt_password);
        delay(500);
    }

    mqtt.loop();
}

void setup() {
    // Configure LEDC PWM and attach GPIO 21
    Serial.begin(115200);

    pinMode(LCD_BACKLIGHT, OUTPUT);
    pinMode(EN_PIN, OUTPUT);

    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    pinMode(BUTTON2_PIN, INPUT_PULLDOWN);
    pinMode(BUTTON3_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), buttonISR, RISING);

    attachInterrupt(digitalPinToInterrupt(BUTTON2_PIN), usageButton1ISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BUTTON3_PIN), usageButton2ISR, CHANGE);

    // Try mounting
    if(!LittleFS.begin()) {
        Serial.println("LittleFS mount failed, formatting...");
        LittleFS.format(); // Create a new filesystem
        LittleFS.begin();  // Mount again
    } else {
        Serial.println("LittleFS mounted successfully.");
    }
    config.loadFromFS();

    // Setup stepper motor
    digitalWrite(EN_PIN, HIGH); // Disable the stepper driver
    stepper.setMaxSpeed(config.StepperSpeed * STEPPER_MICROSTEPS);
    stepper.setAcceleration(config.StepperAccel * STEPPER_MICROSTEPS);

    // Initialize the display
    u8g2.begin();
    u8g2.setContrast(config.LCD_CONTRAST_VAL);
    u8g2.clearDisplay();

    // Set initial backlight brightness
    setBacklight(config.LCD_BACKLIGHT_VAL);
    setContrast(config.LCD_CONTRAST_VAL);

    // Display a welcome message
    u8g2.clearBuffer();
    u8g2.setFont(FONT_SMALL);
    u8g2.drawStr(0, 10, "Welcome");
    u8g2.drawStr(0, 20, "Starting WiFi AP...");
    u8g2.sendBuffer();

    // WiFiManager setup
    wifiManager.setHostname(DEVICE_NAME);

    mqtt_server_param   = new WiFiManagerParameter("server", "MQTT Server", config.mqtt_server, 40);
    mqtt_port_param     = new WiFiManagerParameter("port", "MQTT Port", std::to_string(config.mqtt_port).c_str(), 6);
    mqtt_user_param     = new WiFiManagerParameter("user", "MQTT User", config.mqtt_user, 32);
    mqtt_password_param = new WiFiManagerParameter("password", "MQTT Password", config.mqtt_password, 32);

    wifiManager.addParameter(mqtt_server_param);
    wifiManager.addParameter(mqtt_port_param);
    wifiManager.addParameter(mqtt_user_param);
    wifiManager.addParameter(mqtt_password_param);

    // Setup WiFi Manager
    if(!wifiManager.autoConnect(DEVICE_NAME "-AP", "qqqqqqqq")) {
        // If connection fails, reset and try again
        u8g2.drawStr(0, 30, "Failed to connect");
        u8g2.drawStr(0, 40, "to WiFi");
        u8g2.drawStr(0, 50, "Rebooting...");
        u8g2.sendBuffer();

        delay(3000);
        ESP.restart();
        delay(1000);
    }
    // Connected to WiFi
    u8g2.clearBuffer();
    u8g2.drawStr(0, 10, "WiFi Connected!");
    u8g2.drawStr(0, 20, "IP Address:");
    u8g2.drawStr(0, 30, WiFi.localIP().toString().c_str());
    u8g2.sendBuffer();

    // Initialize NTP time sync
    setupNTP();

    strncpy(config.mqtt_server, mqtt_server_param->getValue(), sizeof(config.mqtt_server));
    config.mqtt_server[sizeof(config.mqtt_server) - 1] = '\0';
    config.mqtt_port                                   = atoi(mqtt_port_param->getValue());
    strncpy(config.mqtt_user, mqtt_user_param->getValue(), sizeof(config.mqtt_user));
    config.mqtt_user[sizeof(config.mqtt_user) - 1] = '\0';
    strncpy(config.mqtt_password, mqtt_password_param->getValue(), sizeof(config.mqtt_password));
    config.mqtt_password[sizeof(config.mqtt_password) - 1] = '\0';

    config.saveToFS();

    // Setup HA Device
    device.setName(DEVICE_NAME);
    device.setSoftwareVersion("0.1");

    // Setup HA Light
    backlight.setName("LCD Backlight");
    backlight.setIcon("mdi:monitor");
    backlight.onStateCommand(onLCDStateCommand);
    backlight.onBrightnessCommand(onLCDBrightnessCommand);
    backlight.setOptimistic(true);

    // Setup HA Contrast
    contrast.setName("LCD Contrast");
    contrast.setIcon("mdi:contrast");
    contrast.setMode(HANumber::ModeSlider);
    contrast.setMin(0.0f);
    contrast.setMax(255.0f);
    contrast.setStep(1.0f);
    contrast.onCommand(onContrastCommand);
    contrast.setOptimistic(true);

    // Setup Stepper Speed
    stepperSpeed.setName("Stepper Speed");
    stepperSpeed.setIcon("mdi:speedometer");
    stepperSpeed.setMode(HANumber::ModeSlider);
    stepperSpeed.setMin(1.0f);
    stepperSpeed.setMax(100.0f);
    stepperSpeed.setStep(1.0f);
    stepperSpeed.onCommand(onStepperSpeedCommand);
    stepperSpeed.setOptimistic(true);

    // Setup Stepper Acceleration
    stepperAccel.setName("Stepper Acceleration");
    stepperAccel.setIcon("mdi:run-fast");
    stepperAccel.setMode(HANumber::ModeSlider);
    stepperAccel.setMin(1.0f);
    stepperAccel.setMax(100.0f);
    stepperAccel.setStep(1.0f);
    stepperAccel.onCommand(onStepperAccelCommand);
    stepperAccel.setOptimistic(true);

    // Setup Rotations Per Feeding
    rotationsPerFeeding.setName("Rotations Per Feeding");
    rotationsPerFeeding.setIcon("mdi:rotate-right");
    rotationsPerFeeding.setMode(HANumber::ModeBox);
    rotationsPerFeeding.setMin(0.01f);
    rotationsPerFeeding.setMax(10.0f);
    rotationsPerFeeding.setStep(0.01f);
    rotationsPerFeeding.onCommand(onRotationsPerFeedingCommand);
    rotationsPerFeeding.setOptimistic(true);

    // Setup Grams Per Feeding
    gramsPerFeeding.setName("Grams Per Feeding");
    gramsPerFeeding.setIcon("mdi:weight-gram");
    gramsPerFeeding.setMode(HANumber::ModeBox);
    gramsPerFeeding.setMin(0.01f);
    gramsPerFeeding.setMax(100.0f);
    gramsPerFeeding.setStep(0.01f);
    gramsPerFeeding.onCommand(onGramsPerFeedingCommand);
    gramsPerFeeding.setOptimistic(true);

    // Setup Max Grams Per Day
    maxGramsPerDay.setName("Max Grams Per Day");
    maxGramsPerDay.setIcon("mdi:scale");
    maxGramsPerDay.setMode(HANumber::ModeBox);
    maxGramsPerDay.setMin(1.0f);
    maxGramsPerDay.setMax(500.0f);
    maxGramsPerDay.setStep(0.01f);
    maxGramsPerDay.onCommand(onMaxGramsPerDayCommand);
    maxGramsPerDay.setOptimistic(true);

    // Setup Feed Now Button
    feedNowButton.setName("Feed Now");
    feedNowButton.setIcon("mdi:food");
    feedNowButton.onCommand(onFeedNowCommand);

    // Grams Fed Today Sensor
    gramsFedTodaySensor.setName("Grams Fed Today");
    gramsFedTodaySensor.setIcon("mdi:counter");
    gramsFedTodaySensor.setUnitOfMeasurement("g");

    mqtt.onMessage(onMqttMessage);
    mqtt.begin(config.mqtt_server, config.mqtt_user, config.mqtt_password);
    mqtt.loop();

    mqtt.subscribe(DATA_PRIMARY_TOPIC);
    mqtt.subscribe(DATA_SECONDARY_TOPIC);
    mqtt.subscribe(DATA3_TOPIC);
    mqtt.subscribe(DATA4_TOPIC);

    // send states
    backlight.setState(config.LCD_BACKLIGHT_VAL > 0);
    backlight.setBrightness(config.LCD_BACKLIGHT_VAL);

    contrast.setState(static_cast<float>(config.LCD_CONTRAST_VAL));

    stepperSpeed.setState(static_cast<float>(config.StepperSpeed));
    stepperAccel.setState(static_cast<float>(config.StepperAccel));
    rotationsPerFeeding.setState(config.RotationsPerFeeding);
    gramsPerFeeding.setState(config.GramsPerFeeding);
    maxGramsPerDay.setState(config.MaxGramsPerDay);

    mqtt.loop();

    // enable light sleep
    WiFi.setSleep(true);                // This is light sleep, not deep sleep
    esp_wifi_set_ps(WIFI_PS_MIN_MODEM); // Minimal power saving

    // Setup hardware timer for stepper (timer 0, 80 divider = 1MHz, count up)
    stepperTimer = timerBegin(0, 80, true);                    // Timer 0, prescaler 80 (1MHz), count up
    timerAttachInterrupt(stepperTimer, &onStepperTimer, true); // Edge triggered
    timerAlarmWrite(stepperTimer, 50, true);                   // 50us interval = 20kHz, auto-reload
    timerAlarmEnable(stepperTimer);                            // Start the timer
}

void IRAM_ATTR onStepperTimer() {
    portENTER_CRITICAL_ISR(&stepperMux);
    if(stepper.isRunning())
        stepper.run();
    portEXIT_CRITICAL_ISR(&stepperMux);
}

void stepperLoop() {
    // Check if stepper finished and disable driver
    if(!stepper.isRunning())
        digitalWrite(EN_PIN, HIGH); // Disable the stepper driver
}

void loop() {
    if(triggered1long) {
        trigger1long.trigger();
        triggered1long = false;
        Serial.println("Long press 1 detected");
    }
    if(triggered1short) {
        trigger1short.trigger();
        triggered1short = false;
        Serial.println("Short press 1 detected");
    }
    if(triggered2long) {
        trigger2long.trigger();
        triggered2long = false;
        Serial.println("Long press 2 detected");
    }
    if(triggered2short) {
        trigger2short.trigger();
        triggered2short = false;
        Serial.println("Short press 2 detected");
    }

    render();
    mqtt.loop();
    serviceCheck();
    stepperLoop(); // Check if stepper finished
    checkNewDay(); // Check if a new day has started

    long currentTime = millis();

    switch(currentActivityState) {
        case ACTIVITY_HIGH:
            if((currentTime - lastButtonInterruptTime) > BACKLIGHT_TIME && currentActivityState != ACTIVITY_STEPPER) {
                currentActivityState = ACTIVITY_LOW;
                // Turn off backlight
                setBacklight(0);
                backlight.setState(false);
            } else {
                backlight.setState(true);
            }
            if(currentActivityState != ACTIVITY_STEPPER)
                delay(10);
            break;

        case ACTIVITY_LOW:
            delay(10);
            break;

        default:
            break;
    }
}

int getTextWidth(const char *text, int charWidth, int spacing) {
    int width = 0;
    while(*text) {
        width += charWidth + spacing;
        text++;
    }
    return width;
}
void drawTextWithSpacing(int x, int y, const char *text, int spacing) {
    u8g2.setCursor(x, y);

    while(*text) {
        char c[2] = { *text++, '\0' };
        u8g2.print(c);
        u8g2.setCursor(u8g2.getCursorX() + spacing, y);
    }
}
void drawFloat(int x, int y, float value, int decimalPlaces, int spacing, const uint8_t *fontPrimary = FONT_PRIMARY_DATA, const uint8_t *fontSecondary = FONT_SMALL) {
    char buf[16];
    snprintf(buf, sizeof(buf), "%.*f", decimalPlaces, value);
    char *dot = strchr(buf, '.');
    if(dot) *dot = '\0'; // terminate integer part
    // Draw integer part (big font)
    u8g2.setFont(fontPrimary);
    drawTextWithSpacing(x, y, buf, spacing);

    x = u8g2.getCursorX();
    if(dot) {
        u8g2.setFont(fontSecondary);
        drawTextWithSpacing(x, y, ".", 0);
        x = u8g2.getCursorX();
        drawTextWithSpacing(x, y, dot + 1, spacing); // decimal digits only
    }
}
void drawArrow(int x, int y, int size, bool up) {
    if(up) {
        u8g2.drawLine(x, y + size, x + size / 2, y);
        u8g2.drawLine(x + size / 2, y, x + size, y + size);
    } else {
        u8g2.drawLine(x, y, x + size / 2, y + size);
        u8g2.drawLine(x + size / 2, y + size, x + size, y);
    }
}
void drawETCTemp(int x, int y, int width, int height, std::string label, float temp, int labelXOffset = 0) {
    u8g2.drawFrame(x, y, width, height);

    u8g2.setFont(FONT_SMALL);
    int xOff = 2 + labelXOffset;
    int yOff = height - 4;
    drawTextWithSpacing(x + xOff, y + yOff, label.c_str(), 0);

    drawFloat(x + width - 30, y + yOff, temp, 1, -1, FONT_SMALL, FONT_SMALL);
}

void render() {
    u8g2.clearBuffer();

    int x               = 0;
    int coY             = 28;
    int cwuY            = 61;
    int spacing         = -2;
    int labelSpacing    = 1;
    int labelHeight     = 10;
    int labelMarginLeft = 2;
    int tempWidth       = 45;

    // Draw Temps Border
    u8g2.drawFrame(x, 0, tempWidth, 64);
    x             += 1;

    // Draw Temps
    int coLabelY   = coY - labelHeight - 10;
    int cwuLabelY  = cwuY;
    u8g2.setFont(FONT_TINY);
    drawTextWithSpacing(x + labelMarginLeft, coLabelY, "CO", labelSpacing);
    drawTextWithSpacing(x + labelMarginLeft, cwuY, "CWU", labelSpacing);

    drawFloat(x, coY, PrimaryData, 1, spacing);
    drawFloat(x, cwuY - labelHeight, SecondaryData, 1, spacing);

    // Draw Arrows
    int size         = 4;
    int arrowX       = tempWidth - size - 4;
    int arrowOffsetY = -5;

    if(PrimaryDelta > 0) {
        if(PrimaryDelta >= PrimaryDeltaThreshold)
            drawArrow(arrowX, coLabelY + arrowOffsetY, size, true);
    } else {
        if(-PrimaryDelta >= PrimaryDeltaThreshold)
            drawArrow(arrowX, coLabelY + arrowOffsetY, size, false);
    }

    if(SecondaryDelta > 0) {
        if(SecondaryDelta >= SecondaryDeltaThreshold)
            drawArrow(arrowX, cwuLabelY + arrowOffsetY, size, true);
    } else {
        if(-SecondaryDelta >= SecondaryDeltaThreshold)
            drawArrow(arrowX, cwuLabelY + arrowOffsetY, size, false);
    }
    x                    += tempWidth;

    // Draw ETC Border
    int etcOffsetX        = -2;
    x                    += etcOffsetX;
    int etcWidth          = 128 - x;
    int etcSegmentHeight  = 16;
    u8g2.drawFrame(x, 0, etcWidth, 64);
    drawETCTemp(x, 0, etcWidth, etcSegmentHeight, "Kamil ", Data3);
    drawETCTemp(x, etcSegmentHeight - 1, etcWidth, etcSegmentHeight, "Magda", Data4, 1);
    drawETCTemp(x, 2 * etcSegmentHeight - 2, etcWidth, etcSegmentHeight, "CO/m", PrimaryDelta);
    drawETCTemp(x, 3 * etcSegmentHeight - 3, etcWidth, etcSegmentHeight, "CWU/m", SecondaryDelta);
    // drawETCTemp(x, 2 * etcSegmentHeight, etcWidth, etcSegmentHeight, "Kuchnia", 21.2f);

    u8g2.sendBuffer();
}

void feedNow() {
    Serial.println("Feeding now...");
    long stepsToMove = static_cast<long>(((float) STEPS_PER_REV * config.RotationsPerFeeding) * 1000.0f) / 1000;
    stepper.move(stepsToMove);
    digitalWrite(EN_PIN, LOW); // Enable the stepper driver
    currentActivityState     = ACTIVITY_STEPPER;

    // Update grams feeded today
    config.GramsFeededToday += static_cast<int>(config.GramsPerFeeding);
    gramsFedTodaySensor.setValue(static_cast<float>(config.GramsFeededToday));
    config.saveToFS();
}

void setBacklight(uint8_t brightness) {
    analogWrite(LCD_BACKLIGHT, brightness);
}

void setContrast(uint8_t contrast) {
    u8g2.setContrast(contrast);
}

void onLCDStateCommand(bool state, HALight *sender) {
    if(state) {
        // Turn on backlight to previous brightness
        setBacklight(config.LCD_BACKLIGHT_VAL);
    } else {
        // Turn off backlight
        setBacklight(0);
    }
    currentActivityState    = ACTIVITY_HIGH;
    lastButtonInterruptTime = millis();
    sender->setState(state); // Update state
}
void onLCDBrightnessCommand(uint8_t brightness, HALight *sender) {
    config.LCD_BACKLIGHT_VAL = brightness;
    setBacklight(brightness);
    config.saveToFS();
    currentActivityState    = ACTIVITY_HIGH;
    lastButtonInterruptTime = millis();
    sender->setBrightness(brightness); // Update brightness
}

void onContrastCommand(HANumeric value, HANumber *sender) {
    uint8_t contrastValue   = value.toUInt8();
    config.LCD_CONTRAST_VAL = contrastValue;
    setContrast(contrastValue);
    config.saveToFS();
    currentActivityState    = ACTIVITY_HIGH;
    lastButtonInterruptTime = millis();
    sender->setState(value); // Update state
}

void onStepperSpeedCommand(HANumeric value, HANumber *sender) {
    config.StepperSpeed = value.toInt16();
    config.saveToFS();
    stepper.setMaxSpeed(config.StepperSpeed * STEPPER_MICROSTEPS);
    sender->setState(value);
}

void onStepperAccelCommand(HANumeric value, HANumber *sender) {
    config.StepperAccel = value.toInt16();
    config.saveToFS();
    stepper.setAcceleration(config.StepperAccel * STEPPER_MICROSTEPS);
    sender->setState(value);
}

void onRotationsPerFeedingCommand(HANumeric value, HANumber *sender) {
    config.RotationsPerFeeding = value.toFloat();
    config.saveToFS();
    sender->setState(value);
}

void onGramsPerFeedingCommand(HANumeric value, HANumber *sender) {
    config.GramsPerFeeding = value.toFloat();
    config.saveToFS();
    sender->setState(value);
}

void onMaxGramsPerDayCommand(HANumeric value, HANumber *sender) {
    config.MaxGramsPerDay = value.toFloat();
    config.saveToFS();
    sender->setState(value);
}

void onFeedNowCommand(HAButton *sender) {
    feedNow();
}

void onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length) {
    Serial.print("MQTT Message received on topic: ");
    Serial.print(topic);
    Serial.print(" with payload: ");
    Serial.write(payload, length);
    Serial.println();
    // Handle incoming MQTT messages if needed
    long currentTime = millis();
    if(strcmp(topic, DATA_PRIMARY_TOPIC) == 0) {
        lastPrimaryData     = PrimaryData;
        PrimaryData         = std::stof(std::string((const char *) payload, length));

        // Calculate delta
        float timeDeltaSec  = (currentTime - lastPrimaryDataTime) / DELTA_TIME_DIVIDER;
        PrimaryDelta        = (PrimaryData - lastPrimaryData) / timeDeltaSec;

        lastPrimaryDataTime = currentTime;
        Serial.print("Primary Delta: ");
        Serial.println(PrimaryDelta);
    } else if(strcmp(topic, DATA_SECONDARY_TOPIC) == 0) {
        lastSecondaryData     = SecondaryData;
        SecondaryData         = std::stof(std::string((const char *) payload, length));

        // Calculate delta
        float timeDeltaSec    = (currentTime - lastSecondaryDataTime) / DELTA_TIME_DIVIDER;
        SecondaryDelta        = (SecondaryData - lastSecondaryData) / timeDeltaSec;

        lastSecondaryDataTime = currentTime;
        Serial.print("Secondary Delta: ");
        Serial.println(SecondaryDelta);
    } else if(strcmp(topic, DATA3_TOPIC) == 0)
        Data3 = std::stof(std::string((const char *) payload, length));
    else if(strcmp(topic, DATA4_TOPIC) == 0)
        Data4 = std::stof(std::string((const char *) payload, length));
}

// NTP Setup - Syncs time from the internet
void setupNTP() {
    configTime(GMT_OFFSET_SEC, DAYLIGHT_OFFSET_SEC, NTP_SERVER);
    Serial.println("Waiting for NTP time sync...");

    // Wait for time to be set (with timeout)
    int retries = 0;
    while(time(nullptr) < 1000000000 && retries < 20) {
        delay(500);
        Serial.print(".");
        retries++;
    }
    Serial.println();

    struct tm timeinfo;
    if(getLocalTime(&timeinfo)) {
        Serial.println("NTP time synchronized!");
        Serial.printf("Current time: %02d:%02d:%02d, Day: %d\n",
        timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec, timeinfo.tm_mday);
        lastDay = timeinfo.tm_mday; // Initialize lastDay
    } else {
        Serial.println("Failed to get NTP time");
    }
}

// Check if a new day has started and reset daily counters
void checkNewDay() {
    struct tm timeinfo;
    if(!getLocalTime(&timeinfo))
        return; // Time not available yet

    int currentDay = timeinfo.tm_mday;

    // Check if day has changed (and lastDay is valid)
    if(lastDay != -1 && currentDay != lastDay) {
        Serial.println("New day detected! Resetting daily counters...");

        // Reset daily grams counter
        config.GramsFeededToday = 0;
        gramsFedTodaySensor.setValue(0.0f);
        config.saveToFS();

        Serial.printf("Daily reset complete. New day: %d\n", currentDay);
    }

    lastDay = currentDay;
}