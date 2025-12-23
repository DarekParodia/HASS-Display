// LED fade in/out on GPIO 21 for ESP32 using LEDC PWM
#include <Arduino.h>
#include <ArduinoHA.h>
#include <U8g2lib.h>
#include <WiFiManager.h>
#include <LittleFS.h>
#include <sstream>
#include <iomanip>
#include <string>

#define LCD_CLOCK            1
#define LCD_DATA             0
#define LCD_CS               4
#define LCD_RS               2
#define LCD_RSE              3
#define LCD_BACKLIGHT        21

#define BUTTON1_PIN          20

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


enum ActivityState {
    ACTIVITY_LOW,
    ACTIVITY_HIGH
};

struct settings {
        char    mqtt_server[64]   = "";
        int     mqtt_port         = 1883;
        char    mqtt_user[64]     = "";
        char    mqtt_password[64] = "";

        uint8_t LCD_CONTRAST_VAL  = 128;
        uint8_t LCD_BACKLIGHT_VAL = 128;

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

settings                           config = {};

HALight                            backlight("light", HALight::BrightnessFeature);
HANumber                           contrast("number");

ActivityState                      currentActivityState    = ACTIVITY_HIGH;

float                              PrimaryData             = 0.0f;
float                              SecondaryData           = 0.0f;
float                              PrimaryDelta            = 10.0f;
float                              SecondaryDelta          = -10.0f;
float                              PrimaryDeltaThreshold   = 0.1f;
float                              SecondaryDeltaThreshold = 0.1f;

float                              Data3                   = 0.0f;
float                              Data4                   = 0.0f;

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
void           onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length);
void           render();
void           drawTextWithSpacing(int x, int y, const char *text, int spacing);
void IRAM_ATTR buttonISR();
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

void setup() {
    // Configure LEDC PWM and attach GPIO 21
    Serial.begin(115200);
    pinMode(LCD_BACKLIGHT, OUTPUT);
    pinMode(BUTTON1_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(BUTTON1_PIN), buttonISR, RISING);

    // Try mounting
    if(!LittleFS.begin()) {
        Serial.println("LittleFS mount failed, formatting...");
        LittleFS.format(); // Create a new filesystem
        LittleFS.begin();  // Mount again
    } else {
        Serial.println("LittleFS mounted successfully.");
    }
    config.loadFromFS();

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

    WiFi.setSleep(true); // Enable WiFi sleep to save power

    mqtt.loop();

    Serial.println("Button interrupt initialized on GPIO 20");
}

void loop() {
    mqtt.loop();
    delay(1000);
    render();

    long currentTime = millis();

    switch(currentActivityState) {
        case ACTIVITY_HIGH:
            if((currentTime - lastButtonInterruptTime) > BACKLIGHT_TIME) {
                currentActivityState = ACTIVITY_LOW;
                // Turn off backlight
                setBacklight(0);
                backlight.setState(false);
            } else {
                backlight.setState(true);
            }
            break;

        case ACTIVITY_LOW:
            /* code */
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
    drawETCTemp(x, etcSegmentHeight, etcWidth, etcSegmentHeight, "Magda", Data4, 1);
    // drawETCTemp(x, 2 * etcSegmentHeight, etcWidth, etcSegmentHeight, "Kuchnia", 21.2f);

    u8g2.sendBuffer();
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

void onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length) {
    Serial.print("MQTT Message received on topic: ");
    Serial.print(topic);
    Serial.print(" with payload: ");
    Serial.write(payload, length);
    Serial.println();
    // Handle incoming MQTT messages if needed
    if(strcmp(topic, DATA_PRIMARY_TOPIC) == 0)
        PrimaryData = std::stof(std::string((const char *) payload, length));
    else if(strcmp(topic, DATA_SECONDARY_TOPIC) == 0)
        SecondaryData = std::stof(std::string((const char *) payload, length));
    else if(strcmp(topic, DATA3_TOPIC) == 0)
        Data3 = std::stof(std::string((const char *) payload, length));
    else if(strcmp(topic, DATA4_TOPIC) == 0)
        Data4 = std::stof(std::string((const char *) payload, length));
}