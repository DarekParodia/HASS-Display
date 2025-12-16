// LED fade in/out on GPIO 21 for ESP32 using LEDC PWM
#include <Arduino.h>
#include <ArduinoHA.h>
#include <U8g2lib.h>
#include <WiFiManager.h>
#include <LittleFS.h>

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

#define DEVICE_NAME          "HASS-Display"

#define DATA_PRIMARY_TOPIC   "GreenThing/27B529/CO/temperature"
#define DATA_SECONDARY_TOPIC "GreenThing/27B529/CWU/temperature"

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

ActivityState                      currentActivityState = ACTIVITY_HIGH;

U8G2_ST7565_NHD_C12864_F_4W_SW_SPI u8g2(U8G2_R0,
/* clock=*/LCD_CLOCK,
/* data=*/LCD_DATA,
/* cs=*/LCD_CS,
/* dc=*/LCD_RS,
/* reset=*/LCD_RSE);


// functions
void setBacklight(uint8_t brightness);
void setContrast(uint8_t contrast);

void onLCDStateCommand(bool state, HALight *sender);
void onLCDBrightnessCommand(uint8_t brightness, HALight *sender);

void onContrastCommand(HANumeric value, HANumber *sender);
void onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length);
void render();
//
void setup() {
    // Configure LEDC PWM and attach GPIO 21
    Serial.begin(115200);
    pinMode(LCD_BACKLIGHT, OUTPUT);
    pinMode(BUTTON1_PIN, INPUT);

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

    mqtt.begin(config.mqtt_server, config.mqtt_user, config.mqtt_password);
    mqtt.loop();

    // send states
    backlight.setState(config.LCD_BACKLIGHT_VAL > 0);
    backlight.setBrightness(config.LCD_BACKLIGHT_VAL);

    contrast.setState(static_cast<float>(config.LCD_CONTRAST_VAL));

    mqtt.loop();
}

void loop() {
    mqtt.loop();
    render();

    switch(currentActivityState) {
        case ACTIVITY_HIGH:
            /* code */
            break;

        case ACTIVITY_LOW:
            /* code */
            break;

        default:
            break;
    }

    delay(1000);
}

void render() {
    u8g2.clearBuffer();

    u8g2.setFont(FONT_MEDIUM);
    u8g2.drawStr(0, 15, "HASS-Display");

    u8g2.setFont(FONT_SMALL);
    u8g2.drawStr(0, 35, "Backlight:");
    char buf[5];
    snprintf(buf, sizeof(buf), "%d", config.LCD_BACKLIGHT_VAL);
    u8g2.drawStr(80, 35, buf);

    u8g2.drawStr(0, 50, "Contrast:");
    snprintf(buf, sizeof(buf), "%d", config.LCD_CONTRAST_VAL);
    u8g2.drawStr(80, 50, buf);

    // Button
    u8g2.drawStr(0, 63, digitalRead(BUTTON1_PIN) == HIGH ? "Button: OFF" : "Button: ON");

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
    sender->setState(state); // Update state
}
void onLCDBrightnessCommand(uint8_t brightness, HALight *sender) {
    config.LCD_BACKLIGHT_VAL = brightness;
    setBacklight(brightness);
    config.saveToFS();
    sender->setBrightness(brightness); // Update brightness
}

void onContrastCommand(HANumeric value, HANumber *sender) {
    uint8_t contrastValue   = value.toUInt8();
    config.LCD_CONTRAST_VAL = contrastValue;
    setContrast(contrastValue);
    config.saveToFS();
    sender->setState(value); // Update state
}

void onMqttMessage(const char *topic, const uint8_t *payload, uint16_t length) {
    // Handle incoming MQTT messages if needed
}