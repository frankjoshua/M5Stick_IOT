
#include <M5StickCPlus.h>
#include "AXP192.h"
#include <ESP_WiFiManager.h> //https://github.com/khoih-prog/ESP_WiFiManager
#include "yunBoard.h"
#include <ESPAsyncWebServer.h>
#include <AsyncElegantOTA.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"
#include <Wire.h>
#include "SHT20.h"
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

TFT_eSprite tftSprite = TFT_eSprite(&M5.Lcd);

AsyncWebServer server(80);

SHT20 sht20;

char response[1024];

void setup()
{
  M5.begin();
  Wire.begin(0, 26, 100000);
  M5.Lcd.setRotation(1);
  tftSprite.createSprite(240, 135);
  tftSprite.setRotation(3);
  M5.Axp.EnableCoulombcounter();

  ESP_WiFiManager ESP_wifiManager("M5StickC_IOT");
  ESP_wifiManager.autoConnect("M5StickC_IOT");
  // Uncomment to reset SSID
  // WiFi.disconnect(true, true);

  bool ready = bmp.begin(0x76);
  if (!ready)
  {
    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "application/json", response);
    });
  }
  else
  {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,      /* Operating Mode. */
                    Adafruit_BMP280::SAMPLING_X2,      /* Temp. oversampling */
                    Adafruit_BMP280::SAMPLING_X16,     /* Pressure oversampling */
                    Adafruit_BMP280::FILTER_X16,       /* Filtering. */
                    Adafruit_BMP280::STANDBY_MS_1000); /* Standby time. */

    server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
      request->send(200, "application/json", response);
    });
  }

  AsyncElegantOTA.begin(&server);
  server.begin();

  while (!ready)
  {
    AsyncElegantOTA.loop();
    // 0x01 long press(1s), 0x02 press
    if (M5.Axp.GetBtnPress() == 0x02)
    {
      esp_restart();
    }
  }
}

unsigned long update_time = 0;

void loop()
{
  AsyncElegantOTA.loop();
  if (millis() > update_time)
  {
    update_time = millis() + 1000;
    int8_t color_light = 1;
    led_set_all((color_light << 16) | (color_light << 8) | color_light);

    tftSprite.fillSprite(RED);
    tftSprite.setCursor(0, 0, 1);
    tftSprite.printf("AXP Temp: %.1fC \r\n", M5.Axp.GetTempInAXP192());
    tftSprite.setCursor(0, 10);
    tftSprite.printf("Bat:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetBatVoltage(), M5.Axp.GetBatCurrent());
    tftSprite.setCursor(0, 30);
    tftSprite.printf("USB:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetVBusVoltage(), M5.Axp.GetVBusCurrent());
    tftSprite.setCursor(0, 50);
    tftSprite.printf("5V-In:\r\n  V: %.3fv  I: %.3fma\r\n", M5.Axp.GetVinVoltage(), M5.Axp.GetVinCurrent());
    tftSprite.setCursor(10, 70);
    tftSprite.printf("Bat power %.3fmw\r\n", M5.Axp.GetBatPower());
    tftSprite.setCursor(0, 90);
    tftSprite.printf(WiFi.localIP().toString().c_str());
    tftSprite.pushSprite(0, 0);

    StaticJsonDocument<1024> data;
    sensors_event_t temp_event, pressure_event;
    data["ip"] = WiFi.localIP().toString();
    data["axp_temp"] = M5.Axp.GetTempInAXP192();
    data["battery_voltage"] = M5.Axp.GetBatVoltage();
    data["battery_current"] = M5.Axp.GetBatCurrent();
    data["usb_voltage"] = M5.Axp.GetVBusVoltage();
    data["usb_current"] = M5.Axp.GetVBusCurrent();
    data["5v_in_voltage"] = M5.Axp.GetVinVoltage();
    data["5v_in_current"] = M5.Axp.GetVinCurrent();
    data["battery_watts"] = M5.Axp.GetBatPower();
    data["temperature_sht20"] = sht20.read_temperature();
    data["humidity"] = sht20.read_humidity();
    bmp_temp->getEvent(&temp_event);
    bmp_pressure->getEvent(&pressure_event);
    data["temperature_bmp280"] = temp_event.temperature;
    data["pressure"] = pressure_event.pressure;
    data["light"] = light_get();
    data["adc_sensor"] = analogRead(G32);
    serializeJson(data, response);
  }

  // 0x01 long press(1s), 0x02 press
  if (M5.Axp.GetBtnPress() == 0x02)
  {
    esp_restart();
  }

  if (M5.BtnA.wasPressed())
  {
    // close tft voltage output
    M5.Axp.SetLDO2(false);
  }

  if (M5.BtnB.wasPressed())
  {
    // close tft voltage output
    M5.Axp.SetLDO2(true);
  }

  // M5.Axp.SetChargeCurrent(CURRENT_100MA);

  M5.update();
  delay(10);
}