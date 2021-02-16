
#include <M5StickCPlus.h>
#include "AXP192.h"
#include <ESP_WiFiManager.h> //https://github.com/khoih-prog/ESP_WiFiManager
#include "yunBoard.h"
#include <ESPAsyncWebServer.h>
#include "AsyncJson.h"
#include "ArduinoJson.h"

TFT_eSprite tftSprite = TFT_eSprite(&M5.Lcd);

AsyncWebServer server(80);

void setup()
{
  M5.begin();
  M5.Lcd.setRotation(1);
  tftSprite.createSprite(240, 135);
  tftSprite.setRotation(3);
  M5.Axp.EnableCoulombcounter();
  Wire.begin(0, 26, 100000);

  ESP_WiFiManager ESP_wifiManager("M5StickC_IOT");
  ESP_wifiManager.autoConnect("M5StickC_IOT");
  // Uncomment to reset SSID
  // WiFi.disconnect(true, true);

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    StaticJsonDocument<100> data;
    String response;
    data["AXP Temp"] = M5.Axp.GetTempInAXP192();
    serializeJson(data, response);
    request->send(200, "application/json", response);
  });
  server.begin();
}

void loop()
{
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
  delay(100);
}