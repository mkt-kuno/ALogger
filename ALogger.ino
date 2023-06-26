////////////////////////////////////
//* Project for Arduino Mega2560 *//
////////////////////////////////////
#include <Arduino_FreeRTOS.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include "LiquidCrystal_I2C_RTOS.h"
#include "ADS1115_WE_RTOS.h"

#define FIRMWARE_VERSION_STR "Aynm0.1"
#define CSV_NAMING_RULE "Aynm_%02d.CSV"
#define CSV_HEADER "time[s],hx711a[i24],hx711b[i24],disp[V],hx711a[N],hx711b[N],disp[mm]"

LiquidCrystal_I2C_RTOS lcd(0x27, 16, 2);
ADS1115_WE adc = ADS1115_WE();

//////////////// Calibration Value //////////////////
#define CALIB_HX711A_AX   (0.001)
#define CALIB_HX711A_B    (0.000)
#define CALIB_HX711B_AX   (0.001)
#define CALIB_HX711B_B    (0.000)
#define CALIB_DISP_AX     (20.884)
#define CALIB_DISP_B      (0.000)
/////////////////////////////////////////////////////

inline void hx711_pulse(uint8_t SCK) {
  digitalWrite(SCK, 1);
  delayMicroseconds(1);
  digitalWrite(SCK, 0);
  delayMicroseconds(1);
}
int32_t hx711_init(uint8_t SCK, uint8_t DOUT) {
  pinMode(SCK, OUTPUT);
  pinMode(DOUT, INPUT);
  digitalWrite(SCK, 1);
  delayMicroseconds(60);
  digitalWrite(SCK, 0);
}
int32_t hx711_read(uint8_t SCK, uint8_t DOUT) {
  int32_t ret = 0;
  while (!digitalRead(DOUT) == LOW) {vTaskDelay(portTICK_PERIOD_MS);}
	portENTER_CRITICAL();
  {
    for (char i = 0; i < 24; i++) { hx711_pulse(SCK); ret = (ret << 1) | (digitalRead(DOUT)); }
    hx711_pulse(SCK);//one pulse -> channel A amp x128
  }
  portEXIT_CRITICAL();
  return (int32_t)(ret << 8);
}

static int32_t st_hx711_a = 0;
static int32_t st_hx711_b = 0;

int32_t hx711_routine(int32_t * p_st_hx711, uint8_t SCK, uint8_t DOUT) {
    vTaskDelay(96/portTICK_PERIOD_MS);
    int32_t temp = hx711_read(SCK, DOUT);
    portENTER_CRITICAL();
    {
      *p_st_hx711 = temp;
    }
    portEXIT_CRITICAL();
}

void HX711Task_A(void *pvParameters) {
  const uint8_t SCK = 2;
  const uint8_t DOUT = 3;
  hx711_init(SCK, DOUT);
  for (;;) {hx711_routine(&st_hx711_a, SCK, DOUT);}
}

void HX711Task_B(void *pvParameters) {
  const uint8_t SCK = 5;
  const uint8_t DOUT = 6;
  hx711_init(SCK, DOUT);
  for (;;) {hx711_routine(&st_hx711_b, SCK, DOUT);}
}

void MainTask(void *pvParameters);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  
  Wire.begin();
  Wire.setClock(400E3);

  xTaskCreate(MainTask, "Main", configMINIMAL_STACK_SIZE+1024 , NULL, 1, NULL);
  xTaskCreate(HX711Task_A, "hx711a", configMINIMAL_STACK_SIZE+128, NULL, 2, NULL);
  xTaskCreate(HX711Task_B, "hx711b", configMINIMAL_STACK_SIZE+128, NULL, 2, NULL);
}

void loop()
{
  // Empty. Things are done in Tasks.
}

void sd_make_filename(char* fn_str, int fn_str_len) {
  int8_t ret = -1;
  char str_buf[16];
  for (int8_t index = 0; index < 100; index++) {
    sprintf(str_buf, CSV_NAMING_RULE, index);
    if (SD.exists(str_buf)) ret = index;
  }
  memset(fn_str, 0x00, fn_str_len);
  sprintf(fn_str, CSV_NAMING_RULE, ret+1);
}

void MainTask(void *pvParameters)  // This is a task.
{
  (void) pvParameters;
  const uint8_t BTN_INPUT = 8;
  const uint8_t BTN_VGND = 9;
  uint8_t btn_store = 0x00;
  uint8_t window_num = 0;

  int32_t local_hx711_a = 0;
  int32_t local_hx711_b = 0;
  float local_disp = 0;
  float phy_hx711_a = 0;
  float phy_hx711_b = 0;
  float phy_displace = 0;

  static File csvFile;

  bool is_recording = false;

  // Naming rule is CSV_NAMING_RULE
  const int sd_fn_len = 16;
  char sd_fn[sd_fn_len+1];
  memset(sd_fn, 0x00, sd_fn_len+1);

  static char dtostrf_buf[17];
  const int sd_buf_size = 256;
  static char sd_buf[sd_buf_size+1];

	lcd.begin();
	lcd.backlight();

  if(!adc.init()){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("ADmodule ADS1115");
    lcd.setCursor(0, 1);
    lcd.print("not connected!!!");
    for(;;) {vTaskDelay(1000/portTICK_PERIOD_MS);};
  }
  adc.setVoltageRange_mV(ADS1115_RANGE_6144);
  adc.setCompareChannels(ADS1115_COMP_0_GND);
  adc.setMeasureMode(ADS1115_CONTINUOUS);
  adc.setConvRate(ADS1115_64_SPS);

  if(!SD.begin(4)){
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("SD CARD ERROR!!!");
    lcd.setCursor(0, 1);
    lcd.print("CheckSD & REBOOT");
    for(;;) {vTaskDelay(1000/portTICK_PERIOD_MS);};
  }

  pinMode(BTN_VGND, OUTPUT);
  digitalWrite(BTN_VGND, 0);
  pinMode(BTN_INPUT, INPUT_PULLUP);

  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    // HX711データ取得
    portENTER_CRITICAL();
    {
      // 裏で動いてるのでRTOSブロックしてコピー
      local_hx711_a = st_hx711_a;
      local_hx711_b = st_hx711_b;
    }
    portEXIT_CRITICAL();

    // 変位計ADCデータ取得
    adc.setCompareChannels(ADS1115_COMP_0_GND);
    local_disp = adc.getResult_V();

    // キャリブレーション値を元に計算
    phy_hx711_a = (float)(CALIB_HX711A_AX * (double)local_hx711_a/256.0 + CALIB_HX711A_B);
    phy_hx711_b = (float)(CALIB_HX711B_AX * (double)local_hx711_b/256.0 + CALIB_HX711B_B);
    phy_displace = (float)(CALIB_DISP_AX * (double)local_disp + CALIB_DISP_B);

    // ボタン短押し・長押し処理
    btn_store = btn_store << 1 | (!digitalRead(BTN_INPUT) & 0x01);
    if ((btn_store & 0b00000111) == 0b00000010 || (btn_store & 0b00001111) == 0b00000110) {
      window_num++;
      if (window_num > 3) {window_num=0;}
    }
    if ((btn_store & 0b00111111) == 0b00111110) {
      //Start or Stop Recording
      if (!is_recording)  {
        sd_make_filename(sd_fn, sd_fn_len);
        //ヘッダーを書き込む
        csvFile = SD.open(sd_fn, FILE_WRITE);
        if (csvFile){
          csvFile.println(CSV_HEADER); // write contents
          csvFile.close();
        }
        is_recording = true;
      }
      else {
        is_recording = false;
        memset(sd_fn, 0x00, sd_fn_len+1);
      }
    }
    
    // 液晶表示処理
    switch(window_num) {
      default:
      case 0:
      lcd.setCursor(0, 0);
      if (!is_recording) {
        lcd.print("NOREC ");
        lcd.print(FIRMWARE_VERSION_STR);
      }
      else {
        lcd.print("REC ");
        static uint8_t progress = 0;
        switch(progress){
          default: lcd.print("*"); break;
          case 0: lcd.print("|"); break;
          case 1: lcd.print("/"); break;
          case 2: lcd.print("-"); break;
          case 3: lcd.print("\\"); break;
        }
        progress++;
        if (progress > 3) progress = 0;
        lcd.print(" ");
        lcd.print(sd_fn);
      }
      lcd.setCursor(0, 1);
      lcd.print("Time:");
      lcd.print(dtostrf((float)xTaskGetTickCount()/configTICK_RATE_HZ, 8, 2, dtostrf_buf));
      lcd.print("[s]");
      break;
      case 1:
      lcd.setCursor(0, 0);
      lcd.print("HxA:");
      lcd.print(dtostrf(phy_hx711_a, 9, 1, dtostrf_buf));
      lcd.print("[N]");
      lcd.setCursor(0, 1);
      lcd.print("HxB:");
      lcd.print(dtostrf(phy_hx711_b, 9, 1, dtostrf_buf));
      lcd.print("[N]");
      break;
      case 2:
      lcd.setCursor(0, 0);
      lcd.print("Disp:");
      lcd.print(dtostrf(phy_displace, 7, 3, dtostrf_buf));
      lcd.print("[mm]");
      lcd.setCursor(0, 1);
      lcd.print("Battery:");
      lcd.print(dtostrf(analogRead(A0)*5.0/1023.0, 4, 3, dtostrf_buf));
      lcd.print("[V]");
      break;
      case 3:
      lcd.setCursor(0, 0);
      lcd.print("RawVal:");
      lcd.print(dtostrf(local_disp, 6, 3, dtostrf_buf));
      lcd.print("[V]");
      lcd.setCursor(0, 1);
      sprintf(dtostrf_buf, "%08lX%08lX", local_hx711_a>>8, local_hx711_b>>8);
      dtostrf_buf[0] = ' '; dtostrf_buf[1] = 'x';
      dtostrf_buf[8] = ' '; dtostrf_buf[9] = 'x';
      lcd.print(dtostrf_buf);
      break;
    }

    // SDカード保存処理
    if (is_recording) {
      csvFile = SD.open(sd_fn, FILE_WRITE);
      if (csvFile){
        int p = 0;
        memset(sd_buf, 0x00, sd_buf_size+1);
        dtostrf((float)xTaskGetTickCount()/configTICK_RATE_HZ, -1, 3, dtostrf_buf);
        p += sprintf(&sd_buf[p], "%s", dtostrf_buf);
        sd_buf[p++] = ',';
        p += sprintf(&sd_buf[p], "0x%08lX", local_hx711_a) - 2;
        sd_buf[p++] = ',';
        p += sprintf(&sd_buf[p], "0x%08lX", local_hx711_b) - 2;
        sd_buf[p++] = ',';
        dtostrf(phy_displace, -1, 3, dtostrf_buf);
        p += sprintf(&sd_buf[p], "%s", dtostrf_buf);
        sd_buf[p++] = ',';
        dtostrf(phy_hx711_a, -1, 3, dtostrf_buf);
        p += sprintf(&sd_buf[p], "%s", dtostrf_buf);
        sd_buf[p++] = ',';
        dtostrf(phy_hx711_b, -1, 3, dtostrf_buf);
        p += sprintf(&sd_buf[p], "%s", dtostrf_buf);
        sd_buf[p++] = ',';
        dtostrf(phy_displace, -1, 3, dtostrf_buf);
        p += sprintf(&sd_buf[p], "%s", dtostrf_buf);
        csvFile.println(sd_buf);
        csvFile.close();
      }
    }
    digitalWrite(LED_BUILTIN, 1);
    vTaskDelay(96/portTICK_PERIOD_MS);
    digitalWrite(LED_BUILTIN, 0);
  }
}
