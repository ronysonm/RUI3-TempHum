/*************************************
   Download RAK1901/RAK1902 library from https://downloads.rakwireless.com/RUI/RUI3/Library/
   Besides, install them into Arduino IDE
 *************************************/

#include "rak1901.h"
//#include "rak1902.h" 

void uplink_routine();

#define SMART_FARM_PERIOD (30000)

// Define channel mask
uint16_t maskBuff = 0x0002;

// Led status
#define LED_OFFSET 0x00000002 //lenght 4 bytes
uint8_t led_status;

/** Temperature & Humidity sensor **/
rak1901 th_sensor;

/** Air Pressure sensor **/
//rak1902 p_sensor;

/** Packet buffer for sending */
uint8_t collected_data[64] = { 0 };

void recvCallback(SERVICE_LORA_RECEIVE_T * data)
{
  if (data->BufferSize > 0) {
    Serial.println("Something received!");
    for (int i = 0; i < data->BufferSize; i++) {
      Serial.printf("%x", data->Buffer[i]);
    }
    Serial.print("\r\n");
  }
}

void joinCallback(int32_t status)
{
  Serial.printf("Join status: %d\r\n", status);
}

void sendCallback(int32_t status)
{
  if (status == 0) {
    Serial.println("Successfully sent");
  } else {
    Serial.println("Sending failed");
  }
}

int led_handle(SERIAL_PORT port, char*cmd, stParam *param)
{
  if (param->argc == 1 && !strcmp(param->argv[0], "?")) {
    Serial.print(cmd);
    Serial.print("=");
    Serial.println(led_status?"HIGH":"LOW");
  } else if (param->argc == 1) {
    for (int i = 0; i < strlen(param->argv[0]); i++) {
      if (!isDigit(*(param->argv[0] +i))) {
        return AT_PARAM_ERROR;
      }
    }

    led_status = strtoul(param->argv[0], NULL, 10);
    if (led_status != 0 && led_status != 1) {
      return AT_PARAM_ERROR;
    }

    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(GREEN_LED, (led_status == 1) ? HIGH : LOW);
    digitalWrite(BLUE_LED, (led_status == 1) ? HIGH : LOW);
    save_at_setting(true);

  } else {
    return AT_PARAM_ERROR;
  }

  return AT_OK;
}

bool get_at_setting(bool get_timeout)
{
  uint8_t flash_value[16];
  uint32_t offset = LED_OFFSET;

  if (!api.system.flash.get(offset, flash_value, 5))
  {
    Serial.println("Get values from flash fail!");
    return false;
  }

  // If read invalid data from flash, will set defaults values 
  if (flash_value[4] != 0xAA)
  {
    Serial.printf("AT_CMD", "No valid LED Status found, read 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X \r\n",
                              flash_value[0], flash_value[1], flash_value[2], flash_value[3], flash_value[4]);
    if (led_status)
    {
      led_status = 0;
    }
    save_at_setting(true);
    return false;
  }
  Serial.printf("Read Led status from flash 0x%02X 0x%02X 0x%02X 0x%02X 0X%02X \r\n",
                                  flash_value[0], flash_value[1], 
                                  flash_value[2], flash_value[3], flash_value[4]);
  if (get_timeout)
  {
    led_status = 0;
    led_status |= flash_value[0] << 0;
    led_status |= flash_value[1] << 8;
    led_status |= flash_value[2] << 16;
    led_status |= flash_value[3] << 24;
    pinMode(GREEN_LED, OUTPUT);
    pinMode(BLUE_LED, OUTPUT);
    digitalWrite(GREEN_LED, (led_status == 1) ? HIGH : LOW);
    digitalWrite(BLUE_LED, (led_status == 1) ? HIGH : LOW);
    Serial.printf("AT_CMD", " Led status found %d\r\n", led_status);
  }
  Serial.printf("LED Status: %d\r\n", led_status);
  return true;
}

bool save_at_setting(bool set_led_status)
{
  uint8_t flash_value[16] = {0};
  bool wr_result = false;
  uint32_t offset = LED_OFFSET;
  if (set_led_status)
  {
    offset = LED_OFFSET;
    flash_value[0] = (uint8_t)(led_status >> 0);
    flash_value[1] = (uint8_t)(led_status >> 8);
    flash_value[2] = (uint8_t)(led_status >> 16);
    flash_value[3] = (uint8_t)(led_status >> 24);
    flash_value[4] = 0xAA;
  }
  Serial.printf("AT_CMD", " Writing time 0X%02 0X%02 0X%02 0X%02 to %d",
                    flash_value[0], flash_value[0], 
                    flash_value[0], flash_value[0], offset);
  wr_result = api.system.flash.set(offset, flash_value, 5);
  if (!wr_result) 
  {
    // Retry
    wr_result = api.system.flash.set(offset, flash_value, 5);
  }
  wr_result = true;
  return wr_result;
}

void setup() 
{
  Serial.begin(115200, RAK_AT_MODE);
  delay(2000);

  Serial.println("RAKWireless Smart Farm Example");
  Serial.println("------------------------------------------------------");

  // Get saved led status
  get_at_setting(true);

  if (!api.lorawan.join()) {
    Serial.printf("LoraWan Smart Farm - join fail! \r\n");
  }

  Serial.println("++++++++++++++++++++++++++");
  Serial.println("RUI3 Environment Sensing");
  Serial.println("++++++++++++++++++++++++++");
  
  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);

  Wire.begin(); // Start I2C Bus
  Serial.printf("RAK1901 init %s\r\n", th_sensor.init() ? "success" : "fail"); // Check if RAK1901 init success

  if (api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER)uplink_routine, RAK_TIMER_PERIODIC) != true) {
    Serial.printf("Lorawan Smart Farm - Creating timer failed! \r\n");
    return;
  }

  if (api.system.timer.start(RAK_TIMER_0, SMART_FARM_PERIOD, NULL) != true) {
    Serial.printf("Lorawan Smart Farm - Starting timer failed! \r\n");
    return;
  }

  api.system.atMode.add("LED", "This controls both green and blue LEDS.", "LED", led_handle, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);

}

void uplink_routine()
{
  th_sensor.update();

  float temp_f = th_sensor.temperature();
  float humid_f = th_sensor.humidity();
  float batt = api.system.bat.get();

  uint16_t t = (uint16_t) (temp_f * 10);
  uint16_t h = (uint16_t) (humid_f * 10);
  uint16_t bat = (uint16_t) (batt * 100);

  Serial.printf("Values from sensor T %f H %f B %f \r\n", temp_f, humid_f, batt);
 
  Serial.printf("Values to transmition T %d H %d Bat %d \r\n", t, h, bat);

  /** Cayenne Low Power Payload **/
  uint8_t data_len = 0;
  collected_data[data_len++] = 0x01; // Data Channel: 1
  collected_data[data_len++] = 0x67; // Type: Temperature Sensor
  collected_data[data_len++] = (uint8_t) (t >> 8);
  collected_data[data_len++] = (uint8_t) t;
  collected_data[data_len++] = 0x02; // Data Channel: 2
  collected_data[data_len++] = 0x68; // Type: Humidity
  collected_data[data_len++] = (uint8_t) h;
  collected_data[data_len++] = 0x04; // Data Channel: 4
  collected_data[data_len++] = 0x02; // Type: Analog Input
  collected_data[data_len++] = (uint8_t) (bat >> 8);
  collected_data[data_len++] = (uint8_t) bat;

  Serial.println("Data Packet:");
  for (int i = 0; i < data_len; i++) {
    Serial.printf("0x%02X ", collected_data[i]);
  }
  Serial.println("");

  /** Send the data package **/
  if (api.lorawan.send(data_len, (uint8_t *) & collected_data, 2, true, 1)) {
    Serial.println("Sending is requested");
  } else {
    Serial.println("Sending failed");
  }
}

void loop() {
  /* 
  *  Destroy this busy loop and use timer to do what you want instead,
  *  so that the system thread can auto enter low power mode by api.system.lpm.set(1); 
  */
  
  api.system.scheduler.task.destroy();
}
