/*************************************
   Download RAK1901/RAK1902 library from https://downloads.rakwireless.com/RUI/RUI3/Library/
   Besides, install them into Arduino IDE
 *************************************/

#include "rak1901.h"

void uplink_routine();

// Send uplink interval
#define SEND_INTERVAL_OFFSET 0x00000002
#define DEFAULT_SEND_INTERVAL (60)                  // Default value if data in flash is empty or corrupted
#define SEND_INTERVAL (30)                          // Hardcoded value, but overwrited by value in flash
uint32_t g_send_interval = SEND_INTERVAL * 1000;

/** Temperature & Humidity sensor **/
rak1901 th_sensor;

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

int send_interval_handler(SERIAL_PORT port, char *cmd, stParam *param)
{
  // Verify if arg is not null and the argument letter is ?
  if(param->argc == 1 && !strcmp(param->argv[0], "?"))
  {
    Serial.print(cmd);
    Serial.print("=");
    Serial.printf("%ld\r\n", g_send_interval /1000);   
  }
  else if (param->argc == 1)
  {
    // MYLOG("AT_CMD", "param->[0] >> %s", param->argv[0]);
    for (int i = 0; i < strlen(param->argv[0]); i++)
    {
      // Verify if each digit is a number
      if(!isDigit(*(param->argv[0]+ i)))
      {
        // MYLOG("AT_CMD", " %d is no digit", i);
        return AT_PARAM_ERROR;
      }
    }
    // Convert string to int
    uint32_t new_send_interval = strtoul(param->argv[0], NULL, 10);

    // MYLOG(("AT_CMD", "Requested interval %ld", new_send_interval);

    g_send_interval = new_send_interval * 1000;

    // MYLOG("AT_CMD", "New interval %ld", g_send_interval);

    // Stop the timer
    api.system.timer.stop(RAK_TIMER_0);
    // Start new timer if not disabled (0)
    if (g_send_interval != 0)
    {
      // Restart the timer
      api.system.timer.start(RAK_TIMER_0, g_send_interval, NULL);
    }
    // Save custom settings
    save_at_setting(true);
    return AT_OK;
  }
  else 
  {
    return AT_PARAM_ERROR;
  }
  return AT_OK;
}

bool get_at_setting(bool get_timeout)
{
  uint8_t flash_value[16];
  uint32_t sendint_offset = SEND_INTERVAL_OFFSET;

  if (!api.system.flash.get(sendint_offset, flash_value, 5))
  {
    Serial.println("Get values from flash fail!");
    return false;
  }

  // If read invalid data from flash, will set defaults values
  // SEND_INTERVAL_OFFSET 4 bytes

  if (flash_value[4] != 0XAA)
  {
    Serial.printf("AT_CMD", " No valid SENDINT found, read 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X from OFFSET %d\r\n",
                              flash_value[0], flash_value[1], flash_value[2], flash_value[3], flash_value[4], sendint_offset);
    save_at_setting(true);
    return false;
  }
  Serial.printf("Read SENDINT from flash 0x%02X 0x%02X 0x%02X 0x%02X \r\n",
                                  flash_value[0], flash_value[1], flash_value[2], flash_value[3]);
  if (get_timeout)
  {
    g_send_interval = 0;
    g_send_interval |= flash_value[0] << 0;
    g_send_interval |= flash_value[1] << 8;
    g_send_interval |= flash_value[2] << 16;
    g_send_interval |= flash_value[3] << 24;
  }
  Serial.printf("SENDINT: %d\r\n", g_send_interval / 1000);
  return true;
}

bool save_at_setting(bool save_settings)
{
  uint8_t flash_value[16] = {0};
  bool write_result = false;
  uint32_t sendint_offset = SEND_INTERVAL_OFFSET;
  
  if (save_settings)
  {
    flash_value[0] = (uint8_t)(g_send_interval >> 0);
    flash_value[1] = (uint8_t)(g_send_interval >> 8);
    flash_value[2] = (uint8_t)(g_send_interval >> 16);
    flash_value[3] = (uint8_t)(g_send_interval >> 24);
    flash_value[4] = 0xAA;
  }
  Serial.printf("AT_CMD", " Writing time 0x%02X 0x%02X 0x%02X 0x%02X to %d\r\n",
                    flash_value[0], flash_value[0], 
                    flash_value[0], flash_value[0], sendint_offset);
  write_result = api.system.flash.set(sendint_offset, flash_value, 5);

  if (!write_result) 
  {
    // Retry
    write_result = api.system.flash.set(sendint_offset, flash_value, 5);
    Serial.printf("Retrying Writing on flash\r\n");
  }
  Serial.printf("Writing on flash\r\n");
  write_result = true;
  return write_result;
}

void setup() 
{
  // Convert seconds to miliseconds
  Serial.begin(115200, RAK_AT_MODE);
  delay(2000);

  Serial.println("RAKWireless Smart Farm Example");
  Serial.println("------------------------------------------------------");

  // Get saved led status
  get_at_setting(true);
  
  // Set custom ATC Commands
  api.system.atMode.add("SENDINT", "Get/Set interval sending time in seconds", "SENDINT", send_interval_handler, RAK_ATCMD_PERM_WRITE | RAK_ATCMD_PERM_READ);

  if (!api.lorawan.join()) {
    Serial.printf("LoraWan Smart Farm - join fail! \r\n");
  }

  api.lorawan.registerRecvCallback(recvCallback);
  api.lorawan.registerJoinCallback(joinCallback);
  api.lorawan.registerSendCallback(sendCallback);

  // Start I2C Bus
  Wire.begin(); 
  // Check if RAK1901 init success
  Serial.printf("RAK1901 init %s\r\n", th_sensor.init() ? "success" : "fail"); 

  if (!api.system.timer.create(RAK_TIMER_0, (RAK_TIMER_HANDLER) uplink_routine, RAK_TIMER_PERIODIC)) {
    Serial.printf("Lorawan Smart Farm - Creating timer failed! \r\n");
    return;
  }

  if (g_send_interval != 0) 
  {
    if (!api.system.timer.start(RAK_TIMER_0, g_send_interval, NULL)) {
      Serial.printf("Lorawan Smart Farm - Starting timer failed! \r\n");
      return;
    }
  }
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
  *  https://github.com/beegee-tokyo/RUI3-LowPower-Example?tab=readme-ov-file#setup-and-loop
  */
  
  api.system.scheduler.task.destroy();
}
