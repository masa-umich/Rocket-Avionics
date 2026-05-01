#include <RadioLib.h>
#include "LoRaBoards.h"
#include "../UART-data/uart-data-struct.h"
#include "HardwareSerial.h"

SX1280 radio = new Module(RADIO_CS_PIN, RADIO_DIO1_PIN, RADIO_RST_PIN, RADIO_BUSY_PIN);
HardwareSerial Nucleo_GS(2);

#define RADIO_SF                12
#define RADIO_BW                406.25
#define RADIO_SYNCWORD          0x12

#define RADIO_PACKET_LENGTH     48

#define DISP_INIT_LINE_SPACE    11

static volatile bool receivedFlag = false;
void setFlag(void) {
    receivedFlag = true;
}

static uint32_t good_packets = 0;
static uint32_t bad_packets = 0;
static float last_rssi = 0;
static float last_snr = 0;

static unsigned long last_packet = 0;

void setup() {
  u8g2_uint_t cur_row = 7;
  setupBoards();
  delay(1000);
  if(disp) {
    disp->clearDisplay();
    disp->setFont(u8g2_font_5x7_mr);
    disp->setCursor(5, cur_row);
    cur_row += DISP_INIT_LINE_SPACE;
    disp->print("Init. Limelight GS...");
    disp->setCursor(5, cur_row);
    cur_row += DISP_INIT_LINE_SPACE;
    disp->print("Connecting to STM32...");
    disp->sendBuffer();
  }

  Nucleo_GS.begin(115200, SERIAL_8N1, 42, 46);

  if(disp) {
    disp->setCursor(5, cur_row);
    cur_row += DISP_INIT_LINE_SPACE;
    disp->print("Initializing SX1280...");
    disp->sendBuffer();
  }
/*#ifdef  RADIO_TCXO_ENABLE
    pinMode(RADIO_TCXO_ENABLE, OUTPUT);
    digitalWrite(RADIO_TCXO_ENABLE, HIGH);
#endif*/

  Serial.print(F("[SX1280] Initializing ... "));
  int state = radio.begin();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  }
  else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Init Failed");
      disp->sendBuffer();
    }
    while (true) { delay(10); }
  }
  
  if(disp) {
    disp->setCursor(5, cur_row);
    cur_row += DISP_INIT_LINE_SPACE;
    disp->print("Config Link Params...");
    disp->sendBuffer();
  }
  
  radio.setPacketReceivedAction(setFlag);

  if (radio.setFrequency(2400.0) == RADIOLIB_ERR_INVALID_FREQUENCY) {
    Serial.println(F("Selected frequency is invalid for this module!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

  if (radio.setBandwidth(RADIO_BW) == RADIOLIB_ERR_INVALID_BANDWIDTH) {
    Serial.println(F("Selected bandwidth is invalid for this module!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

  if (radio.setSpreadingFactor(RADIO_SF) == RADIOLIB_ERR_INVALID_SPREADING_FACTOR) {
    Serial.println(F("Selected spreading factor is invalid for this module!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

  if (radio.setCodingRate(8) == RADIOLIB_ERR_INVALID_CODING_RATE) {
    Serial.println(F("Selected coding rate is invalid for this module!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

  if (radio.setSyncWord(RADIO_SYNCWORD) != RADIOLIB_ERR_NONE) {
    Serial.println(F("Unable to set sync word!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

  if (radio.setPreambleLength(12) == RADIOLIB_ERR_INVALID_PREAMBLE_LENGTH) {
    Serial.println(F("Selected preamble length is invalid for this module!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

  if (radio.setCRC(2) == RADIOLIB_ERR_INVALID_CRC_CONFIGURATION) {
    Serial.println(F("Selected CRC is invalid for this module!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

  if (radio.implicitHeader(RADIO_PACKET_LENGTH) == RADIOLIB_ERR_UNSUPPORTED) {
    Serial.println(F("Selected packet length is invalid for this module!"));
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Config Failed");
      disp->sendBuffer();
    }
    while (true);
  }

/*#ifdef RADIO_RX_PIN
  // SX1280 PA Version
  radio.setRfSwitchPins(RADIO_RX_PIN, RADIO_TX_PIN);
#endif*/

  if(disp) {
    disp->setCursor(5, cur_row);
    cur_row += DISP_INIT_LINE_SPACE;
    disp->print("Starting SX1280 Recv...");
    disp->sendBuffer();
  }

  delay(500);
  Serial.print(F("Radio Starting to listen ... "));
  state = radio.startReceive();
  if (state == RADIOLIB_ERR_NONE) {
    Serial.println(F("success!"));
  } else {
    Serial.print(F("failed, code "));
    Serial.println(state);
    if(disp) {
      disp->setCursor(5, cur_row);
      cur_row += DISP_INIT_LINE_SPACE;
      disp->print("SX1280 Recv Mode failed");
      disp->sendBuffer();
    }
  }
  Serial.print(F("[SX1280] Waiting for incoming transmission ... "));
  if(disp) {
    disp->setCursor(5, cur_row);
    cur_row += DISP_INIT_LINE_SPACE;
    disp->print("LilyGo GS Initialized!");
    disp->sendBuffer();
  }
  delay(1000);
  last_packet = millis();
}

void loop() {
  if(receivedFlag) {
    receivedFlag = false;

    uint8_t payload[RADIO_PACKET_LENGTH];
    int state = radio.readData(payload, RADIO_PACKET_LENGTH);
  
    if (state == RADIOLIB_ERR_NONE) {
      good_packets++;
      Serial.println(F("success!"));
      Serial.print(F("[SX1280] RSSI:\t\t"));
      Serial.print(radio.getRSSI());
      Serial.println(F(" dBm"));
      Serial.print(F("[SX1280] SNR:\t\t"));
      Serial.print(radio.getSNR());
      Serial.println(F(" dB"));
      Serial.print(F("[SX1280] Frequency Error:\t"));
      Serial.print(radio.getFrequencyError());
      Serial.println(F(" Hz"));

      UART_Data_t packet_data = {0};
      parse_radio(&packet_data, payload);
      packet_data.rssi = radio.getRSSI();
      packet_data.snr = radio.getSNR();

      last_rssi = packet_data.rssi;
      last_snr = packet_data.snr;

      uint8_t uart_buf[UART_PACKET_LENGTH];
      if(serialize_radio(&packet_data, uart_buf, UART_PACKET_LENGTH) > 0) {
        Nucleo_GS.write(uart_buf, UART_PACKET_LENGTH);
        Nucleo_GS.flush();
      }

      last_packet = millis();
    } 
    else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
      Serial.println(F("timeout!"));
    } 
    else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
      bad_packets++;
      Serial.println(F("CRC error!"));
      last_packet = millis();
    } 
    else {
      Serial.print(F("failed, code "));
      Serial.println(state);
    }
    radio.startReceive();
    Serial.print(F("[SX1280] Waiting for incoming transmission ... "));
  }
  
  drawRadio();
  delay(50);

  /*UART_Data_t test = {0};
  test.ms_epoch = 12700000ULL;
  test.gps_long = 45.1234;
  test.fc_imu_x = 4512;
  test.fc_fsm_state = 5;
  test.fuel_pres = 345;
  test.rssi = 10.1;
  test.snr = -7.05;
  uint8_t uart_buf[UART_PACKET_LENGTH];
  if(serialize_radio(&test, uart_buf, UART_PACKET_LENGTH) > 0) {
    Nucleo_GS.write(uart_buf, UART_PACKET_LENGTH);
    Nucleo_GS.flush();
  }
  delay(1000);*/
}

void parse_radio(UART_Data_t * uart_dst, uint8_t * buf) {
  memcpy(&(uart_dst->gps_lat), buf + 0, 4);
	memcpy(&(uart_dst->gps_long), buf + 4, 4);
	memcpy(&(uart_dst->gps_alt), buf + 8, 2);
	memcpy(&(uart_dst->fc_imu_x), buf + 10, 2);
	memcpy(&(uart_dst->fc_imu_y), buf + 12, 2);
	memcpy(&(uart_dst->fc_imu_z), buf + 14, 2);
	memcpy(&(uart_dst->fc_w_x), buf + 16, 2);
	memcpy(&(uart_dst->fc_w_y), buf + 18, 2);
	memcpy(&(uart_dst->fc_w_z), buf + 20, 2);
	memcpy(&(uart_dst->fc_bar1), buf + 22, 2);
	memcpy(&(uart_dst->fc_bar2), buf + 24, 2);
	memcpy(&(uart_dst->fc_24v), buf + 26, 2);
	memcpy(&(uart_dst->fc_temp), buf + 28, 2);
	memcpy(&(uart_dst->fc_fsm_state), buf + 30, 1);
	memcpy(&(uart_dst->fc_flutus), buf + 31, 1);
	memcpy(&(uart_dst->chamber_pres), buf + 32, 2);
	memcpy(&(uart_dst->fuel_pres), buf + 34, 2);
	memcpy(&(uart_dst->ox_pres), buf + 36, 2);
	memcpy(&(uart_dst->copv_pres), buf + 38, 2);

  memcpy(&(uart_dst->ms_epoch), buf + 40, 8);
}

void drawRadio() {
  if (disp) {
    disp->clearBuffer();
    disp->drawRFrame(0, 0, 128, 64, 5);
    disp->setFont(u8g2_font_pxplusibmvga8_mr);
    disp->setCursor(6, 25);
    disp->print("RSSI:");
    disp->setCursor(6, 40);
    disp->print("SNR:");
    disp->setCursor(6, 55);
    disp->print("Recv:");

    disp->setFont(u8g2_font_crox1h_tr);
    String rssi_str = isnan(last_rssi) ? " - " : String(last_rssi) + "dBm";
    disp->setCursor(U8G2_HOR_ALIGN_RIGHT(rssi_str.c_str()) - 23, 25);
    disp->print(rssi_str);
    String snr_str = isnan(last_snr) ? " - " : String(last_snr) + "dB";
    disp->setCursor(U8G2_HOR_ALIGN_RIGHT(snr_str.c_str()) - 23, 40);
    disp->print(snr_str);
    String packet_str = String(good_packets) + "/" + String(good_packets + bad_packets);
    disp->setCursor(U8G2_HOR_ALIGN_RIGHT(packet_str.c_str()) - 23, 55);
    disp->print(packet_str);

    String last_str = String((millis() - last_packet) / 1000);
    disp->setCursor(U8G2_HOR_ALIGN_RIGHT(last_str.c_str()) - 3, 13);
    disp->print(last_str);

    disp->sendBuffer();
  }
}