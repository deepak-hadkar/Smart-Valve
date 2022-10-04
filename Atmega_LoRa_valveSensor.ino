// Written by Deepak Hadkar, last modified 03 Oct. 2022, 12:30 am

#include <RadioLib.h>
#include <avr/wdt.h>
#include <avr/sleep.h>

//#define NODENAME "LORA_POWER_1"
String node_id = String("ID:") + "DTH001";

//Set sleep time, when value is 1 almost sleep 8s,when value is 450, almost 1 hour.
#define SLEEP_CYCLE 1 // 225 // 450

//Lora set
//Set Lora frequency
#define FREQUENCY 434.0
// #define FREQUENCY 868.0
// #define FREQUENCY 915.0

// Unique antenna spec.(Must be same for trans-receive)
#define BANDWIDTH 125.0
#define SPREADING_FACTOR 9
#define CODING_RATE 7
#define SX127X_SYNC_WORD 0x12
#define OUTPUT_POWER 20
#define PREAMBLE_LEN 8
#define GAIN 2

#define IN1 5
#define IN2 3

//328p
#define DIO0 2
#define DIO1 6

#define LORA_RST 4
#define LORA_CS 10

#define SPI_MOSI 11
#define SPI_MISO 12
#define SPI_SCK 13

#define VOLTAGE_PIN A0 // Read battery voltage
#define SENSOR_POWER_PIN 5 // RF98 Power pin

#define DEBUG_OUT_ENABLE 1

SX1278 radio = new Module(LORA_CS, DIO0, LORA_RST, DIO1);

bool readSensorStatus = false;
int sensorValue = 0; // variable to store the value coming from the sensor

String valve_status = "";
int current_cursor = 0;
int lora_receive = 0;
int rec_valve_status, done_flag = 0;
int batPercent, batValue = 0;    // the voltage of battery
int count = 0;
int ADC_O_1;           // ADC Output First 8 bits
int ADC_O_2;           // ADC Output Next 2 bits
int16_t packetnum = 0; // packet counter, we increment per xmission

void Lora_init()
{
  int state = radio.begin(FREQUENCY, BANDWIDTH, SPREADING_FACTOR, CODING_RATE, SX127X_SYNC_WORD, OUTPUT_POWER, PREAMBLE_LEN, GAIN);
  if (state == RADIOLIB_ERR_NONE)
  {

#if DEBUG_OUT_ENABLE
    Serial.println(F("Success!"));
    Serial.print(F("[SX1278] Datarate:\t"));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));
#endif
  }
  else
  {
#if DEBUG_OUT_ENABLE
    Serial.print(F("Failed, code "));
    Serial.println(state);
#endif
    // while (true)
    //     ;
  }
}

void setup() {
#if DEBUG_OUT_ENABLE
  Serial.begin(115200);
  Serial.println("Valve start.");
#endif
  delay(100);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(2, OUTPUT);

  pinMode(LORA_RST, OUTPUT);
  digitalWrite(LORA_RST, HIGH);
  delay(100);

  pinMode(SENSOR_POWER_PIN, OUTPUT);
  digitalWrite(SENSOR_POWER_PIN, HIGH); //Sensor power on
  delay(100);

  Lora_init();

  do_some_work();
  //setup over

#if DEBUG_OUT_ENABLE
  Serial.println("[Set]Sleep Mode Set");
#endif
  low_power_set();
}

void Valve_on()
{
  Serial.println(F("Valve ON!"));
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  delay(100);
}
void Valve_off()
{
  Serial.println(F("Valve OFF!"));
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  delay(100);
}

void loop()
{
  wdt_disable();

  if (count > SLEEP_CYCLE) //(7+1) x 8S  450
  {
#if DEBUG_OUT_ENABLE
    //code start
    Serial.println("Code start>>");
#endif

    do_some_work();

#if DEBUG_OUT_ENABLE
    //code end
    Serial.println("Code end<<");
#endif
    //count init
    count = 0;
  }

  low_power_set();
}

ISR(WDT_vect)
{
#if DEBUG_OUT_ENABLE
  Serial.print("[Watch dog]");
  Serial.println(count);
#endif
  delay(100);
  count++;
  //wdt_reset();
  wdt_disable(); // disable watchdog
}

//Set low power mode and into sleep
void low_power_set()
{
  all_pins_low();
  delay(10);
  // disable ADC
  ADCSRA = 0;

  sleep_enable();
  watchdog_init();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  delay(10);
  noInterrupts();
  sleep_enable();

  // turn off brown-out enable in software
  MCUCR = bit(BODS) | bit(BODSE);
  MCUCR = bit(BODS);
  interrupts();

  sleep_cpu();
  sleep_disable();
}

//Enable watch dog
void watchdog_init()
{
  // clear various "reset" flags
  MCUSR = 0;
  // allow changes, disable reset
  WDTCSR = bit(WDCE) | bit(WDE);
  WDTCSR = bit(WDIE) | bit(WDP3) | bit(WDP0); // set WDIE, and 8 seconds delay
  wdt_reset();                                // pat the dog
}

void do_some_work()
{
  digitalWrite(SENSOR_POWER_PIN, HIGH); // Sensor/RF95 power on
  digitalWrite(LORA_RST, HIGH);
  delay(5);

  Lora_init();
  delay(50);

  while (lora_receive == 0)
  {
    receive_lora();
  }

  if (rec_valve_status == 1 && done_flag == 0)
  {
#if DEBUG_OUT_ENABLE
    Serial.println("Turned ON Valve");
#endif
    Valve_on();
    //    Supply_off();
    valve_status = "ON";
    done_flag = 1;
  }

  if (rec_valve_status == 0 && done_flag == 1)
  {
#if DEBUG_OUT_ENABLE
    Serial.println("Turned OFF Valve");
#endif
    Valve_off();
    //    Supply_off();
    valve_status = "OFF";
    done_flag = 0;
  }

  //ADC3  internal 1.1V as ADC reference voltage
  ADMUX = _BV(REFS1) | _BV(REFS0) | _BV(MUX1) | _BV(MUX0);

  delay(50);
  for (int i = 0; i < 3; i++)
  {
    //start ADC conversion
    ADCSRA |= (1 << ADSC);

    delay(10);

    if ((ADCSRA & 0x40) == 0)
    {
      ADC_O_1 = ADCL;
      ADC_O_2 = ADCH;

      batValue = (ADC_O_2 << 8) + ADC_O_1;
      batPercent = batValue / 10;
      ADCSRA |= 0x40;
#if DEBUG_OUT_ENABLE
      Serial.print("BAT:");
      Serial.println(batValue);
      float bat = (float)batValue * 3.3;
      bat = bat / 1024.0;
      Serial.print(bat);
      Serial.println("V");
#endif
    }
    ADCSRA |= (1 << ADIF); //reset as required
    delay(50);
  }
  send_lora();
  delay(3000);

  radio.sleep();

  packetnum++;
  lora_receive = 0;
  readSensorStatus = false;
  digitalWrite(SENSOR_POWER_PIN, LOW); // Sensor/RF95 power off
  delay(100);
}

void all_pins_low()
{
  //Supply_off();
  delay(50);
}

void send_lora()
{
  String message = " INEDX:" + (String)packetnum + " VALVE_STATUS:" + valve_status + " BAT:" + (String)batPercent + "%";
  String back_str = node_id + " [VALVE]" + message;

#if DEBUG_OUT_ENABLE
  Serial.println();
  Serial.println(back_str);
#endif
  radio.transmit(back_str); //ID:DTH001 [VALVE] INEDX:1 VALVE_STATUS:ON BAT:98%
}

void receive_lora()
{
  String received_str;
  int state = radio.receive(received_str);

  if (state == RADIOLIB_ERR_NONE) {
    // packet was successfully received
    Serial.println(F("Success!"));

    lora_receive = 1;

#if DEBUG_OUT_ENABLE
    Serial.print(F("[SX1278] Data: "));
    Serial.println(received_str);

    Serial.print(F("[SX1278] Datarate: "));
    Serial.print(radio.getDataRate());
    Serial.println(F(" bps"));

    Serial.print(F("[SX1278] RSSI: ")); // print the RSSI (Received Signal Strength Indicator)
    Serial.print(radio.getRSSI());
    Serial.println(F(" dBm"));

    Serial.print(F("[SX1278] SNR: "));
    Serial.print(radio.getSNR());
    Serial.println(F(" dB"));

    Serial.print(F("[SX1278] Frequency error: "));
    Serial.print(radio.getFrequencyError());
    Serial.println(F(" Hz"));
#endif

    current_cursor = 0;
    current_cursor = received_str.indexOf('V', 0); //54
    String raw_valve_status = received_str.substring(current_cursor + 4, current_cursor + 5);
#if DEBUG_OUT_ENABLE
    Serial.print("Valve Status: ");
    Serial.println(raw_valve_status.toInt());
#endif
    rec_valve_status = raw_valve_status.toInt();

  } else if (state == RADIOLIB_ERR_RX_TIMEOUT) {
    // timeout occurred while waiting for a packet
    Serial.println(F("timeout!"));

  } else if (state == RADIOLIB_ERR_CRC_MISMATCH) {
    // packet was received, but is malformed
    Serial.println(F("CRC error!"));

  } else {
    // some other error occurred
    Serial.print(F("failed, code "));
    Serial.println(state);
  }
}
