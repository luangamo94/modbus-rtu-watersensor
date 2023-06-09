/*
  -------------------------------------
  - DO sensor
    + ADDRESS: 01
    + 3 thanh ghi: 0x0000, 0x0001, 0x0002 tuong ung voi do bao hao, nong do oxy hoa tan, nhiet do
    + Baud rate: 4800 bit/s
  - ORP sensor
    + ADDRESS: 01
    + 2 thanh ghi: 0x0000, 0x0050 tuong ung voi
    + Baud rate: 4800 bit/s
*/

#include <ModbusMaster.h>
#include <SoftwareSerial.h>

#define DO_add 2
#define ORP_add 1
/*!
  We're using a MAX485-compatible RS485 Transceiver.
  Rx/Tx is hooked up to the hardware serial port at 'Serial'.
  The Data Enable and Receiver Enable pins are hooked up as follows:
*/


#define relay_pin PC14

bool stt = 0;
float nhietdo, nongdo, dobaohoa;
int ORP_value, ORP_offset, c;
float sum;
// instantiate ModbusMaster object
ModbusMaster node;
SoftwareSerial mySerial(6, 7);



void DO_init(ModbusMaster _node) {
  uint8_t result;
  long int num[3];
  float fvalue[3];  // gia tri dang so thuc
  digitalWrite(relay_pin, 1);
  _node.begin(DO_add, mySerial);
  result = _node.readHoldingRegisters(0x0000, 6);  // doc tu thanh ghi 0x0000, so luong 6
  if (result == node.ku8MBSuccess)                 // nhan thanh cong tra ve 0x00
  {
    // do bao hoa oxy hoa tan (don vi: %)
    num[0] = (((unsigned long)_node.getResponseBuffer(0x00) & 0xFFFF) << 16) | (_node.getResponseBuffer(0x01) & 0xFFFF);
    memcpy(&fvalue[0], &num[0], 4);
    dobaohoa = 100 * fvalue[0];
    // Nong do oxy hoa tan (don vi: mg/L)
    num[1] = (((unsigned long)_node.getResponseBuffer(0x02) & 0xFFFF) << 16) | (_node.getResponseBuffer(0x03) & 0xFFFF);
    memcpy(&fvalue[1], &num[1], 4);
    nongdo = fvalue[1];
    // Nhiet do (don vi: do C
    num[2] = (((unsigned long)_node.getResponseBuffer(0x04) & 0xFFFF) << 16) | (_node.getResponseBuffer(0x05) & 0xFFFF);
    memcpy(&fvalue[2], &num[2], 4);
    nhietdo = fvalue[2];
  }
}

void ORP_init(ModbusMaster _node) {
  uint8_t result;

  _node.begin(ORP_add, mySerial);
  result = _node.readHoldingRegisters(0x0000, 1);  // doc tu thanh ghi 0x0000, so luong 6
  if (result == node.ku8MBSuccess)                 // nhan thanh cong tra ve 0x00
  {
    // gia tri ORP (don vi: mV)
    ORP_value = _node.getResponseBuffer(0x00);
  }
  // result = _node.readHoldingRegisters(0x0050, 1);  // doc tu thanh ghi 0x0000, so luong 6
  // if (result == node.ku8MBSuccess)                 // nhan thanh cong tra ve 0x00
  // {
  //   // gia tri ORP (don vi: mV)
  //   ORP_offset = _node.getResponseBuffer(0x00);
  // }
}
void setup() {
  pinMode(relay_pin, OUTPUT);


  //digitalWrite(MAX485_DE, 0);

  // Modbus communication runs at 115200 baud
  Serial.begin(4800);
  Serial.begin(4800);
  mySerial.begin(4800);
  // Modbus slave ID 1
  //node.begin(1, Serial);

  // Callbacks allow us to configure the RS485 transceiver correctly

}

void loop() {

  //  uint8_t result;
  //  uint16_t data[6];
  //
  //  result = node.readHoldingRegisters(0x0A00, 1);
  //  //mySerial.println(result);
  //  if (result == node.ku8MBSuccess) // nhan thanh cong tra ve 0x00
  //  {
  //    mySerial.println(node.getResponseBuffer(0x00));
  //    delay(2000);
  //  }
  //  else mySerial.println("Read Fail!!");
  //for (int i=0; i < 20; i++){
  //    DO_init(node);
  //    if(dobaohoa != 0)
  //    {
  //      sum += dobaohoa;
  //      c++;
  //      }
  DO_init(node);
  Serial.print(dobaohoa); Serial.println(" %");
  delay(500);
  Serial.print(nongdo); Serial.println(" mg/L");
  delay(500);
  Serial.print(nhietdo); Serial.println(" do C");
  delay(500);

//  ORP_init(node);
//  mySerial.print(ORP_value); mySerial.println(" mV");
// //  mySerial.print(ORP_offset); mySerial.println(" mV");
//  delay(500);
}
