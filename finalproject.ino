#include "BLEDevice.h"
#include "WiFi.h"
#include "StreamIO.h"
#include "VideoStream.h"
#include "RTSP.h"

#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100
#define MaxNumValue     2

#define value1 0
#define value2 1

#define MotoA_1A 1    // GPIO
#define MotoA_1B 2     // PWM
#define MotoB_1A 3    // GPIO
#define MotoB_1B 4     // PWM

#define CHANNEL 1

// Default preset configurations for each video channel:
// Channel 0 : 1920 x 1080 30FPS H264
// Channel 1 : 1280 x 720  30FPS H264
// Channel 2 : 1920 x 1080 30FPS MJPEG

// VideoSetting config(CHANNEL);
VideoSetting config(VIDEO_D1, CAM_FPS, VIDEO_H264, 0);
RTSP rtsp1;
RTSP rtsp2;
StreamIO videoStreamer(1, 2);    // 1 Input Video -> 1 Output RTSP

char ssid[] = "Death";    // your network SSID (name)
char pass[] = "07102231";        // your network password
int status = WL_IDLE_STATUS;

typedef struct {
    bool reciveCMDFlag;
    int ReciveValue;
} _rCMD;

BLEService UartService(UART_SERVICE_UUID);
BLECharacteristic Rx(CHARACTERISTIC_UUID_RX);
BLECharacteristic Tx(CHARACTERISTIC_UUID_TX);
BLEAdvertData advdata;
BLEAdvertData scndata;
bool notify = false;
uint8_t Count;

String CMDRefer[5] = {"SS2", "SS4", "SRT", "SR2", "SRV"};
_rCMD bleReciveData[MaxNumValue];

void forward()
{
    digitalWrite(MotoA_1A, 1);
    analogWrite(MotoA_1B, 5);

    digitalWrite(MotoB_1A, 1);
    analogWrite(MotoB_1B, 5);

    delay(50);
}

void backward()
{
    digitalWrite(MotoA_1A, 0);
    analogWrite(MotoA_1B, 250);

    digitalWrite(MotoB_1A, 0);
    analogWrite(MotoB_1B, 250);

    delay(50);
}

void turnRight()
{
    digitalWrite(MotoA_1A, 1);
    analogWrite(MotoA_1B, 5);

    digitalWrite(MotoB_1A, 0);
    analogWrite(MotoB_1B, 250);

    delay(50);
}

void turnLeft()
{
    digitalWrite(MotoA_1A, 0);
    analogWrite(MotoA_1B, 250);

    digitalWrite(MotoB_1A, 1);
    analogWrite(MotoB_1B, 5);

    delay(50);
}

void BrakeAll()
{
    digitalWrite(MotoA_1A, 0);
    analogWrite(MotoA_1B, 0);

    digitalWrite(MotoB_1A, 0);
    analogWrite(MotoB_1B, 0);

    delay(50);
}

void readCB(BLECharacteristic* chr, uint8_t connID)
{
    printf("Characteristic %s read by connection %d \n", chr->getUUID().str(), connID);
}

void writeCB(BLECharacteristic* chr, uint8_t connID)
{
    if (chr->getDataLen() > 0) {
        ParseCMDString(chr->readString());
    }
}

void notifCB(BLECharacteristic* chr, uint8_t connID, uint16_t cccd)
{
    if (cccd & GATT_CLIENT_CHAR_CONFIG_NOTIFY) {
        printf("Notifications enabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = true;
    } else {
        printf("Notifications disabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        notify = false;
    }
}

void ParseCMDString(String cmd)
{
    int comdLength = cmd.length();
    int chkx;
    int CMDMaxNUM = sizeof(CMDRefer) / sizeof(String);

    for (chkx = 0; chkx < CMDMaxNUM; chkx++) {
        if (cmd.indexOf(CMDRefer[chkx].c_str()) > -1) {
            break;
        }
    }

    if (chkx >= CMDMaxNUM && cmd.charAt(comdLength - 1) != '#') {
        return;
    }

    if (cmd.indexOf("SRT") > -1) {
        int x = 3;
        int ValueIndex = 0;

        while (x < (comdLength - 1)) {
            if ((x + 3) < comdLength) {
                String _NumString = cmd.substring(x, (x + 4));
                if (ValueIndex < MaxNumValue) {
                    if (bleReciveData[ValueIndex].ReciveValue != _NumString.toInt()) {
                        bleReciveData[ValueIndex].ReciveValue = _NumString.toInt();
                        bleReciveData[ValueIndex].reciveCMDFlag = true;
                    }
                }
            }
            ValueIndex++;
            x += 4;
        }
    }
}

void setup()
{
    Serial.begin(115200);

    advdata.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advdata.addCompleteName("AMB82-TANK");
    scndata.addCompleteServices(BLEUUID(UART_SERVICE_UUID));

    Rx.setWriteNRProperty(true);
    Rx.setWritePermissions(GATT_PERM_WRITE);
    Rx.setWriteCallback(writeCB);
    Rx.setBufferLen(STRING_BUF_SIZE);

    Tx.setReadProperty(true);
    Tx.setReadPermissions(GATT_PERM_READ);
    Tx.setReadCallback(readCB);
    Tx.setNotifyProperty(true);
    Tx.setCCCDCallback(notifCB);
    Tx.setBufferLen(STRING_BUF_SIZE);

    UartService.addCharacteristic(Rx);
    UartService.addCharacteristic(Tx);

    BLE.init();
    BLE.configAdvert()->setAdvData(advdata);
    BLE.configAdvert()->setScanRspData(scndata);
    BLE.configServer(1);
    BLE.addService(UartService);

    BLE.beginPeripheral();

    while (status != WL_CONNECTED) {
        status = WiFi.begin(ssid, pass);
        delay(2000);
    }

    Camera.configVideoChannel(CHANNEL, config);
    Camera.videoInit();

    rtsp1.configVideo(config);
    rtsp1.begin();
    rtsp2.configVideo(config);
    rtsp2.begin();

    videoStreamer.registerInput(Camera.getStream(CHANNEL));
    videoStreamer.registerOutput1(rtsp1);
    videoStreamer.registerOutput2(rtsp2);
    if (videoStreamer.begin() != 0) {
        Serial.println("StreamIO link start failed");
    }

    Camera.channelBegin(CHANNEL);

    delay(1000);
    // printInfo();

    pinMode(MotoA_1A, OUTPUT);
    pinMode(MotoA_1B, OUTPUT);
    pinMode(MotoB_1A, OUTPUT);
    pinMode(MotoB_1B, OUTPUT);

    digitalWrite(MotoA_1A, 0);
    digitalWrite(MotoA_1B, 0);
    digitalWrite(MotoB_1A, 0);
    digitalWrite(MotoB_1B, 0);
}

void loop()
{
    while (Count < MaxNumValue) {
        if (bleReciveData[Count].reciveCMDFlag) {
            bleReciveData[Count].reciveCMDFlag = false;

            if (abs(bleReciveData[value1].ReciveValue - 1500) < 100 && abs(bleReciveData[value2].ReciveValue - 1500) < 100) {
                BrakeAll();
            } else if (abs(bleReciveData[value1].ReciveValue - 1500) > abs(bleReciveData[value2].ReciveValue - 1500)) {
                if (bleReciveData[value1].ReciveValue > 1500) {
                    turnRight();
                } else {
                    turnLeft();
                }
            } else {
                if (bleReciveData[value2].ReciveValue > 1500) {
                    forward();
                } else {
                    backward();
                }
            }
        }
        Count++;
    }
    Count = 0;
    delay(1);
}
