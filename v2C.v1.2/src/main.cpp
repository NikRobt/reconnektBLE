#include <Arduino.h>
#include "BLEDevice.h"
#include "Esp.h"
// #include <Wire.h>

#define LED_RESOLUTION 10
#define LED_FREQ 1
#define LED_1_CH 0
#define LED_2_CH 1
#define LED_1 GPIO_NUM_18
#define LED_2 GPIO_NUM_32

unsigned long previousMillis1 = 0;
unsigned long interval1 = 30000;

hw_timer_t *timerReConnectF = NULL;

#define bleServerName "LARS_Bracelet"
// UUID для сервиса:
static BLEUUID bleServiceUUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
// UUID для  (данные одной строчки)
static BLEUUID bleStringCharacteristicUUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");
BLEClient *pClient = BLEDevice::createClient();
BLEScan *pBLEScan = BLEDevice::getScan();
// переменные, используемые для определения того,
// нужно ли начинать подключение или завершено ли подключение:
static bool doConnect = false;
void IRAM_ATTR onTimer();
//- метод который позволяет понять состояние связи
volatile bool flagCorect;

static void bleStringNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                    uint8_t *pData, size_t length, bool isNotify);

// адрес периферийного устройства (он должен быть найден во время сканирования)
static BLEAddress *pServerAddress;

// характеристики, данные которых необходимо считать
static BLERemoteCharacteristic *bleStringCharacteristic;

// включение/выключение уведомлений:
////////////////////////////////////////////////////////////////
const uint8_t notificationOn[] = {0x1, 0x0}; // посмотреть к какому функционалу привязаны
const uint8_t notificationOff[] = {0x0, 0x0};
////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////
static void my_gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
  // ESP_LOGW(LOG_TAG, "custom gap event handler, event: %d", (uint8_t)event);
}

static void my_gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
  // ESP_LOGW(LOG_TAG, "custom gattc event handler, event: %d", (uint8_t)event);
}

////////////////////////////////////////////////////////////////////////////////////
// подключаемся к BLE-серверу,
// у которого есть название, сервис и характеристики:
bool connectToServer(BLEAddress pAddress)
{
  // BLEClient *pClient = BLEDevice::createClient();
  pClient->connect(pAddress); // подключаемся к удаленному BLE-серверу:

  // Serial.println(" - Connected to server");
  //  " – Подключились к серверу"
  ////////////////////////////////////////////////////////////////////////////////
  // считываем UUID искомого сервиса:

  BLERemoteService *pRemoteService = pClient->getService(bleServiceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    //  "Не удалось найти UUID нашего сервиса: "
    Serial.println(bleServiceUUID.toString().c_str());
    return (false);
  }
  /////////////////////////////////////////////////////////////////////////////////
  // считываем UUID искомых характеристик:
  bleStringCharacteristic = pRemoteService->getCharacteristic(bleStringCharacteristicUUID);

  if (bleStringCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID"); //  "Не удалось найти UUID нашей характеристики"
    return false;
  }
  Serial.println(" - Сharacteristics were found"); //  " – Наши характеристики найдены"

  // присваиваем характеристикам функции обратного вызова:
  bleStringCharacteristic->registerForNotify(bleStringNotifyCallback);
  return true;
}

// функция обратного вызова, которая будет вызвана
// при получении оповещения от другого устройства:
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice advertisedDevice)
  {
    // проверяем, совпадает ли название
    // BLE-сервера, рассылающего оповещения:
    if (advertisedDevice.getName() == bleServerName)
    {
      // мы нашли, что искали,
      // поэтому сканирование можно завершить:
      advertisedDevice.getScan()->stop();
      // сохраняем адрес устройства, рассылающего оповещения:
      pServerAddress = new BLEAddress(advertisedDevice.getAddress());
      // задаем индикатор, дающий понять,
      // что мы готовы подключиться:
      doConnect = true;
      Serial.println("Device is found. Connecting!");
      //  "Устройство найдено. Подключаемся!"
    }
  }
};

static void bleStringNotifyCallback(BLERemoteCharacteristic *pBLERemoteCharacteristic,
                                    uint8_t *pData, size_t length, bool isNotify)
{

  char tmp[40];
  memset(tmp, 0, sizeof(tmp));
  memcpy(tmp, pData, length);
  Serial.print(tmp);
}

void setup()
{

  // запускаем последовательную коммуникацию:
  Serial.begin(115200);

  // setCpuFrequencyMhz(80);

  ledcSetup(LED_1_CH, LED_FREQ, LED_RESOLUTION); //- первый тип
  ledcSetup(LED_2_CH, LED_FREQ, LED_RESOLUTION); // - второй тип

  ledcAttachPin(LED_1, LED_1_CH);
  ledcAttachPin(LED_2, LED_2_CH);

  //////////////////////////////////////////////иницилизация и подключение протокола ble
  // инициализируем BLE-устройство:h
  BLEDevice::init("ESP32_Client");
  BLEDevice::setCustomGapHandler(my_gap_event_handler);
  BLEDevice::setCustomGattcHandler(my_gattc_event_handler);
  //////////////////////////////////////////////////////////////////////////////

  BLEAddress address("d8:c5:de:83:0f:db"); // ef:05:04:79:49:bf - black, d8:c5:de:83:0f:db - white
  // BLEClient *pClient = BLEDevice::createClient();
  pClient->isConnected();
  bool connect(BLEAddress address, esp_ble_addr_type_t type = BLE_ADDR_TYPE_PUBLIC);
  bool success = pClient->connect(address);
  if (esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_P9) == ESP_FAIL)
  {
    Serial.println("Tx power set failed");
  };

  timerReConnectF = timerBegin(0, 80, true);
  timerAttachInterrupt(timerReConnectF, &onTimer, true);
  timerAlarmWrite(timerReConnectF, 10000, true);
  // создаем экземпляр класса «BLEScan» для сканирования
  // и задаем для этого объекта функцию обратного вызова,
  // которая будет информировать о том, найдено ли новое устройство;
  // дополнительно указываем, что нам нужно активное сканирование,
  // а потом запускаем 30-секундное сканирование:

  ledcWrite(LED_2_CH, 240);
  // BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(true);
  pBLEScan->start(1);
}

void IRAM_ATTR onTimer()
{

  if (pClient->isConnected() == false)
  {
    flagCorect = true;
  }
  else if (pClient->isConnected() == true)
  {
    flagCorect = false;
  }
}

void loop()
{
  // если в переменной «doConnect» значение «true»,
  // то это значит, что сканирование завершено,
  // и мы нашли нужный BLE-сервер, к которому хотим подключиться;
  // теперь пора, собственно, подключиться к нему;

  if (doConnect == true)
  {
    if (connectToServer(*pServerAddress))
    {
      timerWrite(timerReConnectF, 0); //обнуляем таймер
      timerAlarmEnable(timerReConnectF);
      // активируем свойство «notify» у каждой характеристики:
      bleStringCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2901))->writeValue((uint8_t *)notificationOn, 2, true); // 0x2902
      ledcWrite(LED_2_CH, 0);
      ledcWrite(LED_1_CH, 512);
    }
    else
    {
      Serial.println("We have failed to connect to the server; Restart your device to scan for nearby BLE server again.");
      //  "Подключиться к серверу не получилось.
      //   Перезапустите устройство, чтобы снова
      //   просканировать ближайший BLE-сервер."
      ledcWrite(LED_1_CH, 160);
      ledcWrite(LED_2_CH, 160);
    }
    doConnect = false;
  }
  if (flagCorect == true)
  {
    // timerWrite(timerReConnectF, 0); //обнуляем таймер
    // timerAlarmEnable(timerReConnectF);
    timerAlarmDisable(timerReConnectF);
    pBLEScan->start(1);
    flagCorect = false;
    // if (connectToServer(*pServerAddress))
    // {
    //   // активируем свойство «notify» у каждой характеристики:
    //   Serial.println("rew");

    //   bleStringCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2901))->writeValue((uint8_t *)notificationOn, 2, true); // 0x2902
    //   ledcWrite(LED_2_CH, 0);
    //   ledcWrite(LED_1_CH, 512);
    // }

    // if (flagCorect == false)
    //   break;
  }
}
