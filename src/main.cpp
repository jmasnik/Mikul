#include "esp_camera.h"
#include <WiFi.h>
#include "soc/soc.h"             //disable brownout problems
#include "soc/rtc_cntl_reg.h"    //disable brownout problems
#include "SPIFFS.h"
#include <HTTPClient.h>
#include "mbedtls/base64.h"
#include "time.h"
#include <Arduino.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define PIN_LED 33
#define PIN_LED_FLASH 4 
#define PIN_BAT_ADC 2       // nefunguje, je to ADC2 a to nejde spolu s wifi, jedina moznost je asi GPIO33, na te je ale LED, musela by se odpajet 

// interval mezi snimky
#define PHOTO_DELAY 40

#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

void setFlashOn();
void setFlashOff();
void setLedOn();
void setLedOff();

WiFiUDP Udp;
ESP32PWM pwm1;
ESP32PWM pwm2;
Servo servo;

TaskHandle_t ControllTaskHandle = NULL;
TaskHandle_t CameraTaskHandle = NULL;

int servo_want = 90;
int servo_have = 90;
unsigned long servo_last;

#define SERVO_DELAY 1

#define REMOTE_UDP_PORT 4210
#define LOCAL_UDP_PORT 4211

#define PIN_PWM1 14
#define PIN_PWM2 15
#define PWM_RESOLUTION 8
#define PWM_FREQ 1000
#define PIN_SERVO 12
#define MOTOR_DEADBAND 12
#define MOTOR_MIN_PWM 250

// kolik pixelu dava kamera
#define IMG_CAM_WIDTH       96

// jakou sirku obrazu posilam
#define IMG_OUT_WIDTH       96

// o kolik orezavam
#define IMG_CROP_SKIP       0

// 7 radek obrazu o sirce 96px 
#define IMG_PACKET_LEN      96 * 7 * 2

// kolik ma jedna radka co posilam
#define IMG_LENGTH_LINE     IMG_OUT_WIDTH * 2

// orez nahore - 14 prvnich radek pryc
#define IMG_CROP_TOP        (IMG_CAM_WIDTH * 2 * 14) + (IMG_CROP_SKIP / 2)

// packet s rizenim
typedef struct mikulControls {
    uint8_t ident;
    int8_t motor;
    int8_t servo;
    uint8_t lights;
} mikulControls;

/**
 * Ridici vlakno ktere ovlada servo, motor
 */
void ControllTask(void *parameter)
{
    int packet_size;
    uint8_t packet[100];
    mikulControls mikul_controls;
    unsigned long millis_act;
    unsigned int packet_rx_cnt = 0;
    uint8_t led_state = 0;
    char buff[100];

    while (1){
        
        // neprislo neco po UDP?
        packet_size = Udp.parsePacket();
        if(packet_size > 0){
            // pocet prijatych packetu
            packet_rx_cnt++;
            
            // nacteme packet
            Udp.read(packet, packet_size);
            
            // TODO: zkontrolovat ze to je spravnej packet
            memcpy(&mikul_controls, packet, sizeof(mikul_controls));
            
            // kde mame mit servo
            servo_want = map(mikul_controls.servo, -100, 100, 50, 165); 

            // blikani LED
            if(packet_rx_cnt % 20 == 0){
                if(led_state == 0){
                    setLedOn();
                    led_state = 1;
                } else {
                    setLedOff();
                    led_state = 0;
                }
    
                sprintf(buff, "Sw: %d Cm: %d", servo_want, mikul_controls.motor);
                Serial.println(buff);
            }
        }

        // aktualni ms
        millis_act = millis();

        // pohnuti servem do nove polohy
        if(millis_act - servo_last >= SERVO_DELAY){
            if(servo_have < servo_want) servo_have++;
            if(servo_have > servo_want) servo_have--;
            servo.write(servo_have);
            servo_last = millis_act;
        }

        // motor
        if(mikul_controls.motor > -MOTOR_DEADBAND && mikul_controls.motor < MOTOR_DEADBAND){
            // stoji
            pwm1.writeScaled(0);
            pwm2.writeScaled(0);
        } else if (mikul_controls.motor >= MOTOR_DEADBAND) {
            // dopredu
            pwm1.writeScaled(0);
            pwm2.writeScaled(map(mikul_controls.motor, MOTOR_DEADBAND, 100, MOTOR_MIN_PWM, 1000) / 1000.0);
        } else if (mikul_controls.motor <= -MOTOR_DEADBAND) {
            // dozadu
            pwm1.writeScaled(map(-mikul_controls.motor, MOTOR_DEADBAND, 100, MOTOR_MIN_PWM, 1000) / 1000.0);
            pwm2.writeScaled(0);
        }

        // malej delay
        vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}

/**
 * Task ktery resi kameru a posilani obrazu z ni pres UDP
 */
void CameraTask(void *parameter)
{
    unsigned long millis_act;
    uint8_t i;
    uint8_t j;
    uint8_t packet[1500];
    uint8_t *ptr_fbuf;
    uint8_t *ptr_pack;
    unsigned long millis_last = 0;
    
    while(1)
    {
        millis_act = millis();

        // posilani obrazu v intervalu
        if ((millis_act - millis_last) >= PHOTO_DELAY) {
            
            //capture a frame
            camera_fb_t *fb = esp_camera_fb_get();
            if (fb) {
                ptr_fbuf = fb->buf + IMG_CROP_TOP;

                // prvnich 14 radek ignorujeme, pak 70 radek (v jednom paketu je 7 radek)
                for(i = 0; i < 10; i++){

                    // hlavicka packetu
                    packet[0] = 0xF0;
                    packet[1] = i;
                    packet[2] = WiFi.RSSI();

                    ptr_pack = packet + 3;

                    //memcpy(packet + 3, fb->buf + ((i + 2) * IMG_PACKET_LEN), IMG_PACKET_LEN);
                    for(j = 0; j < 7; j++){
                        memcpy(ptr_pack, ptr_fbuf, IMG_LENGTH_LINE);
                        ptr_fbuf += IMG_LENGTH_LINE + IMG_CROP_SKIP;
                        ptr_pack += IMG_LENGTH_LINE;
                    }

                    Udp.beginPacket("192.168.4.1", REMOTE_UDP_PORT);
                    Udp.write(packet, IMG_PACKET_LEN + 3); 
                    Udp.endPacket();

                    vTaskDelay(3 / portTICK_PERIOD_MS);
                }
            }
            esp_camera_fb_return(fb);            

            millis_last = millis_act;
        }

        // malej delay
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

/**
 * Rozne LED
 */
void setLedOn()
{
    digitalWrite(PIN_LED, LOW);
}

/**
 * Zhasne LED
 */
void setLedOff()
{
    digitalWrite(PIN_LED, HIGH);
}

/**
 * Setup
 */
void setup()
{
    char buff[100];
    sensor_t *cam_sensor;

    servo_last = 0;

    //disable brownout detector
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);	

    // pin mode
    pinMode(PIN_LED, OUTPUT);
    pinMode(PIN_LED_FLASH, OUTPUT);

    // mala cervena sviti pri low
    setLedOff();   

    // velka bila sviti pri high, otazka jestli muze svitit dele, dost hreje
    setFlashOff();   

    // seriak
    Serial.begin(115200);
    Serial.setDebugOutput(false);
    
    // msg
    Serial.println("- Mikul v0.2 --------------------------------");   

    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);

    // nastaveni serva
    servo.setPeriodHertz(50);
    servo.attach(PIN_SERVO, 1000, 2000); 

    // nastaveni PWM
    pwm1.attachPin(PIN_PWM1, PWM_FREQ, PWM_RESOLUTION);
    pwm2.attachPin(PIN_PWM2, PWM_FREQ, PWM_RESOLUTION);

    pwm1.writeScaled(0);
    pwm2.writeScaled(0);

    // camera config
    camera_config_t config;
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sccb_sda = SIOD_GPIO_NUM;
    config.pin_sccb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    //config.pixel_format = PIXFORMAT_RGB565; 
    //config.pixel_format = PIXFORMAT_JPEG; 
    config.pixel_format = PIXFORMAT_RGB565; 
    
    if(!psramFound()){
        Serial.println("PSRAM not found!");
        while(1){ }
    }

    config.frame_size = FRAMESIZE_96X96;
    config.jpeg_quality = 5;
    config.fb_count = 1;

    // Camera init
    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        Serial.printf("Camera init failed with error 0x%x", err);
        Serial.println();
    } else {
        Serial.println("Camera init OK");
    }

    // trochu barevnejsi a kontrastnejsi obraz
    cam_sensor = esp_camera_sensor_get();
    cam_sensor->set_saturation(cam_sensor, 2);
    cam_sensor->set_contrast(cam_sensor, 2);
    cam_sensor->set_sharpness(cam_sensor, 2);
    //cam_sensor->set_brightness(cam_sensor, 2);

    // Wifi STA
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    delay(100);

   WiFi.begin("mController");
   Serial.println("mController / bez");
   Serial.print("Connecting to WiFi");
   while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(1000);
   }
   Serial.println(".");   

   // Print ESP Local IP Address
   Serial.println(WiFi.localIP());   
   Serial.println(WiFi.gatewayIP());

    // UDP
    Udp.begin(LOCAL_UDP_PORT);

    // Task resici ovladani
    xTaskCreatePinnedToCore(
        ControllTask,           // Task function
        "ControllTask",         // Task name
        10000,                  // Stack size (bytes)
        NULL,                   // Parameters
        1,                      // Priority
        &ControllTaskHandle,    // Task handle
        1                       // Core 1
    );  

    // Task resici kameru
    xTaskCreatePinnedToCore(
        CameraTask,           // Task function
        "CameraTask",         // Task name
        100000,                  // Stack size (bytes)
        NULL,                   // Parameters
        1,                      // Priority
        &CameraTaskHandle,    // Task handle
        1                       // Core 1
    );      
}

/**
 * Hlavni smycka
 */
void loop()
{

}

/**
 * Rozne blesk
 */
void setFlashOn(){
    digitalWrite(PIN_LED_FLASH, HIGH);
}

/**
 * Zhasne blesk
 */
void setFlashOff(){
    digitalWrite(PIN_LED_FLASH, LOW);
}

