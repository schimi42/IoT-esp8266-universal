
// ----------------------- Bedroom
//#define BEDROOM_ENV_SENSOR      //nodemcu 1.0
// ----------------------- Livingroom
//#define LIVINGROOM_ENV_SENSOR   //nodemcu 1.0
//#define LIVINGROOM_ENV_DISPLAY  //wemos
//#define LIVINGROOM_TV_SWITCH    //sonoff old
//#define LIVINGROOM_WALLPAPER    //sonoff old
// ----------------------- Bathroom

// ----------------------- Hallway
//#define HALLWAY_LAMP            //sonoff old

// ----------------------- Utility Room
//#define CURRENT_COUNTER         //wemos

// ----------------------- Garden
//#define GARDEN_ENV_SENSOR       //wemos
//#define GARDEN_SOIL_SENSOR      //nodemcu 0.9
//#define GARDEN_PUMP             //sonoff old
//#define GARDEN_PUMP_SWITCH      //wemos
//#define GARDEN_SPRINKLER        //wemos

//#define GARDEN_LIGHT_2        //sonoff old
//#define GARDEN_LIGHT_3        //sonoff new

// ----------------------- Garages
//#define GARAGE_DOOR_1         //wemos
//#define GARAGE_DOOR_2         //wemos

// ----------------------- Under Construction
//#define GARAGE_DOOR_3         //wemos
//#define BATHROOM1               //nodemcu
//#define BUTTON_PANEL_GARDEN   //wemos
//#define GARDEN_LIGHT_1        //sonoff
//#define VACATION_INDOOR   //wemos



// WiFi configuration
const char* WIFI_SSID = DEFAULT_WIFI_SSID;
const char* WIFI_PASSWORD = DEFAULT_WIFI_PASSWORD;
const char* MQTT_BROKER = DEFAULT_MQTT_BROKER;

// ----------------------- Bedroom
#ifdef BEDROOM_ENV_SENSOR
#define LOCATION "home"
#define DEVICE_NAME "bedroom/env_sensor"
#define HOST_NAME "bedroom_env_sensor"
#define BME_SENSOR
#define SDA D4
#define SCL D5
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

// ----------------------- Livingroom
#ifdef LIVINGROOM_ENV_SENSOR
#define LOCATION "home"
#define DEVICE_NAME "livingroom/env_sensor"
#define HOST_NAME "livingroom_env_sensor"
#define BME_SENSOR
#define SDA D4
#define SCL D5
#define LIGHT_SENSOR
#define BUTTON1
#define BUTTON1_PIN D6
//#define BUTTON1_LED_PIN D8
//#define BUTTON1_LED_ACTIVE HIGH
//#define BUTTON1_LED_N_ACTIVE LOW
#define BUTTON1_DIR FALLING
#define BUTTON1_STATE_TOPIC "home/livingroom/tv_switch/sonoff"
#define BUTTON1_SET_TOPIC "home/livingroom/tv_switch/sonoff/set"
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef LIVINGROOM_ENV_DISPLAY
#define LOCATION "home"
#define DEVICE_NAME "livingroom/env_display"
#define HOST_NAME "livingroom_env_display"
#define OLED
#define ENV_DISPLAY
#define DISPLAY_TOPIC_1 "home/livingroom/env_sensor/bme_temperature"
#define DISPLAY_HEAD_1 "Wohnzimmer"
#define DISPLAY_UNIT_1 "°C"
#define DISPLAY_TOPIC_2 "home/livingroom/env_sensor/bme_humidity"
#define DISPLAY_HEAD_2 "Wohnzimmer"
#define DISPLAY_UNIT_2 "%"
#define DISPLAY_TOPIC_3 "home/garden/env_sensor/bme_temperature"
#define DISPLAY_HEAD_3 "Außen"
#define DISPLAY_UNIT_3 "°C"
#define DISPLAY_TOPIC_4 "home/garden/env_sensor/bme_humidity"
#define DISPLAY_HEAD_4 "Außen"
#define DISPLAY_UNIT_4 "%"
#define DISPLAY_TOPIC_5 "home/garden/soil_sensor/ds18_temperature"
#define DISPLAY_HEAD_5 "Boden"
#define DISPLAY_UNIT_5 "°C"
#define DISPLAY_TOPIC_6 "home/garden/soil_sensor/soil_humidity"
#define DISPLAY_HEAD_6 "Boden"
#define DISPLAY_UNIT_6 "%"
#define DISPLAY_TOPIC_7 "home/bedroom/env_sensor/bme_temperature"
#define DISPLAY_HEAD_7 "Schlafzimmer"
#define DISPLAY_UNIT_7 "°C"
#define DISPLAY_TOPIC_8 "home/bedroom/env_sensor/bme_humidity"
#define DISPLAY_HEAD_8 "Schlafzimmer"
#define DISPLAY_UNIT_8 "%"
#define DISPLAY_COUNT 8 
#define DISPLAY_TIME_TOPIC "home/orangepi/time"
#define DISPLAY_DATE_TOPIC "home/orangepi/date"
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#define TIME_DISPLAY_UPDATE 3
#endif

#ifdef LIVINGROOM_TV_SWITCH
#define LOCATION "home"
#define DEVICE_NAME "livingroom/tv_switch"
#define HOST_NAME "livingroom_tv_switch"
#define SONOFF
#define SONOFF_PIN 12
#define SONOFF_BUTTON_PIN 0
#define SONOFF_LED_PIN 13
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef LIVINGROOM_WALLPAPER
#define LOCATION "home"
#define DEVICE_NAME "livingroom/wallpaper"
#define HOST_NAME "livingroom_wallpaper"
#define SONOFF
#define SONOFF_PIN 12
#define SONOFF_BUTTON_PIN 0
#define SONOFF_LED_PIN 13
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

// ----------------------- Bathroom
#ifdef BATHROOM1
#define LOCATION "home"
#define ROOM_NAME "bathroom1"
#define BME_SENSOR
#define SDA 2  // D4
#define SCL 14 // D5
#define FORCE_DEEPSLEEP
#define TIME_DEEPSLEEP 10 // in minutes
#define TIME_REPROGRAM 5 // in seconds
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

// ----------------------- Hallway
#ifdef HALLWAY_LAMP
#define LOCATION "home"
#define DEVICE_NAME "hallway/lamp"
#define HOST_NAME "hallway_lamp"
#define SONOFF
#define SONOFF_PIN 12
#define SONOFF_BUTTON_PIN 0
#define SONOFF_LED_PIN 13
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

// ----------------------- Utility Room
#ifdef CURRENT_COUNTER
#define LOCATION "home"
#define DEVICE_NAME "hwr/current_counter"
#define HOST_NAME "hwr_current_counter"
#define LIGHT_SENSOR
#define LIGHT_SENSOR_PIN D1
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#define CURRENT_SENSOR
#endif

// ----------------------- Garden
#ifdef GARDEN_ENV_SENSOR
#define LOCATION "home"
#define DEVICE_NAME "garden/env_sensor"
#define HOST_NAME "garden_env_sensor"
#define BME_SENSOR
#define SDA D6
#define SCL D5
//#define FORCE_DEEPSLEEP
//#define TIME_DEEPSLEEP 10 // in minutes
//#define TIME_REPROGRAM 5 // in seconds
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef GARDEN_SOIL_SENSOR
#define LOCATION "home"
#define DEVICE_NAME "garden/soil_sensor"
#define HOST_NAME "garden_soil_sensor"
#define SOIL_HUMIDITY_SENSOR
#define SOIL_HUMIDITY_SUPPLY_PIN D2
#define ONE_WIRE_SENSOR
#define ONE_WIRE_BUS D4
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef GARDEN_PUMP
#define LOCATION "home"
#define DEVICE_NAME "garden/pump"
#define HOST_NAME "garden_pump"
#define SONOFF
#define SONOFF_PIN 12
#define SONOFF_BUTTON_PIN 0
#define SONOFF_LED_PIN 13
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef GARDEN_PUMP_SWITCH
#define LOCATION "home"
#define DEVICE_NAME "garden/pump/switch"
#define HOST_NAME "garden_pump_switch"
#define TWO_WAY_SWITCH
#define TWO_WAY_SWITCH_ON_PIN D1
#define TWO_WAY_SWITCH_OFF_PIN D2
#define TWO_WAY_SWITCH_ON_LEVEL LOW
#define TWO_WAY_SWITCH_OFF_LEVEL HIGH
#define TWO_WAY_SWITCH_LED_PIN D3 
#define TWO_WAY_SWITCH_LED_ON_LEVEL LOW
#define TWO_WAY_SWITCH_LED_OFF_LEVEL HIGH
#define TWO_WAY_SWITCH_DIR CHANGE
#define TWO_WAY_SWITCH_SET_TOPIC "home/garden/pump/sonoff/set"
#define TWO_WAY_SWITCH_STATE_TOPIC "home/garden/pump/sonoff"
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef BUTTON_PANEL_GARDEN
#define LOCATION "home"
#define ROOM_NAME "button_panel_garden"
#define BUTTON1
#define BUTTON1_PIN D1
#define BUTTON1_LED_PIN D4
#define BUTTON1_LED_ACTIVE HIGH
#define BUTTON1_LED_N_ACTIVE LOW
#define BUTTON1_DIR FALLING
#define BUTTON1_STATE_TOPIC "home/garden_pump/sonoff"
#define BUTTON1_SET_TOPIC "home/garden_pump/sonoff/set"
#define BUTTON2
#define BUTTON2_PIN D5
#define BUTTON2_LED_PIN D8
#define BUTTON2_LED_ACTIVE HIGH
#define BUTTON2_LED_N_ACTIVE LOW
#define BUTTON2_DIR FALLING
#define BUTTON2_STATE_TOPIC "home/garden_light_1/sonoff"
#define BUTTON2_SET_TOPIC "home/garden_light_1/sonoff/set"
#define BUTTON3
#define BUTTON3_PIN D2
#define BUTTON3_LED_PIN D3
#define BUTTON3_LED_ACTIVE HIGH
#define BUTTON3_LED_N_ACTIVE LOW
#define BUTTON3_DIR FALLING
#define BUTTON3_STATE_TOPIC "home/garden_light_2/sonoff"
#define BUTTON3_SET_TOPIC "home/garden_light_2/sonoff/set"
#define BUTTON4
#define BUTTON4_PIN D6
#define BUTTON4_LED_PIN D7
#define BUTTON4_LED_ACTIVE HIGH
#define BUTTON4_LED_N_ACTIVE LOW
#define BUTTON4_DIR FALLING
#define BUTTON4_STATE_TOPIC "home/wallpaper/sonoff"
#define BUTTON4_SET_TOPIC "home/wallpaper/sonoff/set"
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef GARDEN_SPRINKLER
#define LOCATION "home"
#define DEVICE_NAME "garden/sprinkler"
#define HOST_NAME "garden_sprinkler"
//#define SOIL_HUMIDITY_SENSOR
//#define SOIL_HUMIDITY_SUPPLY_PIN D0 //D0
//#define ONE_WIRE_SENSOR
//#define ONE_WIRE_BUS D6
#define RELAY0
#define RELAY0_PIN D1 //D1
#define RELAY1
#define RELAY1_PIN D2 //D1
#define RELAY2
#define RELAY2_PIN D3 //D1
#define RELAY3
#define RELAY3_PIN D7 //D1
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef GARDEN_LIGHT_1
#define LOCATION "home"
#define DEVICE_NAME "garden/light_1"
#define HOST_NAME "garden_light_1"
#define SONOFF
#define SONOFF_PIN 12
#define SONOFF_BUTTON_PIN 0
#define SONOFF_LED_PIN 13
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef GARDEN_LIGHT_2
#define LOCATION "home"
#define DEVICE_NAME "garden/light_2"
#define HOST_NAME "garden_light_2"
#define SONOFF
#define SONOFF_PIN 12
#define SONOFF_BUTTON_PIN 0
#define SONOFF_LED_PIN 13
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

#ifdef GARDEN_LIGHT_3
#define LOCATION "home"
#define DEVICE_NAME "garden/light_3"
#define HOST_NAME "garden_light_3"
#define SONOFF
#define SONOFF_PIN 12
#define SONOFF_BUTTON_PIN 0
#define SONOFF_LED_PIN 13
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

// ----------------------- Garages

#ifdef GARAGE_DOOR_1
#define LOCATION "home"
#define DEVICE_NAME "garage/door_1"
#define HOST_NAME "garage_door_1"
#define COVER
#define COVER_TRIGGER1_PIN D8
#define COVER_TRIGGER2_PIN D7
#define COVER_STATE1_PIN D1
#define COVER_STATE1_DIR CHANGE
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds 
#endif

#ifdef GARAGE_DOOR_2
#define LOCATION "home"
#define DEVICE_NAME "garage/door_2"
#define HOST_NAME "garage_door_2"
#define COVER
#define COVER_TRIGGER1_PIN D8
#define COVER_TRIGGER2_PIN D7
#define COVER_STATE1_PIN D1
#define COVER_STATE1_DIR CHANGE
#define GPIO_STATIC_OUTPUT D3
#define GPIO_STATIC_OUTPUT_LEVEL HIGH
#define BME_SENSOR
#define SDA D6
#define SCL D5
#define TIME_STATE_UPDATE 10 // in seconds
#define TIME_SENSOR_UPDATE 300 // in seconds 
#endif

// ----------------------- Under Construction

#ifdef VACATION_INDOOR
#define LOCATION "vacation"
#define DEVICE_NAME "indoor/env_sensor"
#define HOST_NAME "vacation_indoor_env_sensor"
#define BME_SENSOR
#define SDA D6
#define SCL D5
//#define FORCE_DEEPSLEEP
//#define TIME_DEEPSLEEP 10 // in minutes
//#define TIME_REPROGRAM 5 // in seconds
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#endif

// ----------------------- Eval Kit
#ifdef EVAL_KIT
#define LOCATION "home"
#define DEVICE_NAME "eval_kit"
#define HOST_NAME "eval_kit"
//#define BME_SENSOR
//#define SDA D8
//#define SCL D3
#define SONOFF
#define SONOFF_PIN D0
#define SONOFF_BUTTON_PIN D5
#define SONOFF_LED_PIN D0
#define OLED
#define ENV_DISPLAY
#define DISPLAY_TOPIC_1 "home/livingroom/env_sensor/bme_temperature"
#define DISPLAY_HEAD_1 "Wohnzimmer"
#define DISPLAY_UNIT_1 "°C"
#define DISPLAY_TOPIC_2 "home/livingroom/env_sensor/bme_humidity"
#define DISPLAY_HEAD_2 "Wohnzimmer"
#define DISPLAY_UNIT_2 "%"
#define DISPLAY_TOPIC_3 "home/garden/env_sensor/bme_temperature"
#define DISPLAY_HEAD_3 "Außen"
#define DISPLAY_UNIT_3 "°C"
#define DISPLAY_TOPIC_4 "home/garden/env_sensor/bme_humidity"
#define DISPLAY_HEAD_4 "Außen"
#define DISPLAY_UNIT_4 "%"
#define DISPLAY_TOPIC_5 "home/garden/soil_sensor/ds18_temperature"
#define DISPLAY_HEAD_5 "Boden"
#define DISPLAY_UNIT_5 "°C"
#define DISPLAY_TOPIC_6 "home/garden/soil_sensor/soil_humidity"
#define DISPLAY_HEAD_6 "Boden"
#define DISPLAY_UNIT_6 "%"
#define DISPLAY_TOPIC_7 "home/bedroom/env_sensor/bme_temperature"
#define DISPLAY_HEAD_7 "Schlafzimmer"
#define DISPLAY_UNIT_7 "°C"
#define DISPLAY_TOPIC_8 "home/bedroom/env_sensor/bme_humidity"
#define DISPLAY_HEAD_8 "Schlafzimmer"
#define DISPLAY_UNIT_8 "%"
#define DISPLAY_COUNT 8 
#define DISPLAY_TIME_TOPIC "home/orangepi/time"
#define DISPLAY_DATE_TOPIC "home/orangepi/date"
#define TIME_SENSOR_UPDATE 300 // in seconds
#define TIME_STATE_UPDATE 10 // in seconds
#define TIME_DISPLAY_UPDATE 3
#endif