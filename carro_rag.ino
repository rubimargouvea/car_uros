/*
Comandos executados no terminalpara inicializar o ROS2:
->ros2 topic echo /battery
->ros2 topic echo /cmd_vel
->ros2 topic echo /right_motor_ticks
->ros2 topic echo /left_motor_ticks
->ros2 run teleop_twist_keyboard teleop_twist_keyboard
->ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
considerando que a inicialização do ROS2 já ocorre via .bashrc
*/
#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h> // Incluido para mandar dados da bateria
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>


// Definição do Display


LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display
//GND do módulo I2C pino GND do ESP32;
//VCC do módulo I2C pino VIN do ESP32;
//SDA do módulo I2C pino D21 (GPIO 21) do ESP32;
//SCL do módulo I2C pino D22 (GPIO 22) da ESP32.
// https://www.blogdarobotica.com/2022/12/23/como-utilizar-o-display-lcd-16x02-com-modulo-i2c-na-esp32/


// Definição para Monitoramento de Bateria
#define Bateria 36


// Definição dos pinos do L298N
#define ENA 14  // PWM Motor A
#define IN1 26  // Direção Motor A
#define IN2 27
#define ENB 25  // PWM Motor B
#define IN3 32  // Direção Motor B
#define IN4 33


// Definição dos pinos dos encoders (apenas um sinal de saída por encoder)
#define ENCODER_A 34
#define ENCODER_B 35
//////////////////////////////////////////////


volatile int encoder_count_A = 0;
volatile int encoder_count_B = 0;


/////////////////////////////////////////////
// Encoder variables
int LeftEncoderCount = 0;
int RightEncoderCount = 0;






// Configuração do WI-FI
char ssid[] = "RAG";
char password[] = "AER101023";
char agent_ip[] = "192.168.1.12";
uint agent_port = 8888;




// Configuração do ROS


rcl_publisher_t battery_publisher;
std_msgs__msg__Int8 battery_msg;


rcl_publisher_t left_encoder_publisher;
rcl_publisher_t right_encoder_publisher;


std_msgs__msg__Int32 left_encoder_msg;
std_msgs__msg__Int32 right_encoder_msg;
/////////////////////
rcl_subscription_t twist_subscriber;
geometry_msgs__msg__Twist twist_msg;


rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;


// Verificação de erro de comunicação
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


//Function prototypes
//void setMotorSpeed(int speedLeft, int speedRight);
int8_t get_battery_percentage();
int map_to_percentage(int raw_value);




void error_loop(){
 while(1){
   delay(100);
 }
}
//////////////////////////////////////////////////////////////////


// Callback dos encoders
void IRAM_ATTR encoder_A_ISR() {
   encoder_count_A++;
}


void IRAM_ATTR encoder_B_ISR() {
   encoder_count_B++;
}


/////////////////////////////////////////////////////////////////////
//Controle do motor pelo teclado
// I= frente linear.x=0.5
// J= esquerda angular.z=1.0
// ,= ré linear.x=-0.5
// L= direita angular.z=-1.0
// K= parado linear.x=0.0 e angular.z=0.0






// Callback para receber comandos do ROS 2
void twist_callback(const void *msg_in) {
   const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msg_in;
  
   float linear = msg->linear.x;
   float angular = msg->angular.z;
   int speed = abs(linear * 255);
   int turn = abs(angular * 255);


   if (linear > 0) {
       digitalWrite(IN1, HIGH);
       digitalWrite(IN2, LOW);
       digitalWrite(IN3, HIGH);
       digitalWrite(IN4, LOW);
       Serial.println("Frente");
   } else if (linear < 0) {
       digitalWrite(IN1, LOW);
       digitalWrite(IN2, HIGH);
       digitalWrite(IN3, LOW);
       digitalWrite(IN4, HIGH);
       Serial.println("Ré");
   } else if (angular > 0) {
       speed = abs(0.45 * 255);
       digitalWrite(IN1, HIGH);
       digitalWrite(IN2, LOW);
       digitalWrite(IN3, LOW);
       digitalWrite(IN4, HIGH);
       Serial.println("Direita");
   } else if (angular < 0) {
       speed = abs(0.45 * 255);
       digitalWrite(IN1, LOW);
       digitalWrite(IN2, HIGH);
       digitalWrite(IN3, HIGH);
       digitalWrite(IN4, LOW);
       Serial.println("Esquerda");
   } else {
       digitalWrite(IN1, LOW);
       digitalWrite(IN2, LOW);
       digitalWrite(IN3, LOW);
       digitalWrite(IN4, LOW);
       Serial.println("Parado");
   }


   analogWrite(ENA, speed);
   analogWrite(ENB, speed);
}
////////////////////////////////////////////////////////////////
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
 RCLC_UNUSED(last_call_time);
 if (timer != NULL) {




   RCSOFTCHECK(rcl_publish(&left_encoder_publisher, &left_encoder_msg, NULL));
   RCSOFTCHECK(rcl_publish(&right_encoder_publisher, &right_encoder_msg, NULL));




  right_encoder_msg.data = encoder_count_A;
  left_encoder_msg.data = -encoder_count_B;
    int8_t battery_percentage = get_battery_percentage();
   battery_msg.data = battery_percentage;
   RCSOFTCHECK(rcl_publish(&battery_publisher, &battery_msg, NULL));
 }
}
int8_t get_battery_percentage() {
 // Read the voltage from the BATTERY_PIN
 int raw_value = analogRead(Bateria);


 // Convert the raw value to battery percentage (0 to 100)
 int battery_percentage = map_to_percentage(raw_value);


 return static_cast<int8_t>(battery_percentage);
}


int map_to_percentage(int raw_value) {
 // Assuming the raw_value represents the battery voltage in the range of 0 to 4095
 // Adjust the following values based on your battery voltage range and voltage divider setup (if any).
 int min_voltage = 2730;    // Minimum voltage reading (corresponding to 0% battery)
 int max_voltage = 3740; // Maximum voltage reading (corresponding to 100% battery)


 // Map the raw value to the battery percentage
 int battery_percentage = map(raw_value, min_voltage, max_voltage, 0, 100);
 return battery_percentage;
}
//////////////////////////////////////////////////////////////////////
void setup() {
   Serial.begin(115200);
   lcd.init();                      // initialize the lcd
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) {
       delay(1000);
   }


   set_microros_wifi_transports(ssid, password, agent_ip, agent_port);
   Serial.println("Conectando ao micro-ROS via Wi-Fi...");
   lcd.setBacklight(255);
   lcd.setCursor(3,0);
   lcd.home();
   lcd.clear();
   lcd.print("Conectado!!!");
   lcd.setCursor(0,1);
   lcd.print("Bateria: ");
  
   pinMode(ENA, OUTPUT);
   pinMode(IN1, OUTPUT);
   pinMode(IN2, OUTPUT);
   pinMode(ENB, OUTPUT);
   pinMode(IN3, OUTPUT);
   pinMode(IN4, OUTPUT);
  
   pinMode(Bateria, INPUT);
   pinMode(ENCODER_A, INPUT_PULLUP);
   pinMode(ENCODER_B, INPUT_PULLUP);
   ////////////////////////////////////////////////////////
   attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoder_A_ISR, RISING);
   attachInterrupt(digitalPinToInterrupt(ENCODER_B), encoder_B_ISR, RISING);
  
   /////////////////////////////////////////////////////////////
   allocator = rcl_get_default_allocator();
  //create init_options
 RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));


 // create node
 RCCHECK(rclc_node_init_default(&node, "CarRAG_esp32", "", &support));


 // Create Subscriber
 RCCHECK(rclc_subscription_init_default(
     &twist_subscriber,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
     "cmd_vel"));
  RCCHECK(rclc_publisher_init_default(
     &battery_publisher,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int8),
     "battery"));


 RCCHECK(rclc_publisher_init_default(
     &left_encoder_publisher,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
     "left_motor_ticks"));


 RCCHECK(rclc_publisher_init_default(
     &right_encoder_publisher,
     &node,
     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
     "right_motor_ticks"));


 // create timer,
 const unsigned int timer_timeout = 100;
 RCCHECK(rclc_timer_init_default(
   &timer,
   &support,
   RCL_MS_TO_NS(timer_timeout),
   timer_callback));


// create executor
 RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
 RCCHECK(rclc_executor_add_timer(&executor, &timer));
 //RCCHECK(rclc_executor_add_subscription(&executor, &LEDs_subscriber, &LEDs_msg, &LEDs_subscription_callback, ON_NEW_DATA));
 //RCCHECK(rclc_executor_add_subscription(&executor, &servo_subscriber, &servo_msg, &servo_callback, ON_NEW_DATA));
 RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));


 left_encoder_msg.data = 0;
 right_encoder_msg.data = 0;


}


void loop() {
   rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
   RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));


   lcd.setCursor(10,1);
   //lcd.print("%d", ); ajustar dado para global
   /////////////////////////////////////////////////////////////////
   /*
   //Serial.print("Encoder A: "); Serial.print(encoder_count_A);
   //Serial.print(" | Encoder B: "); Serial.println(encoder_count_B);
   */
}


/* Este codigo foi modificado
usando referencias para medir a tensão da bateria,
alem de enviar dados referentes ao encoder,
porem não esta completa a etapa de encoder,
necessita ser adequada para um tipo de encoder mais simples HC-20k
*/

