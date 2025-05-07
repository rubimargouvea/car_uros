// https://github.com/sam-tj/freertos_apps/blob/hardware_example/apps/twist_joystick/app.c
//link onde retirei a ideia para a criação do joystick


#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int8.h> // Incluido para mandar dados da bateria
#include <std_msgs/msg/int32.h>
#include <geometry_msgs/msg/twist.h>

//Definir duas variaveis para portas analogicas
#define horizontal 35
#define vertical 34
//Fazer config de bits da porta

// Verificação de erro de comunicação
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


//initialização
rclc_executor_t executor;
rcl_allocator_t allocator;
rclc_support_t support;
rcl_timer_t timer;
rcl_publisher_t publisher;
rcl_node_t node;


void error_loop(){
 while(1){
   delay(100);
 }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
 RCLC_UNUSED(last_call_time);
 if (timer != NULL) {
    //initialization of twist msg
		geometry_msgs__msg__Twist twist;
		geometry_msgs__msg__Twist__init(&twist);
    // read values
		int x_axis_read = analogRead(vertical) * 1.15 ;	// 1.15 simulação necessita ser testado 
		int y_axis_read = analogRead(horizontal) * 1.15 ;	// 1.15 simulação necessita ser testado 
		// Conversão do valor para variação de velocidade min 65 max 255
    twist.linear.x = map(x_axis_read, 0, 511, 65, 255);
    twist.angular.z = map(y_axis_read, 0, 511, 65, 255);
    // print values in serial monitor 
		//printf("analog read x= %d \t y= %d \n", x_axis_read , y_axis_read );
		//printf("twist out x= %f \t z= %f \n", twist.linear.x , twist.angular.z );
		
		RCSOFTCHECK(rcl_publish(&publisher, &twist, NULL));		
	}	
}
void setup() {
  Serial.begin(115200);
  pinMode(vertical, INPUT);
  pinMode(horizontal, INPUT);

  allocator = rcl_get_default_allocator();
  // create init_options
	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	
	RCCHECK(rclc_node_init_default(&node, "freertos_twist_pub", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
		"/CarRAG_esp32/cmd_vel"));
	
	// create timer,

	const unsigned int timer_timeout = 500;  //publish timeout
	RCCHECK(rclc_timer_init_default(
		&timer,
		&support,
		RCL_MS_TO_NS(timer_timeout),
		timer_callback));

	// create executor
	
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

}
