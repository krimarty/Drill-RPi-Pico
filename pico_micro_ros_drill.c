#include <stdio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/u_int8.h>
#include <std_msgs/msg/int16.h>
#include <std_msgs/msg/int64_multi_array.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "storage_driver.h"
#include "linear_driver.h"
#include "motor_driver.h"
#include "math.h"

#define I2C_PORT i2c0

struct storage storage;
struct linear linear;
struct motor motor;
int counter;

const uint LED_PIN = 25;

rcl_subscription_t array_subscriber;
std_msgs__msg__Int64MultiArray* msg;

rcl_publisher_t publisher;
std_msgs__msg__Int16 weight;
rcl_subscription_t subscriber_linear_state;
std_msgs__msg__UInt8 linear_sta;

rcl_subscription_t subscriber_linear_speed;
std_msgs__msg__UInt8 linear_spe;

rcl_subscription_t subscriber_motor_state;
std_msgs__msg__UInt8 motor_sta;

rcl_subscription_t subscriber_motor_speed;
std_msgs__msg__UInt8 motor_spe;

rcl_subscription_t subscriber_storage_command;
std_msgs__msg__UInt8 storage_com;


void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    //if (storage_read(&storage) < 0)
    //if (motor_read(&motor) < 0)
    if (linear_read(&linear) < 0)
    {
        weight.data = counter;
        rcl_ret_t ret = rcl_publish(&publisher, &weight, NULL);
    }
    else
    {
    //weight.data = storage.raw;
    //weight.data = storage.command;
    weight.data = storage.weight;
    //weight.data = motor.direction;
    rcl_ret_t ret = rcl_publish(&publisher, &weight, NULL);
    }
    
}

void array_callback(const void* msgin)
{
    const std_msgs__msg__Int64MultiArray * msg = (const std_msgs__msg__Int64MultiArray *)msgin;
    counter++;
    
}

void linear_state_callback(const void* msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    linear.command = msg->data;
    linear_read(&linear);
    if (linear.states == 1 && msg->data == 1) //kdyz bude uplne nahore a zaroven se macka jed nahoru
    {
        linear.command = 4; //wait untill 
    }
    //storage_write(&storage);
}

void linear_speed_callback(const void* msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    linear.speed = msg->data;
    //storage_write(&storage);
}

void motor_state_callback(const void* msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    motor.direction = msg->data;
    //storage_write(&storage);
}

void motor_speed_callback(const void* msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    motor.torque = msg->data;
    //storage_write(&storage);
}

void storage_command_callback(const void* msgin)
{
    const std_msgs__msg__UInt8 * msg = (const std_msgs__msg__UInt8 *)msgin;
    uint8_t old_command = storage.command;
    storage.command = msg->data;
    if (old_command != storage.command && storage.command > 9) {storage_write(&storage);}
}


int main()
{
    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    //Allocation for ros array
    msg->data.capacity = 100; 
    msg->data.size = 0;
    msg->data.data = (int64_t*) malloc(msg->data.capacity * sizeof(int64_t));

    msg->layout.dim.capacity = 100;
    msg->layout.dim.size = 0;
    msg->layout.dim.data = (std_msgs__msg__MultiArrayDimension*) malloc(msg->layout.dim.capacity * sizeof(std_msgs__msg__MultiArrayDimension));

    for(size_t i = 0; i < msg->layout.dim.capacity; i++){
        msg->layout.dim.data[i].label.capacity = 20;
        msg->layout.dim.data[i].label.size = 0;
        msg->layout.dim.data[i].label.data = (char*) malloc(msg->layout.dim.data[i].label.capacity * sizeof(char));
    }
    //

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int16),
        "sample_weight");

    rclc_subscription_init_default(
        &array_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int64MultiArray),
        "array");

    //init subscriber
    rcl_ret_t rc = rclc_subscription_init_default(
        &subscriber_linear_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "linear_state");

    rcl_ret_t rc1 = rclc_subscription_init_default(
        &subscriber_linear_speed, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "linear_speed");

    rcl_ret_t rc2 = rclc_subscription_init_default(
        &subscriber_motor_state, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "motor_state");

    rcl_ret_t rc3 = rclc_subscription_init_default(
        &subscriber_motor_speed, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "motor_speed");

    rcl_ret_t rc4 = rclc_subscription_init_default(
        &subscriber_storage_command, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt8),
        "storage_command");

    rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback);

    rclc_executor_init(&executor, &support.context, 7, &allocator); //the number of executors
    rclc_executor_add_timer(&executor, &timer);
    //init executor for subscriber
    rclc_executor_add_subscription(
        &executor, &array_subscriber, &msg,
        &array_callback, ON_NEW_DATA);

    rc = rclc_executor_add_subscription(
        &executor, &subscriber_linear_state, &linear_sta,
        &linear_state_callback, ON_NEW_DATA);

    rc1 = rclc_executor_add_subscription(
        &executor, &subscriber_linear_speed, &linear_spe,
        &linear_speed_callback, ON_NEW_DATA);

    rc2 = rclc_executor_add_subscription(
        &executor, &subscriber_motor_state, &motor_sta,
        &motor_state_callback, ON_NEW_DATA);

    rc3 = rclc_executor_add_subscription(
        &executor, &subscriber_motor_speed, &motor_spe,
        &motor_speed_callback, ON_NEW_DATA);

    rc4 = rclc_executor_add_subscription(
        &executor, &subscriber_storage_command, &storage_com,
        &storage_command_callback, ON_NEW_DATA);


    //init i2c
    i2c_init(I2C_PORT, 400000);
    gpio_set_function(4, GPIO_FUNC_I2C);
    gpio_set_function(5, GPIO_FUNC_I2C);
    gpio_pull_up(4);
    gpio_pull_up(5);


    gpio_put(LED_PIN, 1);

    storage_init(&storage);
    linear_init(&linear);
    motor_init(&motor);

    while (true)
    {
        sleep_ms(10);
        motor_write(&motor);
        sleep_ms(10);
        linear_write(&linear);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
