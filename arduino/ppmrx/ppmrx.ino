

#include <ros.h>
#include <rcppm2ros/rcppm.h>
#include <stdint.h>

// Pins on port B
#define D8      1<<4
#define D9      1<<5
#define D10     1<<6
#define D11     1<<7
#define D14     1<<3
#define D15     1<<1
#define D16     1<<2
#define D17     1<<0

uint8_t           rxPins[]   = {D10,D14,D15,D16};
const uint8_t     rxPinCount = 4;
volatile uint16_t rxPulseLength[rxPinCount];

// Setup funcion for pin change interrupt on rx pins.
void configureRX()
{
  for( uint8_t channel = 0; channel < rxPinCount; channel++ ){
    DDRB   &= ~rxPins[channel]; // Set pin as input.
    PORTB  |=  rxPins[channel]; // Enable pullup on pin.
    PCMSK0 |=  rxPins[channel]; // Mask pin for port change interrupt.
  }
  PCICR |= (1<<PCIE0); // Enable port change interrupt.
}

// Interrupt routine for pin change on port B
ISR(PCINT0_vect)
{
  static uint16_t risingEdgeTime[rxPinCount];
  uint16_t        currentTime;
  uint16_t        pulseLength;
  uint8_t         port;
  static uint8_t  portLast;
  uint8_t         portChange;
  
  // Find out which pins changed.
  port       = PINB;
  portChange = port^portLast;
  
  // Store the current time and re-enable interrupts.
  currentTime = micros();
  sei();
  
  // Store current state of the port.
  portLast = port;
 
  // Check all pins for change.
  for( uint8_t channel = 0; channel < rxPinCount; channel++ ){
    // Did the pin change?
    if( portChange & rxPins[channel] ){
      // Rising edge?
      if( !(port & rxPins[channel]) ){
        rxPulseLength[channel] = currentTime - risingEdgeTime[channel];
      }
      // Falling edge?
      else{
        risingEdgeTime[channel] = currentTime;
      }
    }
  }
}

// Declare ROS node, message and publisher.
ros::NodeHandle node;
rcppm2ros::rcppm rcppm_msg;
ros::Publisher rcppm_publisher("rcppm", &rcppm_msg, 10);

void setup(void)
{
  // Configure receiver interrupt.
  configureRX();
  sei();
  
  // Initialize ROS node.
  node.initNode();
  
  // Wait for connection.
  while(!node.connected()){
    node.spinOnce();
  }
  
  // Advertise rcppm messages.
  node.advertise(rcppm_publisher);
}

void loop(void)
{
  // Build and publish message.
  for( uint8_t channel = 0; channel < rxPinCount; channel++ ){
    rcppm_msg.channel[channel] = rxPulseLength[channel];
  }
  rcppm_msg.channel_length = rxPinCount;
  rcppm_publisher.publish(&rcppm_msg);
  
  // Spin ROS.
  node.spinOnce();
}

