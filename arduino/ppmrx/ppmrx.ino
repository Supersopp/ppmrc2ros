/*
Copyright (C) 2015 Per Magnus Auby <peramgnus@gmail.com github/Supersopp>
License: GPLv3
Code inspired by the MultiWii rx code.

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Use USB serial port
#define USE_USBCON

#include <ros.h>
#include <rcppm2ros/rc_input.h>
#include <stdint.h>

// Pins on port B
#define D8  (1<<4)
#define D9  (1<<5)
#define D10 (1<<6)
#define D11 (1<<7) // Not exposed on Sparkfun Pro Micro
#define D14 (1<<3)
#define D15 (1<<1)
#define D16 (1<<2)
#define D17 (1<<0)  // Not exposed on Sparkfun Pro Micro

int               frameTimeout = 30; //ms
const int8_t      rxPins[]     = {D10,D14,D15,D16};
const uint8_t     rxPinCount   = 4;
volatile uint16_t rxPulseLength[rxPinCount];
uint8_t           rxChannelRecived;
uint8_t           rxChannelActive;
volatile uint16_t edgeToEdgeTime[rxPinCount];

// Setup funcion for pin change interrupt on rx pins.
void configureRX()
{
  for( uint8_t channel = 0; channel < rxPinCount; channel++ ){
    DDRB   &= ~rxPins[channel]; // Set pin as input.
    PORTB  |=  rxPins[channel]; // Enable pullup on pin.
    PCMSK0 |=  rxPins[channel]; // Mask pin for port change interrupt.
    rxChannelActive |= (1<<channel); // Mark channel as active.
  }
  PCICR |= (1<<PCIE0); // Enable port change interrupt.
}

// Interrupt routine for pin change on port B
ISR(PCINT0_vect)
{
  static uint16_t risingEdgeTime[rxPinCount];
  uint16_t        currentTime;
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
	rxChannelRecived |= (1<<channel);
      }
      // Falling edge?
      else{
        edgeToEdgeTime[channel] = currentTime - risingEdgeTime[channel];
        risingEdgeTime[channel] = currentTime;
      }
    }
  }
}

// Declare ROS node, message and publisher.
ros::NodeHandle node;
rcppm2ros::rc_input rc_msg;
ros::Publisher rc_publisher("rc_input", &rc_msg);

void setup(void)
{
  // Initialize ROS node.
  node.initNode();
  
  // Advertise rc_input messages.
  node.advertise(rc_publisher);

  // Configure receiver interrupt.
  configureRX();
  sei();

  // Wait for connection.
  do{
    node.spinOnce();
  }while(!node.connected());

  node.getParam("~frame_timeout", &frameTimeout);
}

void loop(void)
{
  static uint32_t messageDeadline;

  // Only publish when all channels have new data or after timeout
  if( (rxChannelRecived == rxChannelActive) || (messageDeadline < millis()) ){

    // Store value and clear rxChannelRecived.
    uint8_t recivedChannels = rxChannelRecived;
    rxChannelRecived = 0;

    // Store deadline for next message.
    messageDeadline =  millis() + frameTimeout;

    uint16_t rxPulse[rxPinCount];
    bool     rxValid[rxPinCount];
    float    rxValue[rxPinCount];
    uint8_t  rxRate[rxPinCount];
  
    // Copy and calculate values
    for( uint8_t channel = 0; channel < rxPinCount; channel++ ){
      // Was a pulse recived on the channel?
      if( !(recivedChannels & (1<<channel)) ){
        rxValid[channel] = false;
	rxValue[channel] = 0.0;
      }
      else{
        rxRate[channel] = 1e6/edgeToEdgeTime[channel];
        rxPulse[channel] = rxPulseLength[channel];
        // Valid pulse?
        if(900 < rxPulse[channel] && rxPulse[channel] < 2100){
          rxValid[channel] = true;
          rxValue[channel] = ((int)rxPulse[channel] - 1500)/4;
        }
        else{
          rxValid[channel] = false;
          rxValue[channel] = 0.0;
        }
      }
    }

    // Build and publish message.
    rc_msg.pulse_length = rxPulse;
    rc_msg.valid_pulse  = rxValid;
    rc_msg.value        = rxValue;
    rc_msg.rate         = rxRate;

    rc_msg.pulse_length_length = rxPinCount;
    rc_msg.valid_pulse_length  = rxPinCount;
    rc_msg.value_length        = rxPinCount;
    rc_msg.rate_length         = rxPinCount;

    rc_publisher.publish(&rc_msg);
  }
  
  // Spin ROS.
  node.spinOnce();
}

