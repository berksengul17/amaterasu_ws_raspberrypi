#ifndef SINGLE_PIN_ENCODER_HPP
#define SINGLE_PIN_ENCODER_HPP

#include <stdint.h>

typedef void (*encoder_callback_t)(int);

class single_pin_encoder
{
   int mygpio, lastLevel;

   encoder_callback_t mycallback;

   void _pulse(int gpio, int level, uint32_t tick);

   /* Need a static callback to link with C. */
   static void _pulseEx(int gpio, int level, uint32_t tick, void *user);

   public:

   single_pin_encoder(int gpio, encoder_callback_t callback);
   /*
      This function establishes an encoder on the specified GPIO pin.

      When the encoder detects a pulse, the callback function is called.
   */

   void cancel(void);
   /*
      This function releases the resources used by the encoder.
   */
};

class dual_encoder
{
   single_pin_encoder encoder1;
   single_pin_encoder encoder2;

   public:

   dual_encoder(int gpio1, encoder_callback_t callback1, int gpio2, encoder_callback_t callback2);
   /*
      This function establishes two single-pin encoders.

      When either encoder detects a pulse, their respective callback function is called.
   */

   void cancel(void);
   /*
      This function releases the resources used by both encoders.
   */
};

#endif
