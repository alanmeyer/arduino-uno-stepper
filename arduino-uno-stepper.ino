//--------------------------------------------------------------------------------------------------
// arduino-uno-stepper
// 2024-01-21
//
// based on the arduino uno atmega328p
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// config
// values in this section are expected to
// be changed and by user preferences
//--------------------------------------------------------------------------------------------------
#define    MY_PROGRAM           "stepper"
#define    MY_VERSION           "1.0"                    // version string


//--------------------------------------------------------------------------------------------------
// debug
//--------------------------------------------------------------------------------------------------
// section for debugging options


//--------------------------------------------------------------------------------------------------
// includes
//--------------------------------------------------------------------------------------------------
#include <Wire.h>
#include <GOFi2cOLED.h>
#include <avr/pgmspace.h>


//--------------------------------------------------------------------------------------------------
// types
//--------------------------------------------------------------------------------------------------
#define    BOOL            unsigned char
#define    UINT8           unsigned char
#define    UINT16          unsigned int
#define    UINT32          unsigned long
#define    UINT64          unsigned double
#define    SINT8           signed char
#define    SINT16          signed int
#define    SINT32          signed long
#define    SINT64          signed double


//--------------------------------------------------------------------------------------------------
// system defines
//
// note: use of "L" suffix required for
// pre-processor math operations > 16 bit
//--------------------------------------------------------------------------------------------------
#define    CPU_CLOCK            16000000L               // uno runs at 16MHz
#define    T_USEC_PER_SEC       1000000L                // microseconds per second
#define    T_USEC_PER_MSEC      1000L                   // microseconds per millesecond
#define    T_100USEC_PER_SEC    (T_USEC_PER_SEC/100)    // 100 microseconds per second
#define    T_100USEC_PER_MSEC   10L                     // 100 microseconds per millisecond
#define    T_USEC_PER_100USEC   100L                    // microseconds per 100 microseconds
#define    TRUE                 1
#define    FALSE                0


//--------------------------------------------------------------------------------------------------
// macros
//--------------------------------------------------------------------------------------------------
// bit operations
#define    BMSK(x)         (1<<x)                       // converts bit number to a mask for bit set/clr
#define    BSET(reg,x)     reg |= (BMSK(x))             // register bit set
#define    BCLR(reg,x)     reg &= (~BMSK(x))            // register bit clear
#define    BTGL(reg,x)     reg ^= (BMSK(x))             // register bit toggle
#define    BGETM(name)     ((name##_I & BMSK(name##_B)) \
                           >> name##_B)                 // get port pin by root name only
#define    BSETM(name)     BSET(name##_O,name##_B)      // port pin = high
#define    BCLRM(name)     BCLR(name##_O,name##_B)      // port pin = low
#define    BTGLM(name)     BTGL(name##_O,name##_B)      // port pin = ~Port pin
#define    BSETDM(name)    BSET(name##_D,name##_B)      // port direction = output
#define    BCLRDM(name)    BCLR(name##_D,name##_B)      // port direction = input

// ascii & string conversion
#define    CHAR_TO_INT(c)  (c - '0')                    // convert an ASCII character to a number
#define    INT_TO_CHAR(i)  (i + '0')                    // convert a number to an ASCII character
#define    HEX_TO_CHAR(x)  (((x) < 10) ? (x+'0'): \
                           (x-10+'A'))                  // convert a hex value to an ASCII character
#define    STRINGIFY(x)          #x                     // Convert a constant (number) to ASCII character
#define    CONST_TO_CHAR(x)      STRINGIFY(x)           // note: preprocessor requires indirection here

// interrupts
#define    INTERRUPTS_DISABLE    cli()                  // global interrupt disable
#define    INTERRUPTS_ENABLE     sei()                  // global interrput enable
#define    INTERRUPT_T0_DISABLE  BCLR(TIMSK0, OCIE0A)   // timer 0 disable
#define    INTERRUPT_T0_ENABLE   BSET(TIMSK0, OCIE0A)   // timer 0 enable

// min/max
#define    MIN(a, b)             ({ __typeof__ (a) _a = (a); \
                                    __typeof__ (b) _b = (b); \
                                    _a < _b ? _a : _b; })

#define    MAX(a, b)             ({ __typeof__ (a) _a = (a); \
                                    __typeof__ (b) _b = (b); \
                                    _a > _b ? _a : _b; })

     
//--------------------------------------------------------------------------------------------------
// port pin defines
//--------------------------------------------------------------------------------------------------
//  port B                   port C                port D
//   B0                       C0                    D0 RXD
//   B1                       C1                    D1 TXD
//   B2                       C2                    D2
//   B3                       C3                    D3
//   B4                       C4 SDA                D4
//   B5                       C5 SCL                D5
//                                                  D6
//                                                  D7
//
// atmel atmega328 pin names
//   use the following conventions:
//   _O = output register
//   _I = input register
//   _D = direction register
//   _B = bit number

// motor controller pins A1/A2 B1/B2
#define     M1A_O        PORTC
#define     M1A_I        PINC
#define     M1A_D        DDRC
#define     M1A_B        2

#define     M1B_O        PORTC
#define     M1B_I        PINC
#define     M1B_D        DDRC
#define     M1B_B        3

#define     M2A_O        PORTC
#define     M2A_I        PINC
#define     M2A_D        DDRC
#define     M2A_B        1

#define     M2B_O        PORTC
#define     M2B_I        PINC
#define     M2B_D        DDRC
#define     M2B_B        0

#define     M1S          BCLRM(M1A); BCLRM(M1B)
#define     M1F          BSETM(M1A); BCLRM(M1B)
#define     M1R          BCLRM(M1A); BSETM(M1B)
#define     M2S          BCLRM(M2A); BCLRM(M2B)
#define     M2F          BSETM(M2A); BCLRM(M2B)
#define     M2R          BCLRM(M2A); BSETM(M2B)
#define     MSTOP        M1S; M2S

#define     MOTOR_IDLE   50


// encoder pins
#define     ENCPUSH_O    PORTD
#define     ENCPUSH_I    PIND
#define     ENCPUSH_D    DDRD
#define     ENCPUSH_B    6

#define     ENC0_O       PORTD
#define     ENC0_I       PIND
#define     ENC0_D       DDRD
#define     ENC0_B       5

#define     ENC1_O       PORTD
#define     ENC1_I       PIND
#define     ENC1_D       DDRD
#define     ENC1_B       4

#define     ENC_MIN      0
//#define     ENC_MAX      255L
#define     ENC_MAX      100000L
#define     ENC_STEP     1

#define     ENCPUSH_UP   TRUE
#define     ENCPUSH_DOWN FALSE

#define     ENC_OFF      FALSE
#define     ENC_ON       TRUE
#define     ENC_CW       FALSE
#define     ENC_CCW      TRUE

#define     DEBOUNCE     4
#define     HOLD_COUNT   40


//--------------------------------------------------------------------------------------------------
// defines
//--------------------------------------------------------------------------------------------------
// interrupt timer
#define     INTERRUPT_PERIOD_US     20                          // interrupt period
#define     INTERRUPT_FREQ_HZ       (T_USEC_PER_SEC / \
                                     INTERRUPT_PERIOD_US)
#define     INTERRUPTS_PER_MS       (T_USEC_PER_MSEC / \
                                     INTERRUPT_PERIOD_US)       // number of interrupts per msec

// main loop timer
#define     LOOP_4MS        4
#define     LOOP_4MS_COMP   (LOOP_4MS >> 1)
#define     LOOP_4MS_MASK   (LOOP_4MS - 1)
#define     LOOP_8MS        8
#define     LOOP_8MS_COMP   (LOOP_8MS >> 1)
#define     LOOP_8MS_MASK   (LOOP_8MS - 1)
#define     LOOP_16MS       16
#define     LOOP_16MS_COMP  (LOOP_16MS >> 1)
#define     LOOP_16MS_MASK  (LOOP_16MS - 1)
#define     LOOP_32MS       32
#define     LOOP_32MS_COMP  (LOOP_32MS >> 1)
#define     LOOP_32MS_MASK  (LOOP_32MS - 1)
#define     LOOP_64MS       64
#define     LOOP_64MS_COMP  (LOOP_64MS >> 1)
#define     LOOP_64MS_MASK  (LOOP_64MS - 1)
#define     LOOP_128MS      128
#define     LOOP_128MS_COMP (LOOP_128MS >> 1)
#define     LOOP_128MS_MASK (LOOP_128MS - 1)

#define     MODE_FOLLOW     0
#define     MODE_CHASE      1
#define     MODE_CYCLE      2
#define     MODE_ENCODER    3
#define     MODE_DEFAULT    MODE_FOLLOW
//#define     MODE_DEFAULT    MODE_CHASE
//#define     MODE_DEFAULT    MODE_CYCLE


#define     CYCLE_DEFAULT   200 // the default # of motor values to cycle
#define     CYCLE_DELAY     64  // count to pause before cycle change

// performance counters
#define     PERFORMANCE
#define     PERF_COUNT_BITS     5
#define     PERF_COUNTS         (1 << (PERF_COUNT_BITS-1))
#define     PERF_COUNT_MASK     (PERF_COUNTS-1)

// debug
//#define     DEBUG           FALSE
#define     DEBUG           TRUE
#define     DEBUG_DEFAULT   0x80000000

// chase performance
#define     CHASE_SLOW      1000  // threshold for chase count too far behind hard counter
#define     CHASE_OK        10    // threshold for chase count returned ok (hysteresis)


//--------------------------------------------------------------------------------------------------
// structures
//--------------------------------------------------------------------------------------------------

typedef struct {
  UINT16   time_secs = 0;
  UINT16   time_ms = 0;
  UINT16   chase_time_ms = 0;
  UINT16   chase_behind_ms = 0;
  UINT8    time_4ms = 0;
  UINT8    time_100ms = 0;
  BOOL     busy = FALSE;
  UINT8    mode = MODE_DEFAULT;
} system_t;

typedef struct {
  BOOL     enable = DEBUG;
  UINT32   value  = DEBUG_DEFAULT;
} debug_t;

typedef struct {
  BOOL        enable = TRUE;  // flag to disable display updates
  BOOL        update = FALSE; // flag to request a display update
  GOFi2cOLED  oled;
} display_t;

typedef struct {
  UINT16     max = 0;
  UINT16     avg = 0;
} performance_t;

typedef struct {
  BOOL     scan;      // a copy of the most recent scan results
  BOOL     state;     // switch is up/down or open/closed
  BOOL     snew;      // new flag
  UINT8    debounce;  // debounce counter
  BOOL     ignore_up; // ignore next up event
  BOOL     hnew;      // hold new flag
  UINT16   hcount;    // hold counter
} switch_t;

typedef struct {
  switch_t push;
  switch_t enc0;
  switch_t enc1;
  UINT32   min = ENC_MIN;
  UINT32   max = ENC_MAX;
  UINT16   value = ENC_MIN;
  UINT8    debounce = 0;
  UINT16   step = ENC_STEP;
  BOOL     dir = ENC_CW;
  BOOL     snew = FALSE;
  UINT8    laststate;
  UINT8    state;
  BOOL     busy = FALSE;
} encoder_t;

typedef struct {
  BOOL     enable = TRUE;
  BOOL     dir = FALSE;
  UINT8    speed = 1;
  UINT16   position = 0;
  UINT16   target = 0;
  BOOL     busy = FALSE;
  BOOL     idle = FALSE;
  UINT8    state = 0;
  UINT8    save_state = 0;
} motor_t;


//--------------------------------------------------------------------------------------------------
// globals
//--------------------------------------------------------------------------------------------------
volatile system_t       g_sys;
volatile debug_t        g_debug;
volatile performance_t  g_performance;
volatile encoder_t      g_encoder;
volatile motor_t        g_motor;
volatile display_t      g_disp;


//--------------------------------------------------------------------------------------------------
// routines
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// routine automatically called by the system at startup
// use this routine to initalize our system
//--------------------------------------------------------------------------------------------------
void setup() {
  INTERRUPT_T0_DISABLE;       // disable timer 0 interrupt

  // oled 128x64
  // default address is 0x3C.
  g_disp.oled.init(0x3C);
  g_disp.oled.clearDisplay();
  g_disp.oled.setTextSize(1);
  g_disp.oled.setTextColor(WHITE);
  display_update();

  // encoder
  switch (g_sys.mode) {
    case (MODE_CYCLE):
      encoder_init(ENC_MIN, ENC_MAX, ENC_STEP, CYCLE_DEFAULT);
      break;
    default:
      encoder_init(ENC_MIN, ENC_MAX, ENC_STEP, ENC_MIN);
      break;
  }
  
  // motor
  motor_init();

  // do this last
  timer_setup();              // init our timer interrupt
  INTERRUPT_T0_ENABLE;        // enable timer 0 interrupt
}


//--------------------------------------------------------------------------------------------------
// display a message on the oled
//--------------------------------------------------------------------------------------------------
void display_update() {
  static UINT16 target = 0;
  static UINT16 position = 0;
  static UINT16 value = 0;


  if (g_disp.enable) {
    g_disp.oled.clearDisplay();
    g_disp.oled.setCursor(0,0);
    g_disp.oled.print("project: "); g_disp.oled.print(MY_PROGRAM) ; g_disp.oled.print(" v"); g_disp.oled.println(MY_VERSION); 

    // mode
    g_disp.oled.print("mode   : "); 
    switch (g_sys.mode) {
      case 0:  g_disp.oled.println("follow");        break;
      case 1:  g_disp.oled.println("chase");         break;
      case 2:  g_disp.oled.println("cycle");         break;
      default: g_disp.oled.println("<undefined>");   break;
    }

    g_disp.oled.print("time   : "); g_disp.oled.println(g_sys.time_secs); 
    g_disp.oled.print("idle   : "); g_disp.oled.print(g_performance.max); g_disp.oled.print(" "); g_disp.oled.println(g_performance.avg);

    // prevent timer interrput conflict
    g_encoder.busy = TRUE; value = g_encoder.value; g_encoder.busy = FALSE;
    g_disp.oled.print("encoder: "); g_disp.oled.println(g_encoder.value);
    g_disp.oled.print("step:  : "); g_disp.oled.println(g_encoder.step);

    // get the target and position values
    // prevent timer interrput conflict
    g_motor.busy = TRUE;
    target = g_motor.target;
    position = g_motor.position;
    g_motor.busy = FALSE;

    if (g_motor.enable) { g_disp.oled.print("motor  : "); g_disp.oled.print(target); g_disp.oled.print(" "); g_disp.oled.println(position); }
    else                { g_disp.oled.print("motor d: "); g_disp.oled.print(target); g_disp.oled.print(" "); g_disp.oled.println(position); }

    if (g_debug.enable) {
      g_debug.value = DEBUG_DEFAULT | g_motor.idle;
      g_disp.oled.print("debug  : "); g_disp.oled.print("0x"); g_disp.oled.println(g_debug.value, HEX); 
    }

    g_disp.oled.display();
  }
}


//--------------------------------------------------------------------------------------------------
// check for display update
//--------------------------------------------------------------------------------------------------
void display_check_update() {
  if (g_disp.update) {
    g_disp.update = false;
    display_update();
  }
}


//--------------------------------------------------------------------------------------------------
// this is the main() that runs after setup()
//--------------------------------------------------------------------------------------------------
void loop() {
  static UINT16 idle_counter = 0;
  UINT16 time_ms;
  
  
  // check for time update and run timed events as needed
  g_sys.busy = TRUE; time_ms = g_sys.time_ms; g_sys.busy = FALSE;
  if (g_sys.chase_time_ms != time_ms) {
    g_sys.chase_time_ms++;

    // get difference but check for wrap
    if (time_ms > g_sys.chase_time_ms) g_sys.chase_behind_ms = (time_ms - g_sys.chase_time_ms);
        
    // Use 2^n time values to reduce comparison overhead
    if ((g_sys.chase_time_ms & LOOP_4MS_MASK )  == LOOP_4MS_COMP )   loop_4ms();
    if ((g_sys.chase_time_ms & LOOP_16MS_MASK)  == LOOP_16MS_COMP)   loop_16ms();

    // optional microcontroller performance measurements
    #ifdef PERFORMANCE
    measure_performance(idle_counter);
    #endif

    idle_counter = 0;
  }

  // check if we are really far behind
  // if so, temporarily disable anything
  // that is slow here (like the display)
  if (g_disp.enable) {
    if (g_sys.chase_behind_ms > CHASE_SLOW) {
      g_disp.enable = FALSE;
    }
  }
  else {
    if (g_sys.chase_behind_ms < CHASE_OK) {
      g_disp.enable = TRUE;
    }
  }

  // run any remaing tasks after foreground timer tasks
  // do stuff here

  // track our idle time as a way to tell how busy the system is
  idle_counter++;
}


//--------------------------------------------------------------------------------------------------
// a helper routine to calculate some performance metrics
// useful to see how much of the cpu bandwith is being used
//--------------------------------------------------------------------------------------------------
#ifdef PERFORMANCE
void measure_performance(UINT16 count) {
  static UINT16 counts[PERF_COUNTS] = {0};
  static UINT16 count_index;
  static UINT16 acc = 0;

  // calc performance metrics
  g_performance.max = MAX(count, g_performance.max);

  // update the accumulator by subtracting
  // out the oldest and adding newest
  acc -= counts[count_index];
  counts[count_index] = count;
  acc += count;

  // moving average is taken by the sum of all
  // shifted to perform a fast divide by 2^n
  g_performance.avg = (acc >> (PERF_COUNT_BITS-1)); 

  // update our index, wrap if necessary
  count_index++;
  count_index &= PERF_COUNT_MASK;
}
#endif


//--------------------------------------------------------------------------------------------------
// run this routine every 4 ms (on average)
// note: use 4ms to update 100ms counter
//       because 4ms is evenly divisble and doesn't
//       need to run as often in 1ms
// important note: minmize work here since it needs to complete in < 4ms
//--------------------------------------------------------------------------------------------------
void loop_4ms() {
  static UINT16 value = 0;
  static UINT16 target = 0;
  static UINT8 counter4ms = 0;
  static UINT8 counter100ms = 0;

  // consume if the encoder has been rotated  
  if (g_encoder.snew) {
    g_encoder.snew = FALSE;
    g_disp.update = true;
  }

  // 100ms loop
  if ((++counter4ms) >= 25) {
    counter4ms -= 25;
    if ((++counter100ms) >= 10) {
      counter100ms -= 10;  // subtract here to chase if behind
      g_sys.time_secs++;
      loop_1sec();
    }
  }
}


//--------------------------------------------------------------------------------------------------
// run this routine every 16 ms (on average)
//--------------------------------------------------------------------------------------------------
void loop_16ms() {
  static UINT16 value = 0;
  static UINT16 target = 0;

  // check for new encoder push button hold
  if (switch_check_hold(&g_encoder.push)) {
    g_encoder.push.hnew = FALSE;
    switch (g_sys.mode) {
      case (MODE_FOLLOW):  g_sys.mode = MODE_CHASE;   break;
      case (MODE_CHASE) :  g_sys.mode = MODE_CYCLE;   break;
      case (MODE_CYCLE) :  g_sys.mode = MODE_FOLLOW;  break;
      default           :  g_sys.mode = MODE_DEFAULT; break;
    }
  }

  // check for new encoder push button event
  if (g_encoder.push.snew) {
    g_encoder.push.snew = FALSE; // consume the new flag
    if (g_encoder.push.state == ENCPUSH_UP) {
      if (g_encoder.push.ignore_up) {
        g_encoder.push.ignore_up = FALSE;
      }
      else {
        g_encoder.busy = TRUE;
        switch (g_encoder.step) {
          case (1)   : g_encoder.step = 10; ; break;
          case (10)  : g_encoder.step = 100 ; break;
          case (100) : g_encoder.step = 1000; break;
        //case (1000): g_encoder.step = 1;    break;
          default    : g_encoder.step = 1;    break;
        }
        g_encoder.busy = FALSE;
        g_disp.update = true;
      }
    }
  }

  // consume if the encoder has been rotated  
  if (g_encoder.snew) {
    g_encoder.snew = FALSE;
    g_disp.update = true;
  }

  chase_handler();
  cycle_handler();

  display_check_update();

}

//--------------------------------------------------------------------------------------------------
// run this routine every second (on average)
//--------------------------------------------------------------------------------------------------
void loop_1sec() {
  static UINT8 count = 0;
  // display update request
  // for the time counter
  g_disp.update = true;
}


//--------------------------------------------------------------------------------------------------
// motor chases the encoder but outside of the interrupt loop
// (expect delay / jitter)
//--------------------------------------------------------------------------------------------------
void chase_handler() {
  if (g_sys.mode == MODE_CHASE) {
    g_motor.busy == TRUE;
    g_encoder.busy == TRUE;
    g_motor.target = g_encoder.value;
    g_motor.busy == FALSE;
    g_encoder.busy == FALSE;
  }
}

//--------------------------------------------------------------------------------------------------
// cycle the motor based on the encoder value
//--------------------------------------------------------------------------------------------------
void cycle_handler() {
  static UINT16 delay = 0;

  if (g_sys.mode == MODE_CYCLE) {
    g_motor.busy = TRUE;
    g_encoder.busy = TRUE;

    // wait for the position to catch up with the target
    if (g_motor.position == g_motor.target) {
      if (delay == 0) {
        // toggle the target
        if (g_motor.target == 0) g_motor.target = g_encoder.value;
        else                     g_motor.target = 0;
        delay = CYCLE_DELAY;
      }
      else {
        delay--;
      }
    }

    g_motor.busy = FALSE;
    g_encoder.busy = FALSE;
  }
}


//--------------------------------------------------------------------------------------------------
// check if a switch has been pushed and held
//--------------------------------------------------------------------------------------------------
BOOL switch_check_hold(switch_t* sw) {
  static BOOL state = FALSE;

  if (sw->state == ENCPUSH_UP) {
    sw->hcount = 0;
    state = FALSE;
  }
  else if ((!sw->hnew) && (sw->state == ENCPUSH_DOWN) && (state == FALSE)) {
    if (++sw->hcount > HOLD_COUNT) {
      sw->hnew = TRUE;
      state = TRUE;

      // ignore the next up event
      g_encoder.push.ignore_up = TRUE;
    }
  }

  return(sw->hnew);
}


//--------------------------------------------------------------------------------------------------
// switch init
//--------------------------------------------------------------------------------------------------
void switch_init(switch_t* sw, UINT8 state = 0) {
  sw->state = state;
  sw->snew = FALSE;
  sw->debounce = 0;
  sw->hnew = FALSE;
  sw->hcount = 0;
  sw->ignore_up = FALSE;
}

//--------------------------------------------------------------------------------------------------
// init the encoder at startup or re-init the encoder 
// during operation to change the min, max, step size
//--------------------------------------------------------------------------------------------------
void encoder_init(UINT32 min, UINT32 max, UINT16 step, UINT32 value) {
  static BOOL portinit = FALSE;
  BOOL check = TRUE;

  // only run this part once
  if (!portinit) {
    portinit = TRUE;

    // encoder push button
    BCLRDM(ENCPUSH); // set switch as input
    BSETM(ENCPUSH);  // set pull-up

    // encoder rotary switches
    BCLRDM(ENC0);
    BSETM(ENC0);
    BCLRDM(ENC1);
    BSETM(ENC1);

    // init the switches
    switch_init(&g_encoder.push, ENCPUSH_UP);
    switch_init(&g_encoder.enc0, ENC_OFF);
    switch_init(&g_encoder.enc1, ENC_OFF);
    
    g_encoder.step = ENC_STEP;
  }

  // rule checking
  if (min >= max)  check = FALSE;
  if (value < min) check = FALSE;
  if (value > max) check = FALSE;
  if (check) {
    g_encoder.min = min;
    g_encoder.max = max;
    g_encoder.step = step;
    g_encoder.value = value;
  }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// use a countdown to check for repeated stability
// before updating the switch state to the new value
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void switch_debounce(switch_t * sw) {
  if (sw->debounce) {
    sw->debounce--;
    sw->scan = sw->state;
  }
  else {
    if (sw->scan != sw->state) {
      sw->state = sw->scan;
      sw->snew = TRUE;
      sw->debounce = DEBOUNCE;
    }
  }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// raw scan the switches and debounce as necessary
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void encoder_scan() {

  // push switch
  g_encoder.push.scan = BGETM(ENCPUSH);
  switch_debounce(&g_encoder.push);

  // rotary encoder switches
  // create a 2-bit state variable with the last state
  g_encoder.laststate = (g_encoder.enc1.state << 1) ^ (g_encoder.enc0.state);

  // check encoder 0
  g_encoder.enc0.scan = BGETM(ENC0);
  switch_debounce(&g_encoder.enc0);

  // check encoder 1
  g_encoder.enc1.scan = BGETM(ENC1);
  switch_debounce(&g_encoder.enc1);

  // now create a 2-bit state variable with the current state
  g_encoder.state = ((g_encoder.enc1.scan << 1) ^ g_encoder.enc0.scan);

}  


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// call this periodically to scan the encoder for changes (e.g. 4ms)
//
// there are 3 switches associated with the encoder
// switch 1: push switch
// switch 2 & 3: encoder switches
//
// for the encoder, the switches have quadature encoding meaning that as the user rotates 
// the encoder each of the 2 switches will toggle off and on, offset from each other 
// the offset will be positive or negative depending on the direction being turned
// note that the switch bits only change one at a time, which is a gray code
//
// we can see how the switch bits change depending on the rotatation
//
// clockwise rotation
//   switch state    encoder bit 1   encoder bit 0
//        t0               0               0
//        t1               1               0
//        t2               1               1
//        t3               0               1
//
// counter-clockwise rotation
//   switch state    encoder bit 1   encoder bit 0
//        t0               0               0
//        t1               0               1
//        t2               1               1
//        t3               1               0
//
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void encoder_handler() {
  BOOL update = FALSE;
  static BOOL init = TRUE;
  static UINT16 value = 0;
  static UINT16 step = 0;

  // one time init
  if (init) { 
    value = g_encoder.value;
    step  = g_encoder.step;
    init = FALSE;
  }

  encoder_scan();

  // for the rotary encoder use a state machine to determine how to change value or direction
  // we use a separate update flag to denote when we want to update the value
  // since the encoder is a quadrature we don't want to increment on every state change
  // in this implementation, either leaving or returning to state 0x0 will trigger an update
  // CW  = 0 (u) -> 2 -> 3 -> 1 (u) -> 0
  // CCW = 0 (u) -> 1 -> 3 -> 2 (u) -> 0
  if (g_encoder.enc0.snew || g_encoder.enc1.snew) {
    // consume the new flags
    g_encoder.enc0.snew = FALSE;  
    g_encoder.enc1.snew = FALSE; 

    // options that do nothing are left as comments for completeness
    switch (g_encoder.laststate) {
      case 0x0:
        if      ((g_encoder.dir == ENC_CW)  && (g_encoder.state == 0x2)) { update = TRUE; }
        else if ((g_encoder.dir == ENC_CCW) && (g_encoder.state == 0x1)) { update = TRUE; }
        break;

      case 0x1:
        if      ((g_encoder.dir == ENC_CW)  && (g_encoder.state == 0x0)) { update = TRUE; }
      //else if ((g_encoder.dir == ENC_CCW) && (g_encoder.state == 0x3)) { } // no update for quarter step
        break;

      case 0x2:
      //if      ((g_encoder.dir == ENC_CW)  && (g_encoder.state == 0x3)) { } // no update for quarter step
             if ((g_encoder.dir == ENC_CCW) && (g_encoder.state == 0x0)) { update = TRUE; }
        break;

      default: // case 0x3
        if      ((g_encoder.dir == ENC_CW)  && (g_encoder.state == 0x1)) { } // no update for quarter step
      //else if ((g_encoder.dir == ENC_CCW) && (g_encoder.state == 0x2)) { } // no update for quarter step
        break;
    }

    // update tells us that we want to increase or decrease the encoder value
    // so based on direction and step size increment/decrement the value
    // making sure not to exceed the max/min limits
    if (update) {
      if (!g_encoder.busy) step = g_encoder.step;
      if (g_encoder.dir == ENC_CW) {
        if (value < (g_encoder.max - g_encoder.step)) value += g_encoder.step;
        else value = g_encoder.max;
        g_encoder.debounce = DEBOUNCE;
      }
      else {
        if (g_encoder.value > (g_encoder.min + g_encoder.step)) value -= g_encoder.step;
        else value = g_encoder.min;
        g_encoder.debounce = DEBOUNCE;
      }
    }

    // if we get here, there was no state change consistent with the direction
    // so we use this as an indicator that the user has started turning the encoder 
    // in the opposite direction so we toggle our boolean flag
    else g_encoder.dir = !g_encoder.dir;
  }

  // value only updates if not busy and value is new
  // TODO this could be more efficient
  if (g_encoder.debounce) {
      g_encoder.debounce--;
  }
  else if (!g_encoder.busy) {
    if (value != g_encoder.value) {
      g_encoder.value = value;
      g_encoder.snew = TRUE;
    }
  }
  
}


//--------------------------------------------------------------------------------------------------
// init the motor driver
//--------------------------------------------------------------------------------------------------
void motor_init() {
  static BOOL portinit = FALSE;
  BOOL check = TRUE;

  // motor control outputs
  // BSETDM sets pin as an output pin
  BSETDM(M1A);
  BSETDM(M1B);
  BSETDM(M2A);
  BSETDM(M2B);

  MSTOP;
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// charge motor windings
// based on state value
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void motor_go(UINT8 state) {
  switch(state) {
    case 0:  M1R; M2R; break;
    case 1:  M1R; M2F; break;
    case 2:  M1F; M2R; break;
    case 3:  M1F; M2F; break;
    default: MSTOP;    break;
  }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// call this routine based on a timer (e.g. 16ms)
// turns the motor as requested in g_motor
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void motor_handler() {
  static UINT16 target = 0;
  static UINT16 position = 0;
  static UINT8 state = 0; // 0=M1R/M2R,1=M1R/M2F,2=M1F/MR2,3=M1F/M2F
  static UINT8 save_state = 0;
  static UINT16 idle_count = 0;

  // quick exit if not enabled
  if (!g_motor.enable) {
    MSTOP; 
    return;
  }

  // sharing data outside of the interrupt 
  // only if not currently being read
  if (!g_motor.busy) target = g_motor.target;

  // if we need to move and idle
  // restore the previous state
  if ((position != target) && (g_motor.idle)) {
    state = save_state;
    motor_go(state);
    g_motor.idle = FALSE;
  }

  // clockwise
  // (0)M1R/M2R -> (1)M1R/M2F -> (3)M1F/M2F -> (2)M1F/M2R -> (0)...
  else if (position < target) {
    idle_count = 0;
    switch(state) {
      case 0:  state = 1; break;
      case 1:  state = 3; break;
      case 2:  state = 0; break;
      case 3:  state = 2; break;
      default: state = 0; break;
    }
    motor_go(state);
    position++;
  }

  // counter clockwise
  // (0)M1R-M2R -> (2)M1F/M2R -> (3)M1F/M2F -> (1)M1R/M2F -> (0)...
  else if (position > target) {
    idle_count = 0;
    switch(state) {
      case 0:  state = 2; M1R; M2R; break;
      case 1:  state = 0; M1F; M2R; break;
      case 2:  state = 3; M1F; M2F; break;
      case 3:  state = 1; M1R; M2F; break;
      default: state = 0; ; break;
    }
    motor_go(state);
    position--;
  }

  // if no request to change position then
  // check / set idle to power off motor
  // save state when coming out of idle
  else if (!g_motor.idle) {
    if (++idle_count > MOTOR_IDLE) {
      g_motor.idle = TRUE;
      save_state = state;
      MSTOP;
    }
  }

  // sharing data outside of the interrupt 
  // only if not currently busy
  if (!g_motor.busy) g_motor.position = position;

}


//--------------------------------------------------------------------------------------------------
// timer
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
// setup timer0 to start counting
//
// notes:  (1) using this timer steps on the Arduino library timer, so
//             running this prevents the use of the micros() function
//         (2) interrupt status not changed
//         (3) timing changes may require multiple registers to be updated
//         (4) don't touch this code unless you know what you're doing
//
//               prescalar
//
//               CS02 CS01 CS00   description
//               0    0    0      timer stopped
//               0    0    1      cLKio/1 (no prescale)
//               0    1    0      CLKio/8
//               0    1    1      cLKio/64
//               1    0    0      cLKio/256
//               1    0    1      cLKio/1024
//               1    1    0      external clock T0 falling edge
//               1    1    1      external clock T0 rising edge
//--------------------------------------------------------------------------------------------------
void timer_setup() {
  TCCR0A = 0;                                     // clear TCCR0A register
  TCCR0B = 0;                                     // clear TCCR0B register
  TCNT0 = 0;                                      // clear starting counter
  OCR0A = (((CPU_CLOCK/INTERRUPT_FREQ_HZ)/8)-1);  // compare reg = ((16MHz/FREQ)/Prescalar=8)-1
  TCCR0A |= BMSK(WGM01);                          // mode: WGM[2:0] = 010b CTC (compare match)
  TCCR0B |= BMSK(CS01);                           // prescalar = 8:  CS[02:00]=010b
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// interrupt
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// unterrupt handler for timer0 interrupt
//
// !!! important note:  minimize time spent in this routine
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
ISR(TIMER0_COMPA_vect) {
  static UINT8 counter = 0;
  static UINT16 time_ms = 0;

  // if 1ms has gone by, increment
  // the timer and reset the counter
  if ((++counter) >= INTERRUPTS_PER_MS) {             // update only if we've gone 1ms
    counter = 0;                                      // force counter to reset
    time_ms++;
    if (!g_sys.busy) g_sys.time_ms = time_ms;

                                    isr_1ms();
    if (!(time_ms & LOOP_4MS_MASK)) isr_4ms();
  }
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// manage encoder here
// manage motor here
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void isr_1ms() {
  encoder_handler();
  motor_handler();
}

//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// run mode selector
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void isr_4ms() {
  UINT16 value = 0;
  UINT16 target = 0;
  UINT8 state = 0;
  static UINT16 delay;
  switch (g_sys.mode) {
    case (MODE_FOLLOW):  isr_mode_follow(); break;
    default:             break;
  }
}


//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
// follow inside of the interrupt
//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
void isr_mode_follow() {
  if (g_encoder.snew) {
    g_encoder.snew = FALSE;
    g_disp.update = TRUE;
  }

  if (!g_encoder.busy && !g_motor.busy) {
    g_motor.target = g_encoder.value;
  }
}


//--------------------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------------------
