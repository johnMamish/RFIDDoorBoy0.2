/**
 * @file        main.c
 *
 * @brief       The source file containing this project's main routine.
 */

#include <xc.h>
#include <htc.h>
#include "cfg_bits.h"
#include "UART_lib.h"
#include "stepper_motor_lib.h"

//RFID defines and vars

/**
 * @brief       Defines the number of keys that have permissions
 */
#define NUM_ALLOWED_KEYS 4

/**
 * @brief       Defines the length (in bytes) of each key's unique ID.
 */
#define KEY_NAME_LEN 5

/**
 * @brief       If RFID_MILLIS_TIMEOUT passes between recieving bytes of an RFID
 *              card, the byte recieved is considered to belong to a new card.
 * 
 * Each time a card begins reading, the system time is logged.  
 */
#define RFID_MILLIS_TIMEOUT 10

/**
 * @brief       A list of permitted key ID's.
 */
#if 0
uint8_t allowedKeys[NUM_ALLOWED_KEYS][KEY_NAME_LEN] =
{
    {0x02, 0x36, 0x36, 0x30, 0x30, 0x36, 0x42, 0x46, 0x43, 0x43, 0x31, 0x33, 0x30, 0x0d, 0x0a, 0x03},
    {0x02, '6', '6', '0', '0', '6', 'C', '4', '4', '5', '6', '1', '8', 0x0d, 0x0a, 0x03},
    {0x02, '6', '6', '0', '0', '6', 'C', '2', '9', 'C', '9', 'E', 'A', 0x0d, 0x0a, 0x03}
};
#endif

uint8_t allowedKeys[NUM_ALLOWED_KEYS][KEY_NAME_LEN] =
{
    {0x66, 0x00, 0x6c, 0x44, 0x56},
    {0x66, 0x00, 0x6c, 0x29, 0xc9},
    {0x66, 0x00, 0x6c, 0x06, 0xaa},
    {0x66, 0x00, 0x6b, 0xf6, 0x9b}
};

/**
 * @brief       An array of bytes holding the ID of the currently read key.
 */
uint8_t keyName[KEY_NAME_LEN+1];

/**
 * @brief       Because each byte of the key's ID is recieved sequentially, we
 *              need to keep track of which byte we are recieving.  This
 *              variable does that.
 */
volatile uint8_t keyNameIndex = 0;

/**
 * @brief       This variable is set if a complete key ID is prepared to be read
 *              by the application code.  It is unset if a key is being read in
 *              and may be unset be application code after a key ID has
 *              been processed.
 */
volatile uint8_t keyReadyFlag = 0;

//timer defines and values

/**
 * @brief       The value to preload TIMER0 with to have a tick frequency of
 *              1 millisecond.
 *
 * We want 1 tick every 1 millisecond and we have a TIMER0 clock of 4,000 clk/ms
 * and a 1:32 prescaler on TIMER0.  This means that each time TIMER0 counts 125
 * times, one millisecond will have passed.  If we preload it with
 * (256-125) = 131, it will rollover after 1 millisecond.
 */
#define TMR0_PRELOAD_VALUE 131

/**
 * @brief       System count of milliseconds since power-on.
 */
volatile uint32_t sysTick = 0;

//application defines and vars
typedef enum
{
    DOOR_CLOSED,
    DOOR_OPENING,
    DOOR_OPEN,
    DOOR_CLOSING
} DoorState;

//enum for state of card reader transmission
typedef enum
{
    STX,
    DATA,
    CHECKSUM,
    CRLFETX
} ID20State;

/**
 * @brief       Number of time units to wait until the door closes.
 *
 * lol what are these time units?  IDK lol.
 */
#define DOOR_TIMEOUT_VALUE 100000

#define DOOR_STEP_DISTANCE 160

/**
 * @brief       Describes the current state of the door (assuming the stepper
 *              motor works perfectly).
 */
DoorState currentDoor = DOOR_CLOSED;

/**
 * @brief       Time when the door started opening.
 */
uint32_t doorOpenTime;

const char crlfetx_str[] = "\n\r\x03";

int asciiDigitToInt(uint8_t c)
{
    if((c >= '0') && (c <= '9'))
    {
        return c - '0';
    }

    if((c >= 'A') && (c <= 'F'))
    {
        return (c - 'A') + 10;
    }

    if((c >= 'a') && (c <= 'f'))
    {
        return c - ('0' - ('a' - 'A'));
    }

    return 0xff;
}

void interrupt isr()
{
    //check TIMER0
    if(INTCONbits.T0IE && INTCONbits.T0IF)
    {
        INTCONbits.T0IF = 0b0;
        TMR0 = TMR0_PRELOAD_VALUE;
        sysTick++;
        return;
    }

    //run the UART isr.
    UARTISR();

    //run the stepper motor isr.
    STEPPERISR();

    //static variable to keep track of which field we are reading in
    static ID20State read_state;
    static uint8_t field_count;
    static uint8_t msb_lsb;
    static uint8_t read_byte;
    static uint8_t key_name_index;
    static uint8_t calcd_checksum;
    static uint8_t checksum;

    //add to the key buffer if necessary.
    uint8_t e;
    uint8_t readVal = readUART(&e);
    if(e == 0)
    {
        switch(read_state)
        {
            case STX:
            {
                keyReadyFlag = 0;

                if(readVal != 0x02)
                {
                    //error!!
                }
                else
                {
                    read_state = DATA;
                    field_count = 0;
                    msb_lsb = 0;
                }

                break;
            }

            case DATA:
            {
                if(msb_lsb)
                {
                    read_byte <<= 4;
                    read_byte |= asciiDigitToInt(readVal);
                    msb_lsb = 0;
                    keyName[field_count] = read_byte;
                    if(field_count == 0)
                    {
                        calcd_checksum = read_byte;
                    }
                    else
                    {
                        calcd_checksum ^= read_byte;
                    }
                    field_count++;
                }
                else
                {
                    read_byte = asciiDigitToInt(readVal);
                    msb_lsb = 1;
                }

                if(field_count == 5)
                {
                    read_state = CHECKSUM;
                    field_count = 0;
                    msb_lsb = 0;
                }

                break;
            }

            case CHECKSUM:
            {
                if(field_count == 0)
                {
                    checksum = asciiDigitToInt(readVal);
                }
                else
                {
                    checksum <<= 4;
                    checksum |= asciiDigitToInt(readVal);
                }

                field_count++;

                if(field_count == 2)
                {
                    read_state = CRLFETX;
                    field_count = 0;
                }

                break;
            }

            case CRLFETX:
            {
                if(crlfetx_str[field_count] != readVal)
                {
                    //error
                }

                field_count++;

                if(field_count == 3)
                {
                    keyReadyFlag = 1;
                    read_state = STX;
                }

                break;
            }
        }


#if 0
        keyReadyFlag = 0;
        keyName[keyNameIndex] = readVal;
        keyNameIndex++;
        if(keyNameIndex == KEY_NAME_LEN)
        {
            keyNameIndex = 0;
            keyReadyFlag = 1;
        }
#endif
    }
}

const static IOPin motor_pins[] =
{
    {&LATC, 4},
    {&LATC, 5},
    {&LATC, 6},
    {&LATC, 7}
};

void main()
{
    //testtesttest
    CCP1CONbits.CCP1M = 0b0;
    ANSELH = 0x00;
    ANSEL = 0x00;
    TRISC = 0x00;

    //setup sys clock
    OSCCONbits.IRCF = 0b111;
    OSCCONbits.SCS = 0b10;
    
    //initialize vars
    currentDoor = DOOR_CLOSED;

    //initialize UART
    initUART();

    //initialize steppers
    initStepperMotor(motor_pins);

    //initialize TIMER0 for system clock
    T0CONbits.T08BIT = 0b1;         //TIMER0 is 8-bit
    T0CONbits.PSA = 0b1;            //Use prescaler
    T0CONbits.T0PS = 0b111;         //1:256 prescaler
    T0CONbits.T0CS = 0b0;
    TMR0 = TMR0_PRELOAD_VALUE;
    T0CONbits.TMR0ON = 0b1;         //turn TIMER0 on.
    INTCONbits.T0IE = 0b1;

    //reset info
    uint8_t e;
    writeUARTMessage("RCON = ", &e);
    writeUART(RCON, &e);

    //disable interrupt priorities
    RCONbits.IPEN = 0b0;

    //global interrupt enable
    INTCONbits.PEIE = 0b1;
    INTCONbits.GIE = 0b1;

    keyName[KEY_NAME_LEN] = 0x00;

    uint8_t UARTErrorCode = 0x00;

    //writeUARTMessage("starting main loop\r\n", &UARTErrorCode);

    //startStepperMotor(0xffff, FORWARD, 0x50);
    uint32_t wait = 0;

    while(1)
    {
        switch(currentDoor)
        {
            case DOOR_CLOSED:
            {
                if(keyReadyFlag)
                {
                    //check key against all allowed keys.
                    //this operation has a very small chance of being failure prone!  If the
                    //RFID reader sees a key as this code is running, the key code being
                    //read in this code will be overwritten, however, this will never happen
                    //in practice.
                    keyReadyFlag = 0;
                    uint8_t keyGood = 1;
                    for(int i = 0; i < NUM_ALLOWED_KEYS; i++)
                    {
                        keyGood = 1;
                        for(int j = 0; j < KEY_NAME_LEN; j++)
                        {
                            if(keyName[j] != allowedKeys[i][j])
                            {
                                keyGood = 0;
                                break;
                            }
                        }
                        if(keyGood == 1)
                        {
                            break;
                        }
                    }

                    //print message
                    if(keyGood)
                    {
                        startStepperMotor(DOOR_STEP_DISTANCE, FORWARD, 0x50);
                        currentDoor = DOOR_OPENING;
                    }
                    else
                    {
                    }
                    writeUARTMessage("keynum = ", &UARTErrorCode);
                    writeUARTMessage(keyName, &UARTErrorCode);
                    writeUARTMessage(" ", &UARTErrorCode);
                }

                break;
            }

            case DOOR_OPENING:
            {
                //if the stepper motor has finished running, advance the state
                //of the door and start the "door open" timer.
                if(stepperMotorFinished())
                {
                    currentDoor = DOOR_OPEN;
                    doorOpenTime = sysTick;
                }

                break;
            }

            case DOOR_OPEN:
            {
                //check to see if its time to close the door yet.  If it is,
                //start closing the door and change the state.
                if((sysTick - doorOpenTime) > DOOR_TIMEOUT_VALUE)
                {
                    startStepperMotor(DOOR_STEP_DISTANCE, BACKWARD, 0);
                    currentDoor = DOOR_CLOSING;
                }

                break;
            }

            case DOOR_CLOSING:
            {
                //check to see if the door is closed
                if(stepperMotorFinished())
                {
                    currentDoor = DOOR_CLOSED;

                    //invalidate any key that has been read during the time the
                    //door was open.
                    keyReadyFlag = 0;

                    release_stepper_motor();
                }

                break;
            }
        }
    }
}