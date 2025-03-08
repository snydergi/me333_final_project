#include "nu32dip.h" // config bits, constants, funcs for startup and UART
#include "encoder.h"
#include "utilities.h"
#include "ina219.h"
#define BUF_SIZE 200
#define SAMPLE_TIME 6 // 24MHz*250ns

volatile int pwmDuty = 0;
volatile float kp_mA = 0;
volatile float ki_mA = 0;
volatile float kp_deg = 0;
volatile float ki_deg = 0;
volatile float kd_deg = 0;

void timer2init()
{
    T2CONbits.ON = 0;
    int prescaler = 1; // Using 1:1 prescaler
    PR2 = (NU32DIP_SYS_FREQ / (5000 * prescaler)) - 1;
    IFS0bits.T2IF = 0;
    IEC0bits.T2IE = 1;
    IPC2bits.T2IP = 5;
    IPC2bits.T2IS = 0;
    T2CONbits.ON = 1;
}

void pwmInit()
{
    ANSELBbits.ANSB15 = 0;     // Ensure RB15 is digital
    RPB15Rbits.RPB15R = 0b101; // Map RB15 to OC1 (PWM output)
    OC1CONbits.OCM = 0b110;    // Set OC1 to PWM mode (no fault pin)
    OC1CONbits.OCTSEL = 1;     // Use Timer3
    TMR3 = 0;                  // Clear Timer3 counter
    OC1RS = 0;
    OC1R = 0;
    int prescaler = 1; // Using 1:1 prescaler
    PR3 = (NU32DIP_SYS_FREQ / (20000 * prescaler)) - 1;
    T3CONbits.ON = 1;  // Turn on Timer3
    OC1CONbits.ON = 1; // Turn on OC1
}

void initHBridgeDir()
{
    TRISBbits.TRISB2 = 0; // Set B2 as output
    LATBbits.LATB3 = 0;   // Set B3 to 1
}

void sendDataToPython(int *refArray, int *actualArray, int length)
{
    // Send the number of data points
    char buffer[50];
    sprintf(buffer, "%d\n", length);
    NU32DIP_WriteUART1(buffer);

    // Send the reference then the actual current data
    for (int i = 0; i < length; i++)
    {
        sprintf(buffer, "%d %d\n", refArray[i], actualArray[i]);
        NU32DIP_WriteUART1(buffer);
    }
}

void __ISR(_TIMER_2_VECTOR, IPL5SOFT) Controller(void)
{
    static int counter = 0;             // Counter to keep track of number of samples
    static int refCurrent = -200;       // Reference current in mA (+200 or -200)
    static int actualCurrentArray[100]; // Array to store actual current data
    static int refCurrentArray[100];    // Array to store reference current data
    switch (get_mode())
    {
    case IDLE:
    {
        OC1RS = 0;
        LATBbits.LATB2 = 0; // Set direction to 0
        break;
    }
    case PWM:
    {
        // Set PWM duty cycle based on pwmDuty (-100 to 100)
        int pwmValue = (PR3 + 1) * (abs(pwmDuty) / 100.0);
        OC1RS = pwmValue;

        // Set direction based on sign of pwmDuty
        LATBbits.LATB2 = (pwmDuty >= 0) ? 1 : 0;
        break;
    }
    case ITEST:
    {
        if (counter % 25 == 0) // Toggle reference at 0, 25, 50, 75 samples
        {
            refCurrent = -refCurrent; // Toggle between +200 and -200 mA
        }
        int actualCurrent = INA219_read_current(); // read current in mA

        // PI Controller
        int error = refCurrent - actualCurrent;
        static int integral = 0;
        integral += error;
        pwmDuty = (kp_mA * error) + (ki_mA * integral);
        if (pwmDuty > 100)
        {
            pwmDuty = 100; // upper bound
        }
        else if (pwmDuty < -100)
        {
            pwmDuty = -100; // lower bound
        }
        OC1RS = (PR3 + 1) * (abs(pwmDuty) / 100.0); // Set PWM duty cycle
        LATBbits.LATB2 = (pwmDuty >= 0) ? 1 : 0;    // Set direction

        // Save reference and actual current data
        refCurrentArray[counter] = refCurrent;
        actualCurrentArray[counter] = actualCurrent;
        counter++;          // Increment counter
        if (counter >= 100) // End ITEST mode after 100 samples (two full cycles)
        {
            set_mode(IDLE);
            sendDataToPython(refCurrentArray, actualCurrentArray, 100);
            counter = 0;  // Reset counter
            integral = 0; // Reset integral
        }
        break;
    }
    default:
    {
        break;
    }
    }
    IFS0bits.T2IF = 0;
}

int main()
{
    UART2_Startup();
    INA219_Startup();
    set_mode(IDLE);
    char buffer[BUF_SIZE];
    NU32DIP_Startup(); // cache on, min flash wait, interrupts on, LED/button init, UART init
    NU32DIP_GREEN = 1; // turn off the LEDs
    NU32DIP_YELLOW = 1;
    __builtin_disable_interrupts();
    pwmInit();
    timer2init();
    initHBridgeDir();
    __builtin_enable_interrupts();
    while (1)
    {
        NU32DIP_ReadUART1(buffer, BUF_SIZE); // we expect the next character to be a menu command
        NU32DIP_YELLOW = 1;                  // clear the error LED
        switch (buffer[0])
        {
        case 'c': // read encoder counts
        {
            WriteUART2("a");
            while (!get_encoder_flag())
            {
            };
            set_encoder_flag(0);
            char m[50];
            int p = get_encoder_count();
            sprintf(m, "%d\r\n", p);
            NU32DIP_WriteUART1(m);
            break;
        }
        case 'd': // read encoder degrees
        {
            WriteUART2("a");
            while (!get_encoder_flag())
            {
            };
            set_encoder_flag(0);
            char m[50];
            int p = get_encoder_count();
            float deg = 360.0 * ((float)p / (334.0 * 4.0));
            sprintf(m, "%f\r\n", deg);
            NU32DIP_WriteUART1(m);
            break;
        }
        case 'e': // reset encoder
        {
            WriteUART2("b");
            break;
        }
        case 'r': // get mode
        {
            char m[50];
            switch (get_mode())
            {
            case IDLE: // IDLE mode
                sprintf(m, "IDLE\n");
                break;
            case PWM: // PWM mode
                sprintf(m, "PWM\n");
                break;
            case ITEST: // ITEST mode
                sprintf(m, "ITEST\n");
                break;
            case HOLD: // HOLD mode
                sprintf(m, "HOLD\n");
                break;
            case TRACK: // TRACK mode
                sprintf(m, "TRACK\n");
                break;
            }
            NU32DIP_WriteUART1(m);
            break;
        }
        case 'q': // quit
        {
            set_mode(IDLE);
            break;
        }
        case 'a': // read current sensor (ADC counts)
        {
            signed short adcval = readINA219(0x04);
            char m[50];
            sprintf(m, "%hu\r\n", adcval);
            NU32DIP_WriteUART1(m);
            break;
        }
        case 'b': // read current sensor (mA)
        {
            float adcval = INA219_read_current();
            char m[50];
            sprintf(m, "%f\r\n", adcval);
            NU32DIP_WriteUART1(m);
            break;
        }
        case 'f': // set PWM (-100 to 100)
        {
            int n = 0;
            NU32DIP_ReadUART1(buffer, BUF_SIZE);
            sscanf(buffer, "%d", &n);
            if (n >= -100 && n <= 100)
            {
                pwmDuty = n;
                set_mode(PWM);
            }
            break;
        }
        case 'p': // unpower the motor
        {
            set_mode(IDLE);
            break;
        }
        case 'g': // Set Current Gains
        {
            NU32DIP_ReadUART1(buffer, BUF_SIZE);
            sscanf(buffer, "%f %f", &kp_mA, &ki_mA);
            break;
        }
        case 'h': // Get Current Gains
        {
            char m[50];
            sprintf(m, "%f %f\r\n", kp_mA, ki_mA);
            NU32DIP_WriteUART1(m);
            break;
        }
        case 'k': // Test Current Control
        {
            set_mode(ITEST);
            break;
        }
        case 'i': // Set Position Gains
        {
            NU32DIP_ReadUART1(buffer, BUF_SIZE);
            sscanf(buffer, "%f %f %f", &kp_deg, &ki_deg, &kd_deg);
            break;
        }
        case 'j': // Get Position Gains
        {
            char m[50];
            sprintf(m, "%f %f %f\r\n", kp_deg, ki_deg, kd_deg);
            NU32DIP_WriteUART1(m);
            break;
        }
        case 'l': // Go to angle (deg)
        {
            break;
        }
        case 'm': // Load step trajectory
        {
            break;
        }
        case 'n': // Load cubic trajectory
        {
            break;
        }
        case 'o': // Execute trajectory
        {
            break;
        }
        default:
        {
            NU32DIP_YELLOW = 0; // turn on LED2 to indicate an error
            break;
        }
        }
    }
    return 0;
}