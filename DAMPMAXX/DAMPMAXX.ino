// --------------------------------------------------------------
// Dikins project
//
// by Morgan
// 2021.11.17
// --------------------------------------------------------------
#include <Wire_slave.h> // In this wire_slave library, wire use I2C1 and wire1 use I2C2
//#include <Wire.h> // In this wire_slave library, wire use I2C1 and wire1 use I2C2

#define UART0 Serial  // USB Virtual COM
#define UART1 Serial1 // on PA9(Tx1), PA10(Rx1)

#define USBCOM UART0 // via USB
#define DEBUG UART1  // via UART

#define DebugConsole DEBUG

// Encoder
#define SW_1 PB5
#define SW_2 PB4
#define SW_3 PB3
#define SW_4 PA15

// ADC
#define Voltage_Sensing PB1

// Dante (I2C1)
#define I2C1_SLK PB6
#define I2C1_SDA PB7
#define Dante_Mute PB9
#define Amp_En_Pin PA3
#define Mute_En_Pin PA4

// MA12070(AMP) I2C (I2C2)
#define I2C2_SLK PB10
#define I2C2_SDA PB11

// LED
#define LED7_PIN PA7
#define LED8_PIN PB0

// #define Dante_Mute PB9
#define Amp_A_Clip PC15
#define Amp_A_Err PA0
#define Amp_B_Clip PA1
#define Amp_B_Err PA2

#define MSW_1 PB8 // Hi-BTL, Lo-Single End
// #define MSW_2 PC14
#define SWDIO PA13
#define SWCLK PA14
#define MSEL_A_1 PA6
#define MSEL_B_1 PA5
#define AMP_01 0x20
#define AMP_02 0x21
#define Amp_Write I2C2_Write
#define Amp_Read I2C2_Read
#define TASK_ErrorLED_DURATION 200
#define AmpConfigValue 0x1B // it means enable VLP and i2s standard, no VLP is 0x00
// #define AmpConfigValue 0x00 // it means enable VLP and i2s standard, no VLP is 0x00, 2020.11.27
// -------------------- Matrix ----------------------------------
// 7-bit address
#define Matrix_Addr (0xEA >> 1) // (0xE8 >> 1)
#define MatrixConsole DebugConsole

#define Matrix_Write I2C2_Write
#define Matrix_Read I2C2_Read
// --------------------------------------------------------------
// Multiplexer
// format: {COM A(CH4), COM B(CH3), COM C(CH2), COM D(CH1)}
//
// M-SW Select:
// 0: Single End
// 1: BTL
//
// Table for Value vs Channel
#define NO0 0x00 // 0b0000 0000
#define NO1 0x01 // 0b0000 0001
#define NO2 0x02 // 0b0000 0010
#define NO3 0x04 // 0b0000 0100
#define NO4 0x08 // 0b0000 1000
#define NO5 0x10 // 0b0001 0000
#define NO6 0x20 // 0b0010 0000
#define NO7 0x40 // 0b0100 0000
#define NO8 0x80 // 0b1000 0000

static const uint8_t Switch_Table[2][16][4] = {
    // [MSW Select] [SWITCH CHANNEL] [INPUT NUMBER]
    {
        {NO4, NO3, NO2, NO1}, // 0
        {NO8, NO7, NO6, NO5}, // 1
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    },
    {
        {NO0, NO2, NO0, NO1}, // 0
        {NO0, NO4, NO0, NO3}, // 1
        {NO0, NO6, NO0, NO5}, // 2
        {NO0, NO8, NO0, NO7}, // 3
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
    },
};
// clang-format on
uint8_t swFlag = 0, swFlagCompare = 0xFF;
uint8_t mSwFlag = 0, mSwFlagCompare = 0xFF;
uint32_t swTimer = 0;
uint32_t swCounter = 0;
uint8_t AmpEnableFlag = 1; // default = 1
uint8_t AmpMuteFlag = 0;
uint8_t AmpMuteFlagStatus = 0;
uint8_t AmpMuteFlagStatusPast = 0;
uint16_t AmpInitTimer = 0;
uint16_t AmpRoutine_Timer = 0;
uint8_t TASK_DanteMute_STEP = 0;
uint32_t TASK_DanteMute_TIMER = 0;
uint8_t LED_Toggle;
uint8_t AMPErrorFlag1, AMPErrorFlag2;
uint8_t fClipPinError, fErrorPinError;
uint32_t TASK_Multiplexer_TIMER;
uint8_t TASK_ErrorLED_STEP;
uint32_t TASK_ErrorLED_TIMER;

uint8_t TASK_ErrorLED_COUNT;
uint8_t TASK_ErrorLED_GOAL;

uint8_t AmpRegVal[2];

const int analogInput = Voltage_Sensing;
int analogVal;
volatile uint8_t Volt_Level = 3;
// const uint8_t AmpVolume[4] = { // Reg = 0x40, Volume control, see table 8-9 in datasheet
//     14,
//     24,
//     34,
//     44,
// };

const byte AmpVolume = 0x1A; // based on Glenn's email in 2021.11.16, modified by Morgan, 2020.11.27
const byte AmpVolumeFrac = 0x02; // new parameter, based on Glenn's email in 2021.11.16, modified by Morgan, 2020.11.27
const byte AmpLimit = 0x1A;  // based on Glenn's email in 2021.11.16, modified by Morgan, 2020.11.27
const byte AmpProfile = 0x02; // by Glenn, 2021.3.5

enum
{
    DIR0 = 0,
    DIR1,
    DIR2,
    DIR3,
    SHDW0 = 0x10,
    SHDW1,
    SHDW2,
    SHDW3,
    CMD0,
    CMD1,
    __REG_NUM
};

uint8_t COMA_Reg = 0;
uint8_t COMB_Reg = 0;
uint8_t COMC_Reg = 0;
uint8_t COMD_Reg = 0;
// --------------------------------------------------------------
void ParaInitial(void)
{
    // Task initial
    TASK_DanteMute_STEP = 0;
    AmpMuteFlagStatus = 0;
    AmpMuteFlagStatusPast = 3;
    AmpMuteFlag = 0;
    AmpEnableFlag = 0;
    LED_Toggle = 0;
    AMPErrorFlag1 = 0;
    AMPErrorFlag2 = 0;

    TASK_Multiplexer_TIMER = 0;
    TASK_ErrorLED_STEP = 0;
    TASK_ErrorLED_TIMER = 0;
    TASK_ErrorLED_COUNT = 0;
    TASK_ErrorLED_GOAL = 0;
}
// --------------------------------------------------------------
void PinDefinition(void)
{
    // Encoder
    pinMode(SW_1, INPUT);
    pinMode(SW_2, INPUT);
    pinMode(SW_3, INPUT);
    pinMode(SW_4, INPUT);

    // ADC
    pinMode(Voltage_Sensing, INPUT);

    // MA12070 AMP
    pinMode(Amp_En_Pin, OUTPUT);
    pinMode(Mute_En_Pin, OUTPUT);
    pinMode(Dante_Mute, INPUT_PULLUP); // for RD developing FW
    pinMode(Amp_A_Clip, INPUT_PULLUP);
    pinMode(Amp_A_Err, INPUT_PULLUP);
    pinMode(Amp_B_Clip, INPUT_PULLUP);
    pinMode(Amp_B_Err, INPUT_PULLUP);

    // BTL/Single End Mode
    pinMode(MSW_1, INPUT);
    // Mode Select
    pinMode(MSEL_A_1, OUTPUT);
    pinMode(MSEL_B_1, OUTPUT);
    digitalWrite(MSEL_A_1, LOW);
    digitalWrite(MSEL_B_1, LOW);

    // LED
    pinMode(LED7_PIN, OUTPUT);
    pinMode(LED8_PIN, OUTPUT);
    digitalWrite(LED7_PIN, HIGH); // Low Enable
    digitalWrite(LED8_PIN, HIGH); // Low Enable

    // I2C
    pinMode(I2C1_SLK, INPUT); // set INPUT for protecting GPIO
    pinMode(I2C1_SDA, INPUT); // set INPUT for protecting GPIO
    pinMode(I2C2_SLK, INPUT); // set INPUT for protecting GPIO
    pinMode(I2C2_SDA, INPUT); // set INPUT for protecting GPIO
}
// --------------------------------------------------------------
// ADC Read
void ADC_Read(void)
{
    // ADC Resolution: 12-bit, 0~4095
    analogVal = analogRead(analogInput);
    DEBUG.print("ADC(PB1): ");
    DEBUG.println(analogVal);
}
// --------------------------------------------------------------
void Read_MA12070_Reg(void)
{
    DebugConsole.println("AMP Reg:");
    for (uint8_t addr = 0; addr < 128; addr++)
    {
        DebugConsole.print("addr = 0x");
        DebugConsole.print(addr, HEX);
        DebugConsole.print(" , A1 = 0x");
        DebugConsole.print(Amp_Read(AMP_01, addr), HEX);
        DebugConsole.print(" , A2 = 0x");
        DebugConsole.println(Amp_Read(AMP_02, addr), HEX);
    }
}
// --------------------------------------------------------------
void TASK_AMP(void)
{
    switch (TASK_DanteMute_STEP)
    {
    case 0:
        TASK_DanteMute_TIMER = millis();
        TASK_DanteMute_STEP = 1;
        break;
    case 1:
        if ((millis() - TASK_DanteMute_TIMER) >= 500)
        {
            // Check MA12070 status first
            MA12070_ErrorCheck();

            // Heartbeat
            digitalWrite(LED7_PIN, LED_Toggle); // system alive light
            LED_Toggle ^= 1;
            TASK_DanteMute_STEP = 0;
        }
        break;
    }
}
// --------------------------------------------------------------
void TASK_Multiplexer(void)
{
    swCounter = millis();

    TASK_Multiplexer_TIMER = swCounter - swTimer; // avoid error like 0-65535
    if (TASK_Multiplexer_TIMER < 100)
        return;

    swTimer = swCounter;

    mSwFlag = digitalRead(MSW_1);
    swFlag = digitalRead(SW_1) | (digitalRead(SW_2) << 1) | (digitalRead(SW_3) << 2) | (digitalRead(SW_4) << 3);
    swFlag ^= 0x0F;

    if (mSwFlag != mSwFlagCompare || swFlag != swFlagCompare)
    {
        mSwFlagCompare = mSwFlag;
        swFlagCompare = swFlag;
        // M-SW Status:
        // 0: Single End
        // 1: BTL
        DebugConsole.print("M-SW : ");
        DebugConsole.println(mSwFlag, DEC);

        DebugConsole.print("SW-SEL : ");
        DebugConsole.println(swFlag, DEC);

        if (mSwFlag < 4 && swFlag < 16)
        {
            // Set Reg
            COMA_Reg = Switch_Table[mSwFlag][swFlag][0]; // COM A
            COMB_Reg = Switch_Table[mSwFlag][swFlag][1]; // COM B
            COMC_Reg = Switch_Table[mSwFlag][swFlag][2]; // COM C
            COMD_Reg = Switch_Table[mSwFlag][swFlag][3]; // COM D

            Matrix_Write(Matrix_Addr, DIR0, COMA_Reg);
            Matrix_Write(Matrix_Addr, DIR1, COMB_Reg);
            Matrix_Write(Matrix_Addr, DIR2, COMC_Reg);
            Matrix_Write(Matrix_Addr, DIR3, COMD_Reg);

            DebugConsole.print("COMA : 0x");
            DebugConsole.println(COMA_Reg, HEX);
            DebugConsole.print("COMB : 0x");
            DebugConsole.println(COMB_Reg, HEX);
            DebugConsole.print("COMC : 0x");
            DebugConsole.println(COMC_Reg, HEX);
            DebugConsole.print("COMD : 0x");
            DebugConsole.println(COMD_Reg, HEX);
        }
        else
        {
            DebugConsole.println("Parameter Error");
        }
    }
}
// --------------------------------------------------------------
void TASK_ErrorLED(void)
{
    if (fErrorPinError || fClipPinError || AMPErrorFlag1 || AMPErrorFlag2)
    {
        digitalWrite(LED8_PIN, LOW); // Error LED, Low enable
    }
    else
    {
        digitalWrite(LED8_PIN, HIGH); // Error LED, Low enable
    }
}
// --------------------------------------------------------------
void MA12070_ClipPinCheck(void)
{
    if ((digitalRead(Amp_A_Clip) == 0) || (digitalRead(Amp_B_Clip) == 0))
    {
        fClipPinError = 1;
    }
    else
    {
        fClipPinError = 0;
    }
}
// --------------------------------------------------------------
void MA12070_ErrorPinCheck(void)
{
    if ((digitalRead(Amp_A_Err) == 0) || (digitalRead(Amp_A_Err) == 0))
    {
        fErrorPinError = 1;
    }
    else
    {
        fErrorPinError = 0;
    }
}
// --------------------------------------------------------------
void MA12070_ErrorCheck(void) // check it per 500ms
{

    // ----------------------------------------------------------
    // Check AMP 1
    AmpRegVal[0] = Amp_Read(AMP_01, 0x7C);
    if (AMPErrorFlag1 == 0) // orange status: normal
    {
        if (AmpRegVal[0] != 0) // get error
        {
            DebugConsole.print("[Amp1] Error: 0x");
            DebugConsole.println(AmpRegVal[0], HEX);
            AMPErrorFlag1 = 1;
            // Clear Error
            // Amp_Write(AMP_01, 0x2D, 0xFF);
            // Amp_Write(AMP_01, 0x2D, 0x00);
            // DebugConsole.println("[Amp1] Clear Error");
        }
    }
    else // back from error status
    {
        if (AmpRegVal[0] == 0)
        {
            DebugConsole.print("[Amp1]: OK\n");
            AMPErrorFlag1 = 0;
        }
    }
    // ----------------------------------------------------------
    // Check AMP 2
    AmpRegVal[1] = Amp_Read(AMP_02, 0x7C);
    if (AMPErrorFlag2 == 0)
    {
        if (AmpRegVal[1] != 0)
        {
            DebugConsole.print("[Amp2] Error: 0x");
            DebugConsole.println(AmpRegVal[1], HEX);
            AMPErrorFlag2 = 1;
            // Clear Error
            // Amp_Write(AMP_02, 0x2D, 0xFF);
            // Amp_Write(AMP_02, 0x2D, 0x00);
            // DebugConsole.println("[Amp2] Clear Error");
        }
    }
    else
    {
        if (AmpRegVal[1] == 0)
        {
            DebugConsole.print("[Amp2]: OK\n");
            AMPErrorFlag2 = 0;
        }
    }
    // ----------------------------------------------------------
}
// --------------------------------------------------------------
void AMP_MODE_SELECT(void) // BTL or Single End switch
{
    if (digitalRead(MSW_1))
    {
        pinMode(MSEL_A_1, OUTPUT);
        pinMode(MSEL_B_1, OUTPUT);
        digitalWrite(MSEL_A_1, LOW);
        digitalWrite(MSEL_B_1, LOW);
        DebugConsole.println("BTL Mode");
    }
    else
    {
        pinMode(MSEL_A_1, INPUT); // Do it as open drain for 5V, need pull high
        pinMode(MSEL_B_1, INPUT);
        DebugConsole.println("Single End");
    }
}
// --------------------------------------------------------------
void CURRENT_AMP_MODE_CHECK(void)
{
    uint8_t tmp;
    // Caution: I2C function has to run under MA12070 is enable, otherwise, it will not work
    // --------------------------------------
    tmp = Amp_Read(AMP_01, 0x35);
    DebugConsole.print("[AMP1] Check I2S :");
    DebugConsole.print(tmp, HEX);
    if (tmp != AmpConfigValue)
    {
        DebugConsole.print(", FAIL");
        Amp_Write(AMP_01, 0x35, AmpConfigValue);
        DebugConsole.print(", Change to 0x");
        DebugConsole.println(AmpConfigValue, HEX);
    }
    else
    {
        DebugConsole.println(", OK");
    }
    // --------------------------------------
    tmp = Amp_Read(AMP_02, 0x35);
    DebugConsole.print("[AMP2] Check I2S :");
    DebugConsole.print(tmp, HEX);
    if (tmp != AmpConfigValue)
    {
        DebugConsole.println(", FAIL");
        Amp_Write(AMP_02, 0x35, AmpConfigValue);
        DebugConsole.print(", Change to 0x");
        DebugConsole.println(AmpConfigValue, HEX);
    }
    else
    {
        DebugConsole.println(", OK");
    }
    // --------------------------------------
}
// --------------------------------------------------------------
void AmpVolSet()
{
    // below codes are copied from Glenn's email in 2021.11.16
    // Configure amp A
    Amp_Write(AMP_01, 0x0A, 0x80);          // Enable soft clip
    Amp_Write(AMP_01, 0x35, 0xA8);          // Fast attack, slow release, (should be +1 at end)
    Amp_Write(AMP_01, 0x40, AmpVolume);
    Amp_Write(AMP_01, 0x41, AmpVolumeFrac);
    Amp_Write(AMP_01, 0x1D, AmpProfile);    // Power mode Profile 2 - Optimized Audio Performance

//    Amp_Write(AMP_01, 0x36, 0x41);          // Use the limiter
    Amp_Write(AMP_01, 0x36, 0x00);          // Bypass the limiter

    Amp_Write(AMP_01, 0x47, AmpLimit);    
    Amp_Write(AMP_01, 0x48, AmpLimit);
    Amp_Write(AMP_01, 0x49, AmpLimit);
    Amp_Write(AMP_01, 0x4A, AmpLimit);

    // Configure amp B
    Amp_Write(AMP_02, 0x0A, 0x80);          // Enable soft clip
    Amp_Write(AMP_02, 0x35, 0xA8);
    Amp_Write(AMP_02, 0x40, AmpVolume);
    Amp_Write(AMP_02, 0x41, AmpVolumeFrac);
    Amp_Write(AMP_02, 0x1D, AmpProfile);

//    Amp_Write(AMP_02, 0x36, 0x41);
    Amp_Write(AMP_02, 0x36, 0x00);          // Bypass the limiter

    Amp_Write(AMP_02, 0x47, AmpLimit);
    Amp_Write(AMP_02, 0x48, AmpLimit);
    Amp_Write(AMP_02, 0x49, AmpLimit);
    Amp_Write(AMP_02, 0x4A, AmpLimit);

    DebugConsole.println("Amplifiers Volume Set to -2dB");
    delay(50);
}
// --------------------------------------------------------------
void AmpEnable(void)
{
    // uint8_t temp = 0xFF;
    if (AmpEnableFlag == 0)
    {
        AmpEnableFlag = 1;

        Wire1.end();                   //   restart Wire to skip SCL freeze when Amp Enable
        digitalWrite(Amp_En_Pin, LOW); // assert amp enables
        delay(100);
        DebugConsole.println("[Amp] Enable : Done");

        Wire1.begin();
        delay(100);

        // Config AMP 1
        Amp_Write(AMP_01, 0x35, AmpConfigValue);
        Amp_Write(AMP_01, 0x1D, AmpProfile);    // Power mode Profile 2 - Optimized Audio Performance, by Glenn, 2021.3.5

        // Config AMP 2
        Amp_Write(AMP_02, 0x35, AmpConfigValue);
        Amp_Write(AMP_02, 0x1D, AmpProfile);    // Power mode Profile 2 - Optimized Audio Performance, by Glenn, 2021.3.5

        // Check
        CURRENT_AMP_MODE_CHECK();

        //!! if you want write different value to 0x35, should afer CURRENT_AMP_MODE_CHECK() or this comment
        // change 0x35 register value should start at here
        //        if(Volt_Change())
        //            AmpVolSet(AmpVolume[Volt_Level]);
        AmpVolSet();
        delay(50);
        DebugConsole.println("[Amp] First Initial : Done");
    }
    else
    {
        DebugConsole.println("[Amp] Enable : Skip");
    }
}
// --------------------------------------------------------------
void AmpDisable(void)
{
    AmpEnableFlag = 0;
    digitalWrite(Amp_En_Pin, HIGH);
    delay(50);
    DebugConsole.println("[Amp] Disable : Done");
}
// --------------------------------------------------------------
void AmpMute(void)
{
    AmpMuteFlag = 1;
    digitalWrite(Mute_En_Pin, LOW);
    delay(50);
    DebugConsole.println("[Amp] Mute : Done");
}
// --------------------------------------------------------------
void AmpUnMute(void)
{
    AmpMuteFlag = 0;
    digitalWrite(Mute_En_Pin, HIGH);
    delay(50);
    DebugConsole.println("[Amp] UnMute : Done");
}
// --------------------------------------------------------------
void ClearError(void)
{
    Amp_Write(AMP_01, 0x2D, 0xFF);
    Amp_Write(AMP_01, 0x2D, 0x00);
    Amp_Write(AMP_02, 0x2D, 0xFF);
    Amp_Write(AMP_02, 0x2D, 0x00);
    DebugConsole.println("[Amp] Clear Error");
}
// --------------------------------------------------------------
uint8_t I2C1_Read(int device, int address)
{
    uint8_t data = 0;

    Wire.beginTransmission(device);
    Wire.write(byte(address));
    Wire.endTransmission(); // stop transmitting

    Wire.requestFrom(device, 1);
    data = Wire.read();
    delay(1);

    return data;
}
// --------------------------------------------------------------
void I2C1_Write(uint8_t device, uint8_t address, uint8_t value)
{
    uint8_t temp = 0;
    Wire.beginTransmission(device);   // transmit to device
    temp = Wire.write(byte(address)); // Reg address
    temp = Wire.write(byte(value));
    if (Wire.endTransmission()) // stop transmitting
    {
        DebugConsole.println("I2C1 no ACK");
    }
    delay(1);
}
// --------------------------------------------------------------
uint8_t I2C2_Read(int device, int address)
{
    uint8_t data = 0;

    Wire1.beginTransmission(device);
    Wire1.write(byte(address));
    Wire1.endTransmission(); // stop transmitting

    Wire1.requestFrom(device, 1);
    data = Wire1.read();
    delay(1);

    return data;
}
// --------------------------------------------------------------
void I2C2_Write(uint8_t device, uint8_t address, uint8_t value)
{
    Wire1.beginTransmission(device); // transmit to device
    Wire1.write(byte(address));      // Reg address
    Wire1.write(byte(value));
    if (Wire1.endTransmission()) // stop transmitting
    {
        DebugConsole.println("I2C2 no ACK");
    }
    delay(1);
}
// --------------------------------------------------------------
void setup()
{
    // Pin definition
    PinDefinition();
    // Console
    USBCOM.begin(115200); // USB
    DEBUG.begin(115200);  // UART
    DEBUG.println("\n\nSW Ver: 2.0.4b");

    // Parameters initialize
    ParaInitial();

    AmpDisable(); // pin MA12070_ENABLE is high(it's low enable)
    AmpMute();    // pin MA12070_MUTE is low(it's high un-mute)
    // Select AMP Mode and do AMP OFF, Mute On
    AMP_MODE_SELECT();

    // I2C1 Master, for Dante
    // Wire.begin();
    // I2C2 Master, for MA12070(AMP)
    Wire1.begin();
    delay(100);
    // ADC
    ADC_Read();

    DEBUG.println("Wait Dante Mute signal..");

    // LED7: waiting Dante connection, led always turn on
    digitalWrite(LED7_PIN, LOW);
    // wait Dante Mute signal (at 14th pin of the 40-pins connector)
    while (digitalRead(Dante_Mute))
        ;
    // LED7: waiting Dante connection, led always turn on
    digitalWrite(LED7_PIN, LOW);
    delay(50);

    // Set Output
    AmpEnable();
    delay(50);
    AmpUnMute();
    delay(50);

    // MA12070 Register Check, after doing AmpEnable()
    Read_MA12070_Reg();
}
// -------------------- Main Loop -------------------------------
void loop()
{
    // main routine
    TASK_AMP(); // Morgan
    TASK_Multiplexer();

    MA12070_ClipPinCheck();
    MA12070_ErrorPinCheck();
    TASK_ErrorLED();
}
// --------------------------------------------------------------
