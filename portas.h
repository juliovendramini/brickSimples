#ifndef _PORTAS_H_
#define _PORTAS_H_
#include <Arduino.h>

#define PINO_BATERIA A7


//para cada tipo de sensor, eu devo definir uma struct com os pinos e descrição e 
//depois criar as constantes dentro da classe Portas das portas que são compatíveis com aquele tipo de sensor
struct PortaI2C{
    const uint8_t sda;
    const uint8_t scl;
    const char *descricao;
};

struct PortaServo{
    const uint8_t porta;
    const uint8_t pino;
    const char *descricao;
};

//mesmas portas do servo
struct PortaLed{
    const uint8_t porta;
    const uint8_t pino;
    const char *descricao;
};

struct PortaAnalogica{
    const uint8_t pino1;
    const uint8_t pino2;
};

struct PortaUltrassonico{
    const uint8_t trig;
    const uint8_t echo;
    const char *descricao;
};

struct PortaSerial{
    const uint8_t rx;
    const uint8_t tx;
    const char *descricao;
};

class Portas{
    public:
    constexpr static PortaI2C PORTA_I2C_1 = {14, 15, "I2C-1"};
    constexpr static PortaI2C PORTA_I2C_2 = {16, 17, "I2C-2"};
    constexpr static PortaI2C PORTA_I2C_3 = {8, 9, "I2C-3"};
    constexpr static PortaI2C PORTA_I2C_4 = {2, 3, "I2C-4"}; 
    constexpr static PortaI2C PORTA_I2C_5 = {A4, A5, "I2C-5"};//pinos i2c nativos

    //software serial
    constexpr static PortaSerial PORTA_SERIAL_1 = {14, 15, "SER-1"};
    constexpr static PortaSerial PORTA_SERIAL_2 = {16, 17, "SER-2"};
    constexpr static PortaSerial PORTA_SERIAL_3 = {8, 9, "SER-3"};
    constexpr static PortaSerial PORTA_SERIAL_4 = {2, 3, "SER-4"}; 
    constexpr static PortaSerial PORTA_SERIAL_5 = {A4, A5, "SER-5"};


    constexpr static PortaServo PORTA_SERVO_1 = {1, 13, "S1"};
    constexpr static PortaServo PORTA_SERVO_2 = {2, 12, "S2"};
    constexpr static PortaServo PORTA_SERVO_3 = {3, 11, "S3"};
    constexpr static PortaServo PORTA_SERVO_4 = {4, 10, "S4"};

    constexpr static PortaLed PORTA_LED_1 = {1, 13, "S1"};
    constexpr static PortaLed PORTA_LED_2 = {2, 12, "S2"};
    constexpr static PortaLed PORTA_LED_3 = {3, 11, "S3"};
    constexpr static PortaLed PORTA_LED_4 = {4, 10, "S4"};

    constexpr static PortaAnalogica PORTA_ANALOGICA_1 = {A0, A1};
    constexpr static PortaAnalogica PORTA_ANALOGICA_2 = {A2, A3};
    //porta 3 e 4 nao tem suporta para analogico
    constexpr static PortaAnalogica PORTA_ANALOGICA_5 = {A4, A5};

    constexpr static PortaUltrassonico PORTA_ULTRASSONICO_1 = {14, 15, "US-1"};
    constexpr static PortaUltrassonico PORTA_ULTRASSONICO_2 = {16, 17, "US-2"};
    constexpr static PortaUltrassonico PORTA_ULTRASSONICO_3 = {8, 9, "US-3"};
    constexpr static PortaUltrassonico PORTA_ULTRASSONICO_4 = {2, 3, "US-4"};
    constexpr static PortaUltrassonico PORTA_ULTRASSONICO_5 = {A4, A5, "US-5"};

};



#endif