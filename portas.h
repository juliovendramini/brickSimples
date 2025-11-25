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

// Portas I2C
#define PORTA_I2C_1 {14, 15, "I2C-1"}
#define PORTA_I2C_2 {16, 17, "I2C-2"}
#define PORTA_I2C_3 {8, 9, "I2C-3"}
#define PORTA_I2C_4 {2, 3, "I2C-4"}
#define PORTA_I2C_5 {18, 19, "I2C-5"}

// Portas Serial (Software Serial)
#define PORTA_SERIAL_1 {14, 15, "SER-1"}
#define PORTA_SERIAL_2 {16, 17, "SER-2"}
#define PORTA_SERIAL_3 {8, 9, "SER-3"}
#define PORTA_SERIAL_4 {2, 3, "SER-4"}
#define PORTA_SERIAL_5 {18, 19, "SER-5"}

// Portas Servo
#define PORTA_SERVO_1 {1, 13, "S1"}
#define PORTA_SERVO_2 {2, 12, "S2"}
#define PORTA_SERVO_3 {3, 11, "S3"}
#define PORTA_SERVO_4 {4, 10, "S4"}

// Portas LED
#define PORTA_LED_1 {1, 13, "S1"}
#define PORTA_LED_2 {2, 12, "S2"}
#define PORTA_LED_3 {3, 11, "S3"}
#define PORTA_LED_4 {4, 10, "S4"}

// Portas Analógicas
#define PORTA_ANALOGICA_1 {A0, A1}
#define PORTA_ANALOGICA_2 {A2, A3}
#define PORTA_ANALOGICA_5 {A4, A5}

// Portas Ultrassônico
#define PORTA_ULTRASSONICO_1 {14, 15, "US-1"}
#define PORTA_ULTRASSONICO_2 {16, 17, "US-2"}
#define PORTA_ULTRASSONICO_3 {8, 9, "US-3"}
#define PORTA_ULTRASSONICO_4 {2, 3, "US-4"}
#define PORTA_ULTRASSONICO_5 {18, 19, "US-5"}

// // Definições das constantes static (necessário para linker)
// constexpr PortaI2C Portas::PORTA_I2C_1;
// constexpr PortaI2C Portas::PORTA_I2C_2;
// constexpr PortaI2C Portas::PORTA_I2C_3;
// constexpr PortaI2C Portas::PORTA_I2C_4;
// constexpr PortaI2C Portas::PORTA_I2C_5;

#endif