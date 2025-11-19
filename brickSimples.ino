#include "brickSimples.h"

TCS34725 sensor1 = TCS34725(Portas::PORTA_I2C_1, TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
TCS34725 sensor2 = TCS34725(Portas::PORTA_I2C_2, TCS34725_INTEGRATIONTIME_24MS, TCS34725_GAIN_4X);
VL53L0X sensorDistancia = VL53L0X(Portas::PORTA_I2C_3);
//Servo servo;
bool sensor1Detectado = false;
//Ultrassonico ultrassonico;
//Giroscopio giroscopio;

uint16_t red, green, blue, clear;
void setup(){
    brick.inicializa();
    sensor1.begin();
    sensor2.begin();
    sensorDistancia.init();
    //giroscopio.inicializar(Portas::PORTA_SERIAL_1);
    //ultrassonico.inicializar(Portas::PORTA_ULTRASSONICO_1);
    //ultrassonico.teste(60);
    //Serial.println(ultrassonico.distanciaCmInt());
    /*servos.iniciaServo(Portas::PORTA_SERVO_1);
    //CASO OS SENSORES COMPARTILHEM A MESMA PORTA I2C, O A LASER DEVE SEMPRE SER INICIALIZADO PRIMEIRO
    sensorDistancia.init();
    sensor.begin();
    
    
    sensor.getRawData(&red, &green, &blue, &clear);
    Serial.print("Valores iniciais - R:");
    Serial.print(red);
    Serial.print(" G:");
    Serial.print(green);
    Serial.print(" B:");
    Serial.print(blue);
    Serial.print(" C:");
    Serial.println(clear);


*/
    //buzzer.inicializar();
    //buzzer.powerRangers();
    //buzzer.jingleBells();
    //ledStrip.inicializar(Portas::PORTA_LED_3, 10); //inicializa fita de 1 led na porta led 1
    //ledStrip.demo();
    //ledStrip.arcoIrisRotativo();
    //ledStrip.knightRider();
   /* uint8_t i=0;
    while(1){
        i=0;
        while(i<10){
            ledStrip.setLED(i, 255, 0, 0); //define o led como vermelho
            i++;
        }
        ledStrip.atualizar(); //atualiza a fita de leds
        delay(300);
        i=0;
        while(i<10){
            ledStrip.setLED(i, 0, 0, 255); //define o led como vermelho
            i++;
        }
        ledStrip.atualizar(); //atualiza a fita de leds
        delay(300);
        i=0;
        while(i<10){
            ledStrip.setLED(i, 0, 255, 0); //define o led como verde
            i++;
        }
        ledStrip.atualizar(); //atualiza a fita de leds
        delay(300);
    }
        */
    //while(1);
    /*while(1){ 
        servos.moveServo(Servos::PORTA_SERVO_1, 90); //move servo 1 para 90 graus
        delay(1000);
        servos.moveServo(Servos::PORTA_SERVO_1, 0); //move servo 1 para 0 graus
        delay(1000);
        sensor.getRawData(&red, &green, &blue, &clear);
        Serial.print("Sensor de cor TCS34725 inicializado. Valores iniciais - R:");
        Serial.print(red);
        Serial.print(" G:");
        Serial.print(green);
        Serial.print(" B:");
        Serial.print(blue);
        Serial.print(" C:");
        Serial.println(clear);
        Serial.println(sensorDistancia.readRangeSingleMillimeters());
    }*/
}
void loop(){
    
    sensor1.getRawData(&red, &green, &blue, &clear);
    Serial.print("Sensor1 - R:");
    Serial.print(red);
    Serial.print(" G:");
    Serial.print(green);
    Serial.print(" B:");
    Serial.print(blue);
    Serial.print(" C:");
    Serial.println(clear);

    sensor2.getRawData(&red, &green, &blue, &clear);
    Serial.print("Sensor2 - R:");
    Serial.print(red);
    Serial.print(" G:");
    Serial.print(green);
    Serial.print(" B:");
    Serial.print(blue);
    Serial.print(" C:");
    Serial.println(clear);

    uint16_t dist = sensorDistancia.readRangeSingleMillimeters();
    Serial.print("Distancia: ");   
    Serial.print(dist);
    Serial.println(" mm"); 
    brick.espera(1000);
    /*int dist = sensorDistancia.readRangeSingleMillimeters();
    servos.moveServo(Portas::PORTA_SERVO_1, dist); //mapeia a distância para o ângulo do servo
    sensor.getRawData(&red, &green, &blue, &clear);
    Serial.print("Valores iniciais - R:");
    Serial.print(red);
    Serial.print(" G:");
    Serial.print(green);
    Serial.print(" B:");
    Serial.print(blue);
    Serial.print(" C:");
    Serial.println(clear);
    delay(10);
    */
   
}