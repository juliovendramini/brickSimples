#include "brickSimples.h"

TCS34725 sensor1 = TCS34725(PORTA_I2C_1);
TCS34725 sensor2 = TCS34725(PORTA_I2C_2);
TCS34725 sensor3 = TCS34725(PORTA_I2C_3);
VL53L0X sensorDistancia = VL53L0X(PORTA_I2C_4);
//VL53L0X sensorDistancia2 = VL53L0X(PORTA_I2C_5);
Ultrassonico ultrassonico = Ultrassonico(PORTA_ULTRASSONICO_5);
VL53L0X sensorDistancia3 = VL53L0X(PORTA_I2C_3);
//Servo servo;
bool sensor1Detectado = false;
//Ultrassonico ultrassonico;
Giroscopio giroscopio;

uint16_t red, green, blue, clear;
uint16_t red2, green2, blue2, clear2;
uint16_t red3, green3, blue3, clear3;
uint32_t tempoAnterior = 0;
uint8_t i=0;
void setup(){
    brick.inicializa(); //essa linha é obrigatória existir e ser a primeira do setup
    servos.iniciaServo(PORTA_SERVO_1);
    servos.iniciaServo(PORTA_SERVO_2);
    servos.iniciaServo(PORTA_SERVO_3);
    while(true){
        if(Serial.available()){
            uint8_t grau = Serial.parseInt();
            servos.moveServo(PORTA_SERVO_1, grau); //move servo 1 para 90 graus
            Serial.print(F("Movendo servo para "));
            Serial.print(grau);
        }
    }
    delay(2000);
    //delay(1000);
    //servos.moveServo(PORTA_SERVO_1, 0); //move servo 1 para 0 graus
    // brick.potenciaMotores(60, 0);
    // delay(2000);
    // brick.potenciaMotores(-60, 0);
    // delay(2000);
    // brick.potenciaMotores(0, 60);
    // delay(2000);
    // brick.potenciaMotores(0, -60);
    // delay(2000);
    // brick.potenciaMotores(0, 0);
    //sensor1.limpaCalibracao();
    //sensor2.limpaCalibracao();
    //brick.adiciona(&sensorDistancia);
    //brick.adiciona(&sensorDistancia2);
    //brick.adiciona(&sensorDistancia3);
    brick.adiciona(&sensor1);
    //brick.adiciona(&sensor2);
    brick.adiciona(&sensor3);
    //brick.adiciona(&ultrassonico);
    //brick.inverteMotorEsquerdo(true);
    //sensorDistancia.init();
    //giroscopio.inicializa(PORTA_SERIAL_1);
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
    //buzzer.inicializa();
    //buzzer.powerRangers();
    //buzzer.jingleBells();
    //ledStrip.inicializa(PORTA_LED_4, 1); //inicializa fita de 1 led na porta led 1
    //buzzer.sucesso();
    //buzzer.alerta();
    tempoAnterior=0;
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
    if(brick.botaoApertado()){
        Serial.println("Botao apertado no inicio");
        Serial.println("calibrando sensores...");
        sensor1.calibrar();
        sensor2.calibrar();
    }
}

uint8_t contador = 0;
int16_t erro = 0;
void loop(){
    brick.atualiza(); //essa linha é obrigatória existir e ser a primeira do loop

    // sensor1.getRGBC(red, green, blue, clear);
    // // seguidor.getRGBCCalibrado(&red, &green, &blue, &clear,
    // //                         &red2, &green2, &blue2, &clear2);
    // //sensor1.getRawDataOneShot(&red, &green, &blue, &clear);
    // Serial.print("Sensor1 - R:");
    // Serial.print(red);
    // Serial.print(" G:");
    // Serial.print(green);
    // Serial.print(" B:");
    // Serial.print(blue);
    // Serial.print(" C:");
    // Serial.println(clear);
    
    // sensor2.getRGBC(red2, green2, blue2, clear2);
    // // //sensor2.getRawDataOneShot(&red, &green, &blue, &clear2);
    // Serial.print("Sensor2 - R:");
    // Serial.print(red2);
    // Serial.print(" G:");
    // Serial.print(green2);
    // Serial.print(" B:");
    // Serial.print(blue2);
    // Serial.print(" C:");
    // Serial.println(clear2);

    sensor3.getRGBC(red3, green3, blue3, clear3);
    // //sensor2.getRawDataOneShot(&red, &green, &blue, &clear2);
    Serial.print("Sensor3 - R:");
    Serial.print(red3);
    Serial.print(" G:");
    Serial.print(green3);
    Serial.print(" B:");
    Serial.print(blue3);
    Serial.print(" C:");
    Serial.println(clear3);

    // uint16_t dist = sensorDistancia.getDistancia();
    // Serial.print("Distancia: ");
    // Serial.print(dist);
    // Serial.println(" mm"); 


    // // uint16_t dist2 = sensorDistancia2.getDistancia();
    // // Serial.print("Distancia2: ");
    // // Serial.print(dist2);
    // // Serial.println(" mm"); 
    // int16_t distUltrassonico = ultrassonico.getDistancia();
    // Serial.print("Distancia Ultrassonico: ");
    // Serial.print(distUltrassonico);
    // Serial.println(" cm");

    // uint16_t dist3 = sensorDistancia3.getDistancia();
    // Serial.print("Distancia3: ");
    // Serial.print(dist3);
    // Serial.println(" mm");
    //brick.espera(1000);
    // //delay(50);
    // /*int dist = sensorDistancia.readRangeSingleMillimeters();
    // servos.moveServo(Portas::PORTA_SERVO_1, dist); //mapeia a distância para o ângulo do servo
    // sensor.getRawData(&red, &green, &blue, &clear);
    // Serial.print("Valores iniciais - R:");
    // Serial.print(red);
    // Serial.print(" G:");
    // Serial.print(green);
    // Serial.print(" B:");
    // Serial.print(blue);
    // Serial.print(" C:");
    // Serial.println(clear);
    // delay(10);
    // */
    // ledStrip.setLED(0, red/32, green/32, blue/32);
    // ledStrip.atualiza();
    // contador++;
    // if(millis() - tempoAnterior >= 1000){
    //     tempoAnterior = millis();
    //     Serial.println(contador);
    //     contador = 0;
    // }
    // if(brick.botaoApertado()){
    //     Serial.println("Botao apertado");
    //     buzzer.alerta();
    // }
    // erro = clear2 - clear;
    // erro = erro*1;
    // Serial.print("Erro: ");
    // Serial.println(erro);
    //delay(500);
    // brick.potenciaMotores(150+erro, 150-erro);
    // if(clear > 100 && clear2 > 100){
    //     brick.potenciaMotores(25, 25);
    // }else if (clear < 100){
    //     brick.potenciaMotores(0, -25);
    // }else if (clear2 < 100){
    //     brick.potenciaMotores(-25, 0);
    // }
}