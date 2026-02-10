#define SUPORTE_SENSOR_BMI160 1
//#define SUPORTE_SENSOR_GIROSCOPIO 1
// #define SUPORTE_SENSOR_LINHA 1
//#define SUPORTE_SENSOR_ULTRASSONICO 0
#define SUPORTE_SENSOR_TCS34725 1
#define SUPORTE_SENSOR_VL53L0X 1

#include "brickSimples.h"

TCS34725 sensor1 = TCS34725(PORTA_I2C_1);
TCS34725 sensor2 = TCS34725(PORTA_I2C_2);
// TCS34725 sensor3 = TCS34725(PORTA_I2C_3);
VL53L0X sensorDistancia = VL53L0X(PORTA_I2C_4);
// //VL53L0X sensorDistancia2 = VL53L0X(PORTA_I2C_5);
LEDStrip led1 = LEDStrip(PORTA_LED_4,1);
// Ultrassonico ultrassonico = Ultrassonico(PORTA_ULTRASSONICO_5);
// VL53L0X sensorDistancia3 = VL53L0X(PORTA_I2C_3);

BMI160 bmi160Sensor = BMI160(PORTA_I2C_5);
Buzzer buzzer = Buzzer(PORTA_BUZZER_3);

//Servo servo;
bool sensor1Detectado = false;
//Ultrassonico ultrassonico;
// Giroscopio giroscopio(PORTA_SERIAL_1);
//SensorLinha sensorLinha(PORTA_SERIAL_1);
// Bluetooth bluetooth(PORTA_SERIAL_3);

Motor Motor1 = Motor(PORTA_MOTOR_1, MOTOR_NORMAL);
Motor Motor2 = Motor(PORTA_MOTOR_2, MOTOR_INVERTIDO);


uint16_t red, green, blue, clear;
uint16_t red2, green2, blue2, clear2;
uint16_t red3, green3, blue3, clear3;
uint32_t tempoAnterior = 0;
uint8_t i=0;
void setup(){
    brick.inicializa(); //essa linha é obrigatória existir e ser a primeira do setup
    // bluetooth.begin();
    // brick.adiciona(giroscopio);
    brick.adiciona(Motor1, Motor2); //adiciona de uma vez (mas podemos fazer a função de adicionar somente um motor)
    brick.adiciona(led1); 
    brick.adiciona(buzzer);   
    brick.adiciona(bmi160Sensor);
    //brick.adiciona(sensorLinha);
    servos.iniciaServo(PORTA_SERVO_1);
    // servos.iniciaServo(PORTA_SERVO_3);
    //servos.iniciaServo(PORTA_SERVO_3);
    // giroscopio.setModo(BMI160_GYRO);    
    delay(500);
    servos.moveServo(PORTA_SERVO_1, 90); //move servo 1 para 0 graus
    
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
    brick.adiciona(sensorDistancia);
    //brick.adiciona(&sensorDistancia2);
    //brick.adiciona(&sensorDistancia3);
    brick.adiciona(sensor1);
    brick.adiciona(sensor2);
    //brick.adiciona(sensor3);
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
    led1.inicializa();
    //buzzer.sucesso();
    //buzzer.alerta();
    tempoAnterior=0;
    //ledStrip.demo();
    //ledStrip.arcoIrisRotativo();
    led1.setLED(0, 255, 0, 0); //define o led como vermelho
    led1.atualiza();
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
        //bmi160Sensor.calibrar();
        // giroscopio.calibrar();
        // sensor1.calibrar();
        // sensor2.calibrar();
    }
    brick.ativaLedInterno();
}

uint8_t contador = 0;
int16_t erro = 0;
void loop(){
    brick.atualiza(); //essa linha é obrigatória existir e ser a primeira do loop
    if(brick.botaoApertado()){
        bmi160Sensor.resetaZ();
    }
    brick.potenciaMotores(50, 50);
    Serial.print("Teste BMI160: ");
    Serial.print("Plano X: ");
    Serial.print(bmi160Sensor.getPlanoX());
    Serial.print(" Plano Y: "); 
    Serial.print(bmi160Sensor.getPlanoY());
    Serial.print(" Eixo Z: ");
    Serial.println(bmi160Sensor.getEixoZ());
    //Serial.println(bmi160Sensor.getAccelerationZ());
    // brick.espera(2000);
    // brick.potenciaMotores(-50, 50);
    // brick.espera(2000);

    // Serial.println("teste bluetooth");
    // bluetooth.println("Teste Bluetooth");
    // delay(1000);
    // if(bluetooth.available()){
    //     char c = bluetooth.read();
    //     Serial.print("Recebido via Bluetooth: ");
    //     Serial.println(c);
    // }
    // Serial.print("Linha1: ");
    // Serial.print(sensorLinha.getLinha(0));
    // Serial.print(" Linha2: ");
    // Serial.print(sensorLinha.getLinha(1));
    // Serial.print(" Linha3: ");
    // Serial.print(sensorLinha.getLinha(2));
    // Serial.print(" Linha4: ");
    // Serial.println(sensorLinha.getLinha(3));
    // Serial.print("SensorCor1: ");
    // Serial.print(" R:");
    // Serial.print(sensorLinha.getRedEsquerda());
    // Serial.print(" G:");
    // Serial.print(sensorLinha.getGreenEsquerda());
    // Serial.print(" B:");
    // Serial.print(sensorLinha.getBlueEsquerda());
    // Serial.print(" C:");
    // Serial.println(sensorLinha.getClearEsquerda());
    // delay(1000);
    // Serial.println(giroscopio.getAnguloX());
    // delay(100);
    sensor1.getRGBC(red, green, blue, clear);
    // // seguidor.getRGBCCalibrado(&red, &green, &blue, &clear,
    // //                         &red2, &green2, &blue2, &clear2);
    // //sensor1.getRawDataOneShot(&red, &green, &blue, &clear);
    Serial.print("Sensor1 - R:");
    Serial.print(red);
    Serial.print(" G:");
    Serial.print(green);
    Serial.print(" B:");
    Serial.print(blue);
    Serial.print(" C:");
    Serial.println(clear);
    
    sensor2.getRGBC(red2, green2, blue2, clear2);
    // //sensor2.getRawDataOneShot(&red, &green, &blue, &clear2);
    Serial.print("Sensor2 - R:");
    Serial.print(red2);
    Serial.print(" G:");
    Serial.print(green2);
    Serial.print(" B:");
    Serial.print(blue2);
    Serial.print(" C:");
    Serial.println(clear2);

    // sensor3.getRGBC(red3, green3, blue3, clear3);
    // // //sensor2.getRawDataOneShot(&red, &green, &blue, &clear2);
    // Serial.print("Sensor3 - R:");
    // Serial.print(red3);
    // Serial.print(" G:");
    // Serial.print(green3);
    // Serial.print(" B:");
    // Serial.print(blue3);
    // Serial.print(" C:");
    // Serial.println(clear3);

    uint16_t dist = sensorDistancia.getDistancia();
    Serial.print("Distancia: ");
    Serial.print(dist);
    Serial.println(" mm"); 
    if(dist < 100){
        brick.pararMotores();
        buzzer.alerta();
        led1.setLED(0, random(0, 256), random(0, 256), random(0, 256));
        led1.atualiza();
        int8_t x = rand()%2;
        if(x==0){
            brick.potenciaMotores(50, -50);
        }else{
            brick.potenciaMotores(-50, 50);
        }
        brick.espera(300);
    }

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
    delay(30);
}
