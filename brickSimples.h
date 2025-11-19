#include "portas.h"
#include "Servo.h"
#include "SoftWire.h"
#include "TCS34725.h"
#include "VL53L0X.h"
#include "led.h"
#include "ultrassonico.h"
#include "giroscopio.h"



class BrickSimples{
public:
    private:

    public:
    BrickSimples(){
    }
    ~BrickSimples(){
    }
    void inicializa(){
        Serial.begin(115200);
        
        // Configura Timer0 para PWM em ~244Hz (prescaler 256)
        // Timer0 controla PWM dos pinos 5 e 6 (motores)
        TCCR0B = (TCCR0B & 0b11111000) | 0x04; // Prescaler 256
        // Frequência PWM resultante: 16MHz / (256 * 256) = 244.14Hz
        
        //PINOS MOTOR ESQUERDO
        pinMode(6, OUTPUT);
        pinMode(7, OUTPUT);

        //PINOS MOTOR DIREITO
        pinMode(5, OUTPUT);
        pinMode(4, OUTPUT);
        Serial.println("Hello, Brick Simples!");
        Serial.print("Tensao da bateria: ");
        uint32_t tensao = analogRead(PINO_BATERIA);
        tensao = 4887 * tensao; //microvolt (estou fazendo isso para nao usar float)
        tensao = tensao / 1000; //milivolt
        Serial.print(tensao);
        Serial.println(" mV");
        if(tensao < 3100){ //milivolt
            Serial.println("Bateria fraca!");
            Serial.println("Coloque o brick para carregar e aguarde.");
            while(1);
        }
    }
    void espera(uint32_t ms){
        ::delay(ms/4);
    }


    uint32_t millis(){
        return ::millis()*4;
    }

    
    // Controla ambos os motores com a mesma potência
    // potencia: -255 a 255 (negativo = reverso, positivo = frente)
    void potenciaMotores(int potencia){
        potenciaMotorEsquerdo(potencia);
        potenciaMotorDireito(potencia);
    }

    // Controla motores independentemente
    // potenciaEsq, potenciaDir: -255 a 255
    void potenciaMotores(int potenciaEsq, int potenciaDir){
        potenciaMotorEsquerdo(potenciaEsq);
        potenciaMotorDireito(potenciaDir);
    }

    // Controla motor esquerdo (pinos 6 e 7)
    void potenciaMotorEsquerdo(int potencia){
        // Limita a potência entre -255 e 255
        potencia = constrain(potencia, -255, 255);
        
        if(potencia >= 0){
            // Frente: pino 6 com PWM, pino 7 LOW
            analogWrite(6, potencia);
            digitalWrite(7, LOW);
        } else {
            // Reverso: pino 6 LOW, pino 7 HIGH (sem PWM no pino 7)
            potencia = -potencia;
            analogWrite(6, 255 - potencia); //inverte o valor para evitar usar valor negativo no analogWrite
            digitalWrite(7, HIGH);
        }
    }

    // Controla motor direito (pinos 5 e 4)
    void potenciaMotorDireito(int potencia){
        // Limita a potência entre -255 e 255
        potencia = constrain(potencia, -255, 255);
        
        if(potencia >= 0){
            // Frente: pino 5 com PWM, pino 4 LOW
            analogWrite(5, potencia);
            digitalWrite(4, LOW);
        } else {
            // Reverso: pino 5 LOW, pino 4 HIGH (sem PWM no pino 4)
            potencia = -potencia;
            analogWrite(5, 255 - potencia); //inverte o valor para evitar usar valor negativo no analogWrite
            digitalWrite(4, HIGH);
        }
    }

    // Para ambos os motores
    void pararMotores(){
        digitalWrite(6, LOW);
        digitalWrite(7, LOW);
        digitalWrite(5, LOW);
        digitalWrite(4, LOW);
    }

    void frearMotores(){
        digitalWrite(6, HIGH);
        digitalWrite(7, HIGH);
        digitalWrite(5, HIGH);
        digitalWrite(4, HIGH);
    }

};


BrickSimples brick;

class Servos{
    private:
    Servo *servos;
    // const static uint8_t PINO_SERVO_1 = 13;
    // const static uint8_t PINO_SERVO_2 = 12;
    // const static uint8_t PINO_SERVO_3 = 11;
    // const static uint8_t PINO_SERVO_4 = 10;



    public:
    //definições de constantes
    // const static uint8_t PORTA_SERVO_1 = 1;
    // const static uint8_t PORTA_SERVO_2 = 2;
    // const static uint8_t PORTA_SERVO_3 = 3;
    // const static uint8_t PORTA_SERVO_4 = 4;

    Servos(){
        servos = new Servo[4]();
    }
    ~Servos(){
        delete[] servos;
    }
    void iniciaServo(PortaServo porta){
        if(porta.porta < 1 || porta.porta > 4){
            Serial.println(F("Erro: numero de porta de servo invalido!"));
            while(1);
        }  
        uint8_t pinoServo = 0;
        switch(porta.porta){
            case 1:
                pinoServo = porta.pino;
                break;
            case 2:
                pinoServo = porta.pino;
                break;
            case 3:
                pinoServo = porta.pino;
                break;
            case 4:
                pinoServo = porta.pino;
                break;
        }
        servos[porta.porta - 1].attach(pinoServo);
        Serial.print(F("Servo iniciado na porta "));
        Serial.println(porta.descricao);
    }
    void moveServo(PortaServo porta, int angulo){
        if(porta.porta < 1 || porta.porta > 4) return; //número inválido
        servos[porta.porta - 1].write(angulo);
    }
    void desanexaServo(PortaServo porta){
        if(porta.porta < 1 || porta.porta > 4) return; //número inválido
        servos[porta.porta - 1].detach();
    }

};


Servos servos;

class Buzzer{
    private:
    const static uint8_t PINO_BUZZER = 11; // PORTA_SERVO_3 - Timer2 OC2A
    bool inicializado;
    bool tocando;

    public:
    Buzzer() : inicializado(false), tocando(false) {}
    
    ~Buzzer() {
        if(inicializado) {
            parar();
        }
    }

    // Inicializa o buzzer na PORTA_SERVO_3 com lógica invertida
    void inicializar() {
        pinMode(PINO_BUZZER, OUTPUT);
        digitalWrite(PINO_BUZZER, HIGH); // Buzzer invertido: HIGH = desligado
        inicializado = true;
        Serial.println(F("Buzzer inicializado na PORTA_SERVO_3 (pino 11) - Logica invertida"));
    }

    // Emite um tom em frequência específica por um tempo específico
    // frequencia: Hz (ex: 440 para Lá, 262 para Dó, etc)
    // duracao: milissegundos (0 = som contínuo até chamar parar())
    void tocar(uint16_t frequencia, uint32_t duracao = 0) {
        if(!inicializado) {
            Serial.println(F("Erro: Buzzer nao inicializado!"));
            return;
        }
        
        // Gera PWM manual com lógica invertida
        uint32_t periodo = 1000000UL / frequencia; // período em microssegundos
        uint32_t tempoAlto = periodo / 2; // 50% duty cycle (HIGH = desligado)
        uint32_t tempoBaixo = periodo / 2; // 50% duty cycle (LOW = ligado)
        
        if(duracao > 0) {
            uint32_t inicio = millis();
            while(millis() - inicio < duracao) {
                digitalWrite(PINO_BUZZER, HIGH); // Buzzer desligado
                delayMicroseconds(tempoAlto);
                digitalWrite(PINO_BUZZER, LOW);  // Buzzer ligado
                delayMicroseconds(tempoBaixo);
            }
            digitalWrite(PINO_BUZZER, HIGH); // Garante que termina desligado
        } else {
            // Som contínuo (não recomendado, bloqueia o código)
            tocando = true;
            while(tocando) {
                digitalWrite(PINO_BUZZER, HIGH);
                delayMicroseconds(tempoAlto);
                digitalWrite(PINO_BUZZER, LOW);
                delayMicroseconds(tempoBaixo);
            }
        }
    }

    // Para o som
    void parar() {
        if(inicializado) {
            digitalWrite(PINO_BUZZER, HIGH); // Desligado para buzzer invertido
            tocando = false;
        }
    }

    // Toca uma melodia simples (beep curto)
    void beep(uint16_t frequencia = 1000, uint16_t duracao = 100) {
        tocar(frequencia, duracao);
    }

    // Toca um beep de alerta (dois beeps rápidos)
    void alerta() {
        beep(1500, 100);
        delay(100);
        beep(1500, 100);
    }

    // Toca um beep de sucesso (tom ascendente)
    void sucesso() {
        beep(523, 100);  // Dó
        delay(50);
        beep(659, 100);  // Mi
        delay(50);
        beep(784, 150);  // Sol
    }

    // Toca um beep de erro (tom descendente)
    void erro() {
        beep(784, 100);  // Sol
        delay(50);
        beep(659, 100);  // Mi
        delay(50);
        beep(523, 150);  // Dó
    }

    // Toca a melodia de Jingle Bells
    void jingleBells() {
        // Jingle bells, jingle bells, jingle all the way
        tocar(MI, 250);
        delay(100);;
        tocar(MI, 250);
        delay(100);;
        tocar(MI, 500);
        delay(100);;
        
        tocar(MI, 250);
        delay(100);;
        tocar(MI, 250);
        delay(100);;
        tocar(MI, 500);
        delay(100);;
        
        tocar(MI, 250);
        delay(100);;
        tocar(SOL, 250);
        delay(100);;
        tocar(DO, 375);
        delay(125);
        tocar(RE, 125);
        delay(125);
        tocar(MI, 500);
        delay(500);
        
        // Oh what fun it is to ride
        tocar(FA, 250);
        delay(100);;
        tocar(FA, 250);
        delay(100);;
        tocar(FA, 375);
        delay(125);
        tocar(FA, 125);
        delay(125);
        tocar(FA, 250);
        delay(100);;
        tocar(MI, 250);
        delay(100);;
        tocar(MI, 250);
        delay(125);
        tocar(MI, 125);
        delay(125);
        
        tocar(MI, 250);
        delay(100);;
        tocar(RE, 250);
        delay(100);;
        tocar(RE, 250);
        delay(100);;
        tocar(MI, 250);
        delay(100);;
        tocar(RE, 500);
        delay(100);;
        tocar(SOL, 500);
        delay(500);
    }

    // Toca o tema dos Power Rangers (Go Go Power Rangers!)
    void powerRangers() {
        // "Go Go Power Rangers!"
        // Parte característica: Sol Sol La Sol Mi Re
        tocar(SOL, 150);
        delay(50);
        tocar(SOL, 150);
        delay(50);
        tocar(LA, 150);
        delay(50);
        tocar(SOL, 200);
        delay(50);
        tocar(MI, 200);
        delay(50);
        tocar(RE, 300);
        delay(200);
        
        // Repetição mais intensa
        tocar(SOL, 150);
        delay(50);
        tocar(SOL, 150);
        delay(50);
        tocar(LA, 150);
        delay(50);
        tocar(SOL, 200);
        delay(50);
        tocar(MI, 200);
        delay(50);
        tocar(RE, 300);
        delay(200);
        
        // Parte final energética
        tocar(DO_ALTO, 200);
        delay(50);
        tocar(SI, 200);
        delay(50);
        tocar(LA, 200);
        delay(50);
        tocar(SOL, 200);
        delay(50);
        tocar(LA, 150);
        delay(50);
        tocar(SOL, 150);
        delay(50);
        tocar(MI, 400);
        delay(100);
    }

    // Notas musicais (frequências em Hz)
    enum Notas {
        DO = 262,
        DO_S = 277,
        RE = 294,
        RE_S = 311,
        MI = 330,
        FA = 349,
        FA_S = 370,
        SOL = 392,
        SOL_S = 415,
        LA = 440,
        LA_S = 466,
        SI = 494,
        DO_ALTO = 523
    };
};

Buzzer buzzer;

