#ifndef _SENSOR_LINHA_H_
#define _SENSOR_LINHA_H_

#include <Arduino.h>
#include "SoftwareSerial.h"

// Modos de operação do sensor de linha
#define MODO_RGB_HSV_4X      0  // Modo RGB/HSV com gain 4X
#define MODO_RGB_HSV_16X     1  // Modo RGB/HSV com gain 16X
#define MODO_RGB_HSV_AUTO    2  // Modo RGB/HSV com calibração automática
#define MODO_CALIBRA_BRANCO  3  // Calibração branco
#define MODO_CALIBRA_PRETO   4  // Calibração preto
#define MODO_RAW_AUTO        5  // Modo raw com calibração
#define MODO_RAW_4X          6  // Modo raw 4X
#define MODO_RAW_16X         7  // Modo raw 16X

// Definições de cores
enum Cor { 
    COR_PRETO = 0, 
    COR_BRANCO = 1, 
    COR_VERMELHO = 2, 
    COR_VERDE = 3, 
    COR_AZUL = 4, 
    COR_AMARELO = 5, 
    COR_INDEFINIDA = -1 
};

class SensorLinha {
private:
    SoftwareSerial* serial;
    uint8_t pinoRX;
    uint8_t pinoTX;
    bool inicializado;
    uint8_t modoAtual;
    uint32_t baudRate;
    char descricaoPorta[10];
    
    // Buffer para receber dados
    uint8_t bufferRx[32];  // 32 bytes: 4 sensores linha + 3 sensores cor RGB (4 valores cada)
    
    // Valores dos sensores de linha (0-100)
    uint8_t linha[4];      // 4 sensores de linha
    
    // Valores dos sensores de cor (RGB + Clear)
    // Sensor esquerdo (TCS2)
    uint8_t r_esq, g_esq, b_esq, c_esq;
    // Sensor meio (TCSMeio)  
    uint8_t r_meio, g_meio, b_meio, c_meio;
    // Sensor direito (TCS1)
    uint8_t r_dir, g_dir, b_dir, c_dir;
    
    uint32_t ultimaAtualizacao;
    
    // Timeout para leitura
    const uint32_t TIMEOUT = 66; // ms
    
public:
    SensorLinha(PortaSerial porta, uint32_t baudRate = 115200) 
        : serial(nullptr), pinoRX(0), pinoTX(0), inicializado(false), modoAtual(MODO_RGB_HSV_AUTO) {
        pinoRX = porta.rx;
        pinoTX = porta.tx;
        this->baudRate = baudRate;
        strcpy(this->descricaoPorta, porta.descricao);
        ultimaAtualizacao = 0;
        
        // Inicializa arrays
        for(uint8_t i = 0; i < 4; i++) {
            linha[i] = 0;
        }
        r_esq = g_esq = b_esq = c_esq = 0;
        r_meio = g_meio = b_meio = c_meio = 0;
        r_dir = g_dir = b_dir = c_dir = 0;
    }
    
    ~SensorLinha() {
        if(serial != nullptr) {
            delete serial;
        }
    }
    
    // Inicializa comunicação serial com o sensor
    void inicializa() {
        serial = new SoftwareSerial(pinoRX, pinoTX);
        serial->begin(baudRate);
        serial->setTimeout(10); // 10ms timeout
        
        inicializado = true;
        modoAtual = MODO_RGB_HSV_AUTO;
        
        Serial.print(F("SensorLinha Serial inicializado na porta "));
        Serial.print(descricaoPorta);
        Serial.print(F(" (RX="));
        Serial.print(pinoRX);
        Serial.print(F(", TX="));
        Serial.print(pinoTX);
        Serial.print(F(") @ "));
        Serial.print(baudRate);
        Serial.println(F(" baud"));
        
        delay(100);
        setModo(MODO_RGB_HSV_AUTO);
    }
    
    // Define o modo de operação
    void setModo(uint8_t modo) {
        if(!inicializado) {
            Serial.println(F("Erro: SensorLinha nao inicializado!"));
            return;
        }
        
        if(modo > 7) {
            Serial.println(F("Erro: Modo invalido! Use 0-7"));
            return;
        }
        
        modoAtual = modo;
    }
    
    // Calibra branco
    bool calibrarBranco() {
        if(!inicializado) return false;
        
        Serial.println(F("Calibrando branco..."));
        uint8_t modoAnterior = modoAtual;
        setModo(MODO_CALIBRA_BRANCO);
        
        uint16_t timeout_count = 0;
        uint8_t confirmacao = 0;
        
        cli();
        serial->write(MODO_CALIBRA_BRANCO);
        
        while(timeout_count++ < 30000) {
            if(serial->available()) {
                confirmacao = serial->read();
                break;
            }
        }
        sei();
        
        setModo(modoAnterior);
        
        if(confirmacao == 1) {
            Serial.println(F("Calibracao branco concluida!"));
            return true;
        } else {
            Serial.println(F("Erro na calibracao branco!"));
            return false;
        }
    }
    
    // Calibra preto
    bool calibrarPreto() {
        if(!inicializado) return false;
        
        Serial.println(F("Calibrando preto..."));
        uint8_t modoAnterior = modoAtual;
        setModo(MODO_CALIBRA_PRETO);
        
        uint16_t timeout_count = 0;
        uint8_t confirmacao = 0;
        
        cli();
        serial->write(MODO_CALIBRA_PRETO);
        
        while(timeout_count++ < 30000) {
            if(serial->available()) {
                confirmacao = serial->read();
                break;
            }
        }
        sei();
        
        setModo(modoAnterior);
        
        if(confirmacao == 1) {
            Serial.println(F("Calibracao preto concluida!"));
            return true;
        } else {
            Serial.println(F("Erro na calibracao preto!"));
            return false;
        }
    }
    
    // Lê os dados do sensor (solicita e aguarda resposta)
    bool lerDados() {
        if(!inicializado) return false;
        
        uint16_t inicio = 1;
        uint8_t bytesRecebidos = 0;
        
        cli();
        serial->write(modoAtual);
        
        while(inicio++) {
            if(serial->available()) {
                bytesRecebidos = serial->readBytes(bufferRx, 32);
                break;
            }
        }
        sei();
        
        if(bytesRecebidos < 32) {
            Serial.println(F("Timeout: Dados incompletos do SensorLinha"));
            return false;
        }
        
        // Decodifica os dados recebidos
        // Bytes 0-3: Sensores de linha (0-100)
        linha[0] = bufferRx[0];
        linha[1] = bufferRx[1];
        linha[2] = bufferRx[2];
        linha[3] = bufferRx[3];
        
        // Bytes 4-7: Sensor cor esquerdo (TCS2) - RGBC
        r_esq = bufferRx[4];
        g_esq = bufferRx[5];
        b_esq = bufferRx[6];
        c_esq = bufferRx[7];
        
        // Bytes 8-11: Sensor cor meio (TCSMeio) - RGBC
        r_meio = bufferRx[8];
        g_meio = bufferRx[9];
        b_meio = bufferRx[10];
        c_meio = bufferRx[11];
        
        // Bytes 12-15: Sensor cor direito (TCS1) - RGBC
        r_dir = bufferRx[12];
        g_dir = bufferRx[13];
        b_dir = bufferRx[14];
        c_dir = bufferRx[15];
        
        // Bytes 16-31: Dados adicionais (luz ambiente, etc)
        
        ultimaAtualizacao = millis();
        return true;
    }
    
    // Atualiza dados com timeout automático
    void atualizaDadosTimeOut() {
        if(millis() - ultimaAtualizacao > TIMEOUT) {
            lerDados();
        }
    }
    
    // ===== Métodos para sensores de linha =====
    
    // Retorna valor do sensor de linha (0-100, 0=branco, 100=preto)
    uint8_t getLinha(uint8_t sensor) {
        if(sensor >= 4) return 0;
        atualizaDadosTimeOut();
        return linha[sensor];
    }
    
    // Retorna array com todos os sensores de linha
    void getLinhas(uint8_t valores[4]) {
        atualizaDadosTimeOut();
        for(uint8_t i = 0; i < 4; i++) {
            valores[i] = linha[i];
        }
    }
    
    // Retorna posição estimada da linha (-150 a +150, 0=centro)
    int16_t getPosicaoLinha() {
        atualizaDadosTimeOut();
        
        // Calcula posição ponderada
        int32_t soma_ponderada = 0;
        int32_t soma_valores = 0;
        
        // Posições: -150, -50, +50, +150
        int16_t posicoes[4] = {-150, -50, 50, 150};
        
        for(uint8_t i = 0; i < 4; i++) {
            soma_ponderada += (int32_t)linha[i] * posicoes[i];
            soma_valores += linha[i];
        }
        
        if(soma_valores == 0) return 0;
        
        return soma_ponderada / soma_valores;
    }
    
    // ===== Métodos para sensores de cor =====
    
    // Sensor esquerdo
    void getCorEsquerda(uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &c) {
        atualizaDadosTimeOut();
        r = r_esq; g = g_esq; b = b_esq; c = c_esq;
    }
    
    uint8_t getRedEsquerda() { atualizaDadosTimeOut(); return r_esq; }
    uint8_t getGreenEsquerda() { atualizaDadosTimeOut(); return g_esq; }
    uint8_t getBlueEsquerda() { atualizaDadosTimeOut(); return b_esq; }
    uint8_t getClearEsquerda() { atualizaDadosTimeOut(); return c_esq; }
    
    // Sensor meio
    void getCorMeio(uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &c) {
        atualizaDadosTimeOut();
        r = r_meio; g = g_meio; b = b_meio; c = c_meio;
    }
    
    uint8_t getRedMeio() { atualizaDadosTimeOut(); return r_meio; }
    uint8_t getGreenMeio() { atualizaDadosTimeOut(); return g_meio; }
    uint8_t getBlueMeio() { atualizaDadosTimeOut(); return b_meio; }
    uint8_t getClearMeio() { atualizaDadosTimeOut(); return c_meio; }
    
    // Sensor direito
    void getCorDireita(uint8_t &r, uint8_t &g, uint8_t &b, uint8_t &c) {
        atualizaDadosTimeOut();
        r = r_dir; g = g_dir; b = b_dir; c = c_dir;
    }
    
    uint8_t getRedDireita() { atualizaDadosTimeOut(); return r_dir; }
    uint8_t getGreenDireita() { atualizaDadosTimeOut(); return g_dir; }
    uint8_t getBlueDireita() { atualizaDadosTimeOut(); return b_dir; }
    uint8_t getClearDireita() { atualizaDadosTimeOut(); return c_dir; }
    
    // Detecta cor predominante (simplificado)
    Cor detectarCor(uint8_t r, uint8_t g, uint8_t b, uint8_t c) {
        // Se muito escuro, é preto
        if(c < 10) return COR_PRETO;
        
        // Se muito claro, é branco
        if(c > 100) return COR_BRANCO;
        
        // Detecta cor predominante
        if(r > g && r > b) {
            if(g > b * 1.5) return COR_AMARELO;  // Amarelo = vermelho + verde
            return COR_VERMELHO;
        }
        if(g > r && g > b) return COR_VERDE;
        if(b > r && b > g) return COR_AZUL;
        
        return COR_INDEFINIDA;
    }
    
    Cor getCorEsquerda() { return detectarCor(r_esq, g_esq, b_esq, c_esq); }
    Cor getCorMeio() { return detectarCor(r_meio, g_meio, b_meio, c_meio); }
    Cor getCorDireita() { return detectarCor(r_dir, g_dir, b_dir, c_dir); }
    
    // ===== Métodos auxiliares =====
    
    // Imprime dados no Serial Monitor
    void imprimirDados() {
        Serial.print(F("Linha: ["));
        for(uint8_t i = 0; i < 4; i++) {
            Serial.print(linha[i]);
            if(i < 3) Serial.print(F(", "));
        }
        Serial.print(F("] | Pos: "));
        Serial.print(getPosicaoLinha());
        
        Serial.print(F(" | Esq(RGBC): "));
        Serial.print(r_esq); Serial.print(F(","));
        Serial.print(g_esq); Serial.print(F(","));
        Serial.print(b_esq); Serial.print(F(","));
        Serial.print(c_esq);
        
        Serial.print(F(" | Meio(RGBC): "));
        Serial.print(r_meio); Serial.print(F(","));
        Serial.print(g_meio); Serial.print(F(","));
        Serial.print(b_meio); Serial.print(F(","));
        Serial.print(c_meio);
        
        Serial.print(F(" | Dir(RGBC): "));
        Serial.print(r_dir); Serial.print(F(","));
        Serial.print(g_dir); Serial.print(F(","));
        Serial.print(b_dir); Serial.print(F(","));
        Serial.println(c_dir);
    }
    
    // Verifica se está inicializado
    bool estaInicializado() {
        return inicializado;
    }
    
    // Retorna o modo atual
    uint8_t getModoAtual() {
        return modoAtual;
    }
};

#endif
