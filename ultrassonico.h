#ifndef _ULTRASSONICO_H_
#define _ULTRASSONICO_H_

#include <Arduino.h>
#include "portas.h"

class Ultrassonico {
private:
    uint8_t pinoTrig;
    uint8_t pinoEcho;
    bool inicializado;
    char descricao[5];
    
    // Timeout em microsegundos (19ms = ~3.25m de alcance máximo)
    const uint32_t TIMEOUT = 19000UL;
    
    // Constantes para cálculo de distância
    // Som viaja ~343m/s = 0.0343cm/µs
    // Distância = (tempo_ida_e_volta / 2) * velocidade_som
    // Distância(cm) = tempo(µs) / 58.0
    // Distância(mm) = tempo(µs) / 5.8
    
public:
    Ultrassonico(){
        inicializado = false;
    }
    
    ~Ultrassonico() {}
    
    // Inicializa o sensor ultrassônico em uma porta específica
    void inicializar(PortaUltrassonico porta) {
        pinoTrig = porta.trig;
        pinoEcho = porta.echo;
        strcpy(descricao, porta.descricao);
        
        pinMode(pinoTrig, OUTPUT);
        pinMode(pinoEcho, INPUT);
        digitalWrite(pinoTrig, LOW);
        
        inicializado = true;
        
        Serial.print(F("Sensor Ultrassonico HC-SR04 inicializado na porta "));
        Serial.println(descricao);
    }
    
    // Envia pulso de trigger (10µs)
    void enviarPulso() {
        digitalWrite(pinoTrig, LOW);
        delayMicroseconds(2);
        digitalWrite(pinoTrig, HIGH);
        delayMicroseconds(10);
        digitalWrite(pinoTrig, LOW);
    }
    
    // Lê a duração do pulso de echo em microsegundos
    uint32_t lerPulso() {
        return pulseIn(pinoEcho, HIGH, TIMEOUT);
    }
    
    // Mede a distância em centímetros
    float distanciaCm() {
        if(!inicializado) {
            Serial.println(F("Erro: Sensor ultrassonico nao inicializado!"));
            return -1.0;
        }
        
        enviarPulso();
        uint32_t duracao = lerPulso();
        
        if(duracao == 0) {
            // Timeout - objeto muito longe ou sem retorno
            return -1.0;
        }
        
        // Converte tempo para distância em cm
        float distancia = duracao / 58.0;
        return distancia;
    }
    
    // Mede a distância em milímetros
    float distanciaMm() {
        if(!inicializado) {
            Serial.println(F("Erro: Sensor ultrassonico nao inicializado!"));
            return -1.0;
        }
        
        enviarPulso();
        uint32_t duracao = lerPulso();
        
        if(duracao == 0) {
            return -1.0;
        }
        
        // Converte tempo para distância em mm
        float distancia = duracao / 5.8;
        return distancia;
    }
    
    // Mede a distância em centímetros (versão inteira)
    int distanciaCmInt() {
        float dist = distanciaCm();
        if(dist < 0) return -1;
        return (int)(dist + 0.5); // Arredonda
    }
    
    // Mede a distância em milímetros (versão inteira)
    int distanciaMmInt() {
        float dist = distanciaMm();
        if(dist < 0) return -1;
        return (int)(dist + 0.5); // Arredonda
    }
    
    // Faz múltiplas leituras e retorna a média (mais preciso)
    float distanciaCmMedia(uint8_t amostras = 5) {
        if(!inicializado || amostras == 0) return -1.0;
        
        float soma = 0;
        uint8_t leituras_validas = 0;
        
        for(uint8_t i = 0; i < amostras; i++) {
            float dist = distanciaCm();
            if(dist > 0) {
                soma += dist;
                leituras_validas++;
            }
            delay(10); // Pequeno delay entre leituras
        }
        
        if(leituras_validas == 0) return -1.0;
        
        return soma / leituras_validas;
    }
    
    // Faz múltiplas leituras e retorna a mediana (remove outliers)
    float distanciaCmMediana(uint8_t amostras = 5) {
        if(!inicializado || amostras == 0) return -1.0;
        
        float leituras[amostras];
        uint8_t leituras_validas = 0;
        
        // Coleta amostras
        for(uint8_t i = 0; i < amostras; i++) {
            float dist = distanciaCm();
            if(dist > 0) {
                leituras[leituras_validas++] = dist;
            }
            delay(10);
        }
        
        if(leituras_validas == 0) return -1.0;
        
        // Ordena (bubble sort simples para poucos elementos)
        for(uint8_t i = 0; i < leituras_validas - 1; i++) {
            for(uint8_t j = 0; j < leituras_validas - i - 1; j++) {
                if(leituras[j] > leituras[j + 1]) {
                    float temp = leituras[j];
                    leituras[j] = leituras[j + 1];
                    leituras[j + 1] = temp;
                }
            }
        }
        
        // Retorna mediana
        if(leituras_validas % 2 == 0) {
            return (leituras[leituras_validas/2 - 1] + leituras[leituras_validas/2]) / 2.0;
        } else {
            return leituras[leituras_validas/2];
        }
    }
    
    // Verifica se há obstáculo dentro de uma distância específica
    bool detectarObstaculo(float distanciaLimite) {
        float dist = distanciaCm();
        return (dist > 0 && dist <= distanciaLimite);
    }
    
    // Teste do sensor
    void teste(uint8_t segundos = 10) {
        Serial.println(F("=== TESTE SENSOR ULTRASSONICO ==="));
        Serial.print(F("Lendo distancia por "));
        Serial.print(segundos);
        Serial.println(F(" segundos..."));
        
        uint32_t inicio = millis();
        while(millis() - inicio < (segundos * 1000UL)) {
            float dist = distanciaCm();
            
            Serial.print(F("Distancia: "));
            if(dist < 0) {
                Serial.println(F("Fora de alcance ou erro"));
            } else {
                Serial.print(dist, 1);
                Serial.println(F(" cm"));
            }
            
            delay(500);
        }
        
        Serial.println(F("Teste concluido!"));
    }
};

#endif
