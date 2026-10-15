/*
 * Exemplo: DHT22/DHT11 (temperatura e umidade) e DS18B20 (temperatura)
 * nas portas de 1 fio digital do Paçoca (as mesmas dos servos: S1 a S4).
 *
 * Para usar, copie este conteúdo para o brickSimples.ino (o Arduino só
 * compila o .ino da pasta principal).
 */
#define SUPORTE_MOTOR 1
#define SUPORTE_DHT 1
#define SUPORTE_DS18B20 1

#include "brickSimples.h"

DHT dht = DHT(PORTA_DHT_1, DHT22);                // ou DHT11
DS18B20 termometro = DS18B20(PORTA_DS18B20_2);   // até 4 DS18B20 no mesmo fio

void setup() {
    brick.inicializa();              // obrigatório, sempre a primeira linha
    brick.adiciona(dht);
    brick.adiciona(termometro);      // mostra quantos DS18B20 achou na porta
    termometro.imprimeEnderecos();
    // termometro.resolucao(10);     // 0,25 °C em 190 ms (padrão: 12 bits, 750 ms)
}

void loop() {
    // Pode chamar sempre: entre uma leitura e outra do sensor a biblioteca
    // devolve o último valor, sem travar o programa.
    float t = dht.temperatura();
    float u = dht.umidade();
    Serial.print(F("DHT: "));
    if (dht.ok()) {
        Serial.print(t, 1);
        Serial.print(F(" C, "));
        Serial.print(u, 1);
        Serial.print(F(" %  sensacao "));
        Serial.print(dht.sensacaoTermica(), 1);
        Serial.print(F(" C"));
    } else {
        Serial.print(dht.textoErro());
    }

    Serial.print(F("  |  DS18B20:"));
    for (uint8_t i = 0; i < termometro.quantidade(); i++) {
        Serial.print(' ');
        Serial.print(termometro.temperatura(i), 2);
        Serial.print(F(" C"));
    }
    Serial.println();

    delay(500);
}
