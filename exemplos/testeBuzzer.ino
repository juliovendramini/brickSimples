/*
 * testeBuzzer.ino - Descobre como o buzzer (ou alto-falante) da porta S3 responde.
 *
 * Parte 1: toca tons puros (senoidais) de 200 Hz a 4000 Hz, todos com o mesmo
 *          volume, e escreve no monitor serial (115200) qual está tocando.
 *          Anote quais soam ALTO e quais quase não se ouvem.
 * Parte 2: fala "Olá! Eu sou o Paçoca." com volume 100, 200 e 300.
 *
 * Um buzzer costuma ter um pico forte em uma frequência (2 a 4 kHz) e quase
 * nada abaixo de 1 kHz, que é onde fica boa parte da voz. Com essas anotações
 * dá para ajustar as falas para o seu buzzer no gerar_falas.py.
 *
 * Ligações: módulo do buzzer (ou alto-falante) na porta S3.
 * Como usar: copie para o brickSimples.ino, grave e abra o monitor serial.
 */
#define SUPORTE_VOZ 1
#define SUPORTE_MUSICA 1
#include "brickSimples.h"
#include "falasAulas.h"

Voz voz = Voz(PORTA_BUZZER_3);
Musica musica = Musica(PORTA_BUZZER_3);

const uint16_t frequencias[] = {200, 300, 400, 500, 600, 800, 1000, 1200, 1500,
                                1800, 2000, 2200, 2500, 2800, 3000, 3300, 3600, 4000};
const uint8_t TOTAL = sizeof(frequencias) / sizeof(frequencias[0]);

void setup() {
    brick.inicializa();
    brick.adiciona(voz);
    brick.adiciona(musica);
    musica.timbre(Musica::SENOIDAL);
    musica.volume(100);
}

void loop() {
    Serial.println(F("Parte 1: tons de mesmo volume"));
    for (uint8_t i = 0; i < TOTAL; i++) {
        Serial.print(F("Tom "));
        Serial.print(i + 1);
        Serial.print(F(": "));
        Serial.print(frequencias[i]);
        Serial.println(F(" Hz"));
        musica.notaAteOFim(frequencias[i], 800);
        brick.espera(400);
    }

    brick.espera(1000);
    Serial.println(F("Parte 2: voz com volume 100, 200 e 300"));
    const uint16_t volumes[] = {100, 200, 300};
    for (uint8_t i = 0; i < 3; i++) {
        Serial.print(F("Volume "));
        Serial.println(volumes[i]);
        voz.volume(volumes[i]);
        voz.falar(FALA_OLA_PACOCA);
        brick.espera(800);
    }

    Serial.println(F("Fim. Recomeça em 5 s."));
    brick.espera(5000);
}
