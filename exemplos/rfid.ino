/*
 * Exemplo: leitor RFID MFRC522 I2C (nulllab, endereço 0x28) no Paçoca.
 *
 * Ligue o leitor em qualquer porta I2C (PORTA_I2C_1 a PORTA_I2C_5).
 * Dá para usar até 5 leitores ao mesmo tempo, um em cada porta.
 *
 * Para usar, copie este conteúdo para o brickSimples.ino (o Arduino só
 * compila o .ino da pasta principal).
 */
#define SUPORTE_MOTOR 1
#define SUPORTE_RFID 1

#include "brickSimples.h"

RFID rfid = RFID(PORTA_I2C_2);

// UID de um cartão conhecido (descubra o seu com rfid.imprimeUid())
const uint32_t CARTAO_PROFESSOR = 0xDEADBEEF;

void setup() {
    brick.inicializa();              // obrigatório, sempre a primeira linha
    brick.adiciona(rfid);
    // rfid.ganhoAntena(7);          // mais alcance (padrão do chip: 4)
    Serial.println(F("Aproxime um cartao..."));
}

void loop() {
    if (rfid.cartaoNovo()) {         // true uma vez para cada cartão que chega
        rfid.imprimeUid();
        Serial.print(F("Tipo: "));
        Serial.println(rfid.nomeTipo());

        if (rfid.uidIgual(CARTAO_PROFESSOR)) {
            Serial.println(F("Ola, professor!"));
        }

        // Lê o texto gravado no bloco 1 (MIFARE Classic ou NTAG)
        char texto[17];
        if (rfid.lerTexto(1, texto) == RFID::OK) {
            Serial.print(F("Texto no bloco 1: "));
            Serial.println(texto);
        } else {
            Serial.print(F("Nao consegui ler: "));
            Serial.println(rfid.textoStatus(rfid.ultimoStatus()));
        }

        // Para gravar (o cartão precisa continuar perto):
        // if (rfid.escreveTexto(1, "VITIMA") == RFID::OK) Serial.println(F("Gravado"));
    }
    delay(50);
}
