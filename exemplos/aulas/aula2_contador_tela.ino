/*
 * Aula 2 - Contador: apertar é diferente de segurar (versão com tela)
 *
 * O que faz: um contador de 0 a 255 controlado pelo teclado.
 *   Botão 1: diminui 1
 *   Botão 2: aumenta 1
 *   Botão 3: volta para zero
 *   Botão 4: mostra ou esconde o número em binário
 * Os 4 LEDs do teclado mostram os 4 últimos bits do número (o LED 4 é o bit 0).
 *
 * Ligações: tela na porta I2C-5 e teclado na porta I2C-3.
 * Versão falada: aula2_contador_voz.ino (mesma lógica).
 *
 * Ideia principal: o loop roda dezenas de vezes por segundo. Se a gente
 * somasse 1 sempre que o botão está APERTADO, um toque rápido somaria
 * vários. Por isso contamos só no momento em que o botão PASSA de solto
 * para apertado (a "borda"). Quem faz isso é a função apertou().
 */
#define SUPORTE_DISPLAY_SSD1306 1
#define SUPORTE_TECLADO 1
#include "brickSimples.h"

SSD1306 tela = SSD1306(PORTA_I2C_3);
Teclado teclado = Teclado(PORTA_I2C_3);

int contador = 0;
bool mostraBinario = false;

/*
 * apertou(botao) devolve true só uma vez a cada toque.
 * Guardamos como o botão estava na última vez (estadoAnterior) e só
 * respondemos quando ele estava solto e agora está apertado.
 * Também ignoramos mudanças muito rápidas (menos de 30 ms): o contato do
 * botão "pula" algumas vezes quando encosta (isso se chama bounce).
 */
uint8_t estadoAnterior[5] = {Teclado::LIBERADO, Teclado::LIBERADO, Teclado::LIBERADO,
                             Teclado::LIBERADO, Teclado::LIBERADO};
uint32_t tempoMudanca[5] = {0, 0, 0, 0, 0};

bool apertou(uint8_t botao) {
    uint8_t agora = teclado.leBotao(botao);
    if (agora == estadoAnterior[botao]) return false;          /* nada mudou */
    if (millis() - tempoMudanca[botao] < 30) return false;     /* bounce: ignora */
    estadoAnterior[botao] = agora;
    tempoMudanca[botao] = millis();
    return agora == Teclado::APERTADO;                         /* true só ao apertar */
}

void mostraContador() {
    tela.setCursor(0, 1);
    tela.print(contador);
    tela.print("   ");          /* apaga o que sobrou de um número maior */

    tela.setCursor(0, 2);
    if (mostraBinario) {
        /* do bit 7 (o que vale 128) até o bit 0 (o que vale 1) */
        for (int bit = 7; bit >= 0; bit--) {
            tela.print((contador >> bit) & 1);
        }
    } else {
        tela.print("        ");   /* 8 espaços */
    }

    /* LEDs: os 4 bits mais baixos. LED 4 = bit 0, LED 1 = bit 3 */
    for (int bit = 0; bit < 4; bit++) {
        teclado.alteraLed(4 - bit, (contador >> bit) & 1);
    }
}

void setup() {
    brick.inicializa();
    brick.adiciona(tela);
    brick.adiciona(teclado);

    tela.setFonte(SSD1306::FONTE_MEDIA);
    tela.setCursor(0, 0);
    tela.print("Contador");
    tela.setCursor(0, 3);
    tela.print("1- 2+ 3=0");
    mostraContador();
}

void loop() {
    brick.atualiza();

    if (apertou(1)) {
        if (contador > 0) contador--;
        mostraContador();
    }
    if (apertou(2)) {
        if (contador < 255) contador++;
        mostraContador();
    }
    if (apertou(3)) {
        contador = 0;
        mostraContador();
    }
    if (apertou(4)) {
        mostraBinario = !mostraBinario;     /* troca: true vira false e vice-versa */
        mostraContador();
    }
}

/*
 * Experimente:
 * 1. Troque apertou(2) por teclado.leBotao(2) == Teclado::APERTADO. Dê um
 *    toque rápido no botão 2. Quanto o contador sobe? Por quê?
 * 2. Tire a linha do bounce (a dos 30 ms). Dá para perceber diferença?
 * 3. Quando o contador chega em 255, mostre "MAX" na linha 2 por 1 segundo.
 * 4. Desafio: segurando o botão 2 por mais de meio segundo, o contador
 *    passa a subir sozinho, 5 vezes por segundo (como o volume da TV).
 * 5. Desafio: conte de 0 a 15 e leia o número só olhando os LEDs.
 */
