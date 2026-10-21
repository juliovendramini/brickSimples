/*
 * Aula 5 - Servo comandado pelo teclado (versão com tela)
 *
 * O que faz: os botões giram o servo motor e a tela mostra o ângulo, com
 * números e com uma barra.
 *   Botão 1: gira 10 graus para um lado (diminui o ângulo)
 *   Botão 2: gira 10 graus para o outro lado (aumenta o ângulo)
 *   Botão 3: volta para o meio (90 graus)
 *   Botão 4: vai para o próximo ângulo "redondo": 0, 45, 90, 135, 180
 *
 * Ligações: servo na porta S1, tela na porta I2C-5 e teclado na porta I2C-3.
 * Versão falada: aula5_servo_voz.ino (mesma lógica).
 *
 * Dica de montagem: cole um palito no braço do servo (um ponteiro) e um
 * semicírculo de papelão atrás, com marcas em 0, 45, 90, 135 e 180 graus.
 * Na versão falada, as marcas em relevo (cola quente) deixam o ponteiro
 * ser lido com os dedos.
 */
#define SUPORTE_DISPLAY_SSD1306 1
#define SUPORTE_TECLADO 1
#define SUPORTE_SERVO 1
#include "brickSimples.h"

SSD1306 tela = SSD1306(PORTA_I2C_5);
Teclado teclado = Teclado(PORTA_I2C_3);

int angulo = 90;
const int PASSO = 10;

/* Mesma função da aula 2: true só no momento em que o botão é apertado. */
uint8_t estadoAnterior[5] = {Teclado::LIBERADO, Teclado::LIBERADO, Teclado::LIBERADO,
                             Teclado::LIBERADO, Teclado::LIBERADO};
uint32_t tempoMudanca[5] = {0, 0, 0, 0, 0};

bool apertou(uint8_t botao) {
    uint8_t agora = teclado.leBotao(botao);
    if (agora == estadoAnterior[botao]) return false;
    if (millis() - tempoMudanca[botao] < 30) return false;
    estadoAnterior[botao] = agora;
    tempoMudanca[botao] = millis();
    return agora == Teclado::APERTADO;
}

void atualizaServo() {
    /* constrain mantém o valor entre 0 e 180 (o servo não passa disso) */
    angulo = constrain(angulo, 0, 180);
    servos.moveServo(PORTA_SERVO_1, angulo);

    tela.setCursor(0, 1);
    tela.print(angulo);
    tela.print(" graus  ");

    /* Barra de 10 caracteres: map converte 0..180 graus em 0..10 quadrados */
    int cheios = map(angulo, 0, 180, 0, 10);
    tela.setCursor(0, 2);
    for (int i = 0; i < 10; i++) {
        if (i < cheios) tela.print("#");
        else tela.print("-");
    }
}

void setup() {
    brick.inicializa();
    brick.adiciona(tela);
    brick.adiciona(teclado);
    servos.iniciaServo(PORTA_SERVO_1);

    tela.setFonte(SSD1306::FONTE_MEDIA);
    tela.setCursor(0, 0);
    tela.print("Servo S1");
    tela.setCursor(0, 3);
    tela.print("1- 2+ 3=90");
    atualizaServo();
}

void loop() {
    brick.atualiza();

    if (apertou(1)) {
        angulo = angulo - PASSO;
        atualizaServo();
    }
    if (apertou(2)) {
        angulo = angulo + PASSO;
        atualizaServo();
    }
    if (apertou(3)) {
        angulo = 90;
        atualizaServo();
    }
    if (apertou(4)) {
        /* próximo múltiplo de 45; depois de 180 volta para 0 */
        angulo = (angulo / 45 + 1) * 45;
        if (angulo > 180) angulo = 0;
        atualizaServo();
    }
}

/*
 * Experimente:
 * 1. Mude o PASSO para 1 e para 45. Qual é melhor para posicionar com precisão?
 * 2. Troque servos.moveServo por servos.moveServoTempo(PORTA_SERVO_1, angulo, 500).
 *    O que muda no movimento? E na resposta dos botões?
 * 3. Seu servo chega mesmo em 0 e em 180? Meça com o semicírculo.
 * 4. Desafio: segurando o botão 2, o servo continua girando devagar até
 *    você soltar (veja o desafio 4 da aula 2).
 */
