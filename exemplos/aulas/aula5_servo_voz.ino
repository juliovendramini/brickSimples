/*
 * Aula 5 - Servo comandado pelo teclado (versão com voz)
 *
 * O que faz: os botões giram o servo motor e o Paçoca fala o ângulo.
 *   Botão 1: gira 10 graus para um lado (diminui o ângulo)
 *   Botão 2: gira 10 graus para o outro lado (aumenta o ângulo)
 *   Botão 3: volta para o meio (90 graus)
 *   Botão 4: vai para o próximo ângulo "redondo": 0, 45, 90, 135, 180
 * O ângulo é falado quando você para de apertar (meio segundo depois do
 * último toque), para não falar cada passo.
 *
 * Ligações: servo na porta S1, buzzer passivo na porta S3 e teclado na
 * porta I2C-3.
 * Versão com tela: aula5_servo_tela.ino (mesma lógica).
 *
 * Dica de montagem: cole um palito no braço do servo (um ponteiro) e um
 * semicírculo de papelão atrás, com marcas em relevo (cola quente) em 0,
 * 45, 90, 135 e 180 graus. Assim o ponteiro pode ser lido com os dedos.
 *
 * Atenção: a voz e o servo usam o mesmo temporizador do microcontrolador
 * (o Timer2). Enquanto o Paçoca fala, o servo fica sem receber pulsos e
 * fica "solto". Por isso o servo se move primeiro e a fala vem depois.
 */
#define SUPORTE_VOZ 1
#define SUPORTE_VOZ_NUMEROS 1
#define SUPORTE_TECLADO 1
#define SUPORTE_SERVO 1
#include "brickSimples.h"
#include "falasAulas.h"

Voz voz = Voz(PORTA_BUZZER_3);
Teclado teclado = Teclado(PORTA_I2C_3);

int angulo = 90;
const int PASSO = 10;
bool precisaFalar = false;        /* tem ângulo novo para falar? */
uint32_t ultimoToque = 0;         /* quando foi o último toque (millis) */

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
    voz.parar();                  /* se estava falando, cala para o servo poder se mover */
    if (angulo < 0) {
        angulo = 0;
        voz.iniciaFala(FALA_MINIMO);
    }
    if (angulo > 180) {
        angulo = 180;
        voz.iniciaFala(FALA_MAXIMO);
    }
    servos.moveServo(PORTA_SERVO_1, angulo);
    precisaFalar = true;
    ultimoToque = millis();
}

void falaAngulo() {
    voz.iniciaNumero(angulo);     /* "noventa ... */
    voz.enfileirar(FALA_GRAUS);   /*  ... graus." */
}

void setup() {
    brick.inicializa();
    brick.adiciona(voz);
    brick.adiciona(teclado);
    servos.iniciaServo(PORTA_SERVO_1);
    servos.moveServo(PORTA_SERVO_1, angulo);
    brick.espera(500);            /* dá tempo de o servo chegar antes de falar */
    falaAngulo();
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

    /* Fala meio segundo depois do último toque (o servo já chegou). */
    if (precisaFalar && millis() - ultimoToque > 500) {
        precisaFalar = false;
        falaAngulo();
    }
}

/*
 * Experimente:
 * 1. Mude o PASSO para 1 e para 45. Qual é melhor para posicionar com precisão?
 * 2. Mude o 500 do "meio segundo depois" para 0. O que acontece com o servo?
 *    Por quê? (Releia a "Atenção" lá em cima.)
 * 3. Com o ponteiro e o semicírculo: peça para um colega escolher um ângulo,
 *    você chega nele só pelo tato e confere ouvindo.
 * 4. Desafio: segurando o botão 2, o servo continua girando devagar até
 *    você soltar, e só então fala o ângulo.
 */
