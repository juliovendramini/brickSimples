/*
 * Aula 1 - Olá, Paçoca! (versão com voz)
 *
 * O que faz: o Paçoca se apresenta falando e, enquanto um botão do teclado
 * estiver apertado, acende o LED dele e fala o número do botão.
 *
 * Ligações: buzzer passivo na porta S3 (a voz só funciona nela) e teclado
 * na porta I2C-3.
 *
 * A versão com tela desta aula é a aula1_ola_tela.ino. As duas têm a mesma
 * lógica: só muda a saída (voz aqui, tela lá).
 *
 * Como usar: copie este arquivo para o brickSimples.ino (na pasta da
 * biblioteca), compile e grave. As falas estão no arquivo falasAulas.h.
 */

/* 1. Escolhemos o que vamos usar. Cada linha liga uma parte da biblioteca. */
#define SUPORTE_VOZ 1
#define SUPORTE_VOZ_NUMEROS 1        /* para falar números: voz.falarNumero() */
#define SUPORTE_TECLADO 1
#include "brickSimples.h"
#include "falasAulas.h"              /* as frases gravadas (FALA_...) */

/* 2. Criamos um objeto para cada peça e dizemos em que porta ela está. */
Voz voz = Voz(PORTA_BUZZER_3);
Teclado teclado = Teclado(PORTA_I2C_3);

/* O setup roda uma vez, quando o Paçoca liga. */
void setup() {
    brick.inicializa();          /* sempre a primeira linha do setup */
    brick.adiciona(voz);
    brick.adiciona(teclado);

    /* voz.falar() fala e só passa para a próxima linha quando termina. */
    voz.falar(FALA_OLA_PACOCA);          /* "Olá! Eu sou o Paçoca." */
    voz.falar(FALA_APERTE_UM_BOTAO);     /* "Aperte um botão para começar." */
}

/* O loop roda para sempre, uma vez depois da outra. */
void loop() {
    brick.atualiza();            /* sempre a primeira linha do loop */

    /* Olhamos os 4 botões, um de cada vez. A variável botao vale 1, 2, 3 e 4. */
    for (uint8_t botao = 1; botao <= 4; botao++) {
        if (teclado.leBotao(botao) == Teclado::APERTADO) {
            teclado.alteraLed(botao, true);      /* acende o LED do botão */
            voz.falar(FALA_BOTAO);               /* "Botão" */
            voz.falarNumero(botao);              /* "um", "dois", "três" ou "quatro" */
        } else {
            teclado.alteraLed(botao, false);     /* apaga o LED do botão */
        }
    }
}

/*
 * Experimente:
 * 1. Segure o botão 1 apertado. O que acontece? Por que o Paçoca repete?
 *    (Guarde a resposta: é o assunto da aula 2.)
 * 2. Faça o botão 4 falar a apresentação (FALA_OLA_PACOCA) em vez do número.
 * 3. Use voz.falarNumero() para falar outros números: 7, 15, 100, 2026.
 *    Até que número ele fala? E números negativos?
 * 4. O volume pode ir de 100 a 400: coloque voz.volume(300); no setup.
 */
