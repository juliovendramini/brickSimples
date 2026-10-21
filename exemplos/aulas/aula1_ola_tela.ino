/*
 * Aula 1 - Olá, Paçoca! (versão com tela)
 *
 * O que faz: mostra uma mensagem na tela e, enquanto um botão do teclado
 * estiver apertado, acende o LED dele e escreve o número do botão.
 *
 * Ligações: tela na porta I2C-5 e teclado na porta I2C-3.
 *
 * A versão falada desta aula é a aula1_ola_voz.ino. As duas têm a mesma
 * lógica: só muda a saída (tela aqui, voz lá).
 *
 * Como usar: copie este arquivo para o brickSimples.ino (na pasta da
 * biblioteca), compile e grave.
 */

/* 1. Escolhemos o que vamos usar. Cada linha liga uma parte da biblioteca. */
#define SUPORTE_DISPLAY_SSD1306 1
#define SUPORTE_TECLADO 1
#include "brickSimples.h"

/* 2. Criamos um objeto para cada peça e dizemos em que porta ela está. */
SSD1306 tela = SSD1306(PORTA_I2C_5);
Teclado teclado = Teclado(PORTA_I2C_3);

/* O setup roda uma vez, quando o Paçoca liga. */
void setup() {
    brick.inicializa();          /* sempre a primeira linha do setup */
    brick.adiciona(tela);
    brick.adiciona(teclado);

    /* Fonte média: 10 colunas e 4 linhas (linhas 0, 1, 2 e 3).
     * A tela não tem acentos nem cedilha: escrevemos "Pacoca". */
    tela.setFonte(SSD1306::FONTE_MEDIA);
    tela.setCursor(0, 0);        /* coluna 0, linha 0 */
    tela.print("Ola!");
    tela.setCursor(0, 1);
    tela.print("Eu sou o");
    tela.setCursor(0, 2);
    tela.print("Pacoca");
    brick.espera(2000);          /* espera 2 segundos */

    tela.clear();                /* apaga a tela toda */
    tela.setCursor(0, 0);
    tela.print("Aperte um");
    tela.setCursor(0, 1);
    tela.print("botao");
}

/* O loop roda para sempre, uma vez depois da outra. */
void loop() {
    brick.atualiza();            /* sempre a primeira linha do loop */

    /* Olhamos os 4 botões, um de cada vez. A variável botao vale 1, 2, 3 e 4. */
    for (uint8_t botao = 1; botao <= 4; botao++) {
        if (teclado.leBotao(botao) == Teclado::APERTADO) {
            teclado.alteraLed(botao, true);      /* acende o LED do botão */
            tela.setCursor(0, 3);
            tela.print("Botao ");
            tela.print(botao);
        } else {
            teclado.alteraLed(botao, false);     /* apaga o LED do botão */
        }
    }
}

/*
 * Experimente:
 * 1. Troque a mensagem do começo pelo seu nome.
 * 2. Troque FONTE_MEDIA por FONTE_GRANDE (5 colunas e 2 linhas) e por
 *    FONTE_PEQUENA (21 colunas e 8 linhas). O que cabe em cada uma?
 * 3. Faça cada botão escrever uma palavra diferente na linha 3.
 *    Dica: as palavras têm tamanhos diferentes. O que sobra da palavra
 *    anterior? Como apagar a sobra?
 * 4. Pergunta: quantas vezes por segundo o loop escreve "Botao 1" enquanto
 *    você segura o botão? Crie uma variável que conta e mostre na tela.
 */
