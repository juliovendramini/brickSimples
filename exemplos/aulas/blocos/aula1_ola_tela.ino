#define SUPORTE_DISPLAY_SSD1306 1
#define SUPORTE_TECLADO 1

#include "brickSimples.h"
SSD1306 tela_porta_5(PORTA_I2C_5);
Teclado teclado_porta_3(PORTA_I2C_3);

void setup() {
  brick.inicializa();
  brick.adiciona(tela_porta_5);
  brick.adiciona(teclado_porta_3);
  // Aula 1 - Ola, Pacoca! (versao com tela)
  // Tela na porta I2C-5 e teclado na porta I2C-3.
  // Enquanto um botao estiver apertado, acende o LED dele e escreve o numero.
  tela_porta_5.setFonte(SSD1306::FONTE_MEDIA);
  tela_porta_5.setCursor(0, 0);
  tela_porta_5.print("Ola!");
  tela_porta_5.setCursor(0, 1);
  tela_porta_5.print("Eu sou o");
  tela_porta_5.setCursor(0, 2);
  tela_porta_5.print("Pacoca");
  brick.espera(2000);
  tela_porta_5.clear();
  tela_porta_5.setCursor(0, 0);
  tela_porta_5.print("Aperte um");
  tela_porta_5.setCursor(0, 1);
  tela_porta_5.print("botao");

}

void loop() {
    brick.atualiza();
    if ((teclado_porta_3.leBotao(1) == Teclado::APERTADO)) {
      teclado_porta_3.alteraLed(1, true);
      tela_porta_5.setCursor(0, 3);
      tela_porta_5.print("Botao 1");

    } else {
      teclado_porta_3.alteraLed(1, false);

    }
    if ((teclado_porta_3.leBotao(2) == Teclado::APERTADO)) {
      teclado_porta_3.alteraLed(2, true);
      tela_porta_5.setCursor(0, 3);
      tela_porta_5.print("Botao 2");

    } else {
      teclado_porta_3.alteraLed(2, false);

    }
    if ((teclado_porta_3.leBotao(3) == Teclado::APERTADO)) {
      teclado_porta_3.alteraLed(3, true);
      tela_porta_5.setCursor(0, 3);
      tela_porta_5.print("Botao 3");

    } else {
      teclado_porta_3.alteraLed(3, false);

    }
    if ((teclado_porta_3.leBotao(4) == Teclado::APERTADO)) {
      teclado_porta_3.alteraLed(4, true);
      tela_porta_5.setCursor(0, 3);
      tela_porta_5.print("Botao 4");

    } else {
      teclado_porta_3.alteraLed(4, false);

    }

}