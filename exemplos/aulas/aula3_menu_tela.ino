/*
 * Aula 3 - Menu (versão com tela)
 *
 * O que faz: um menu com 4 opções. A seta ">" mostra a opção escolhida.
 *   Botão 1: sobe (opção anterior)
 *   Botão 2: desce (próxima opção)
 *   Botão 3: executa a opção
 *   Botão 4: ajuda
 * Opções: Dado (sorteia de 1 a 6), Tempo (há quanto tempo o Paçoca está
 * ligado), Contagem (regressiva de 5 a 0) e Sobre.
 * Depois de executar, aperte qualquer botão para voltar ao menu.
 *
 * Ligações: tela na porta I2C-5 e teclado na porta I2C-3.
 * Versão falada: aula3_menu_voz.ino (mesma lógica).
 *
 * Ideias principais: guardar as opções num vetor e usar uma variável
 * (opcao) para saber onde estamos. Descer do último volta para o primeiro.
 */
#define SUPORTE_DISPLAY_SSD1306 1
#define SUPORTE_TECLADO 1
#include "brickSimples.h"

SSD1306 tela = SSD1306(PORTA_I2C_5);
Teclado teclado = Teclado(PORTA_I2C_3);

const int TOTAL_OPCOES = 4;
const char *nomes[TOTAL_OPCOES] = {"Dado", "Tempo", "Contagem", "Sobre"};
int opcao = 0;                  /* 0 = primeira opção */

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

/* Fica esperando até algum dos 4 botões ser apertado. Devolve qual foi. */
int esperaBotao() {
    while (true) {
        brick.atualiza();
        for (int b = 1; b <= 4; b++) {
            if (apertou(b)) return b;
        }
    }
}

void desenhaMenu() {
    tela.setFonte(SSD1306::FONTE_MEDIA);      /* também apaga a tela */
    for (int i = 0; i < TOTAL_OPCOES; i++) {
        tela.setCursor(0, i);
        if (i == opcao) tela.print(">");
        else tela.print(" ");
        tela.print(nomes[i]);
    }
}

/* Só troca a seta de lugar (mais rápido que redesenhar tudo). */
void moveSeta(int anterior) {
    tela.setCursor(0, anterior);
    tela.print(" ");
    tela.setCursor(0, opcao);
    tela.print(">");
}

void opcaoDado() {
    int numero = random(1, 7);                /* de 1 a 6 (o 7 não entra) */
    tela.setFonte(SSD1306::FONTE_GRANDE);     /* 5 colunas, 2 linhas */
    tela.setCursor(0, 0);
    tela.print("Saiu");
    tela.setCursor(2, 1);
    tela.print(numero);
    esperaBotao();
}

void opcaoTempo() {
    tela.setFonte(SSD1306::FONTE_MEDIA);
    tela.setCursor(0, 0);
    tela.print("Ligado ha");
    tela.setCursor(0, 3);
    tela.print("Aperte");
    /* Atualiza o tempo na tela até alguém apertar um botão. */
    while (true) {
        brick.atualiza();
        tela.setCursor(0, 1);
        tela.print(millis() / 1000);
        tela.print(" s   ");
        for (int b = 1; b <= 4; b++) {
            if (apertou(b)) return;
        }
    }
}

void opcaoContagem() {
    tela.setFonte(SSD1306::FONTE_GRANDE);
    for (int n = 5; n >= 1; n--) {
        tela.setCursor(2, 0);
        tela.print(n);
        brick.espera(1000);
    }
    tela.setCursor(0, 0);
    tela.print(" JA! ");
    esperaBotao();
}

void opcaoSobre() {
    tela.setFonte(SSD1306::FONTE_PEQUENA);    /* 21 colunas, 8 linhas */
    tela.setCursor(0, 0);
    tela.print("Eu sou o Pacoca");
    tela.setCursor(0, 2);
    tela.print("Biblioteca");
    tela.setCursor(0, 3);
    tela.print("brickSimples");
    tela.setCursor(0, 7);
    tela.print("Aperte um botao");
    esperaBotao();
}

void ajuda() {
    tela.setFonte(SSD1306::FONTE_MEDIA);
    tela.setCursor(0, 0);
    tela.print("1 sobe");
    tela.setCursor(0, 1);
    tela.print("2 desce");
    tela.setCursor(0, 2);
    tela.print("3 executa");
    tela.setCursor(0, 3);
    tela.print("4 ajuda");
    esperaBotao();
}

void setup() {
    brick.inicializa();
    brick.adiciona(tela);
    brick.adiciona(teclado);
    desenhaMenu();
}

void loop() {
    brick.atualiza();

    if (apertou(1)) {                          /* sobe */
        int anterior = opcao;
        opcao = opcao - 1;
        if (opcao < 0) opcao = TOTAL_OPCOES - 1;     /* do primeiro vai para o último */
        moveSeta(anterior);
    }
    if (apertou(2)) {                          /* desce */
        int anterior = opcao;
        opcao = opcao + 1;
        if (opcao >= TOTAL_OPCOES) opcao = 0;        /* do último volta para o primeiro */
        moveSeta(anterior);
    }
    if (apertou(3)) {                          /* executa */
        randomSeed(micros());     /* o momento exato do toque deixa o sorteio imprevisível */
        if (opcao == 0) opcaoDado();
        if (opcao == 1) opcaoTempo();
        if (opcao == 2) opcaoContagem();
        if (opcao == 3) opcaoSobre();
        desenhaMenu();
    }
    if (apertou(4)) {
        ajuda();
        desenhaMenu();
    }
}

/*
 * Experimente:
 * 1. Tire a linha do randomSeed. Ligue o Paçoca e jogue o dado 3 vezes.
 *    Desligue, ligue e jogue de novo. O que você percebe?
 * 2. Mude o dado para sortear de 1 a 20.
 * 3. Troque a ordem das opções no vetor nomes. O que mais precisa mudar?
 *    (Dica: use switch (opcao) com case 0, case 1... em vez dos ifs.)
 * 4. Desafio: crie uma quinta opção. A tela média só tem 4 linhas: faça o
 *    menu rolar (mostrar as opções 1 a 4 quando a seta estiver na quinta).
 */
