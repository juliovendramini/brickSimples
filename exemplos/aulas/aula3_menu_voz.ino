/*
 * Aula 3 - Menu (versão com voz)
 *
 * O que faz: um menu falado com 4 opções, como um leitor de tela.
 *   Botão 1: opção anterior (fala o nome)
 *   Botão 2: próxima opção (fala o nome)
 *   Botão 3: executa a opção
 *   Botão 4: repete o nome da opção atual
 * Opções: Dado (sorteia de 1 a 6), Tempo (há quanto tempo o Paçoca está
 * ligado), Contagem (regressiva de 5 a 0) e Sobre.
 * Depois de executar, o menu continua onde estava: é só navegar de novo.
 *
 * Ligações: buzzer passivo na porta S3 e teclado na porta I2C-3.
 * Versão com tela: aula3_menu_tela.ino (mesma lógica).
 *
 * Ideias principais: guardar as opções num vetor e usar uma variável
 * (opcao) para saber onde estamos. Descer do último volta para o primeiro.
 * Aqui o vetor guarda as FALAS das opções em vez dos textos.
 */
#define SUPORTE_VOZ 1
#define SUPORTE_VOZ_NUMEROS 1
#define SUPORTE_TECLADO 1
#include "brickSimples.h"
#include "falasAulas.h"

Voz voz = Voz(PORTA_BUZZER_3);
Teclado teclado = Teclado(PORTA_I2C_3);

const int TOTAL_OPCOES = 4;
const uint8_t *falas[TOTAL_OPCOES] = {FALA_DADO, FALA_TEMPO, FALA_CONTAGEM, FALA_SOBRE};
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

void falaOpcao() {
    voz.iniciaFala(falas[opcao]);    /* interrompe o que estava falando */
}

void opcaoDado() {
    int numero = random(1, 7);                /* de 1 a 6 (o 7 não entra) */
    voz.iniciaFala(FALA_SAIU);                /* "Saiu ... */
    voz.enfileirarNumero(numero);             /*  ... quatro" */
}

void opcaoTempo() {
    voz.iniciaNumero(millis() / 1000);        /* "trinta e dois ... */
    voz.enfileirar(FALA_SEGUNDOS);            /*  ... segundos" */
}

void opcaoContagem() {
    for (int n = 5; n >= 1; n--) {
        voz.iniciaNumero(n);
        brick.espera(1000);                   /* um número por segundo */
    }
    voz.falar(FALA_JA);                       /* "Já!" */
}

void opcaoSobre() {
    voz.iniciaFala(FALA_OLA_PACOCA);
}

void setup() {
    brick.inicializa();
    brick.adiciona(voz);
    brick.adiciona(teclado);
    voz.iniciaFala(FALA_MENU);                /* "Menu. ... */
    voz.enfileirar(falas[opcao]);             /*  ... Jogar o dado." */
}

void loop() {
    brick.atualiza();

    if (apertou(1)) {                          /* anterior */
        opcao = opcao - 1;
        if (opcao < 0) opcao = TOTAL_OPCOES - 1;     /* do primeiro vai para o último */
        falaOpcao();
    }
    if (apertou(2)) {                          /* próxima */
        opcao = opcao + 1;
        if (opcao >= TOTAL_OPCOES) opcao = 0;        /* do último volta para o primeiro */
        falaOpcao();
    }
    if (apertou(3)) {                          /* executa */
        randomSeed(micros());     /* o momento exato do toque deixa o sorteio imprevisível */
        if (opcao == 0) opcaoDado();
        if (opcao == 1) opcaoTempo();
        if (opcao == 2) opcaoContagem();
        if (opcao == 3) opcaoSobre();
    }
    if (apertou(4)) {
        falaOpcao();
    }
}

/*
 * Experimente:
 * 1. Tire a linha do randomSeed. Ligue o Paçoca e jogue o dado 3 vezes.
 *    Desligue, ligue e jogue de novo. O que você percebe?
 * 2. Mude o dado para sortear de 1 a 20.
 * 3. Quando a lista dá a volta (do último para o primeiro), leitores de tela
 *    costumam avisar. Toque um bipe ou fale "Menu" antes do nome nessa hora.
 * 4. Desafio: crie uma quinta opção. Que frase nova ela precisa? Escreva a
 *    frase em ferramentas/frases_aulas.txt e gere o falasAulas.h de novo.
 */
