/*
 * Aula 4 - Jogo da memória (Genius) (versão com tela)
 *
 * O que faz: o Paçoca toca uma sequência de botões (cada botão tem a sua
 * nota e o seu LED) e você repete apertando os botões na mesma ordem.
 * A cada rodada a sequência ganha mais um botão e fica um pouco mais rápida.
 *   Botão 1: dó    Botão 2: mi    Botão 3: sol    Botão 4: dó agudo
 * A tela mostra o nível, o recorde e de quem é a vez.
 *
 * Ligações: buzzer passivo na porta S3, tela na porta I2C-5 e teclado na
 * porta I2C-3.
 * Versão falada: aula4_genius_voz.ino (mesma lógica).
 *
 * Ideias principais: guardar a sequência num vetor, percorrer o vetor com
 * for, sortear com random() e comparar o que a pessoa apertou com o vetor.
 * O jogo é igual para quem enxerga e para quem não enxerga: cada botão
 * tem um som diferente.
 */
#define SUPORTE_DISPLAY_SSD1306 1
#define SUPORTE_TECLADO 1
#define SUPORTE_MUSICA 1
#include "brickSimples.h"

SSD1306 tela = SSD1306(PORTA_I2C_5);
Teclado teclado = Teclado(PORTA_I2C_3);
Musica musica = Musica(PORTA_BUZZER_3);

const int MAXIMO = 32;                  /* tamanho máximo da sequência */
uint8_t sequencia[MAXIMO];              /* os botões sorteados, de 1 a 4 */
int tamanho = 0;                        /* quantos botões a sequência tem agora */
int recorde = 0;
const int notas[5] = {0, 262, 330, 392, 523};   /* Hz; a posição 0 não é usada */

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

/* Espera um botão por até limiteMs. Devolve o botão (1 a 4) ou 0 se o tempo acabou. */
int esperaBotao(uint32_t limiteMs) {
    uint32_t inicio = millis();
    while (millis() - inicio < limiteMs) {
        brick.atualiza();
        for (int b = 1; b <= 4; b++) {
            if (apertou(b)) return b;
        }
    }
    return 0;
}

/* Acende o LED e toca a nota do botão por duracao milissegundos. */
void tocaBotao(int botao, int duracao) {
    teclado.alteraLed(botao, true);
    musica.notaAteOFim(notas[botao], duracao);
    teclado.alteraLed(botao, false);
}

void escreveLinha(int linha, const char *texto) {
    tela.limpaLinha(linha);
    tela.print(texto);
}

void mostraPlacar() {
    tela.setCursor(0, 0);
    tela.print("Nivel ");
    tela.print(tamanho);
    tela.print("  ");
    tela.setCursor(0, 1);
    tela.print("Rec. ");
    tela.print(recorde);
    tela.print("  ");
}

void comecaPartida() {
    tamanho = 0;
    tela.clear();
    escreveLinha(2, "Aperte um");
    escreveLinha(3, "botao");
    esperaBotao(4000000000UL);        /* espera "para sempre" */
    randomSeed(micros());             /* o momento do toque deixa o sorteio imprevisível */
    musica.tocarAteOFim(EFEITO_INICIO);
    tela.clear();
}

void fimDeJogo() {
    musica.tocarAteOFim(EFEITO_ERRO);
    int pontos = tamanho - 1;          /* a última rodada não foi completada */
    tela.clear();
    tela.setCursor(0, 0);
    tela.print("Fim! ");
    tela.print(pontos);
    tela.print(" pts");
    if (pontos > recorde) {
        recorde = pontos;
        escreveLinha(1, "Recorde!");
        musica.tocarAteOFim(EFEITO_SUCESSO);
    }
    brick.espera(1500);
    comecaPartida();
}

void setup() {
    brick.inicializa();
    brick.adiciona(tela);
    brick.adiciona(teclado);
    brick.adiciona(musica);
    tela.setFonte(SSD1306::FONTE_MEDIA);
    comecaPartida();
}

void loop() {
    brick.atualiza();

    /* 1. Sorteia mais um botão e coloca no fim da sequência. */
    sequencia[tamanho] = random(1, 5);       /* de 1 a 4 */
    tamanho++;
    mostraPlacar();
    escreveLinha(3, "Observe");
    brick.espera(700);

    /* 2. Toca a sequência. Fica mais rápido a cada nível. */
    int duracao = 500 - tamanho * 20;
    if (duracao < 180) duracao = 180;
    for (int i = 0; i < tamanho; i++) {
        tocaBotao(sequencia[i], duracao);
        brick.espera(duracao / 3);
    }

    /* 3. A pessoa repete. Errou ou demorou mais de 5 s: fim de jogo. */
    escreveLinha(3, "Sua vez!");
    for (int i = 0; i < tamanho; i++) {
        int botao = esperaBotao(5000);
        if (botao != sequencia[i]) {           /* errou (ou 0 = tempo acabou) */
            fimDeJogo();
            return;                            /* sai do loop; ele começa de novo */
        }
        tocaBotao(botao, 200);
    }

    /* 4. Acertou tudo. */
    escreveLinha(3, "Muito bem");
    brick.espera(600);
    if (tamanho == MAXIMO) {                   /* chegou no fim do vetor */
        escreveLinha(3, "Venceu!");
        musica.tocarAteOFim(MUSICA_ODE_ALEGRIA);
        comecaPartida();
    }
}

/*
 * Experimente:
 * 1. Mude as notas do vetor notas[]. Quais sons ficam mais fáceis de lembrar?
 * 2. Deixe o jogo mais difícil: comece já com 3 botões na sequência.
 * 3. Na linha "if (botao != sequencia[i])", o que acontece se a pessoa
 *    demora mais de 5 s? Por que isso também é um erro?
 * 4. Desafio: modo dois jogadores. Cada um, na sua vez, ADICIONA um botão à
 *    sequência (em vez do random) e o outro precisa repetir tudo.
 */
