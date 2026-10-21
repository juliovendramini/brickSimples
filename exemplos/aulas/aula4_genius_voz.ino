/*
 * Aula 4 - Jogo da memória (Genius) (versão com voz)
 *
 * O que faz: o Paçoca toca uma sequência de botões (cada botão tem a sua
 * nota e o seu LED) e você repete apertando os botões na mesma ordem.
 * A cada rodada a sequência ganha mais um botão e fica um pouco mais rápida.
 *   Botão 1: dó    Botão 2: mi    Botão 3: sol    Botão 4: dó agudo
 * A voz fala o nível, avisa a sua vez e fala os pontos no fim.
 *
 * Ligações: buzzer passivo na porta S3 (voz e música usam o mesmo buzzer,
 * uma de cada vez) e teclado na porta I2C-3.
 * Versão com tela: aula4_genius_tela.ino (mesma lógica).
 *
 * Ideias principais: guardar a sequência num vetor, percorrer o vetor com
 * for, sortear com random() e comparar o que a pessoa apertou com o vetor.
 * O jogo é igual para quem enxerga e para quem não enxerga: cada botão
 * tem um som diferente.
 */
#define SUPORTE_VOZ 1
#define SUPORTE_VOZ_NUMEROS 1
#define SUPORTE_TECLADO 1
#define SUPORTE_MUSICA 1
#include "brickSimples.h"
#include "falasAulas.h"

Voz voz = Voz(PORTA_BUZZER_3);
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

void comecaPartida() {
    tamanho = 0;
    voz.falar(FALA_APERTE_UM_BOTAO);
    esperaBotao(4000000000UL);        /* espera "para sempre" */
    randomSeed(micros());             /* o momento do toque deixa o sorteio imprevisível */
    musica.tocarAteOFim(EFEITO_INICIO);
}

void fimDeJogo() {
    musica.tocarAteOFim(EFEITO_ERRO);
    int pontos = tamanho - 1;          /* a última rodada não foi completada */
    voz.falar(FALA_FIM_DE_JOGO);
    voz.iniciaNumero(pontos);          /* "sete ... */
    voz.enfileirar(FALA_PONTOS);       /*  ... pontos." */
    voz.aguarda();
    if (pontos > recorde) {
        recorde = pontos;
        voz.falar(FALA_RECORDE);
        musica.tocarAteOFim(EFEITO_SUCESSO);
    }
    brick.espera(500);
    comecaPartida();
}

void setup() {
    brick.inicializa();
    brick.adiciona(voz);
    brick.adiciona(teclado);
    brick.adiciona(musica);
    comecaPartida();
}

void loop() {
    brick.atualiza();

    /* 1. Sorteia mais um botão e coloca no fim da sequência. */
    sequencia[tamanho] = random(1, 5);       /* de 1 a 4 */
    tamanho++;
    voz.iniciaFala(FALA_NIVEL);              /* "Nível ... */
    voz.enfileirarNumero(tamanho);           /*  ... três" */
    voz.aguarda();
    brick.espera(400);

    /* 2. Toca a sequência. Fica mais rápido a cada nível. */
    int duracao = 500 - tamanho * 20;
    if (duracao < 180) duracao = 180;
    for (int i = 0; i < tamanho; i++) {
        tocaBotao(sequencia[i], duracao);
        brick.espera(duracao / 3);
    }

    /* 3. A pessoa repete. Errou ou demorou mais de 5 s: fim de jogo. */
    voz.falar(FALA_SUA_VEZ);
    for (int i = 0; i < tamanho; i++) {
        int botao = esperaBotao(5000);
        if (botao != sequencia[i]) {           /* errou (ou 0 = tempo acabou) */
            fimDeJogo();
            return;                            /* sai do loop; ele começa de novo */
        }
        tocaBotao(botao, 200);
    }

    /* 4. Acertou tudo. */
    brick.espera(300);
    voz.falar(FALA_MUITO_BEM);
    if (tamanho == MAXIMO) {                   /* chegou no fim do vetor */
        musica.tocarAteOFim(MUSICA_ODE_ALEGRIA);
        comecaPartida();
    }
}

/*
 * Experimente:
 * 1. Mude as notas do vetor notas[]. Quais sons ficam mais fáceis de lembrar?
 * 2. A voz fala "Nível" em toda rodada. Isso ajuda ou atrapalha o ritmo?
 *    Teste falar só a cada 5 níveis (dica: tamanho % 5 == 0).
 * 3. Na linha "if (botao != sequencia[i])", o que acontece se a pessoa
 *    demora mais de 5 s? Por que isso também é um erro?
 * 4. Desafio: modo dois jogadores. Cada um, na sua vez, ADICIONA um botão à
 *    sequência (em vez do random) e o outro precisa repetir tudo.
 */
