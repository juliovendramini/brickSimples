/*
 * Aula 6 - Sensor de distância, voz e servo (versão com voz)
 *
 * O que faz: lê a distância com o sensor VL53L0X e usa o servo de dois
 * jeitos (modos):
 *   Modo ponteiro: o servo vira um medidor que dá para ler com os dedos.
 *                  Perto = 0 graus, longe = 180.
 *   Modo cancela: o servo abre (90 graus) quando algo chega mais perto
 *                 que o limite, e fecha (0 grau) quando vai embora.
 *                 O Paçoca avisa "Cancela aberta" e "Cancela fechada".
 * Botões:
 *   Botão 1: diminui o limite (5 cm) e fala o limite
 *   Botão 2: aumenta o limite (5 cm) e fala o limite
 *   Botão 3: troca de modo e fala o nome do modo
 *   Botão 4: fala a distância agora
 *
 * Ligações: sensor VL53L0X na porta I2C-1, servo na porta S1, buzzer
 * passivo na porta S3 e teclado na porta I2C-3.
 * Versão com tela: aula6_sensor_tela.ino (mesma lógica).
 *
 * Lembrete da aula 5: enquanto o Paçoca fala, o servo fica sem pulsos.
 * Por isso as falas aqui são curtas e só acontecem quando algo muda.
 *
 * Com o sensor ultrassônico: troque SUPORTE_SENSOR_VL53L0X por
 * SUPORTE_SENSOR_ULTRASSONICO, o objeto por
 * Ultrassonico sensor = Ultrassonico(PORTA_ULTRASSONICO_1); e a leitura por
 * sensor.getDistancia() * 10 (ele já mede em centímetros).
 */
#define SUPORTE_VOZ 1
#define SUPORTE_VOZ_NUMEROS 1
#define SUPORTE_TECLADO 1
#define SUPORTE_SERVO 1
#define SUPORTE_SENSOR_VL53L0X 1
#include "brickSimples.h"
#include "falasAulas.h"

Voz voz = Voz(PORTA_BUZZER_3);
Teclado teclado = Teclado(PORTA_I2C_3);
VL53L0X sensor = VL53L0X(PORTA_I2C_1);

const int MODO_PONTEIRO = 0;
const int MODO_CANCELA = 1;
int modo = MODO_PONTEIRO;
int limite = 15;                  /* cm: mais perto que isto abre a cancela */
bool cancelaAberta = false;
int anguloServo = -1;             /* -1 = ainda não moveu */

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

/* Distância em cm. Devolve -1 se não tem nada na frente (mais de 120 cm). */
int leDistancia() {
    uint16_t mm = sensor.getDistancia();     /* sem nada na frente ele devolve 8190 ou mais */
    if (mm > 1200) return -1;
    return mm / 10;
}

/* Só manda o servo se o ângulo mudou pelo menos 3 graus (evita tremedeira). */
void moveServo(int angulo) {
    if (anguloServo < 0 || abs(angulo - anguloServo) >= 3) {
        anguloServo = angulo;
        servos.moveServo(PORTA_SERVO_1, angulo);
    }
}

void falaLimite() {
    voz.iniciaFala(FALA_LIMITE);          /* "Limite ... */
    voz.enfileirarNumero(limite);         /*  ... quinze ... */
    voz.enfileirar(FALA_CENTIMETROS);     /*  ... centímetros." */
}

void falaModo() {
    if (modo == MODO_PONTEIRO) voz.iniciaFala(FALA_MODO_PONTEIRO);
    else voz.iniciaFala(FALA_MODO_CANCELA);
}

void falaDistancia(int distancia) {
    if (distancia < 0) {
        voz.iniciaFala(FALA_NADA_A_FRENTE);
    } else {
        voz.iniciaNumero(distancia);
        voz.enfileirar(FALA_CENTIMETROS);
    }
}

void setup() {
    brick.inicializa();
    brick.adiciona(voz);
    brick.adiciona(teclado);
    brick.adiciona(sensor);
    servos.iniciaServo(PORTA_SERVO_1);
    falaModo();
}

void loop() {
    brick.atualiza();                     /* aqui o brick lê o sensor */

    int distancia = leDistancia();

    /* Botões */
    if (apertou(1)) {
        limite = max(5, limite - 5);
        falaLimite();
    }
    if (apertou(2)) {
        limite = min(100, limite + 5);
        falaLimite();
    }
    if (apertou(3)) {
        modo = (modo == MODO_PONTEIRO) ? MODO_CANCELA : MODO_PONTEIRO;
        anguloServo = -1;                 /* força mover o servo no modo novo */
        cancelaAberta = false;            /* todo modo começa com a cancela fechada */
        falaModo();
    }
    if (apertou(4)) {
        falaDistancia(distancia);
    }

    /* Servo */
    if (modo == MODO_PONTEIRO) {
        int d = (distancia < 0) ? 100 : distancia;
        d = constrain(d, 0, 100);
        moveServo(map(d, 0, 100, 0, 180));        /* 0..100 cm vira 0..180 graus */
    } else {
        /* Histerese: abre abaixo do limite, só fecha 3 cm acima dele.
         * Sem isso, um objeto parado bem no limite faz a cancela abrir e
         * fechar sem parar (e o Paçoca falaria sem parar). */
        bool antes = cancelaAberta;
        if (!cancelaAberta && distancia >= 0 && distancia < limite) cancelaAberta = true;
        if (cancelaAberta && (distancia < 0 || distancia > limite + 3)) cancelaAberta = false;
        moveServo(cancelaAberta ? 90 : 0);
        if (cancelaAberta != antes) {             /* só fala quando muda */
            brick.espera(400);                    /* deixa o servo chegar antes de falar */
            if (cancelaAberta) voz.iniciaFala(FALA_CANCELA_ABERTA);
            else voz.iniciaFala(FALA_CANCELA_FECHADA);
        }
    }
}

/*
 * Experimente:
 * 1. No modo ponteiro, mude a escala para 0..50 cm. O ponteiro fica mais
 *    sensível ou menos? Confira com os dedos.
 * 2. No modo cancela, tire a histerese (use só "distancia < limite").
 *    Coloque a mão bem no limite. O que acontece com a voz?
 * 3. Por que existe o brick.espera(400) antes de falar "Cancela aberta"?
 * 4. Desafio: modo "sensor de ré": bipes mais rápidos quanto mais perto
 *    (use a classe Musica, como na aula 4: musica.nota(1000, 50)).
 */
