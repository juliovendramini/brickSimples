/*
 * Exemplo: voz e música no buzzer passivo do Paçoca.
 *
 * Para usar, copie este conteúdo para o brickSimples.ino (o Arduino só compila
 * o .ino da pasta principal) e ligue o buzzer passivo na porta S3.
 *
 * As falas vêm do falas.h, gerado no PC com:
 *     cd ferramentas
 *     python gerar_falas.py frases.txt
 */
#define SUPORTE_MOTOR 1
#define SUPORTE_VOZ 1
#define SUPORTE_VOZ_SERIAL 1      // receber fala pelo enviar_fala.py (opcional)
#define SUPORTE_MUSICA 1

#include "brickSimples.h"
#include "falas.h"

Voz voz = Voz(PORTA_BUZZER_3);           // a voz só funciona na S3
Musica musica = Musica(PORTA_BUZZER_3);  // na S3 tem volume e timbre

void setup() {
    brick.inicializa();                  // obrigatório, sempre a primeira linha
    brick.adiciona(voz);
    brick.adiciona(musica);

    voz.volume(200);                     // 100 = limpo; 200-300 = mais alto
    voz.falar(FALA_OLA);                 // espera terminar

    musica.volume(80);
    musica.timbre(Musica::QUADRADA);
    musica.tocarAteOFim(EFEITO_INICIO);

    // Não bloqueante: o robô continua fazendo outras coisas enquanto fala
    voz.iniciaFala(FALA_VITIMA_ENCONTRADA);
    while (voz.falando()) {
        // ... ler sensores, decidir, etc.
    }
    Serial.print(F("Amostras atrasadas: "));
    Serial.println(voz.atrasos());       // deve ser 0

    musica.timbre(Musica::TRIANGULAR);
    musica.decaimento(true);
    musica.tocar(MUSICA_ODE_ALEGRIA);    // toca em segundo plano
    delay(3000);
    musica.parar();

    musica.nota(440, 300);               // Lá por 300 ms
    musica.aguarda();
    musica.tocar(F("Escala:d=8,o=5,b=180:c,d,e,f,g,a,b,c6"));   // RTTTL próprio
    musica.aguarda();

    Serial.println(F("Pronto. Rode no PC: python enviar_fala.py --porta COM? \"Texto qualquer\""));
}

void loop() {
    // Fala o que chegar pela serial (ferramentas/enviar_fala.py).
    // Enquanto isso, não use a Serial para outra coisa.
    voz.atualizaSerial(Serial);
}
