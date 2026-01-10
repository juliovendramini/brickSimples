//Precisei transformar o tratamento dessas funções em algo global, pois mais de uma classe vai fazer isso desse tipo de interrupção, e não posso ter mais uma 
//FUNCAO de ISR por programa.

//atualmente usando esse código: ultrassonico e bluetooth


// ISRs PCINT simplificadas - apenas para pinos usados: 15, 17, 6, 3, 19
// ISRs PCINT simplificadas - apenas para pinos usados: 14, 16, 8, 2, 18 (precisei trocar as posições dos pinos do ultrassonico para a recepção dele ser o mesmo pino do RX de serial)


// PCINT0: Pino D8 (PB0)
ISR(PCINT0_vect) {
    // Apenas D8 usa PCINT0
    uint8_t idx = Ultrassonico::pinoParaInstancia[8];
    if(Ultrassonico::instancias[idx]) {
        Ultrassonico::instancias[idx]->handleInterrupt();
    }
    if(Bluetooth::bluetooth_active_object){
            Bluetooth::bluetooth_active_object->handle_interrupt();
    }
}

// PCINT1: Pinos D14 (A0/PC0), D16 (A2/PC2), D18 (A4/PC4)
ISR(PCINT1_vect) {
    uint8_t estadoAtual = PINC;
    uint8_t mudancas = estadoAtual ^ Ultrassonico::estadoAnteriorPCINT1;
    Ultrassonico::estadoAnteriorPCINT1 = estadoAtual;
    
    // Verifica apenas os 3 pinos usados
    if(mudancas & (1)) {  // PC0 = D14 (A0)
        uint8_t idx = Ultrassonico::pinoParaInstancia[14];
        if(Ultrassonico::instancias[idx]) Ultrassonico::instancias[idx]->handleInterrupt();
        if(Bluetooth::bluetooth_active_object){
            Bluetooth::bluetooth_active_object->handle_interrupt();
        }
    }
    if(mudancas & (1 << 2)) {  // PC2 = D16 (A2)
        uint8_t idx = Ultrassonico::pinoParaInstancia[16];
        if(Ultrassonico::instancias[idx]) Ultrassonico::instancias[idx]->handleInterrupt();
        if(Bluetooth::bluetooth_active_object){
            Bluetooth::bluetooth_active_object->handle_interrupt();
        }
    }
    if(mudancas & (1 << 4)) {  // PC4 = D18 (A4)
        uint8_t idx = Ultrassonico::pinoParaInstancia[18];
        if(Ultrassonico::instancias[idx]) Ultrassonico::instancias[idx]->handleInterrupt();
        if(Bluetooth::bluetooth_active_object){
            Bluetooth::bluetooth_active_object->handle_interrupt();
        }
    }
}

// PCINT2: Pino D2 (PD2)
ISR(PCINT2_vect) {
    // Apenas D2 usa PCINT2
    uint8_t idx = Ultrassonico::pinoParaInstancia[2];
    if(Ultrassonico::instancias[idx]) {
        Ultrassonico::instancias[idx]->handleInterrupt();
    }
    if(Bluetooth::bluetooth_active_object){
        Bluetooth::bluetooth_active_object->handle_interrupt();
    }
}

