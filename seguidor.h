//essa classe atualiza dois sensores para o seguidor de linha baseado em cor, usando dessa forma os sensores ficam sincronizados nas leituras dos valores

class Seguidor {
private:
    TCS34725 *sensor1;
    TCS34725 *sensor2;
public:
    Seguidor(){}
    void begin(TCS34725 *s1, TCS34725 *s2){
        sensor1 = s1;
        sensor2 = s2;
    }

    void getRGBCCalibrado(uint16_t *r1, uint16_t *g1, uint16_t *b1, uint16_t *c1,
                    uint16_t *r2, uint16_t *g2, uint16_t *b2, uint16_t *c2){
        //sensor1->enable(false); //false para nao esperar
        //sensor2->enable(false); //false para nao esperar
        sensor1->enablePON();
        sensor2->enablePON();   
        delayMicroseconds(2500); //pequena espera para garantir que os sensores estejam prontos
        sensor1->enablePON_AEN();
        sensor2->enablePON_AEN();
        delayMicroseconds(3400);
        sensor1->getRawData(r1, g1, b1, c1);
        sensor2->getRawData(r2, g2, b2, c2);
        sensor1->disable();
        sensor2->disable();
        //sensor1->enableLedOff(false); //false para nao esperar
        //sensor2->enableLedOff(false); //false para nao esperar
        sensor1->enablePON();
        sensor2->enablePON();
        delayMicroseconds(2500); //pequena espera para garantir que os sensores estejam prontos
        sensor1->enablePON_AEN();
        sensor2->enablePON_AEN();
        sensor1->ledOff();
        sensor2->ledOff();
        delayMicroseconds(3400);
        uint16_t r1_off, g1_off, b1_off, c1_off;
        uint16_t r2_off, g2_off, b2_off, c2_off;
        sensor1->getRawData(&r1_off, &g1_off, &b1_off, &c1_off);
        sensor2->getRawData(&r2_off, &g2_off, &b2_off, &c2_off);
        sensor1->disable();
        sensor2->disable();
        *r1 = *r1 - r1_off;
        if(*r1 > 64000) *r1 = 0;
        *g1 = *g1 - g1_off;
        if(*g1 > 64000) *g1 = 0;
        *b1 = *b1 - b1_off;
        if(*b1 > 64000) *b1 = 0;
        *c1 = *c1 - c1_off;
        if(*c1 > 64000) *c1 = 0;
        *r2 = *r2 - r2_off;
        if(*r2 > 64000) *r2 = 0;
        *g2 = *g2 - g2_off;
        if(*g2 > 64000) *g2 = 0;
        *b2 = *b2 - b2_off;
        if(*b2 > 64000) *b2 = 0;
        *c2 = *c2 - c2_off;
        if(*c2 > 64000) *c2 = 0;

        // Aplica calibração
        *r1 = (uint32_t)*r1 * 255 / sensor1->dadosCalibracao.r;
        *g1 = (uint32_t)*g1 * 255 / sensor1->dadosCalibracao.g;
        *b1 = (uint32_t)*b1 * 255 / sensor1->dadosCalibracao.b;
        *c1 = (uint32_t)*c1 * 255 / sensor1->dadosCalibracao.c;
        if(*r1 > 255) *r1 = 255;
        if(*g1 > 255) *g1 = 255;
        if(*b1 > 255) *b1 = 255;
        if(*c1 > 255) *c1 = 255;

        *r2 = (uint32_t)*r2 * 255 / sensor2->dadosCalibracao.r;
        *g2 = (uint32_t)*g2 * 255 / sensor2->dadosCalibracao.g;
        *b2 = (uint32_t)*b2 * 255 / sensor2->dadosCalibracao.b;
        *c2 = (uint32_t)*c2 * 255 / sensor2->dadosCalibracao.c;
        if(*r2 > 255) *r2 = 255;
        if(*g2 > 255) *g2 = 255;
        if(*b2 > 255) *b2 = 255;
        if(*c2 > 255) *c2 = 255;
            
    }
};