#include <avr/pgmspace.h>
#include <math.h>
#include <stdlib.h>
#include "TCS34725.h"
#include "portas.h"
/*!
 *  @brief  Implements missing powf function
 *  @param  x
 *          Base number
 *  @param  y
 *          Exponent
 *  @return x raised to the power of y
 */
float powf(const float x, const float y) {
  return (float)(pow((double)x, (double)y));
}

/*!
 *  @brief  Writes a register and an 8 bit value over I2C
 *  @param  reg
 *  @param  value
 */
void TCS34725::write8(uint8_t reg, uint8_t value) {
  uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg), value};
  bus->beginTransmission(TCS34725_ADDRESS);
  bus->write(buffer, 2);
  last_status = bus->endTransmission();
}

/*!
 *  @brief  Reads an 8 bit value over I2C
 *  @param  reg
 *  @return value
 */
uint8_t TCS34725::read8(uint8_t reg) {
  uint8_t buffer[1] = {(uint8_t)(TCS34725_COMMAND_BIT | reg)};
  bus->beginTransmission(TCS34725_ADDRESS);
  bus->write(buffer[0]);
  last_status = bus->endTransmission();
  bus->requestFrom(TCS34725_ADDRESS, (uint8_t)1);
  buffer[0] = bus->read();
  return buffer[0];
}

/*!
 *  @brief  Reads a 16 bit values over I2C
 *  @param  reg
 *  @return value
 */
uint16_t TCS34725::read16(uint8_t reg) {
  uint8_t buffer[2] = {(uint8_t)(TCS34725_COMMAND_BIT | reg), 0};
  uint16_t value ;
  
  bus->beginTransmission(TCS34725_ADDRESS);
  bus->write(buffer[0]);
  last_status = bus->endTransmission();
  bus->requestFrom(TCS34725_ADDRESS, (uint8_t)2);
  buffer[0]  = bus->read(); // value high byte
  buffer[1]  = bus->read();      // value low byte
  //return buffer[0] << 8 | buffer[1];
  return (uint16_t(buffer[1]) << 8) | (uint16_t(buffer[0]) & 0xFF);
}

void TCS34725::enablePON() {
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON);
}

void TCS34725::enablePON_AEN() {
  write8(TCS34725_ENABLE, TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN);
}

void TCS34725::ledOff() {
  pinMode(this->sda, OUTPUT);
  digitalWrite(this->sda, LOW); //desliga o led
}

/*!
 *  @brief  Enables the device
 */
void TCS34725::enable(bool wait=true) {
  enablePON();
  delay(3);
  enablePON_AEN();
  /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  if (wait) {
    delay((256 - _tcs34725IntegrationTime) * 12 / 5 + 2);
  }
  //delay(4);
}


void TCS34725::enableLedOff(bool wait = true) {
  enablePON();
  delay(3);
  enablePON_AEN();
  ledOff();
  /* Set a delay for the integration time.
    This is only necessary in the case where enabling and then
    immediately trying to read values back. This is because setting
    AEN triggers an automatic integration, so if a read RGBC is
    performed too quickly, the data is not yet valid and all 0's are
    returned */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  if (wait) {
    delay((256 - _tcs34725IntegrationTime) * 12 / 5 + 2);
  }
  //delay(4);
}

/*!
 *  @brief  Disables the device (putting it in lower power sleep mode)
 */
void TCS34725::disable() {
  /* Turn the device off to save power */
  uint8_t reg = 0;
  reg = read8(TCS34725_ENABLE);
  write8(TCS34725_ENABLE, reg & ~(TCS34725_ENABLE_PON | TCS34725_ENABLE_AEN));
}

/*!
 *  @brief  Constructor
 *  @param  it
 *          Integration Time
 *  @param  gain
 *          Gain
 */
TCS34725::TCS34725(PortaI2C porta, uint8_t it, tcs34725Gain_t gain) {
  _tcs34725Initialised = false;
  _tcs34725IntegrationTime = it;
  _tcs34725Gain = gain;
  this->sda = porta.sda;
  this->scl = porta.scl;
  strcpy(this->descricaoPorta, porta.descricao);
  
  // Extrai número da porta da descrição (ex: "I2C-1" -> 1)
  if (porta.descricao[4] >= '1' && porta.descricao[4] <= '5') {
    this->numeroPorta = porta.descricao[4] - '0';
  } else {
    this->numeroPorta = 0;  // Porta inválida
  }
}

/*!
 *  @brief  Initializes I2C and configures the sensor
 *  @param  addr
 *          i2c address
 *  @param  *theWire
 *          The Wire object
 *  @return True if initialization was successful, otherwise false.
 */
// boolean TCS34725::begin(uint8_t addr, uint8_t sda, uint8_t scl) {
//   this->sda = sda;

//   this->scl = scl;
//   bus = new SoftWire(sda, scl); //pinos SDA e SCL escolhidos
//   //return init();
//   return true;
// }

boolean TCS34725::begin() {
  bus = new SoftWire(sda, scl); //pinos SDA e SCL escolhidos
  //return true;
  return init();

}

/*!
 *  @brief  Part of begin
 *  @return True if initialization was successful, otherwise false.
 */
boolean TCS34725::init() {
  bus->setTimeout_ms(10);
  bus->begin();
  bus->setTimeout_ms(10);
  /* Make sure we're actually connected */
  uint8_t x = read8(TCS34725_ID);
  if ((x != 0x4d) && (x != 0x44) && (x != 0x10)) {
    delay(3);
    x = read8(TCS34725_ID); //faço duas tentativas para ler o id
    if ((x != 0x4d) && (x != 0x44) && (x != 0x10)) {
      Serial.print(F("Erro ao detectar o TCS34725 na porta "));
      Serial.println(this->descricaoPorta);
      while(1);
    }
  }
  
  
  _tcs34725Initialised = true;
  
  /* Set default integration time and gain */
  setIntegrationTime(_tcs34725IntegrationTime);
  setGain(_tcs34725Gain);
  
  /* Note: by default, the device is in power down mode on bootup */
  enable();
  Serial.print(F("TCS34725 detectado com sucesso na porta "));
  Serial.println(this->descricaoPorta);
  if(this->carregarCalibracao()) { //carrega calibração padrão
    Serial.println(F("Calibração carregada da EEPROM."));
  } else {
    Serial.println(F("Nenhuma calibração válida encontrada na EEPROM, calibre o sensor."));
    return true;
  }
}

/*!
 *  @brief  Sets the integration time for the TC34725
 *  @param  it
 *          Integration Time
 */
void TCS34725::setIntegrationTime(uint8_t it) {
  /* Update the timing register */
  write8(TCS34725_ATIME, it);

  /* Update value placeholders */
  _tcs34725IntegrationTime = it;
}

/*!
 *  @brief  Adjusts the gain on the TCS34725
 *  @param  gain
 *          Gain (sensitivity to light)
 */
void TCS34725::setGain(tcs34725Gain_t gain) {
  /* Update the timing register */
  write8(TCS34725_CONTROL, gain);

  /* Update value placeholders */
  _tcs34725Gain = gain;
}

/*!
 *  @brief  Reads the raw red, green, blue and clear channel values
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void TCS34725::getRawData(uint16_t *r, uint16_t *g, uint16_t *b,
                                   uint16_t *c) {
  // *c = read16(TCS34725_CDATAL);
  // *r = read16(TCS34725_RDATAL);
  // *g = read16(TCS34725_GDATAL);
  // *b = read16(TCS34725_BDATAL);

uint8_t buffer[8]; // 8 bytes: C_low, C_high, R_low, R_high, G_low, G_high, B_low, B_high
  
  // Envia comando para ler a partir do registrador CDATAL (0x14)
  bus->beginTransmission(TCS34725_ADDRESS);
  bus->write(TCS34725_COMMAND_BIT | TCS34725_CDATAL);
  last_status = bus->endTransmission();
  
  // Solicita 8 bytes consecutivos (auto-incremento ativado)
  bus->requestFrom(TCS34725_ADDRESS, (uint8_t)8);
  
  // Lê todos os 8 bytes de uma vez
  for (uint8_t i = 0; i < 8; i++) {
    buffer[i] = bus->read();
  }
  
  // Monta os valores uint16_t (little-endian: low byte primeiro)
  *c = (uint16_t(buffer[1]) << 8) | uint16_t(buffer[0]);
  *r = (uint16_t(buffer[3]) << 8) | uint16_t(buffer[2]);
  *g = (uint16_t(buffer[5]) << 8) | uint16_t(buffer[4]);
  *b = (uint16_t(buffer[7]) << 8) | uint16_t(buffer[6]);



  //o delay é pelo codigo e nao aqui
  /* Set a delay for the integration time */
  /* 12/5 = 2.4, add 1 to account for integer truncation */
  //delay((256 - _tcs34725IntegrationTime) * 12 / 5 + 1);
}

/*!
 *  @brief  Reads the raw red, green, blue and clear channel values in
 *          one-shot mode (e.g., wakes from sleep, takes measurement, enters
 *          sleep)
 *  @param  *r
 *          Red value
 *  @param  *g
 *          Green value
 *  @param  *b
 *          Blue value
 *  @param  *c
 *          Clear channel value
 */
void TCS34725::getRawDataOneShot(uint16_t *r, uint16_t *g, uint16_t *b,
                                          uint16_t *c) {
  enable();
  getRawData(r, g, b, c);
  disable();
}


void TCS34725::getRawDataOneShotOff(uint16_t *r, uint16_t *g, uint16_t *b,
                                          uint16_t *c) {
  enableLedOff();
  getRawData(r, g, b, c);
  disable();
}  


void TCS34725::getRawDataWithoutInterference(uint16_t *r, uint16_t *g, uint16_t *b,
                                          uint16_t *c) {
  uint16_t red1, green1, blue1, clear1;
  uint16_t red2, green2, blue2, clear2;      
  enable();                                      
  getRawData(&red1, &green1, &blue1, &clear1);
  disable();
  //delay(3);
  enableLedOff();
  getRawData(&red2, &green2, &blue2, &clear2);
  disable();
  //se o led nao estiver desativando, vai dar problema. Então tenho que tratar isso
  *r = red1 - red2;
  if(*r > 64000) *r = 0;
  *g = green1 - green2;
  if(*g > 64000) *g = 0;
  *b = blue1 - blue2;
  if(*b > 64000) *b = 0;
  *c = clear1 - clear2;
  if(*c > 64000) *c = 0;
}  

void TCS34725::getRGBCCalibrado(uint16_t *r, uint16_t *g, uint16_t *b,
                                   uint16_t *c) {
  uint16_t redRaw, greenRaw, blueRaw, clearRaw;
  getRawDataWithoutInterference(&redRaw, &greenRaw, &blueRaw, &clearRaw);
  
  if(*r > 64000) *r = 0;
  if(*g > 64000) *g = 0;
  if(*b > 64000) *b = 0;
  if(*c > 64000) *c = 0;
  // Aplica calibração
  *r = (uint32_t)redRaw * 255 / dadosCalibracao.r;
  *g = (uint32_t)greenRaw * 255 / dadosCalibracao.g;
  *b = (uint32_t)blueRaw * 255 / dadosCalibracao.b;
  *c = (uint32_t)clearRaw * 255 / dadosCalibracao.c;
  if(*r > 255) *r = 255;
  if(*g > 255) *g = 255;
  if(*b > 255) *b = 255;
  if(*c > 255) *c = 255;
}


/*!
 *  @brief  Read the RGB color detected by the sensor.
 *  @param  *r
 *          Red value normalized to 0-255
 *  @param  *g
 *          Green value normalized to 0-255
 *  @param  *b
 *          Blue value normalized to 0-255
 */
void TCS34725::getRGB(float *r, float *g, float *b) {
  uint16_t red, green, blue, clear;
  getRawData(&red, &green, &blue, &clear);
  uint32_t sum = clear;

  // Avoid divide by zero errors ... if clear = 0 return black
  if (clear == 0) {
    *r = *g = *b = 0;
    return;
  }

  *r = (float)red / sum * 255.0;
  *g = (float)green / sum * 255.0;
  *b = (float)blue / sum * 255.0;
}

/*!
 *  @brief  Converts the raw R/G/B values to color temperature in degrees Kelvin
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Color temperature in degrees Kelvin
 */
uint16_t TCS34725::calculateColorTemperature(uint16_t r, uint16_t g,
                                                      uint16_t b) {
  float X, Y, Z; /* RGB to XYZ correlation      */
  float xc, yc;  /* Chromaticity co-ordinates   */
  float n;       /* McCamy's formula            */
  float cct;

  if (r == 0 && g == 0 && b == 0) {
    return 0;
  }

  /* 1. Map RGB values to their XYZ counterparts.    */
  /* Based on 6500K fluorescent, 3000K fluorescent   */
  /* and 60W incandescent values for a wide range.   */
  /* Note: Y = Illuminance or lux                    */
  X = (-0.14282F * r) + (1.54924F * g) + (-0.95641F * b);
  Y = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);
  Z = (-0.68202F * r) + (0.77073F * g) + (0.56332F * b);

  /* 2. Calculate the chromaticity co-ordinates      */
  xc = (X) / (X + Y + Z);
  yc = (Y) / (X + Y + Z);

  /* 3. Use McCamy's formula to determine the CCT    */
  n = (xc - 0.3320F) / (0.1858F - yc);

  /* Calculate the final CCT */
  cct =
      (449.0F * powf(n, 3)) + (3525.0F * powf(n, 2)) + (6823.3F * n) + 5520.33F;

  /* Return the results in degrees Kelvin */
  return (uint16_t)cct;
}

/*!
 *  @brief  Converts the raw R/G/B values to color temperature in degrees
 *          Kelvin using the algorithm described in DN40 from Taos (now AMS).
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @param  c
 *          Clear channel value
 *  @return Color temperature in degrees Kelvin
 */
uint16_t TCS34725::calculateColorTemperature_dn40(uint16_t r,
                                                           uint16_t g,
                                                           uint16_t b,
                                                           uint16_t c) {
  uint16_t r2, b2; /* RGB values minus IR component */
  uint16_t sat;    /* Digital saturation level */
  uint16_t ir;     /* Inferred IR content */

  if (c == 0) {
    return 0;
  }

  /* Analog/Digital saturation:
   *
   * (a) As light becomes brighter, the clear channel will tend to
   *     saturate first since R+G+B is approximately equal to C.
   * (b) The TCS34725 accumulates 1024 counts per 2.4ms of integration
   *     time, up to a maximum values of 65535. This means analog
   *     saturation can occur up to an integration time of 153.6ms
   *     (64*2.4ms=153.6ms).
   * (c) If the integration time is > 153.6ms, digital saturation will
   *     occur before analog saturation. Digital saturation occurs when
   *     the count reaches 65535.
   */
  if ((256 - _tcs34725IntegrationTime) > 63) {
    /* Track digital saturation */
    sat = 65535;
  } else {
    /* Track analog saturation */
    sat = 1024 * (256 - _tcs34725IntegrationTime);
  }

  /* Ripple rejection:
   *
   * (a) An integration time of 50ms or multiples of 50ms are required to
   *     reject both 50Hz and 60Hz ripple.
   * (b) If an integration time faster than 50ms is required, you may need
   *     to average a number of samples over a 50ms period to reject ripple
   *     from fluorescent and incandescent light sources.
   *
   * Ripple saturation notes:
   *
   * (a) If there is ripple in the received signal, the value read from C
   *     will be less than the max, but still have some effects of being
   *     saturated. This means that you can be below the 'sat' value, but
   *     still be saturating. At integration times >150ms this can be
   *     ignored, but <= 150ms you should calculate the 75% saturation
   *     level to avoid this problem.
   */
  if ((256 - _tcs34725IntegrationTime) <= 63) {
    /* Adjust sat to 75% to avoid analog saturation if atime < 153.6ms */
    sat -= sat / 4;
  }

  /* Check for saturation and mark the sample as invalid if true */
  if (c >= sat) {
    return 0;
  }

  /* AMS RGB sensors have no IR channel, so the IR content must be */
  /* calculated indirectly. */
  ir = (r + g + b > c) ? (r + g + b - c) / 2 : 0;

  /* Remove the IR component from the raw RGB values */
  r2 = r - ir;
  b2 = b - ir;

  if (r2 == 0) {
    return 0;
  }

  /* A simple method of measuring color temp is to use the ratio of blue */
  /* to red light, taking IR cancellation into account. */
  uint16_t cct = (3810 * (uint32_t)b2) / /** Color temp coefficient. */
                    (uint32_t)r2 +
                 1391; /** Color temp offset. */

  return cct;
}

/*!
 *  @brief  Converts the raw R/G/B values to lux
 *  @param  r
 *          Red value
 *  @param  g
 *          Green value
 *  @param  b
 *          Blue value
 *  @return Lux value
 */
uint16_t TCS34725::calculateLux(uint16_t r, uint16_t g, uint16_t b) {
  float illuminance;

  /* This only uses RGB ... how can we integrate clear or calculate lux */
  /* based exclusively on clear since this might be more reliable?      */
  illuminance = (-0.32466F * r) + (1.57837F * g) + (-0.73191F * b);

  return (uint16_t)illuminance;
}

/*!
 *  @brief  Sets interrupt for TCS34725
 *  @param  i
 *          Interrupt (True/False)
 */
void TCS34725::setInterrupt(boolean i) {
  uint8_t r = read8(TCS34725_ENABLE);
  if (i) {
    r |= TCS34725_ENABLE_AIEN;
  } else {
    r &= ~TCS34725_ENABLE_AIEN;
  }
  write8(TCS34725_ENABLE, r);
}

/*!
 *  @brief  Clears inerrupt for TCS34725
 */
void TCS34725::clearInterrupt() {
  uint8_t buffer[1] = {TCS34725_COMMAND_BIT | 0x66};
  bus->write(buffer, 1);
}

/*!
 *  @brief  Sets inerrupt limits
 *  @param  low
 *          Low limit
 *  @param  high
 *          High limit
 */
void TCS34725::setIntLimits(uint16_t low, uint16_t high) {
  write8(0x04, low & 0xFF);
  write8(0x05, low >> 8);
  write8(0x06, high & 0xFF);
  write8(0x07, high >> 8);
}

/*!
 *  @brief  Retorna o número da porta I2C (1-5)
 *  @return Número da porta
 */
uint8_t TCS34725::getNumeroPorta() {
  return numeroPorta;
}

/*!
 *  @brief  Calibra o sensor e salva os valores na EEPROM
 *          Endereço base: 100 + (numeroPorta - 1) * sizeof(DadosCalibracao)
 *          Exemplo: Porta 1 = endereço 100, Porta 2 = endereço 109, etc.
 */
void TCS34725::calibrar() {
  if (!_tcs34725Initialised) {
    Serial.println(F("Sensor não inicializado. Não é possível calibrar."));
    return;  // Sensor não inicializado
  }
  
  DadosCalibracao dados;
  
  // Lê valores atuais do sensor
  //getRawData(&dados.r, &dados.g, &dados.b, &dados.c);
  getRawDataWithoutInterference(&dados.r, &dados.g, &dados.b, &dados.c);
  
  // Calcula checksum simples (XOR dos bytes)
  dados.checksum = 0;
  uint8_t *ptr = (uint8_t *)&dados;
  for (uint8_t i = 0; i < sizeof(DadosCalibracao) - 1; i++) {
    dados.checksum ^= ptr[i];
  }
  
  // Calcula endereço na EEPROM: 100 + (numeroPorta - 1) * tamanho
  uint16_t enderecoBase = 100 + (numeroPorta - 1) * sizeof(DadosCalibracao);
  
  // Salva na EEPROM
  EEPROM.put(enderecoBase, dados);
  Serial.println(F("Calibração salva na EEPROM."));
}

/*!
 *  @brief  Carrega valores de calibração da EEPROM
 *  @param  dados
 *          Ponteiro para estrutura onde os dados serão carregados
 *  @return true se calibração foi carregada com sucesso (checksum válido)
 */
boolean TCS34725::carregarCalibracao() {
  
  // Calcula endereço na EEPROM
  uint16_t enderecoBase = 100 + (numeroPorta - 1) * sizeof(DadosCalibracao);
  
  // Carrega da EEPROM
  EEPROM.get(enderecoBase, dadosCalibracao);
  
  // Verifica checksum
  uint8_t checksumCalculado = 0;
  uint8_t *ptr = (uint8_t *)&dadosCalibracao;
  for (uint8_t i = 0; i < sizeof(DadosCalibracao) - 1; i++) {
    checksumCalculado ^= ptr[i];
  }
  
  // Retorna true se checksum é válido
  return (checksumCalculado == dadosCalibracao.checksum);
}

