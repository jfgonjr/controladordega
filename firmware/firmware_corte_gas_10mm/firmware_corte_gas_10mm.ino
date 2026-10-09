/* =============================================================================
   FIRMWARE - CORTE_GAS_10MM
   Valvula borboleta rotativa de 90 graus com CORTE IMEDIATO DE GAS
   acionado por sensor de movimento (PIR) e/ou detector de gas.
   -----------------------------------------------------------------------------
   Derivado de: firmware_controlador_agua_10mm.ino (jfgonjr/controlador-de-fluido-10mm)

   MECANICA (INALTERADA - ver CAD / README do projeto)
     Corpo 88 x 50 x 62 mm, passagem Ø10, obturador Ø32 x 3 mm, eixo Ø6.
     0 graus = FECHADA (disco perpendicular ao fluxo)
     90 graus = ABERTA (disco alinhado ao fluxo)
     Encoder magnetico AS5600 (I2C) na ponta do eixo - sem fins de curso.
     Atuador: motor DC com reducao + ponte H (L9110 / DRV8833).

   CAIXA ELETRONICA - 3 PASSAGENS Ø7 mm (parede +Y)
     x = -20  Alimentacao 12 V
     x =   0  Sensor de movimento PIR (sinal digital, HIGH = movimento)
     x = +20  Detector de gas (saida digital) e, opcionalmente, o botao remoto
              (use cabo multivias de Ø <= 6,5 mm para caber na passagem)

   O QUE MUDOU EM RELACAO AO CONTROLADOR DE AGUA
     - Removidos: sonda de conducao, sensor de fluxo, pre-alarme de 60 s,
       botao de rearme separado e opcao de servo (servo nao e fail-safe).
     - T_CORTE_VAZAMENTO = 0 s: movimento (ou gas) com a valvula ABERTA ou
       ABRINDO fecha a valvula na mesma iteracao do loop().
     - Buzzer continuo + LED vermelho durante e apos o corte de emergencia.
     - Bloqueio persistente em EEPROM, abertura negada com presenca ativa,
       rearme manual so apos 3 s de botao pressionado e SEM presenca.
     - Entrada de detector de gas e saida opcional para eletrovalvula
       normalmente fechada em serie (corte direto na linha de gas).

   BOTAO UNICO (PIN_BTN_CMD, ativo em LOW)
     Toque curto, FECHADA .......... abre (se permitido)
     Toque curto, ABERTA ........... fecha (manual, sem bloqueio)
     Toque durante ABRINDO ......... EMERGENCIA do botao: fecha e BLOQUEIA
     Segurar 3 s, BLOQUEADA/FALHA .. rearme (so sem movimento / sem gas)

   Plataforma: Arduino Nano / Pro Mini (AVR) ou ESP32 / ESP32-C3
               (ESP32: core Arduino-ESP32 >= 3.0, para analogWrite/tone).
   Serial: 115200 baud
     Z = grava posicao atual como FECHADO (0 graus)   I = inverte sentido
     A = abrir   F = fechar   ? = status
   ============================================================================= */

#include <Wire.h>
#include <EEPROM.h>

/* =============================================================================
   1) PINAGEM (mesma placa 54 x 34 mm do projeto original)
   ============================================================================= */
#if defined(ESP32)
const uint8_t PIN_SDA       = 8;
const uint8_t PIN_SCL       = 9;
const uint8_t PIN_MOT_A     = 3;   // ponte H IN1 (PWM)
const uint8_t PIN_MOT_B     = 4;   // ponte H IN2 (PWM)
const uint8_t PIN_PIR       = 5;   // sensor de movimento (HIGH = presenca)
const uint8_t PIN_GAS       = 6;   // detector de gas (ver GAS_NIVEL_ATIVO)
const uint8_t PIN_SOL_GAS   = 1;   // saida p/ eletrovalvula NF (via MOSFET/rele)
const uint8_t PIN_BTN_CMD   = 7;   // botao unico (comando e rearme)
const uint8_t PIN_BUZZER    = 2;
const uint8_t PIN_LED_VERDE = 20;
const uint8_t PIN_LED_VERM  = 21;
#else  // Arduino Nano / Pro Mini
const uint8_t PIN_MOT_A     = 5;   // PWM
const uint8_t PIN_MOT_B     = 6;   // PWM
const uint8_t PIN_PIR       = 2;
const uint8_t PIN_GAS       = 3;
const uint8_t PIN_SOL_GAS   = 7;
const uint8_t PIN_BTN_CMD   = 4;
const uint8_t PIN_BUZZER    = 9;
const uint8_t PIN_LED_VERDE = 10;
const uint8_t PIN_LED_VERM  = 11;
#endif

/* =============================================================================
   2) PARAMETROS DO MECANISMO
   ============================================================================= */
const float   ANG_FECHADO  = 0.0f;
const float   ANG_ABERTO   = 90.0f;
const float   TOLERANCIA   = 2.5f;   // graus - janela de "chegou"
const float   ANG_DESACEL  = 20.0f;  // graus - inicio da rampa de frenagem
const uint8_t PWM_MIN      = 90;     // minimo para vencer o atrito da bucha
const uint8_t PWM_MAX      = 255;

/* =============================================================================
   3) TEMPOS (ms)
   ============================================================================= */
const unsigned long T_CORTE_VAZAMENTO = 0UL;      // SEM pre-alarme: corte imediato
const unsigned long T_MANOBRA_MAX     = 8000UL;   // 90 graus devem sair antes
const unsigned long T_STALL           = 1200UL;   // sem mover = travado
const unsigned long T_DESENCRAVA      = 250UL;    // recuo para soltar travamento
const unsigned long T_SEGURAR_REARME  = 3000UL;   // botao pressionado p/ rearme
const unsigned long T_DEBOUNCE_BOTAO  = 30UL;
const unsigned long T_FILTRO_GAS      = 300UL;    // anti-ruido do modulo de gas
const unsigned long T_AQUECIMENTO     = 30000UL;  // PIR/MQ instaveis na partida
const uint8_t       LIM_FALHAS_I2C    = 10;       // leituras seguidas com erro

/* Sinalizacao de emergencia */
const unsigned int  TOM_ALARME_HZ     = 2500;     // tom CONTINUO

/* =============================================================================
   4) SENSORES
   ============================================================================= */
const bool USAR_SENSOR_GAS     = true;   // false = ignora a entrada de gas
const bool USAR_SOLENOIDE_AUX  = false;  // true = comanda eletrovalvula NF extra
const uint8_t GAS_NIVEL_ATIVO  = LOW;    // modulos MQ: DO cai para LOW com gas

/* =============================================================================
   5) ENCODER MAGNETICO AS5600
   ============================================================================= */
const uint8_t AS5600_ADDR   = 0x36;
const uint8_t AS5600_RAWANG = 0x0C;   // 12 bits, 0..4095
const uint8_t AS5600_STATUS = 0x0B;   // MD/ML/MH - presenca do ima

bool as5600Ler(uint16_t &bruto) {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_RAWANG);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) != 2) return false;
  uint16_t hi = Wire.read(), lo = Wire.read();
  bruto = ((hi << 8) | lo) & 0x0FFF;
  return true;
}

bool as5600ImaOk() {
  Wire.beginTransmission(AS5600_ADDR);
  Wire.write(AS5600_STATUS);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) != 1) return false;
  uint8_t st = Wire.read();
  return (st & 0x20);                 // MD = ima detectado
}

/* =============================================================================
   6) DADOS PERSISTENTES (EEPROM)
   -----------------------------------------------------------------------------
   Mesma estrutura do firmware de agua: a calibracao (zero/sentido) ja gravada
   e preservada. 'bloqueado' sobrevive a quedas de energia.
   ============================================================================= */
struct Config {
  uint16_t magic;
  uint16_t zeroBruto;   // leitura do encoder na posicao FECHADA
  int8_t   sentido;     // +1 ou -1
  uint8_t  bloqueado;   // 1 = corte de emergencia pendente de rearme
};

const uint16_t MAGIC = 0xC0A1;
Config cfg, cfgSalvo;

void salvarCfg() {
  if (memcmp(&cfg, &cfgSalvo, sizeof(Config)) == 0) return;  // poupa a EEPROM
  EEPROM.put(0, cfg);
#if defined(ESP32)
  EEPROM.commit();
#endif
  cfgSalvo = cfg;
}

void carregarCfg() {
  EEPROM.get(0, cfg);
  cfgSalvo = cfg;
  if (cfg.magic != MAGIC) {            // primeira energizacao
    cfg.magic = MAGIC; cfg.zeroBruto = 0; cfg.sentido = 1; cfg.bloqueado = 0;
    cfgSalvo.magic = 0;                // forca a gravacao
    salvarCfg();
  }
}

/* =============================================================================
   7) ESTADOS E VARIAVEIS GLOBAIS
   ============================================================================= */
enum Estado : uint8_t {
  ST_FECHADA, ST_ABRINDO, ST_ABERTA, ST_FECHANDO, ST_BLOQUEADA, ST_FALHA
};
const char* const NOME_ESTADO[] = {
  "FECHADA", "ABRINDO", "ABERTA", "FECHANDO", "BLOQUEADA", "FALHA"
};

Estado estado = ST_FECHANDO;
unsigned long tEstado = 0, tRefStall = 0, tDesencrava = 0;
unsigned long tPerigoIni = 0, tGasMudou = 0, tBeepAte = 0;
unsigned long tBtnMudou = 0, tBtnPress = 0;

float   anguloAtual = 0.0f, anguloRefStall = 0.0f;
uint8_t tentativas = 0, falhasI2C = 0;

bool presenca = false;          // PIR em HIGH agora (sem filtro: 0 s)
bool gasAtivo = false, gasBruto = false;
bool perigoAnterior = false;

bool btnLido = false, btnEstavel = false;
bool btnPressionado = false;    // nivel estavel (debounced)
bool btnEvento = false;         // borda de descida (1 loop)
bool rearmeConsumido = false;
bool buzzerLigado = false;

void trocarEstado(Estado n) {
  estado = n;
  tEstado = millis();
  // Eletrovalvula auxiliar so fica energizada (aberta) com a valvula abrindo/aberta.
  if (USAR_SOLENOIDE_AUX)
    digitalWrite(PIN_SOL_GAS, (n == ST_ABRINDO || n == ST_ABERTA) ? HIGH : LOW);
  // Exige soltar o botao antes de contar os 3 s do rearme.
  if (n == ST_BLOQUEADA || n == ST_FALHA) rearmeConsumido = btnPressionado;
}

/* =============================================================================
   8) LEITURA DE POSICAO
   ============================================================================= */
float lerAngulo() {
  uint16_t bruto;
  if (!as5600Ler(bruto)) {
    if (falhasI2C < 255) falhasI2C++;
    return anguloAtual;               // mantem o ultimo valor
  }
  falhasI2C = 0;
  int16_t delta = (int16_t)bruto - (int16_t)cfg.zeroBruto;
  if (delta >  2048) delta -= 4096;
  if (delta < -2048) delta += 4096;
  return cfg.sentido * delta * (360.0f / 4096.0f);
}

/* =============================================================================
   9) ACIONAMENTO DO MOTOR
   ============================================================================= */
void motorParar() {
  analogWrite(PIN_MOT_A, 0);
  analogWrite(PIN_MOT_B, 0);
}

/* pwm > 0 abre (angulo crescente), pwm < 0 fecha */
void motorAcionar(int pwm) {
  pwm = constrain(pwm, -(int)PWM_MAX, (int)PWM_MAX);
  if (pwm > 0)      { analogWrite(PIN_MOT_B, 0); analogWrite(PIN_MOT_A, pwm); }
  else if (pwm < 0) { analogWrite(PIN_MOT_A, 0); analogWrite(PIN_MOT_B, -pwm); }
  else motorParar();
}

/* PWM cheio longe do alvo, rampa nos ultimos graus (evita bater o disco). */
void irParaAlvo(float destino) {
  float erro = destino - anguloAtual;
  float mod  = fabs(erro);
  if (mod <= TOLERANCIA) { motorParar(); return; }
  float k = (mod >= ANG_DESACEL) ? 1.0f : (mod / ANG_DESACEL);
  int pwm = PWM_MIN + (int)((PWM_MAX - PWM_MIN) * k);
  motorAcionar(erro > 0 ? pwm : -pwm);
}

bool chegou(float destino) { return fabs(destino - anguloAtual) <= TOLERANCIA; }

/* Travamento: motor comandado e angulo praticamente parado por T_STALL. */
bool detectouStall() {
  unsigned long agora = millis();
  if (fabs(anguloAtual - anguloRefStall) > 1.0f) {
    anguloRefStall = anguloAtual;
    tRefStall = agora;
    return false;
  }
  return (agora - tRefStall >= T_STALL);
}

void iniciarManobra() {              // reinicia a referencia de travamento
  anguloRefStall = anguloAtual;
  tRefStall = millis();
}

/* =============================================================================
   10) ENTRADAS: MOVIMENTO, GAS E BOTAO
   ============================================================================= */
void lerSensores() {
  unsigned long agora = millis();

  // Movimento: leitura direta, sem filtro (corte imediato).
  presenca = (digitalRead(PIN_PIR) == HIGH);

  // Gas: filtro curto apenas contra ruido do comparador do modulo.
  if (USAR_SENSOR_GAS) {
    bool g = (digitalRead(PIN_GAS) == GAS_NIVEL_ATIVO);
    if (g != gasBruto) { gasBruto = g; tGasMudou = agora; }
    if (agora - tGasMudou >= T_FILTRO_GAS) gasAtivo = gasBruto;
  } else {
    gasAtivo = false;
  }
}

void lerBotao() {
  unsigned long agora = millis();
  bool raw = (digitalRead(PIN_BTN_CMD) == LOW);
  if (raw != btnLido) { btnLido = raw; tBtnMudou = agora; }
  if ((agora - tBtnMudou >= T_DEBOUNCE_BOTAO) && (raw != btnEstavel)) {
    btnEstavel = raw;
    btnPressionado = raw;
    if (raw) { btnEvento = true; tBtnPress = agora; }
  }
}

/* Abertura e negada com movimento, gas ou sensores ainda aquecendo. */
bool aberturaBloqueada() {
  return presenca || gasAtivo || (millis() < T_AQUECIMENTO);
}

/* =============================================================================
   11) SINALIZACAO
   ============================================================================= */
void bipe(unsigned int hz, unsigned int ms) {
  tone(PIN_BUZZER, hz, ms);
  tBeepAte = millis() + ms;
  buzzerLigado = false;              // forca o tom continuo a retomar depois
}

void buzzerContinuo(bool ligar) {
  if (millis() < tBeepAte) return;   // deixa o bipe terminar
  if (ligar == buzzerLigado) return;
  buzzerLigado = ligar;
  if (ligar) tone(PIN_BUZZER, TOM_ALARME_HZ); else noTone(PIN_BUZZER);
}

void sinalizar() {
  unsigned long ms = millis();
  bool alarme = (cfg.bloqueado || estado == ST_FALHA);   // durante E apos o corte

  if (alarme) {
    bool vm = (estado == ST_FALHA) ? ((ms / 200) % 2) : true;  // FALHA pisca rapido
    digitalWrite(PIN_LED_VERM, vm);
    digitalWrite(PIN_LED_VERDE, LOW);
    buzzerContinuo(true);
    return;
  }

  digitalWrite(PIN_LED_VERM, LOW);
  buzzerContinuo(false);
  switch (estado) {
    case ST_ABERTA:  digitalWrite(PIN_LED_VERDE, HIGH); break;
    case ST_ABRINDO:
    case ST_FECHANDO: digitalWrite(PIN_LED_VERDE, (ms / 400) % 2); break;
    default:         digitalWrite(PIN_LED_VERDE, LOW); break;
  }
}

/* =============================================================================
   12) CORTE DE EMERGENCIA E REARME
   ============================================================================= */
/* Ordem deliberada: primeiro o motor e o alarme, depois a EEPROM (que leva
   alguns ms), para o corte comecar na mesma iteracao da deteccao. */
void cortarEmergencia(const __FlashStringHelper* motivo) {
  if (USAR_SOLENOIDE_AUX) digitalWrite(PIN_SOL_GAS, LOW);   // corte direto na linha
  motorAcionar(-PWM_MAX);                                   // fecha ja, PWM cheio
  digitalWrite(PIN_LED_VERM, HIGH);
  digitalWrite(PIN_LED_VERDE, LOW);
  tBeepAte = 0;
  buzzerLigado = false;
  buzzerContinuo(true);
  iniciarManobra();
  tentativas = 0;
  trocarEstado(ST_FECHANDO);
  cfg.bloqueado = 1;
  salvarCfg();                                              // persiste o bloqueio
  Serial.print(F("EMERGENCIA: "));
  Serial.print(motivo);
  Serial.println(F(" - valvula fechando e sistema BLOQUEADO."));
}

void tentarAbrir(const __FlashStringHelper* origem) {
  if (aberturaBloqueada()) {
    Serial.print(F("Abertura NEGADA ("));
    Serial.print(origem);
    Serial.println(presenca ? F("): movimento ativo.")
                   : gasAtivo ? F("): gas detectado.")
                              : F("): sensores aquecendo."));
    bipe(400, 600);
    return;
  }
  iniciarManobra();
  trocarEstado(ST_ABRINDO);
}

void iniciarFechamentoNormal() {
  iniciarManobra();
  tentativas = 0;
  trocarEstado(ST_FECHANDO);
}

/* Rearme: segurar o botao por 3 s CONSECUTIVOS, sem movimento e sem gas. */
void tratarRearme() {
  if (!btnPressionado) { rearmeConsumido = false; return; }
  if (rearmeConsumido) return;
  if (millis() - tBtnPress < T_SEGURAR_REARME) return;

  rearmeConsumido = true;
  if (aberturaBloqueada()) {
    bipe(400, 600);
    Serial.println(F("REARME NEGADO - movimento/gas ativo ou sensores aquecendo."));
    return;
  }
  cfg.bloqueado = 0;
  salvarCfg();
  bipe(1500, 200);
  Serial.println(F("REARME OK - confirmando fechamento."));
  iniciarFechamentoNormal();          // reconfirma 0 graus antes de liberar
}

/* =============================================================================
   13) CONSOLE SERIAL
   ============================================================================= */
void imprimirStatus() {
  Serial.print(F("estado="));  Serial.print(NOME_ESTADO[estado]);
  Serial.print(F(" ang="));    Serial.print(anguloAtual, 1);
  Serial.print(F(" mov="));    Serial.print(presenca);
  Serial.print(F(" gas="));    Serial.print(gasAtivo);
  Serial.print(F(" bloq="));   Serial.print(cfg.bloqueado);
  Serial.print(F(" sentido="));Serial.print(cfg.sentido);
  Serial.print(F(" ima="));    Serial.println(as5600ImaOk());
}

void tratarConsole() {
  if (!Serial.available()) return;
  char c = toupper(Serial.read());
  uint16_t bruto;
  switch (c) {
    case 'Z':
      if (estado == ST_ABRINDO || estado == ST_FECHANDO) {
        Serial.println(F("Calibracao recusada: valvula em movimento."));
      } else if (as5600Ler(bruto)) {
        cfg.zeroBruto = bruto; salvarCfg();
        Serial.println(F("Zero gravado na posicao atual (FECHADO)."));
      }
      break;
    case 'I':
      cfg.sentido = -cfg.sentido; salvarCfg();
      Serial.print(F("Sentido = ")); Serial.println(cfg.sentido);
      break;
    case 'A':
      if (estado == ST_FECHADA) tentarAbrir(F("serial"));
      else Serial.println(F("Abrir so e aceito com a valvula FECHADA e sem bloqueio."));
      break;
    case 'F':
      if (estado == ST_ABERTA || estado == ST_ABRINDO) iniciarFechamentoNormal();
      break;
    case '?':
      imprimirStatus();
      break;
  }
}

/* =============================================================================
   14) SETUP - PARTIDA SEGURA
   ============================================================================= */
void setup() {
  Serial.begin(115200);
#if defined(ESP32)
  EEPROM.begin(sizeof(Config) + 8);
  Wire.begin(PIN_SDA, PIN_SCL);
#else
  Wire.begin();
#endif
  Wire.setClock(400000);

  carregarCfg();

  pinMode(PIN_MOT_A, OUTPUT);  pinMode(PIN_MOT_B, OUTPUT);
  motorParar();

  pinMode(PIN_SOL_GAS, OUTPUT); digitalWrite(PIN_SOL_GAS, LOW);   // NF: fechada
#if defined(ESP32)
  pinMode(PIN_PIR, INPUT_PULLDOWN);
#else
  pinMode(PIN_PIR, INPUT);     // AVR: use resistor de 10 k para GND no PIR
#endif
  pinMode(PIN_GAS, INPUT_PULLUP);
  pinMode(PIN_BTN_CMD, INPUT_PULLUP);
  pinMode(PIN_BUZZER, OUTPUT);
  pinMode(PIN_LED_VERDE, OUTPUT);
  pinMode(PIN_LED_VERM, OUTPUT);

  if (!as5600ImaOk()) {
    Serial.println(F("ATENCAO: ima do eixo nao detectado pelo AS5600."));
    Serial.println(F("Sem leitura de angulo a valvula entrara em FALHA."));
  }

  // Partida segura: SEMPRE fecha primeiro, referenciando o zero do AS5600.
  anguloAtual = lerAngulo();
  iniciarManobra();
  tentativas = 0;
  trocarEstado(ST_FECHANDO);
  Serial.println(F("Corte_Gas_10mm iniciado - fechando por seguranca."));
  if (cfg.bloqueado) Serial.println(F("Bloqueio de emergencia anterior mantido (EEPROM)."));
}

/* =============================================================================
   15) LOOP
   ============================================================================= */
void loop() {
  unsigned long agora = millis();

  anguloAtual = lerAngulo();
  lerSensores();
  lerBotao();

  /* ---- Corte imediato: movimento ou gas com a valvula ABRINDO/ABERTA ---- */
  bool perigo = presenca || gasAtivo;
  if (perigo && !perigoAnterior) tPerigoIni = agora;
  perigoAnterior = perigo;

  if ((estado == ST_ABRINDO || estado == ST_ABERTA) && perigo &&
      (agora - tPerigoIni >= T_CORTE_VAZAMENTO)) {
    cortarEmergencia(presenca ? F("MOVIMENTO") : F("GAS"));
  }

  /* ---- Perda do encoder durante manobra: nao ha como controlar ---- */
  if ((estado == ST_ABRINDO || estado == ST_FECHANDO) && falhasI2C >= LIM_FALHAS_I2C) {
    motorParar();
    Serial.println(F("FALHA: AS5600 sem resposta."));
    cfg.bloqueado = 1; salvarCfg();
    trocarEstado(ST_FALHA);
  }

  /* ---- Maquina de estados ---- */
  switch (estado) {

    case ST_FECHADA:
      motorParar();
      if (btnEvento) tentarAbrir(F("botao"));
      break;

    case ST_ABRINDO:
      if (btnEvento) {                          // emergencia do botao
        cortarEmergencia(F("BOTAO"));
        break;
      }
      irParaAlvo(ANG_ABERTO);
      if (chegou(ANG_ABERTO)) {
        motorParar(); tentativas = 0; trocarEstado(ST_ABERTA);
      } else if (detectouStall() || agora - tEstado > T_MANOBRA_MAX) {
        motorParar();
        Serial.println(F("Obstrucao na abertura - fechando."));
        iniciarFechamentoNormal();
      }
      break;

    case ST_ABERTA:
      motorParar();
      if (btnEvento) iniciarFechamentoNormal();   // fechamento manual, sem bloqueio
      break;

    case ST_FECHANDO:
      if (agora < tDesencrava) { motorAcionar(PWM_MAX); break; }  // recuo curto
      irParaAlvo(ANG_FECHADO);
      if (chegou(ANG_FECHADO)) {
        motorParar(); tentativas = 0;
        trocarEstado(cfg.bloqueado ? ST_BLOQUEADA : ST_FECHADA);
      } else if (detectouStall() || agora - tEstado > T_MANOBRA_MAX) {
        motorParar();
        if (++tentativas <= 2) {                  // fechar e critico: insiste
          Serial.println(F("Travou ao fechar - tentando desencravar."));
          tDesencrava = agora + T_DESENCRAVA;
          iniciarManobra(); tEstado = agora;
        } else {
          Serial.println(F("FALHA: nao foi possivel fechar."));
          cfg.bloqueado = 1; salvarCfg();
          trocarEstado(ST_FALHA);
        }
      }
      break;

    case ST_BLOQUEADA:
    case ST_FALHA:
      motorParar();
      tratarRearme();
      break;
  }

  btnEvento = false;
  sinalizar();
  tratarConsole();
}
