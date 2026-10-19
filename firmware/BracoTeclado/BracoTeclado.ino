// =====================================================================================
//  Controle do braco robotico pelo TECLADO DO PC  -  Arduino Mega 2560
//  Versao 15/10: todos os eixos comecam em 0 na chave; nomes de ombro/cotovelo/punho
//               conforme o movimento real; garra: Y fecha, H abre (ver [15/10])
//  Versao 14/10: encoder da garra invertido e re-zeramento no fim do homing (ver [13/10]);
//               alvo sem corte e sem rezerar na chave nos eixos com compensacao (ver [14/10])
// =====================================================================================
//  O PC fica ligado ao Arduino pelo cabo USB. Cada tecla digitada no PC chega ao Arduino
//  pela Serial (9600 baud) e movimenta um eixo do braco.
//
//  Mesmo hardware do sketch TCCFinalFinalFinal2.ino: mesmos pinos de ponte H, encoders,
//  chaves fim de curso, limites de posicao e as mesmas compensacoes entre motores
//  (ex.: ao mover o ombro, cotovelo e punho se movem junto para manter a orientacao).
//
//  COMO USAR
//   - Recomendado: PuTTY (ja instalado no PC) -> Connection type "Serial", porta COMx,
//     velocidade 9600. O PuTTY envia cada tecla na hora, entao basta SEGURAR a tecla
//     para o eixo andar e SOLTAR para parar (modo SEGURAR).
//   - Tambem funciona no Monitor Serial da Arduino IDE ("Sem final de linha"/"No line
//     ending"): digite a letra e Enter; no modo SEGURAR o eixo anda um pequeno passo, no
//     modo CONTINUO ele anda ate apertar ESPACO (ou chegar no limite).
//
//  TECLAS (maiuscula ou minuscula)
//     Q / A  -> Base            (recuar / avancar)
//     W / S  -> Ombro           (recuar / avancar)
//     E / D  -> Cotovelo        (recuar / avancar)
//     R / F  -> Movimento punho (recuar / avancar)
//     T / G  -> Giro do punho   (recuar / avancar)
//     Y / H  -> Garra           (fechar / abrir)
//     X      -> define a posicao atual da garra como ABERTA (G=0). A garra nao tem chave
//               fim de curso: abra-a toda com H e aperte X antes de rodar programas.
//     U / J  -> Linha transversal (esquerda / direita)   [setas <- / -> tambem]
//     ESPACO -> PARAR TUDO
//     + / -  -> aumenta / diminui a velocidade (20% a 150%)
//     M      -> alterna modo SEGURAR / CONTINUO
//     P      -> mostra a posicao de todos os encoders
//     V      -> liga/desliga o monitoramento continuo da posicao
//     Z      -> zeramento (homing) - pressione Z duas vezes para confirmar
//     1      -> executa uma vez o programa automatico gravado
//     ?      -> mostra esta ajuda
//
//  PROGRAMA AUTOMATICO (comandos de texto - TODA linha comeca com '#', termina com Enter)
//     #INICIO           comeca a gravar um programa novo (apaga o anterior)
//     #B <pos>          leva a Base ate a posicao <pos> do encoder (0 a 53000)
//     #O <pos>          Ombro          #C <pos>   Cotovelo
//     #MP <pos>         Mov. punho     #GP <pos>  Giro do punho
//     #G <pos>          Garra (0 = aberta, maior = mais fechada)
//     #LT <pos>         Linha transversal
//     #ESPERA <ms>      pausa em milissegundos
//     #VEL <%>          velocidade dos proximos movimentos (20 a 150)
//     #ZERAR            faz o zeramento de todos os eixos
//     #// texto         comentario (ignorado)
//     #FIM              termina a gravacao e mostra o programa
//     #EXECUTAR <n>     executa o programa n vezes (0 = repete ate apertar ESPACO)
//     #LISTAR           mostra o programa gravado       #POS  mostra as posicoes
//   Fora do #INICIO/#FIM, um comando de eixo (ex.: #B 25000) e executado na hora.
//   Para enviar pelo PuTTY: copie o texto (ex.: programa_demo.txt) e clique com o botao
//   direito na janela do PuTTY para colar. ESPACO interrompe o programa a qualquer momento.
//   Ao ligar, ja vem gravado um programa de demonstracao (tecla 1 para executar).
// =====================================================================================

// O pino 53 (LT_encoderPinA) nao suporta attachInterrupt no Mega2560 (so 2,3,18,19,20,21),
// entao o encoder da linha transversal usa interrupcao por mudanca de pino (PCINT0),
// direto nos registradores do AVR. No ATmega2560, o pino 53 = PB0 = PCINT0.

//Pinos das pontes h
//Linha Transversal
const int LT_ENA = 44;   // PWM para velocidade
const int LT_IN1 = 10;
const int LT_IN2 = 45;
//base
const int B_ENA = 4;
const int B_IN1 = 40;
const int B_IN2 = 41;
//ombro
const int O_ENA = 5;
const int O_IN1 = 42;
const int O_IN2 = 25;
//Cotovelo
const int C_ENA = 6;
const int C_IN1 = 26;
const int C_IN2 = 27;
//Giro do punho
const int GP_ENA = 7;
const int GP_IN1 = 28;
const int GP_IN2 = 29;
//Movimento do punho
const int MP_ENA = 8;
const int MP_IN1 = 30;
const int MP_IN2 = 31;
//Garra
const int G_ENA = 9;
const int G_IN1 = 32;
const int G_IN2 = 33;

//Pinos dos encoders
const byte LT_encoderPinA = 53;
const byte LT_encoderPinB = 52;
const byte B_encoderPinA = 18;
const byte B_encoderPinB = 34;
const byte O_encoderPinA = 19;
const byte O_encoderPinB = 35;
const byte C_encoderPinA = 20;
const byte C_encoderPinB = 36;
const byte GP_encoderPinA = 21;
const byte GP_encoderPinB = 37;
const byte MP_encoderPinA = 2;
const byte MP_encoderPinB = 38;
const byte G_encoderPinA = 3;
const byte G_encoderPinB = 39;

//Pinos das chaves fim de curso (HIGH = chave acionada)
const int LT_limitSwitch = 51;
const int B_limitSwitch = 46;
const int O_limitSwitch = 47;
const int C_limitSwitch = 48;
const int GP_limitSwitch = 49;
const int MP_limitSwitch = 50;

//Posicao dos encoders (atualizadas nas interrupcoes)
volatile long LT_encoderPos = 0;
volatile long B_encoderPos = 0;
volatile long O_encoderPos = 0;
volatile long C_encoderPos = 0;
volatile long GP_encoderPos = 0;
volatile long MP_encoderPos = 0;
volatile long G_encoderPos = 0;

//Posicoes maximas e minimas. [15/10] Mesmo curso do sketch original, mas todo eixo comeca em 0
//na chave fim de curso (antes: B -55000..-2000, C -60..19950, MP -80..10600, GP -410..23000).
const long LT_POSICAO_MINIMA = 0;
const long LT_POSICAO_MAXIMA = 100000000;
const long B_POSICAO_MINIMA = 0;        // [15/10] todos os eixos comecam em 0 na chave
const long B_POSICAO_MAXIMA = 53000;
const long O_POSICAO_MINIMA = 0;
const long O_POSICAO_MAXIMA = 18000;
const long C_POSICAO_MINIMA = 0;
const long C_POSICAO_MAXIMA = 20010;
const long MP_POSICAO_MINIMA = 0;
const long MP_POSICAO_MAXIMA = 10680;
const long GP_POSICAO_MINIMA = 0;
const long GP_POSICAO_MAXIMA = 23410;
const long G_POSICAO_MINIMA = 0;
const long G_POSICAO_MAXIMA = 26095;

// ---------------- Configuracoes do controle por teclado ----------------
// Faz o zeramento de todos os eixos ao ligar (como no sketch original).
const bool HOMING_NA_INICIALIZACAO = true;
// Tempo maximo (ms) que o homing de cada eixo espera pela chave fim de curso.
const unsigned long HOMING_TIMEOUT_MS = 15000;
// Modo SEGURAR: o eixo para se nenhuma tecla chegar dentro deste tempo.
// O primeiro tempo e maior porque o Windows espera ~500 ms antes de comecar a repetir
// a tecla segurada; depois que a repeticao comeca, o eixo para logo apos soltar a tecla.
const unsigned long TEMPO_PRIMEIRA_TECLA_MS = 600;
const unsigned long TEMPO_REPETICAO_MS = 150;
// Intervalo do monitoramento continuo de posicao (tecla V).
const unsigned long INTERVALO_MONITOR_MS = 300;
// Tempo para confirmar o zeramento (segunda tecla Z).
const unsigned long TEMPO_CONFIRMA_Z_MS = 2000;

// ---------------- Movimentos ----------------
enum Movimento : uint8_t {
  M_NENHUM = 0,
  M_B_AVANCAR, M_B_RECUAR,
  M_O_AVANCAR, M_O_RECUAR,
  M_C_AVANCAR, M_C_RECUAR,
  M_MP_AVANCAR, M_MP_RECUAR,
  M_GP_AVANCAR, M_GP_RECUAR,
  M_G_ABRIR, M_G_FECHAR,
  M_LT_ESQUERDA, M_LT_DIREITA,
  M_TOTAL
};

// PWM de cada motor para cada movimento (velocidade 100%). Os valores e os motores
// acoplados sao os mesmos do sketch original.
struct DefMovimento {
  const char* nome;
  int lt, b, o, c, mp, gp, g;
};

const DefMovimento MOVIMENTOS[M_TOTAL] = {
  //  nome                      LT     B     O     C    MP    GP     G
  { "Nenhum",                    0,    0,    0,    0,    0,    0,    0 },
  { "Base recuar",              0,  -80,    0,    0,    0,    0,    0 },
  { "Base avancar",               0,   80,    0,    0,    0,    0,    0 },
  // [15/10] Nomes trocados para bater com o movimento real: W/E/R levam o eixo de volta a
  // chave (recuar) e S/D/F o afastam (avancar). As teclas e os motores nao mudaram.
  { "Ombro recuar",              0,    0, -150,   90,  -80,   80,    0 },
  { "Ombro avancar",             0,    0,  150,  -90,   80,  -80,    0 },
  { "Cotovelo recuar",           0,    0,    0,  -80,  100,  -90,    0 },
  { "Cotovelo avancar",          0,    0,    0,   80,  -50,   50,    0 },
  { "Mov. punho recuar",         0,    0,    0,    0,  -80,   80,    0 },
  { "Mov. punho avancar",        0,    0,    0,    0,   80,  -80,    0 },
  { "Giro punho recuar",        0,    0,    0,    0,  -90,  -80,    0 },
  { "Giro punho avancar",         0,    0,    0,    0,   90,   80,    0 },
  // [15/10] Garra: PWM positivo FECHA a garra no braco real. Abrir agora e -80 e fechar +80.
  { "Garra abrir",               0,    0,    0,    0,    0,    0,  -80 },
  { "Garra fechar",              0,    0,    0,    0,    0,    0,   80 },
  { "Linha transv. esquerda",  230,    0,    0,    0,    0,    0,    0 },
  { "Linha transv. direita",  -230,    0,    0,    0,    0,    0,    0 },
};

enum Modo : uint8_t { MODO_SEGURAR, MODO_CONTINUO };

Movimento movAtual = M_NENHUM;
Modo modo = MODO_SEGURAR;
int velocidadePct = 100;
bool teclaRepetindo = false;
unsigned long ultimaTeclaMs = 0;
bool monitorLigado = false;
unsigned long ultimoMonitorMs = 0;
unsigned long pedidoZeramentoMs = 0;
bool aguardandoConfirmaZ = false;
uint8_t estadoEscape = 0;   // para reconhecer as setas (ESC [ C / ESC [ D)

// ---------------- Programa automatico ----------------
// Tempo maximo de um movimento automatico antes de desistir (eixo travado/encoder parado).
const unsigned long TIMEOUT_MOVIMENTO_MS = 30000;

enum TipoPasso : uint8_t { P_B, P_O, P_C, P_MP, P_GP, P_G, P_LT, P_ESPERA, P_VEL, P_ZERAR, P_INVALIDO };
const char* const NOMES_PASSO[] = { "B", "O", "C", "MP", "GP", "G", "LT", "ESPERA", "VEL", "ZERAR" };

struct Passo {
  uint8_t tipo;
  long valor;
};

const uint8_t MAX_PASSOS = 100;
Passo programa[MAX_PASSOS];
uint8_t totalPassos = 0;
bool gravandoPrograma = false;
bool gravacaoComErro = false;

// Linha de comando de texto (comeca com '#', termina com Enter)
char linhaCmd[40];
uint8_t tamLinhaCmd = 0;
bool lendoLinhaCmd = false;
bool ignorandoLinhaNoAbort = false;

// Eixos que podem ir para uma posicao. "sobe" = movimento que AUMENTA a contagem do
// encoder (mesmos sentidos do modo automatico do sketch original).
struct EixoAuto {
  volatile long* pos;
  long minimo, maximo;
  Movimento sobe, desce;
};

const EixoAuto EIXOS_AUTO[] = {
  { &B_encoderPos,  B_POSICAO_MINIMA,  B_POSICAO_MAXIMA,  M_B_AVANCAR,  M_B_RECUAR    },  // P_B
  { &O_encoderPos,  O_POSICAO_MINIMA,  O_POSICAO_MAXIMA,  M_O_RECUAR,   M_O_AVANCAR   },  // P_O
  { &C_encoderPos,  C_POSICAO_MINIMA,  C_POSICAO_MAXIMA,  M_C_RECUAR,   M_C_AVANCAR   },  // P_C
  { &MP_encoderPos, MP_POSICAO_MINIMA, MP_POSICAO_MAXIMA, M_MP_RECUAR,  M_MP_AVANCAR  },  // P_MP
  { &GP_encoderPos, GP_POSICAO_MINIMA, GP_POSICAO_MAXIMA, M_GP_RECUAR,  M_GP_AVANCAR  },  // P_GP
  { &G_encoderPos,  G_POSICAO_MINIMA,  G_POSICAO_MAXIMA,  M_G_FECHAR,   M_G_ABRIR     },  // P_G
  { &LT_encoderPos, LT_POSICAO_MINIMA, LT_POSICAO_MAXIMA, M_LT_DIREITA, M_LT_ESQUERDA },  // P_LT
};

// Programa de demonstracao carregado ao ligar: gira a base, move ombro e cotovelo e
// abre/fecha a garra. AJUSTE as posicoes para o seu braco (use a tecla P para ver as
// posicoes atuais depois de levar o braco ate o ponto desejado com o teclado).
const Passo PROGRAMA_DEMO[] = {
  { P_VEL,    100 },
  { P_G,        0 },   // abre a garra
  { P_B,    15000 },   // gira a base para um lado
  { P_O,     6000 },   // ombro
  { P_C,     8000 },   // cotovelo
  { P_ESPERA, 500 },
  { P_G,    15000 },   // fecha a garra (pega a peca)
  { P_ESPERA, 500 },
  { P_O,     2000 },   // recolhe o ombro
  { P_B,    40000 },   // gira a base para o outro lado
  { P_O,     6000 },
  { P_ESPERA, 500 },
  { P_G,        0 },   // abre a garra (solta a peca)
  { P_ESPERA, 500 },
  { P_O,     2000 },
  { P_C,     2000 },
  { P_B,    25000 },   // volta para o meio
};

// =====================================================================================
void setup() {
  Serial.begin(9600);

  const int saidas[] = { LT_ENA, LT_IN1, LT_IN2, B_ENA, B_IN1, B_IN2, O_ENA, O_IN1, O_IN2,
                         C_ENA, C_IN1, C_IN2, GP_ENA, GP_IN1, GP_IN2, MP_ENA, MP_IN1, MP_IN2,
                         G_ENA, G_IN1, G_IN2 };
  for (unsigned int i = 0; i < sizeof(saidas) / sizeof(saidas[0]); i++) {
    pinMode(saidas[i], OUTPUT);
  }
  pararTudo();

  const byte entradas[] = { LT_encoderPinA, LT_encoderPinB, B_encoderPinA, B_encoderPinB,
                            O_encoderPinA, O_encoderPinB, C_encoderPinA, C_encoderPinB,
                            GP_encoderPinA, GP_encoderPinB, MP_encoderPinA, MP_encoderPinB,
                            G_encoderPinA, G_encoderPinB, LT_limitSwitch, B_limitSwitch,
                            O_limitSwitch, C_limitSwitch, GP_limitSwitch, MP_limitSwitch };
  for (unsigned int i = 0; i < sizeof(entradas) / sizeof(entradas[0]); i++) {
    pinMode(entradas[i], INPUT_PULLUP);
  }

  // Interrupcao por mudanca de pino so no pino 53 (PCINT0) - encoder da linha transversal.
  PCICR |= (1 << PCIE0);
  PCMSK0 |= (1 << PCINT0);

  attachInterrupt(digitalPinToInterrupt(B_encoderPinA), B_updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(O_encoderPinA), O_updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(C_encoderPinA), C_updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(GP_encoderPinA), GP_updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(MP_encoderPinA), MP_updateEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(G_encoderPinA), G_updateEncoder, RISING);

  totalPassos = sizeof(PROGRAMA_DEMO) / sizeof(PROGRAMA_DEMO[0]);
  for (uint8_t i = 0; i < totalPassos; i++) {
    programa[i] = PROGRAMA_DEMO[i];
  }

  Serial.println(F("\r\n=== Braco robotico - controle pelo teclado ==="));
  if (HOMING_NA_INICIALIZACAO) {
    zeramento();
  }
  mostrarAjuda();
}

// =====================================================================================
void loop() {
  while (Serial.available() > 0) {
    processarByte((char)Serial.read());
  }

  atualizarMovimento();

  if (aguardandoConfirmaZ && millis() - pedidoZeramentoMs > TEMPO_CONFIRMA_Z_MS) {
    aguardandoConfirmaZ = false;
    Serial.println(F("Zeramento cancelado."));
  }

  if (monitorLigado && millis() - ultimoMonitorMs >= INTERVALO_MONITOR_MS) {
    ultimoMonitorMs = millis();
    mostrarPosicoes();
  }
}

// ---------------- Leitura do teclado ----------------
void processarByte(char c) {
  // Linha de comando de texto: junta os caracteres ate o Enter.
  if (lendoLinhaCmd) {
    if (c == '\r' || c == '\n') {
      linhaCmd[tamLinhaCmd] = '\0';
      lendoLinhaCmd = false;
      processarLinha(linhaCmd);
    } else if (c == 8 || c == 127) {          // backspace
      if (tamLinhaCmd > 0) tamLinhaCmd--;
    } else if (tamLinhaCmd < sizeof(linhaCmd) - 1) {
      linhaCmd[tamLinhaCmd++] = toupper(c);
    }
    return;
  }
  if (c == '#') {
    pararMovimento(NULL);
    lendoLinhaCmd = true;
    tamLinhaCmd = 0;
    estadoEscape = 0;
    return;
  }
  // Enquanto grava um programa colado, qualquer texto fora de uma linha '#' e ignorado,
  // para que nenhuma letra solta seja tratada como tecla de movimento.
  if (gravandoPrograma) return;

  // Setas chegam como ESC '[' 'A'..'D' (PuTTY / terminais ANSI).
  if (estadoEscape == 1) {
    estadoEscape = (c == '[') ? 2 : 0;
    if (estadoEscape == 2) return;
  } else if (estadoEscape == 2) {
    estadoEscape = 0;
    if (c == 'D') { comandoMovimento(M_LT_ESQUERDA); return; }
    if (c == 'C') { comandoMovimento(M_LT_DIREITA); return; }
    if (c >= '0' && c <= '9') { estadoEscape = 3; }   // ex.: ESC [ 2 ~ (Insert, F5...)
    return;                                          // ignora setas cima/baixo e outras
  } else if (estadoEscape == 3) {
    bool continuaSequencia = (c >= '0' && c <= '9') || c == ';';
    if (!continuaSequencia) estadoEscape = 0;   // termina no '~' ou em qualquer letra
    return;
  }

  if (c == 27) { estadoEscape = 1; return; }
  if (c == '\r' || c == '\n') return;

  if (c != 'z' && c != 'Z') aguardandoConfirmaZ = false;

  switch (c) {
    case 'q': case 'Q': comandoMovimento(M_B_AVANCAR);   break;
    case 'a': case 'A': comandoMovimento(M_B_RECUAR);    break;
    case 'w': case 'W': comandoMovimento(M_O_AVANCAR);   break;
    case 's': case 'S': comandoMovimento(M_O_RECUAR);    break;
    case 'e': case 'E': comandoMovimento(M_C_AVANCAR);   break;
    case 'd': case 'D': comandoMovimento(M_C_RECUAR);    break;
    case 'r': case 'R': comandoMovimento(M_MP_AVANCAR);  break;
    case 'f': case 'F': comandoMovimento(M_MP_RECUAR);   break;
    case 't': case 'T': comandoMovimento(M_GP_AVANCAR);  break;
    case 'g': case 'G': comandoMovimento(M_GP_RECUAR);   break;
    case 'y': case 'Y': comandoMovimento(M_G_FECHAR);    break;   // [15/10] Y fecha
    case 'h': case 'H': comandoMovimento(M_G_ABRIR);     break;   // [15/10] H abre
    case 'u': case 'U': comandoMovimento(M_LT_ESQUERDA); break;
    case 'j': case 'J': comandoMovimento(M_LT_DIREITA);  break;

    case ' ':
      pararMovimento(F("Parado pelo usuario"));
      break;

    case '+': case '=':
      alterarVelocidade(10);
      break;
    case '-': case '_':
      alterarVelocidade(-10);
      break;

    case 'm': case 'M':
      pararMovimento(NULL);
      modo = (modo == MODO_SEGURAR) ? MODO_CONTINUO : MODO_SEGURAR;
      Serial.print(F("Modo: "));
      Serial.println(modo == MODO_SEGURAR ? F("SEGURAR (anda enquanto a tecla estiver pressionada)")
                                          : F("CONTINUO (anda ate ESPACO ou limite)"));
      break;

    case 'p': case 'P':
      mostrarPosicoes();
      break;

    case 'v': case 'V':
      monitorLigado = !monitorLigado;
      Serial.print(F("Monitoramento de posicao: "));
      Serial.println(monitorLigado ? F("LIGADO") : F("DESLIGADO"));
      break;

    case 'z': case 'Z':
      pararMovimento(NULL);
      if (aguardandoConfirmaZ && millis() - pedidoZeramentoMs <= TEMPO_CONFIRMA_Z_MS) {
        aguardandoConfirmaZ = false;
        zeramento();
      } else {
        aguardandoConfirmaZ = true;
        pedidoZeramentoMs = millis();
        Serial.println(F("Pressione Z novamente para confirmar o ZERAMENTO de todos os eixos."));
      }
      break;

    case '1':
      executarPrograma(1);
      break;

    case 'x': case 'X':
      pararMovimento(NULL);
      gravarPos(&G_encoderPos, G_POSICAO_MINIMA);
      Serial.println(F("Garra: posicao atual definida como ABERTA (G=0)."));
      break;

    case '?': case 'i': case 'I':
      mostrarAjuda();
      break;

    default:
      break;   // tecla sem funcao: ignora
  }
}

// ---------------- Controle dos movimentos ----------------
void comandoMovimento(Movimento m) {
  ultimaTeclaMs = millis();

  if (m == movAtual) {
    // Mesma tecla chegando de novo = tecla sendo segurada (auto-repeticao).
    teclaRepetindo = true;
    return;
  }

  pararTudo();
  movAtual = M_NENHUM;

  if (limiteAtingido(m)) {
    // Os movimentos sao pares (avancar/recuar) seguidos no enum: o oposto e o vizinho.
    const char TECLAS[M_TOTAL] = { ' ', 'Q', 'A', 'W', 'S', 'E', 'D', 'R', 'F', 'T', 'G', 'H', 'Y', 'U', 'J' };
    Movimento oposto = (Movimento)((m % 2 == 1) ? m + 1 : m - 1);
    bool porChave = (m == M_B_RECUAR || m == M_O_AVANCAR || m == M_C_AVANCAR ||
                     m == M_MP_AVANCAR || m == M_GP_AVANCAR || m == M_LT_ESQUERDA);
    Serial.print(F(">> "));
    Serial.print(MOVIMENTOS[m].nome);
    Serial.print(porChave ? F(": CHAVE FIM DE CURSO acionada (posicao de zero).")
                          : F(": LIMITE MAXIMO do encoder atingido."));
    Serial.print(F(" Use a tecla "));
    Serial.print(TECLAS[oposto]);
    Serial.println(F(" para mover no outro sentido."));
    return;
  }

  movAtual = m;
  teclaRepetindo = false;
  aplicarPWM(m, velocidadePct);
  Serial.print(F("Movendo: "));
  Serial.print(MOVIMENTOS[m].nome);
  Serial.print(F("  (vel "));
  Serial.print(velocidadePct);
  Serial.println(F("%)"));
}

// Chamado a cada volta do loop: verifica limites e o tempo da tecla.
void atualizarMovimento() {
  if (movAtual == M_NENHUM) return;

  if (limiteAtingido(movAtual)) {
    pararMovimento(F(">> LIMITE atingido"));
    return;
  }

  if (modo == MODO_SEGURAR) {
    unsigned long limite = teclaRepetindo ? TEMPO_REPETICAO_MS : TEMPO_PRIMEIRA_TECLA_MS;
    if (millis() - ultimaTeclaMs > limite) {
      pararMovimento(NULL);
    }
  }
}

void pararMovimento(const __FlashStringHelper* motivo) {
  Movimento anterior = movAtual;
  pararTudo();
  movAtual = M_NENHUM;
  teclaRepetindo = false;

  if (anterior != M_NENHUM) {
    if (motivo != NULL) {
      Serial.print(motivo);
      Serial.print(F(" - "));
    }
    Serial.print(F("Parou: "));
    Serial.print(MOVIMENTOS[anterior].nome);
    Serial.print(F("  ->  "));
    mostrarPosicaoDoEixo(anterior);
  } else if (motivo != NULL) {
    Serial.println(F("Todos os motores parados."));
  }
}

// Retorna true se o movimento m nao pode continuar. Quando a chave fim de curso e
// atingida, o encoder daquele eixo e re-zerado (como no sketch original).
bool limiteAtingido(Movimento m) {
  switch (m) {
    case M_B_AVANCAR:   return lerPos(&B_encoderPos) >= B_POSICAO_MAXIMA;
    case M_B_RECUAR:
      if (digitalRead(B_limitSwitch) == HIGH) { gravarPos(&B_encoderPos, B_POSICAO_MINIMA); return true; }
      return lerPos(&B_encoderPos) <= B_POSICAO_MINIMA;

    case M_O_AVANCAR:
      if (digitalRead(O_limitSwitch) == HIGH) { gravarPos(&O_encoderPos, O_POSICAO_MINIMA); return true; }
      return false;
    case M_O_RECUAR:    return lerPos(&O_encoderPos) >= O_POSICAO_MAXIMA;

    case M_C_AVANCAR:
      // [14/10] Sem rezerar aqui: a compensacao desloca a contagem deste motor, entao chave
      // acionada nao significa contagem = minimo. Este eixo so e zerado no zeramento.
      if (digitalRead(C_limitSwitch) == HIGH) return true;
      return false;
    case M_C_RECUAR:    return lerPos(&C_encoderPos) >= C_POSICAO_MAXIMA;

    case M_MP_AVANCAR:
      // [14/10] Sem rezerar aqui: a compensacao desloca a contagem deste motor, entao chave
      // acionada nao significa contagem = minimo. Este eixo so e zerado no zeramento.
      if (digitalRead(MP_limitSwitch) == HIGH) return true;
      return false;
    case M_MP_RECUAR:   return lerPos(&MP_encoderPos) >= MP_POSICAO_MAXIMA;

    case M_GP_AVANCAR:
      // [14/10] Sem rezerar aqui: a compensacao desloca a contagem deste motor, entao chave
      // acionada nao significa contagem = minimo. Este eixo so e zerado no zeramento.
      if (digitalRead(GP_limitSwitch) == HIGH) return true;
      return false;
    case M_GP_RECUAR:   return lerPos(&GP_encoderPos) >= GP_POSICAO_MAXIMA;

    // A garra nao tem chave fim de curso e o encoder comeca em 0 onde ela estiver ao
    // ligar, entao o limite de software so vale no modo CONTINUO. No modo SEGURAR o
    // usuario controla a garra livremente (e usa a tecla X para marcar "aberta" = 0).
    // Nos programas automaticos o alvo ja e limitado entre G_POSICAO_MINIMA e MAXIMA.
    case M_G_ABRIR:     return modo == MODO_CONTINUO && lerPos(&G_encoderPos) < G_POSICAO_MINIMA;
    case M_G_FECHAR:    return modo == MODO_CONTINUO && lerPos(&G_encoderPos) >= G_POSICAO_MAXIMA;

    case M_LT_ESQUERDA: return digitalRead(LT_limitSwitch) == HIGH;
    case M_LT_DIREITA:  return lerPos(&LT_encoderPos) >= LT_POSICAO_MAXIMA;

    default:            return true;
  }
}

void aplicarPWM(Movimento m, int pct) {
  const DefMovimento& d = MOVIMENTOS[m];
  setMotorLT(escala(d.lt, pct));
  setMotorB(escala(d.b, pct));
  setMotorO(escala(d.o, pct));
  setMotorC(escala(d.c, pct));
  setMotorMP(escala(d.mp, pct));
  setMotorGP(escala(d.gp, pct));
  setMotorG(escala(d.g, pct));
}

int escala(int pwm, int pct) {
  return constrain((long)pwm * pct / 100, -255, 255);
}

void alterarVelocidade(int delta) {
  velocidadePct = constrain(velocidadePct + delta, 20, 150);
  Serial.print(F("Velocidade: "));
  Serial.print(velocidadePct);
  Serial.println(F("%"));
  if (movAtual != M_NENHUM) {
    aplicarPWM(movAtual, velocidadePct);
  }
}

void pararTudo() {
  setMotorLT(0);
  setMotorB(0);
  setMotorO(0);
  setMotorC(0);
  setMotorMP(0);
  setMotorGP(0);
  setMotorG(0);
}

// ---------------- Zeramento (homing) ----------------
// Mesma sequencia do sketch original: Base, Mov. punho, Giro punho, Cotovelo, Ombro e
// por ultimo abre a garra. Cada eixo tem timeout e o processo pode ser abortado com
// ESPACO a qualquer momento. Retorna false se foi abortado.
bool zeramento() {
  Serial.println(F("ZERAMENTO iniciado (ESPACO aborta)..."));
  pararTudo();
  movAtual = M_NENHUM;

  struct EtapaHoming { const char* nome; int chave; Movimento mov; volatile long* pos; long zero; };
  const EtapaHoming etapas[] = {
    { "Base",            B_limitSwitch,  M_B_RECUAR,   &B_encoderPos,  B_POSICAO_MINIMA },
    { "Movimento punho", MP_limitSwitch, M_MP_AVANCAR, &MP_encoderPos, MP_POSICAO_MINIMA },
    { "Giro punho",      GP_limitSwitch, M_GP_AVANCAR, &GP_encoderPos, GP_POSICAO_MINIMA },
    { "Cotovelo",        C_limitSwitch,  M_C_AVANCAR,  &C_encoderPos,  C_POSICAO_MINIMA },
    { "Ombro",           O_limitSwitch,  M_O_AVANCAR,  &O_encoderPos,  O_POSICAO_MINIMA },
  };

  for (unsigned int i = 0; i < sizeof(etapas) / sizeof(etapas[0]); i++) {
    Serial.print(F("  Zerando "));
    Serial.print(etapas[i].nome);
    Serial.print(F("... "));

    unsigned long inicio = millis();
    if (digitalRead(etapas[i].chave) == LOW) {
      aplicarPWM(etapas[i].mov, 100);
    }
    while (digitalRead(etapas[i].chave) == LOW) {
      if (abortarPeloTeclado()) {
        pararTudo();
        Serial.println(F("\r\nZERAMENTO ABORTADO pelo usuario."));
        return false;
      }
      if (millis() - inicio > HOMING_TIMEOUT_MS) {
        Serial.print(F("TIMEOUT (chave nao respondeu) - "));
        break;
      }
    }
    pararTudo();
    gravarPos(etapas[i].pos, etapas[i].zero);
    Serial.println(F("ok"));
  }

  // [13/10] Ao zerar cotovelo e ombro, as compensacoes giram tambem os motores do punho
  // (e o ombro gira o do cotovelo), entao as contagens dos eixos ja zerados mudam mesmo
  // com as juntas paradas na chave. Re-zera todo eixo que continua com a chave acionada.
  for (unsigned int i = 0; i < sizeof(etapas) / sizeof(etapas[0]); i++) {
    if (digitalRead(etapas[i].chave) == HIGH) {
      gravarPos(etapas[i].pos, etapas[i].zero);
    }
  }

  // Abre a garra ate a posicao minima do encoder.
  unsigned long inicio = millis();
  if (lerPos(&G_encoderPos) > G_POSICAO_MINIMA) {
    Serial.print(F("  Abrindo garra... "));
    aplicarPWM(M_G_ABRIR, 100);
    while (lerPos(&G_encoderPos) > G_POSICAO_MINIMA) {
      if (abortarPeloTeclado()) {
        pararTudo();
        Serial.println(F("\r\nZERAMENTO ABORTADO pelo usuario."));
        return false;
      }
      if (millis() - inicio > HOMING_TIMEOUT_MS) break;
    }
    pararTudo();
    Serial.println(F("ok"));
  }

  Serial.println(F("ZERAMENTO concluido."));
  mostrarPosicoes();
  return true;
}

// Le o teclado durante zeramento/programa. ESPACO (ou ESC) interrompe; espacos dentro de
// uma linha de comando '#...' (texto colado) sao ignorados para nao interromper por engano.
bool abortarPeloTeclado() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '#') {
      ignorandoLinhaNoAbort = true;
    } else if (c == '\r' || c == '\n') {
      ignorandoLinhaNoAbort = false;
    } else if (!ignorandoLinhaNoAbort && (c == ' ' || c == 27)) {
      return true;
    }
  }
  return false;
}

// ---------------- Programa automatico ----------------
TipoPasso tipoDoComando(const char* cmd) {
  for (uint8_t i = 0; i < P_INVALIDO; i++) {
    if (strcmp(cmd, NOMES_PASSO[i]) == 0) return (TipoPasso)i;
  }
  return P_INVALIDO;
}

void processarLinha(char* texto) {
  char* cmd = strtok(texto, " \t=:");
  if (cmd == NULL) return;                                   // linha "#" vazia
  if (cmd[0] == '/' || cmd[0] == '-' || cmd[0] == '#') return; // comentario
  char* arg = strtok(NULL, " \t=:");
  long valor = (arg != NULL) ? atol(arg) : 0;

  if (strcmp(cmd, "INICIO") == 0) {
    gravandoPrograma = true;
    gravacaoComErro = false;
    totalPassos = 0;
    Serial.println(F("Gravando programa... (termine com #FIM)"));
    return;
  }
  if (strcmp(cmd, "FIM") == 0) {
    if (!gravandoPrograma) return;
    gravandoPrograma = false;
    Serial.print(F("Programa gravado com "));
    Serial.print(totalPassos);
    Serial.println(F(" passos."));
    if (gravacaoComErro) {
      Serial.println(F("ATENCAO: algumas linhas foram ignoradas (invalidas ou programa cheio)."));
    }
    listarPrograma();
    Serial.println(F("Tecla 1 ou #EXECUTAR <vezes> para rodar."));
    return;
  }
  if (strcmp(cmd, "EXECUTAR") == 0 || strcmp(cmd, "EXEC") == 0) {
    if (gravandoPrograma) return;
    executarPrograma(arg != NULL ? valor : 1);
    return;
  }
  if (strcmp(cmd, "LISTAR") == 0) { listarPrograma(); return; }
  if (strcmp(cmd, "POS") == 0)    { mostrarPosicoes(); return; }

  TipoPasso tipo = tipoDoComando(cmd);
  if (tipo == P_INVALIDO || (tipo != P_ZERAR && arg == NULL)) {
    gravacaoComErro = gravandoPrograma;
    Serial.print(F("Comando invalido: #"));
    Serial.println(cmd);
    return;
  }

  if (gravandoPrograma) {
    // Sem mensagem por linha: imprimir aqui atrasaria a leitura do texto colado.
    if (totalPassos < MAX_PASSOS) {
      programa[totalPassos].tipo = tipo;
      programa[totalPassos].valor = valor;
      totalPassos++;
    } else {
      gravacaoComErro = true;
    }
    return;
  }

  // Fora da gravacao: executa o comando na hora.
  executarPasso(tipo, valor);
  Serial.println(F("PRONTO"));
}

void imprimirPasso(uint8_t tipo, long valor) {
  Serial.print(NOMES_PASSO[tipo]);
  if (tipo != P_ZERAR) {
    Serial.print(' ');
    Serial.print(valor);
  }
}

void listarPrograma() {
  Serial.print(F("Programa ("));
  Serial.print(totalPassos);
  Serial.println(F(" passos):"));
  for (uint8_t i = 0; i < totalPassos; i++) {
    Serial.print(F("  "));
    Serial.print(i + 1);
    Serial.print(F(": #"));
    imprimirPasso(programa[i].tipo, programa[i].valor);
    Serial.println();
  }
}

// vezes = 0 repete ate o usuario apertar ESPACO.
void executarPrograma(long vezes) {
  pararMovimento(NULL);
  if (totalPassos == 0) {
    Serial.println(F("Nenhum programa gravado (use #INICIO ... #FIM)."));
    return;
  }
  int velocidadeAntes = velocidadePct;
  ignorandoLinhaNoAbort = false;
  Serial.println(F("EXECUTANDO programa (ESPACO interrompe)..."));

  bool ok = true;
  for (long ciclo = 1; ok && (vezes <= 0 || ciclo <= vezes); ciclo++) {
    Serial.print(F("-- Ciclo "));
    Serial.println(ciclo);
    for (uint8_t i = 0; ok && i < totalPassos; i++) {
      Serial.print(F("Passo "));
      Serial.print(i + 1);
      Serial.print('/');
      Serial.print(totalPassos);
      Serial.print(F(": "));
      imprimirPasso(programa[i].tipo, programa[i].valor);
      Serial.print(F(" ... "));
      ok = executarPasso(programa[i].tipo, programa[i].valor);
    }
  }

  pararTudo();
  velocidadePct = velocidadeAntes;
  Serial.println(ok ? F("PROGRAMA CONCLUIDO.") : F("PROGRAMA INTERROMPIDO."));
  mostrarPosicoes();
}

// Retorna false se o programa deve ser interrompido.
bool executarPasso(uint8_t tipo, long valor) {
  switch (tipo) {
    case P_ESPERA: {
      unsigned long inicio = millis();
      while (millis() - inicio < (unsigned long)max(valor, 0L)) {
        if (abortarPeloTeclado()) {
          Serial.println(F("interrompido"));
          return false;
        }
      }
      Serial.println(F("ok"));
      return true;
    }
    case P_VEL:
      velocidadePct = constrain(valor, 20L, 150L);
      Serial.print(F("velocidade "));
      Serial.print(velocidadePct);
      Serial.println(F("%"));
      return true;
    case P_ZERAR:
      Serial.println();
      return zeramento();
    default:
      return moverPara(EIXOS_AUTO[tipo], valor);
  }
}

// Leva um eixo ate a posicao 'alvo' do encoder, respeitando limites e chaves.
bool moverPara(const EixoAuto& eixo, long alvo) {
  // [14/10] Ombro, cotovelo e punho: as compensacoes deslocam as contagens desses motores
  // (ex.: depois de mover o ombro, C fica negativo com o cotovelo parado). Cortar o alvo na
  // faixa minimo-maximo impedia voltar a um ponto gravado. A chave fim de curso e o limite
  // maximo (limiteAtingido) continuam protegendo o eixo durante o movimento.
  bool contagemAcoplada = (eixo.pos == &O_encoderPos || eixo.pos == &C_encoderPos ||
                           eixo.pos == &MP_encoderPos || eixo.pos == &GP_encoderPos);
  if (!contagemAcoplada) alvo = constrain(alvo, eixo.minimo, eixo.maximo);
  long pos = lerPos(eixo.pos);
  if (pos == alvo) {
    Serial.println(F("ja esta na posicao"));
    return true;
  }

  bool subindo = alvo > pos;
  Movimento m = subindo ? eixo.sobe : eixo.desce;
  if (limiteAtingido(m)) {
    Serial.println(F("LIMITE/chave fim de curso - pulando"));
    return true;
  }

  bool ok = true;
  unsigned long inicio = millis();
  aplicarPWM(m, velocidadePct);
  while (true) {
    pos = lerPos(eixo.pos);
    if (subindo ? (pos >= alvo) : (pos <= alvo)) break;
    if (limiteAtingido(m)) {
      Serial.print(F("LIMITE atingido "));
      break;
    }
    if (abortarPeloTeclado()) {
      Serial.print(F("interrompido "));
      ok = false;
      break;
    }
    if (millis() - inicio > TIMEOUT_MOVIMENTO_MS) {
      Serial.print(F("TIMEOUT (eixo travado ou encoder sem sinal?) "));
      ok = false;
      break;
    }
  }
  pararTudo();
  Serial.print(F("pos="));
  Serial.println(lerPos(eixo.pos));
  return ok;
}

// ---------------- Mensagens ----------------
void mostrarPosicaoDoEixo(Movimento m) {
  switch (m) {
    case M_B_AVANCAR:  case M_B_RECUAR:    Serial.print(F("B="));  Serial.println(lerPos(&B_encoderPos));  break;
    case M_O_AVANCAR:  case M_O_RECUAR:    Serial.print(F("O="));  Serial.println(lerPos(&O_encoderPos));  break;
    case M_C_AVANCAR:  case M_C_RECUAR:    Serial.print(F("C="));  Serial.println(lerPos(&C_encoderPos));  break;
    case M_MP_AVANCAR: case M_MP_RECUAR:   Serial.print(F("MP=")); Serial.println(lerPos(&MP_encoderPos)); break;
    case M_GP_AVANCAR: case M_GP_RECUAR:   Serial.print(F("GP=")); Serial.println(lerPos(&GP_encoderPos)); break;
    case M_G_ABRIR:    case M_G_FECHAR:    Serial.print(F("G="));  Serial.println(lerPos(&G_encoderPos));  break;
    case M_LT_ESQUERDA: case M_LT_DIREITA: Serial.print(F("LT=")); Serial.println(lerPos(&LT_encoderPos)); break;
    default: Serial.println(); break;
  }
}

void mostrarPosicoes() {
  Serial.print(F("POS  LT="));  Serial.print(lerPos(&LT_encoderPos));
  Serial.print(F("  B="));      Serial.print(lerPos(&B_encoderPos));
  Serial.print(F("  O="));      Serial.print(lerPos(&O_encoderPos));
  Serial.print(F("  C="));      Serial.print(lerPos(&C_encoderPos));
  Serial.print(F("  MP="));     Serial.print(lerPos(&MP_encoderPos));
  Serial.print(F("  GP="));     Serial.print(lerPos(&GP_encoderPos));
  Serial.print(F("  G="));      Serial.println(lerPos(&G_encoderPos));
  // 1 = chave acionada. Se uma chave fica em 1 mesmo longe do fim de curso, verifique o
  // fio: com INPUT_PULLUP, chave desconectada tambem le como acionada.
  Serial.print(F("CHAVES LT=")); Serial.print(digitalRead(LT_limitSwitch));
  Serial.print(F("  B="));       Serial.print(digitalRead(B_limitSwitch));
  Serial.print(F("  O="));       Serial.print(digitalRead(O_limitSwitch));
  Serial.print(F("  C="));       Serial.print(digitalRead(C_limitSwitch));
  Serial.print(F("  MP="));      Serial.print(digitalRead(MP_limitSwitch));
  Serial.print(F("  GP="));      Serial.println(digitalRead(GP_limitSwitch));
}

void mostrarAjuda() {
  Serial.println(F("--------------------------------------------------"));
  Serial.println(F(" Q / A  Base             (recuar / avancar)"));
  Serial.println(F(" W / S  Ombro            (recuar / avancar)"));
  Serial.println(F(" E / D  Cotovelo         (recuar / avancar)"));
  Serial.println(F(" R / F  Movimento punho  (recuar / avancar)"));
  Serial.println(F(" T / G  Giro do punho    (recuar / avancar)"));
  Serial.println(F(" Y / H  Garra            (fechar / abrir)   X  garra atual = aberta (G=0)"));
  Serial.println(F(" U / J  Linha transversal (esquerda / direita)  ou setas <- ->"));
  Serial.println(F(" ESPACO PARAR TUDO"));
  Serial.println(F(" + / -  Velocidade        M  Modo segurar/continuo"));
  Serial.println(F(" P  Posicoes   V  Monitorar posicao   Z Z  Zeramento   ?  Ajuda"));
  Serial.println(F(" 1  Executa o programa automatico gravado (ESPACO interrompe)"));
  Serial.println(F(" Comandos de texto: #INICIO  #B 25000  #O 6000  #C 8000  #G 15000"));
  Serial.println(F("   #ESPERA 500  #VEL 80  #ZERAR  #FIM  #EXECUTAR 3  #LISTAR  #POS"));
  Serial.println(F("--------------------------------------------------"));
  Serial.print(F("Modo: "));
  Serial.print(modo == MODO_SEGURAR ? F("SEGURAR") : F("CONTINUO"));
  Serial.print(F("   Velocidade: "));
  Serial.print(velocidadePct);
  Serial.println(F("%"));
}

// ---------------- Acesso seguro as posicoes (long de 32 bits no AVR de 8 bits) ----------------
long lerPos(volatile long* p) {
  noInterrupts();
  long v = *p;
  interrupts();
  return v;
}

void gravarPos(volatile long* p, long v) {
  noInterrupts();
  *p = v;
  interrupts();
}

// ---------------- Pontes H ----------------
void setMotor(int ena, int in1, int in2, int pwm) {
  pwm = constrain(pwm, -255, 255);
  if (pwm > 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(ena, pwm);
  } else if (pwm < 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(ena, -pwm);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
    analogWrite(ena, 0);
  }
}

void setMotorLT(int pwm) { setMotor(LT_ENA, LT_IN1, LT_IN2, pwm); }
void setMotorB(int pwm)  { setMotor(B_ENA,  B_IN1,  B_IN2,  pwm); }
void setMotorO(int pwm)  { setMotor(O_ENA,  O_IN1,  O_IN2,  pwm); }
void setMotorC(int pwm)  { setMotor(C_ENA,  C_IN1,  C_IN2,  pwm); }
void setMotorGP(int pwm) { setMotor(GP_ENA, GP_IN1, GP_IN2, pwm); }
void setMotorMP(int pwm) { setMotor(MP_ENA, MP_IN1, MP_IN2, pwm); }
void setMotorG(int pwm)  { setMotor(G_ENA,  G_IN1,  G_IN2,  pwm); }

// ---------------- Encoders ----------------
// PCINT0 dispara em qualquer mudanca do pino 53; so conta na subida (igual RISING).
ISR(PCINT0_vect) {
  if (digitalRead(LT_encoderPinA) == HIGH) {
    if (digitalRead(LT_encoderPinB) == HIGH) LT_encoderPos--;
    else LT_encoderPos++;
  }
}

void B_updateEncoder() {
  if (digitalRead(B_encoderPinB) == HIGH) B_encoderPos--;
  else B_encoderPos++;
}

void O_updateEncoder() {
  if (digitalRead(O_encoderPinB) == HIGH) O_encoderPos++;
  else O_encoderPos--;
}

void C_updateEncoder() {
  if (digitalRead(C_encoderPinB) == HIGH) C_encoderPos++;
  else C_encoderPos--;
}

void GP_updateEncoder() {
  if (digitalRead(GP_encoderPinB) == HIGH) GP_encoderPos++;
  else GP_encoderPos--;
}

void MP_updateEncoder() {
  if (digitalRead(MP_encoderPinB) == HIGH) MP_encoderPos++;
  else MP_encoderPos--;
}

// [15/10] Sentido original. Com o motor da garra corrigido (fechar = PWM positivo), fechar
// AUMENTA a contagem (0 = aberta, 26095 = fechada), como o firmware espera.
void G_updateEncoder() {
  if (digitalRead(G_encoderPinB) == HIGH) G_encoderPos++;
  else G_encoderPos--;
}
