/*
  Script da programação do controle duplo PID em cascata do drone desenvolvido pelo
  aluno Eduardo Araujo de Sousa, estudante de Engenharia de Controle e Automação no 
  IFRO, campus Porto Velho - Calama.

  O código foi adaptado de https://arduproject.es/
  dronedesdecero@gmail.com
  
 */

// --------------------------------------------------------------------------------

bool visu = 1;         // Visualizar variáveis pelo monitor serial configurar em 1. Configurar para 0 quando estiver em voo!!
int visu_select = 3;   // 0: Controle RC, 1: giro, 2: acc, 3: ang, 4: esc
bool MODO_voo = 1;   // 0: Modo acrobático, 1: Modo estável (por padrão MODO_voo = 1)

#define usCiclo 6000   // Tempo do ciclo de execução do software em microssegundos

#define pin_motor1 6        // Pin motor 1
#define pin_motor2 9        // Pin motor 2
#define pin_motor3 10        // Pin motor 3
#define pin_motor4 11        // Pin motor 4
#define pin_INT_Throttle 3  // Pin Throttle do controle RC
#define pin_INT_Yaw 2       // Pin Yaw do controle RC
#define pin_INT_Pitch 4    // Pin Pitch do controle RC
#define pin_INT_Roll 5      // Pin Roll do controle RC
 
// --------------------------------------------------------------------------------

#include <EnableInterrupt.h> //Para utilizar nas interrupções do software
#include <Wire.h>            //Para comunicação do sensor MPU6050

// AJUSTE DE PIDs
// Modificar estes parâmetros para melhor sintonia dos controladores PID
float Roll_ang_Kp  = 1, Roll_ang_Ki  = 0.01, Roll_ang_Kd  = 5; //Parâmetros PID Roll controle de inclinação 
float Pitch_ang_Kp = 1, Pitch_ang_Ki = 0.01, Pitch_ang_Kd = 5; //Parâmetros PID Pitch controle de inclinação 
float Pitch_W_Kp   = 0.5,   Pitch_W_Ki   = 0.01, Pitch_W_Kd   = 0; //Parâmetros PI Pitch controle de velocidade angular 
float Roll_W_Kp    = 0.5,   Roll_W_Ki    = 0.01, Roll_W_Kd    = 0; //Parâmetros PI Roll controle de velocidade angular 
float Yaw_W_Kp     = 0,   Yaw_W_Ki     = 0, Yaw_W_Kd     = 0; //Parâmetros PI Yaw controle de velocidade angular 

int PID_W_sat1   = 380;  // Limitar parte integral PID velocidade angular
int PID_W_sat2   = 380;  // Limitar saída do PID velocidade angular
int PID_ang_sat1 = 130;  // Limitar parte integral PID inclinação
int PID_ang_sat2 = 130;  // Limitar saída do PID inclinação

//Variáveis PID a serem utilizadas nos moduladores
float PID_ang_Pitch_error, PID_ang_Pitch_P, PID_ang_Pitch_I, PID_ang_Pitch_D, PID_ang_Pitch_OUT;
float PID_ang_Roll_error, PID_ang_Roll_P, PID_ang_Roll_I, PID_ang_Roll_D, PID_ang_Roll_OUT;
float PID_ang_Yaw_error, PID_ang_Yaw_P, PID_ang_Yaw_I, PID_ang_Yaw_D, PID_ang_Yaw_OUT;
float PID_W_Pitch_error, PID_W_Pitch_P, PID_W_Pitch_I, PID_W_Pitch_D, PID_W_Pitch_OUT;
float PID_W_Roll_error, PID_W_Roll_P, PID_W_Roll_I, PID_W_Roll_D, PID_W_Roll_OUT;
float PID_W_Yaw_error, PID_W_Yaw_P, PID_W_Yaw_I, PID_W_Yaw_D, PID_W_Yaw_OUT;
float PID_W_Pitch_consigna, PID_W_Roll_consigna;

//---------------------------------------------------------------------------------

// AJUSTE CONTROLE RC - THROTLLE
const int us_max_Throttle_adj = 1800;
const int us_min_Throttle_adj = 970;
const float us_max_Throttle_raw = 2004;  // <-- Se a entrada Throttle está invertida substituir este valor
const float us_min_Throttle_raw = 1116;  // <-- por este e vice-versa

// AJUSTE CONTROLE RC - PITCH
const float us_max_Pitch_raw = 1952;
const float us_min_Pitch_raw = 992;
const int us_max_Pitch_adj = -30;  // <-- Se a entrada pitch está invertida substituir este valor
const int us_min_Pitch_adj = 30;   // <-- por este e vice-versa

// AJUSTE CONTROLE RC - ROLL
const float us_max_Roll_raw = 1960;
const float us_min_Roll_raw = 992;
const int us_max_Roll_adj = 30;    // <-- Se a entrada roll está invertida substituir este valor
const int us_min_Roll_adj = -30;   // <-- por este e vice-versa

// AJUSTE CONTROLE RC - YAW
const float us_max_Yaw_raw = 1928;
const float us_min_Yaw_raw = 972;
const int us_max_Yaw_adj = 30;     // <-- Se a entrada yaw está invertida substituir este valor
const int us_min_Yaw_adj = -30;    // <-- por este e vice-versa

//---------------------------------------------------------------------------------

//Variáveis do sensor MPU6050
#define MPU6050_adress 0x68
float angulo_pitch, angulo_roll, angulo_yaw, angulo_pitch_acc, angulo_roll_acc, temperature;
float angulo_pitch_ant, angulo_roll_ant, angulo_yaw_ant;
int gx, gy, gz, gyro_Z, gyro_X, gyro_Y, gyro_X_ant, gyro_Y_ant, gyro_Z_ant;
float gyro_X_cal, gyro_Y_cal, gyro_Z_cal;
float ax, ay, az, acc_X_cal, acc_Y_cal, acc_Z_cal, acc_total_vector;
bool set_gyro_angles, accCalibOK = false;
float tempo_execucao_MPU6050, tempo_MPU6050_1;

//---------------------------------------------------------------------------------

// VARIÁVEIS DE TEMPO
long loop_timer, loop_timer1, tempo_motores_start, tempo_ON, tempo_1, tempo_2;

/// SINAIS PWM
float ESC1_us, ESC2_us, ESC3_us, ESC4_us;

// VARIÁVEIS CONTROLE RC
float RC_Throttle_filt, RC_Pitch_filt, RC_Yaw_filt, RC_Roll_filt;
float RC_Throttle_consigna, RC_Pitch_consigna, RC_Roll_consigna, RC_Yaw_consigna;

//---------------------------------------------------------------------------------

//LEITURA DO SINAL (uS) DOS CANAIS DO RÁDIO RC ATRAVÉS DE INTERRUPÇÕES DE SOFTWARE

// CONTAGEM DE TEMPO - THROTTLE
volatile long Throttle_HIGH_us;
volatile int RC_Throttle_raw;
void INT_Throttle() {
  if (digitalRead(pin_INT_Throttle) == HIGH)Throttle_HIGH_us = micros();
  if (digitalRead(pin_INT_Throttle) == LOW) RC_Throttle_raw  = micros() - Throttle_HIGH_us;
}

// CONTAGEM DE TEMPO - PITCH
volatile long Pitch_HIGH_us;
volatile int RC_Pitch_raw;
void INT_Pitch() {
  if (digitalRead(pin_INT_Pitch) == HIGH)Pitch_HIGH_us = micros();
  if (digitalRead(pin_INT_Pitch) == LOW) RC_Pitch_raw  = micros() - Pitch_HIGH_us;
}

// CONTAGEM DE TEMPO - ROLL
volatile long Pitch_Roll_us;
volatile int RC_Roll_raw;
void INT_Roll() {
  if (digitalRead(pin_INT_Roll) == HIGH)Pitch_Roll_us = micros();
  if (digitalRead(pin_INT_Roll) == LOW) RC_Roll_raw   = micros() - Pitch_Roll_us;
}

// CONTAGEM DE TEMPO - YAW
volatile long Pitch_Yaw_us;
volatile int RC_Yaw_raw;
void INT_Yaw() {
  if (digitalRead(pin_INT_Yaw) == HIGH)Pitch_Yaw_us = micros();
  if (digitalRead(pin_INT_Yaw) == LOW) RC_Yaw_raw   = micros() - Pitch_Yaw_us;
}

//---------------------------------------------------------------------------------

void setup() {

  Wire.begin(); //Inicia a comunicação I2C do MPU6050

  if (visu == 1) {
    Serial.begin(115200); //configura a baud rate para a visualização no monitor serial    
  }
  // Controle RC: declaração de interrupcões
  pinMode(pin_INT_Yaw, INPUT_PULLUP);                
  enableInterrupt(pin_INT_Yaw, INT_Yaw, CHANGE);
  pinMode(pin_INT_Throttle, INPUT_PULLUP);           
  enableInterrupt(pin_INT_Throttle, INT_Throttle, CHANGE);
  pinMode(pin_INT_Pitch, INPUT_PULLUP);              
  enableInterrupt(pin_INT_Pitch, INT_Pitch, CHANGE);
  pinMode(pin_INT_Roll, INPUT_PULLUP);                 
  enableInterrupt(pin_INT_Roll, INT_Roll, CHANGE);

  // Declaração dos pinos dos motores
  pinMode(pin_motor1, OUTPUT);  //Motor 1
  pinMode(pin_motor2, OUTPUT);  //Motor 2
  pinMode(pin_motor3, OUTPUT);  //Motor 3
  pinMode(pin_motor4, OUTPUT);  //Motor 4
  // Forçando os pinos ao estado LOW
  digitalWrite(pin_motor1, LOW);
  digitalWrite(pin_motor2, LOW);
  digitalWrite(pin_motor3, LOW);
  digitalWrite(pin_motor4, LOW);

  //Led do arduino para indicar erro no sensor MPU6050
  pinMode(LED_BUILTIN, OUTPUT);

  // Para conectar o receptor com o rádio RC, ligar o rádio e baixar o stick do throttle no mínimo
  Serial.print("Ligar o rádio");
  if (MODO_voo == 1) Serial.print("-MODO Estável-");
  else Serial.print("-MODO Acro-");
  while (RC_Throttle_consigna < 950 || RC_Throttle_consigna > 1050) RC_procesar(); //condição para conectar receptor e transmissor

  MPU6050_iniciar();         // Iniciar sensor MPU6050
  MPU6050_calibrar();        // Calibrar sensor MPU6050

  // Para entrar no loop principal, deve-se mover o stick de Roll à direita
  Serial.print("ROLL -->");
  while (RC_Roll_consigna < 10)RC_procesar();

  
  loop_timer = micros();
}

//---------------------------------------------------------------------------------

void loop() {
  // Se o ciclo de tempo de 6000 us é excedido, uma mensagem de alerta é impressa na serial monitor
  if (micros() - loop_timer > usCiclo + 50) Serial.print("Tempo do ciclo excedido!!!");
  // Inicia-se um novo ciclo
  while (micros() - loop_timer < usCiclo);
  // Registra o instante do começo do ciclo
  loop_timer = micros();

  PWM();                           // Gera os sinais PWM para os motores
  MPU6050_leer();                  // Leituras do sensor MPU6050
  MPU6050_procesar();              // Processa os dados do sensor MPU6050
  if (MODO_voo == 1)PID_ang();     // Obtem as saídas do PID de inclinação
  PID_w();                         // Obtem as saídas do PID de velocidade angular
  Modulador();                     // Modulador dos sinais de saída do PID para PWM

  // Guardamos as leituras do sensor MPU6050 para o ciclo seguinte (necessário para os dois PIDs)
  angulo_pitch_ant = angulo_pitch;
  angulo_roll_ant  = angulo_roll;
  angulo_yaw_ant   = angulo_yaw;
  gyro_X_ant = gyro_X; // Pitch
  gyro_Y_ant = gyro_Y; // Roll
  gyro_Z_ant = gyro_Z; // Yaw

  // Visualização das variáveis
  if (visu == 1)Visualizacoes(); 
  

}

//---------------------------------------------------------------------------------

// Inicia o sensor MPU6050
void MPU6050_iniciar() {
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x6B);                                                          //Registro 6B hex)
  Wire.write(0x00);                                                          //00000000 para ativar giroscopio
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);                                                          //Register 1B hex
  Wire.write(0x08);                                                          //Girscopio a 500dps (full scale)
  Wire.endTransmission();
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1C);                                                          //Register (1A hex)
  Wire.write(0x10);                                                          //Acelerometro a  +/- 8g (full scale range)
  Wire.endTransmission();

  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1B);
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_adress, 1);
  while (Wire.available() < 1);

  // Se houver um erro no sensor MPU6050, avisamos e travamos programa
  if (Wire.read() != 0x08) {
    Serial.print("MPU6050 error");
    while (1) {
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
    }
  }

  // Ativar e configurar o filtro passa baixas (LPF) que incorpora o sensor
  Wire.beginTransmission(MPU6050_adress);
  Wire.write(0x1A);
  Wire.write(0x04);
  Wire.endTransmission();

  /*
    Frequencia de corte do filtro passa baixas:
    256Hz(0ms):0x00
    188Hz(2ms):0x01
    98Hz(3ms):0x02
    42Hz(4.9ms):0x03
    20Hz(8.5ms):0x04
    10Hz(13.8ms):0x05
    5Hz(19ms):0x06
  */
}

//---------------------------------------------------------------------------------

// Calibrar sensor MPU6050
void MPU6050_calibrar() {
  Serial.print("Calib. MPU6050");

  // Calibrar giroscopio tomando 3000 amostras
  for (int cal_int = 0; cal_int < 3000 ; cal_int ++) {
    MPU6050_leer();
    gyro_X_cal += gx;
    gyro_Y_cal += gy;
    gyro_Z_cal += gz;
    acc_X_cal  += ax;
    acc_Y_cal  += ay;
    acc_Z_cal  += az;
    delayMicroseconds(20);
  }
  // Calcular o valor medio das 3000 amostras
  gyro_X_cal = gyro_X_cal / 3000;
  gyro_Y_cal = gyro_Y_cal / 3000;
  gyro_Z_cal = gyro_Z_cal / 3000;
  acc_X_cal  = acc_X_cal  / 3000;
  acc_Y_cal  = acc_Y_cal  / 3000;
  acc_Z_cal  = acc_Z_cal  / 3000;
  accCalibOK = true;
}

//---------------------------------------------------------------------------------

// Leer sensor MPU6050
void MPU6050_leer() {
  // Ler os dados do giroscópio e acelerômetro que se encontram nos endereçeos 3B à 14
  Wire.beginTransmission(MPU6050_adress);       // Início da comunicação
  Wire.write(0x3B);                             // Solicitação do registro 0x3B (AcX)
  Wire.endTransmission();
  Wire.requestFrom(MPU6050_adress, 14);         // Solicitar um total de 14 registros
  while (Wire.available() < 14);                // Espera até receber os 14 bytes

  ax = Wire.read() << 8 | Wire.read();          // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  ay = Wire.read() << 8 | Wire.read();          // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  az = Wire.read() << 8 | Wire.read();          // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  temperature = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H)   & 0x42 (TEMP_OUT_L)
  gx = Wire.read() << 8 | Wire.read();          // 0x43 (GYRO_XOUT_H)  & 0x44 (GYRO_XOUT_L)
  gy = Wire.read() << 8 | Wire.read();          // 0x45 (GYRO_YOUT_H)  & 0x46 (GYRO_YOUT_L)
  gz = Wire.read() << 8 | Wire.read();          // 0x47 (GYRO_ZOUT_H)  & 0x48 (GYRO_ZOUT_L)

  // Resetar valores de calibração do acelerômetro
  if (accCalibOK == true) {
    ax -= acc_X_cal;
    ay -= acc_Y_cal;
    az -= acc_Z_cal;
    az = az + 4096;
  }
}

//---------------------------------------------------------------------------------

// Cálculo da velocidade angular (º/s) e o ângulo de inclinação (º)
void MPU6050_procesar() {
  // Resetar valores de calibração do acelerômetro e calcular
  // velocidade angular em º/s. Cada sinal cru de 65.5 equivale a 1º/s
  gyro_X = (gx - gyro_X_cal) / 65.5;
  gyro_Y = (gy - gyro_Y_cal) / 65.5;
  gyro_Z = (gz - gyro_Z_cal) / 65.5;

  // Calcula-se exatamente o tempo que se passou desde que se executou o cálculo do ângulo.
  // Ao se obter os sinais PWM variáveis entre 1 e 2ms, o cálculo do ângulo não se executa
  tempo_execucao_MPU6050 = (micros() - tempo_MPU6050_1) / 1000;

  // Cálculo do ângulo de inclinação com os dados do giroscópio:
  // velocidade (º/s) * tempo (s) = graus de inclinação (º)
  angulo_pitch += gyro_X * tempo_execucao_MPU6050 / 1000;
  angulo_roll  += gyro_Y * tempo_execucao_MPU6050 / 1000;
  // 0.000000266 = tempo_execucao / 1000 / 65.5 * PI / 180
  angulo_pitch += angulo_roll  * sin((gz - gyro_Z_cal) * tempo_execucao_MPU6050 * 0.000000266);
  angulo_roll  -= angulo_pitch * sin((gz - gyro_Z_cal) * tempo_execucao_MPU6050 * 0.000000266);
  tempo_MPU6050_1 = micros();

  // Cálculo vetor de aceleração
  // 57.2958 = conversão de radianos para graus 180/PI
  acc_total_vector = sqrt(pow(ay, 2) + pow(ax, 2) + pow(az, 2));
  angulo_pitch_acc = asin((float)ay / acc_total_vector) * 57.2958;
  angulo_roll_acc  = asin((float)ax / acc_total_vector) * -57.2958;

  if (set_gyro_angles) {
    // Filtro complementar
    angulo_pitch = angulo_pitch * 0.995 + angulo_pitch_acc * 0.005;   // Angulo Pitch de inclinação
    angulo_roll  = angulo_roll  * 0.995 + angulo_roll_acc  * 0.005;   // Angulo Roll de inclinação
  }
  else {
    angulo_pitch = angulo_pitch_acc;
    angulo_roll  = angulo_roll_acc;
    set_gyro_angles = true;
  }
}

//---------------------------------------------------------------------------------

void RC_procesar() {
  //  Filtro dos sinais crus lidos do controle RC
  
  RC_Throttle_filt = RC_Throttle_filt * 0.9 + RC_Throttle_raw * 0.1;
  RC_Pitch_filt    = RC_Pitch_filt * 0.9 + RC_Pitch_raw * 0.1;
  RC_Roll_filt     = RC_Roll_filt  * 0.9 + RC_Roll_raw  * 0.1;
  RC_Yaw_filt      = RC_Yaw_filt   * 0.9 + RC_Yaw_raw   * 0.1;

  // Mapeamento dos sinais do controle RC
  RC_Throttle_consigna = map(RC_Throttle_filt, us_min_Throttle_raw, us_max_Throttle_raw, us_min_Throttle_adj, us_max_Throttle_adj);
  RC_Pitch_consigna    = map(RC_Pitch_filt, us_min_Pitch_raw, us_max_Pitch_raw, us_min_Pitch_adj, us_max_Pitch_adj);
  RC_Roll_consigna     = map(RC_Roll_filt, us_min_Roll_raw, us_max_Roll_raw, us_min_Roll_adj, us_max_Roll_adj);
  RC_Yaw_consigna      = map(RC_Yaw_filt, us_min_Yaw_raw, us_max_Yaw_raw, us_min_Yaw_adj, us_max_Yaw_adj);

  // Se as leituras estão próximas de 0, força-se para o valor 0 para evitar a inclinação errônea do drone
  if (RC_Pitch_consigna < 3 && RC_Pitch_consigna > -3)RC_Pitch_consigna = 0;
  if (RC_Roll_consigna  < 3 && RC_Roll_consigna  > -3)RC_Roll_consigna  = 0;
  if (RC_Yaw_consigna   < 3 && RC_Yaw_consigna   > -3)RC_Yaw_consigna   = 0;
}

//---------------------------------------------------------------------------------

void Modulador() {
  // Se o sinal de Throttle for menor que 1300us, o controle de estabilidade se desativa. A parte integral
  // dos controladores PID são forçados para 0.
  if (RC_Throttle_consigna <= 1300) {
    PID_W_Pitch_I = 0;
    PID_W_Roll_I = 0;
    PID_W_Yaw_I  = 0;
    PID_ang_Pitch_I = 0;
    PID_ang_Roll_I = 0;

    ESC1_us = RC_Throttle_consigna;
    ESC2_us = RC_Throttle_consigna;
    ESC3_us = RC_Throttle_consigna;
    ESC4_us = RC_Throttle_consigna;

    // Se os motores giram com o stick de Throttle no mínimo, setar para o valor de 950us
    if (ESC1_us < 1000) ESC1_us = 950;
    if (ESC2_us < 1000) ESC2_us = 950;
    if (ESC3_us < 1000) ESC3_us = 950;
    if (ESC4_us < 1000) ESC4_us = 950;
  }

  // Se o sinal de throttle é maior que 1300us, o controle de estabilidade se ativa.
  else {
    // Limitar throttle a 1800 para deixar margem para os PID
    if (RC_Throttle_consigna > 1800)RC_Throttle_consigna = 1800;

    // Modulador
    ESC1_us = RC_Throttle_consigna + PID_W_Pitch_OUT - PID_W_Roll_OUT - PID_W_Yaw_OUT; // Motor 1
    ESC2_us = RC_Throttle_consigna + PID_W_Pitch_OUT + PID_W_Roll_OUT + PID_W_Yaw_OUT; // Motor 2
    ESC3_us = RC_Throttle_consigna - PID_W_Pitch_OUT + PID_W_Roll_OUT - PID_W_Yaw_OUT; // Motor 3
    ESC4_us = RC_Throttle_consigna - PID_W_Pitch_OUT - PID_W_Roll_OUT + PID_W_Yaw_OUT; // Motor 4
    
    // Evita que nenhum dos motores pare completamente em pleno voo
    if (ESC1_us < 1100) ESC1_us = 1100;
    if (ESC2_us < 1100) ESC2_us = 1100;
    if (ESC3_us < 1100) ESC3_us = 1100;
    if (ESC4_us < 1100) ESC4_us = 1100;
    // Evita mandar sinais maiores que 2000us aos motores
    if (ESC1_us > 2000) ESC1_us = 2000;
    if (ESC2_us > 2000) ESC2_us = 2000;
    if (ESC3_us > 2000) ESC3_us = 2000;
    if (ESC4_us > 2000) ESC4_us = 2000;
  }
}

//---------------------------------------------------------------------------------

void PWM() {
  // Para gerar os 4 sinais PWM, o primeiro passo é colocar esses sinais em estado alto (HIGH).
  digitalWrite(pin_motor1, HIGH);
  digitalWrite(pin_motor2, HIGH);
  digitalWrite(pin_motor3, HIGH);
  digitalWrite(pin_motor4, HIGH);
  tempo_motores_start = micros();

  // ------------------ 1ms max!! ------------------
  tempo_1 = micros();

  RC_procesar();             // Leitura do controle RC deve ser menor que 1ms
  
  // Se a duração entre tempo_1 e tempo_2 for maior que 900us, informar que o tempo foi excedido.
  tempo_2 = micros();
  tempo_ON = tempo_2 - tempo_1;
  if (tempo_ON > 900) Serial.print("Tempo de leitura do rádio excedido");
  // ------------------ ¡¡1ms max!! ------------------

  // Força-se os sinais PWM ao estado LOW quando se ocorre o tempo definido nas variáveis ESCx_us
  while (digitalRead(pin_motor1) == HIGH || digitalRead(pin_motor2) == HIGH || digitalRead(pin_motor3) == HIGH || digitalRead(pin_motor4) == HIGH) {
    if (tempo_motores_start + ESC1_us <= micros()) digitalWrite(pin_motor1, LOW);
    if (tempo_motores_start + ESC2_us <= micros()) digitalWrite(pin_motor2, LOW);
    if (tempo_motores_start + ESC3_us <= micros()) digitalWrite(pin_motor3, LOW);
    if (tempo_motores_start + ESC4_us <= micros()) digitalWrite(pin_motor4, LOW);
  }
}

//---------------------------------------------------------------------------------

// PID ângulo de inclinação
void PID_ang() {
  // PID ângulo - PITCH
  PID_ang_Pitch_error = RC_Pitch_consigna - angulo_pitch;                        // Erro entre sinal de leitura do sensor e do setpoint
  PID_ang_Pitch_P  = Pitch_ang_Kp  * PID_ang_Pitch_error;                        // Parte proporcional
  PID_ang_Pitch_I += (Pitch_ang_Ki * PID_ang_Pitch_error);                       // Parte integral (Somatório do erro no tempo)
  PID_ang_Pitch_I  = constrain(PID_ang_Pitch_I, -PID_ang_sat1, PID_ang_sat1);    // Limite da parte integral
  PID_ang_Pitch_D  = Pitch_ang_Kd * (angulo_pitch - angulo_pitch_ant);           // Parte derivativa (diferença do erro atual e do erro anterior)

  PID_ang_Pitch_OUT =  PID_ang_Pitch_P + PID_ang_Pitch_I + PID_ang_Pitch_D;      // Saída PID
  PID_ang_Pitch_OUT = constrain(PID_ang_Pitch_OUT, -PID_ang_sat2, PID_ang_sat2); // Limite da saída do PID

  // PID ângulo - ROLL
  PID_ang_Roll_error = RC_Roll_consigna - angulo_roll;                           // Erro entre sinal de leitura do sensor e do setpoint
  PID_ang_Roll_P  = Roll_ang_Kp  * PID_ang_Roll_error;                           // Parte proporcional
  PID_ang_Roll_I += (Roll_ang_Ki * PID_ang_Roll_error);                          // Parte integral (Somatório do erro no tempo)
  PID_ang_Roll_I  = constrain(PID_ang_Roll_I, -PID_ang_sat1, PID_ang_sat1);      // Limite da parte integral
  PID_ang_Roll_D  = Roll_ang_Kd * (angulo_roll - angulo_roll_ant);               // Parte derivativa (diferença do erro atual e do erro anterior)

  PID_ang_Roll_OUT = PID_ang_Roll_P + PID_ang_Roll_I + PID_ang_Roll_D;           // Saída PID
  PID_ang_Roll_OUT = constrain(PID_ang_Roll_OUT, -PID_ang_sat2, PID_ang_sat2);   // Limite da saída do PID
}

//---------------------------------------------------------------------------------

// PID velocidade angular
void PID_w() {
  // Dependendo do modo de voo selecionado, o sinal de setpoint deve ser diferente
  if (MODO_voo == 0) {
    // No modo acrobático controla-se apenas a velocidade angular de cada eixo (um PID por eixo). O setpoint do PID se dá em º/s
    // e vem direntamente do controle RC
    PID_W_Pitch_consigna = RC_Pitch_consigna;
    PID_W_Roll_consigna  = RC_Roll_consigna;
  }
  else {
    // Já no modo estável, os setpoints dos PID de velocidade vêm das saídas dos PID de controle de ângulo de inclinação
    PID_W_Pitch_consigna = PID_ang_Pitch_OUT;
    PID_W_Roll_consigna  = PID_ang_Roll_OUT;
  }

  // PID velocidade - PITCH
  PID_W_Pitch_error = PID_W_Pitch_consigna - gyro_X;                       // Erro entre sinal de leitura do sensor e do setpoint
  PID_W_Pitch_P  = Pitch_W_Kp  * PID_W_Pitch_error;                        // Parte proporcional
  PID_W_Pitch_I += (Pitch_W_Ki * PID_W_Pitch_error);                       // Parte integral (Somatório do erro no tempo)
  PID_W_Pitch_I  = constrain(PID_W_Pitch_I, -PID_W_sat1, PID_W_sat1);      // Limite da parte integral
  PID_W_Pitch_D  = Pitch_W_Kd * (gyro_X - gyro_X_ant);                     // Parte derivativa (diferença do erro atual e do erro anterior)

  PID_W_Pitch_OUT = PID_W_Pitch_P + PID_W_Pitch_I + PID_W_Pitch_D;         // Saída PID
  PID_W_Pitch_OUT = constrain(PID_W_Pitch_OUT, -PID_W_sat2, PID_W_sat2);   // Limite da saída do PID

  // PID velocidad - ROLL
  PID_W_Roll_error = PID_W_Roll_consigna - gyro_Y;                         // Erro entre sinal de leitura do sensor e do setpoint
  PID_W_Roll_P  = Roll_W_Kp  * PID_W_Roll_error;                           // Parte proporcional
  PID_W_Roll_I += (Roll_W_Ki * PID_W_Roll_error);                          // Parte integral (Somatório do erro no tempo)
  PID_W_Roll_I  = constrain(PID_W_Roll_I, -PID_W_sat1, PID_W_sat1);        // Limite da parte integral
  PID_W_Roll_D  = Roll_W_Kd * (gyro_Y - gyro_Y_ant);                       // Parte derivativa (diferença do erro atual e do erro anterior)

  PID_W_Roll_OUT = PID_W_Roll_P + PID_W_Roll_I + PID_W_Roll_D;             // Saída PID
  PID_W_Roll_OUT = constrain(PID_W_Roll_OUT, -PID_W_sat2, PID_W_sat2);     // Limite da saída do PID

  // PID velocidad - YAW
  PID_W_Yaw_error = RC_Yaw_consigna - gyro_Z;                              // Erro entre sinal de leitura do sensor e do setpoint
  PID_W_Yaw_P  = Yaw_W_Kp  * PID_W_Yaw_error;                              // Parte proporcional
  PID_W_Yaw_I += (Yaw_W_Ki * PID_W_Yaw_error);                             // Parte integral (Somatório do erro no tempo)
  PID_W_Yaw_I  = constrain(PID_W_Yaw_I, -PID_W_sat1, PID_W_sat1);          // Limite da parte integral
  PID_W_Yaw_D  = Yaw_W_Kd * (gyro_Z - gyro_Z_ant);                         // Parte derivativa (diferença do erro atual e do erro anterior)

  PID_W_Yaw_OUT = PID_W_Yaw_P + PID_W_Yaw_I + PID_W_Yaw_D;                 // Saída PID
  PID_W_Yaw_OUT = constrain(PID_W_Yaw_OUT, -PID_W_sat2, PID_W_sat2);       // Limite da saída do PID
}

//---------------------------------------------------------------------------------

// Visualizar variáveis pelo Monitor Serial
// NOTA: Ao visualizar as variáveis pelo monitor serial é possível que se ultrapasse o ciclo de tempo pre-estabelecido
// devido ao tempo extra utilizado na visualização das variáveis. Este é o motivo de se desativar essa opção em voo

void Visualizacoes() {

  // Visualizar variáveis pelo canal serial
  // Selecionar a variável para se visualizar através da opção no visu_select (0: Controle RC, 1: giro, 2: acc, 3: ang, 4: esc)
  if (visu == 1) {
    if (visu_select == 0) {
      Serial.print(RC_Pitch_consigna);
      Serial.print("\t");
      Serial.print(RC_Roll_consigna);
      Serial.print("\t");
      Serial.print(RC_Yaw_consigna);
      Serial.print("\t");
      Serial.println(RC_Throttle_consigna);
    }

    if (visu_select == 1) {
      Serial.print((gx - gyro_X_cal) / 16.4);
      Serial.print("\t");
      Serial.print((gy - gyro_Y_cal) / 16.4);
      Serial.print("\t");
      Serial.println((gz - gyro_Z_cal) / 16.4);
    }

    if (visu_select == 2) {
      Serial.print(ax / 4096);
      Serial.print("\t");
      Serial.print(ay / 4096);
      Serial.print("\t");
      Serial.println(az / 4096);
    }

    if (visu_select == 3) {
      Serial.print("Pitch: ");
      Serial.print(angulo_pitch);
      Serial.print("\t");
      Serial.print("SP Pitch: ");
      Serial.print(RC_Pitch_consigna);
      Serial.print("\t");      
      Serial.print("Roll: ");
      Serial.print(angulo_roll);
      Serial.print("\t");
      Serial.print("SP Roll: ");
      Serial.println(RC_Roll_consigna);   
    }

    if (visu_select == 4) {
      Serial.print(ESC1_us);
      Serial.print("\t");
      Serial.print(ESC2_us);
      Serial.print("\t");
      Serial.print(ESC3_us);
      Serial.print("\t");
      Serial.println(ESC4_us);

      Pitch_W_Ki = 0;
      Roll_W_Ki  = 0;
      Yaw_W_Ki   = 0;
      Pitch_ang_Ki = 0;
      Roll_ang_Ki  = 0;
    }
    
    if (visu_select == 999) {
      Serial.println(tempo_execucao_MPU6050);
    }
  }
}

/// FIM DO CÓDIGO
//---------------------------------------------------------------------------------
