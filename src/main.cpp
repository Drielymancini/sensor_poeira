/* ==============================================================================

  Projeto Sensor de Poeira

  ESP-WROOM-32
  Board: DevKitV1
  Compilador: Arduino IDE 1.8.4

  Autor: Driely Mancini
  Data: Setembro de 2022

================================================================================== */  

// ==================================================================================
// --- Bibliotecas ---
#include <Arduino.h>
#include <nvs_flash.h>
#include <string>
#include <stdio.h>
#include <SPI.h>
#include <lmic.h>
#include <hal/hal.h>
// ==================================================================================

// *************************** Variaveis para memoria flash *****************************//
/* Definição - baudrate da serial de debug */
//#define BAUDRATE_SERIAL_DEBUG    115200
 
/* Chave atribuida ao valor a ser escrito e lido
   da partição NVS */
#define CHAVE_NVS  "dado_leitura"
/* Protótipos */
void grava_dado_nvs(float dado);
float le_dado_nvs(void);
//===================================================================================================


// *************************** CRIAÇÃO DOS HANDLES DAS TAREFAS 1, 2 , 3 E 4 *****************************//

TaskHandle_t TaskGerenciaLeitura;
//===================================================================================================

// *************************** FUNÇÕES DE TASK ***********************************************//
void gerenciaLeitura(void * pvParameters);

// ==================================================================================
// --- Mapeamento de Hardware ---
#define sensor_VA 12
#define sensor_VB 27
#define servo 33

// --- Variáveis Globais ---
int     timer_val=15;                               //valor de contagem do timer
bool    timer_running=1,                              //flag para indicar quando start foi pressionado
        primeira_leitura = 0;
int sujo = 0;
uint8_t val=0;
bool    ctrl=0;
static uint8_t mydata[8];
static osjob_t sendjob;
void do_send(osjob_t* j);
int flag_start = 1;
float REFERENCIA = 0.05;
float REFERENCIA2 = 0.05;

const unsigned TX_INTERVAL = 10;


// ==================================================================================
// --- Configuração LoraWan ---

static const u1_t PROGMEM APPEUI[8]= {0x49, 0x87, 0x59, 0x54, 0x56, 0x84, 0x79, 0x88 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]= {0x59, 0x74, 0x05, 0xD0, 0x7E, 0xD5, 0xB3, 0x70};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from the TTN console can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xC5, 0xAA, 0xC9, 0x77, 0x2F, 0xD8, 0x73, 0x56, 0x36, 0xBB, 0x04, 0x5E, 0xCC, 0x05, 0x10, 0xA3};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}
// ==================================================================================
// --- Mapeamento de funções ---
/* Função: grava na NVS um dado do tipo interio 32-bits
 *         sem sinal, na chave definida em CHAVE_NVS
 * Parâmetros: dado a ser gravado
 * Retorno: nenhum
 */
void grava_dado_nvs(float dado)
{
    nvs_handle handler_particao_nvs;
    esp_err_t err;

    char dado_8b[8];
    //gcvt(dado, 6, dado_8b)
    dtostrf(dado, 6, 2, dado_8b);
    //char dado_convertido = dado_8b[5];


    err = nvs_flash_init_partition("nvs");
     
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao iniciar partição NVS.");           
        return;
    }
 
    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura"); 
        return;
    }
 
    /* Atualiza valor do horimetro total */
    err = nvs_set_str(handler_particao_nvs, CHAVE_NVS, dado_8b);
    Serial.println("GRAVADO : ");  
    Serial.println("\n");
    Serial.println(dado_8b);  
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Erro ao gravar horimetro");                   
        nvs_close(handler_particao_nvs);
        return;
    }
    else
    {
        Serial.println("Dado gravado com sucesso!");     
        nvs_commit(handler_particao_nvs);    
        nvs_close(handler_particao_nvs);      
    }
}
 
/* Função: le da NVS um dado do tipo interio 32-bits
 *         sem sinal, contido na chave definida em CHAVE_NVS
 * Parâmetros: nenhum
 * Retorno: dado lido
 */
float le_dado_nvs(void)
{
    nvs_handle handler_particao_nvs;
    esp_err_t err;
    char dado_lido;
    size_t required_size;

    err = nvs_flash_init_partition("nvs");
     
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao iniciar partição NVS.");         
        return 0;
    }
 
    err = nvs_open_from_partition("nvs", "ns_nvs", NVS_READWRITE, &handler_particao_nvs);
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao abrir NVS como escrita/leitura");         
        return 0;
    }
 
    /* Faz a leitura do dado associado a chave definida em CHAVE_NVS */
    err = nvs_get_str(handler_particao_nvs, CHAVE_NVS, &dado_lido, &required_size);
     
    if (err != ESP_OK)
    {
        Serial.println("[ERRO] Falha ao fazer leitura do dado");         
        return 0;
    }
    else
    {
        Serial.println("Dado lido com sucesso!");  
        nvs_close(handler_particao_nvs);  
        float dado_convertido = atof(&dado_lido); 
        Serial.println("LIDO : ");  
        Serial.println(dado_convertido);  
        return dado_convertido;
    }
} //end le_dado_nvs

float medirLDR () {

  static float volts_f = 0.0;
  volts_f = fabs(analogRead(sensor_VA) - analogRead(sensor_VB))*(3.3/4095); //calculada a tensao a partir da média
    
    static float volts_ref = 0.0;
    volts_ref = ((analogRead(sensor_VA) * 1.00190363341459) - analogRead(sensor_VB))*(3.3/4095); //calculada a tensao a partir da média

  Serial.println("volts_ref");
  Serial.println(volts_ref);
 // Serial.println("ldr2");
 // Serial.println(ldr2);
  //return (analogRead(sensor_VB)-analogRead(sensor_VA));
  return volts_f;
}

void realizaLimpeza () {
  ledcWrite(9,4095);
  delay(2000);
  ledcWrite(9,1966);
  delay(2000);
 
  Serial.println("realizaLimpeza");
  Serial.println("\n");

};

void envia_Dados (uint32_t leitura1, uint32_t leitura2, int estado) {
  Serial.println("Leitura1");
  Serial.println(leitura1);
  Serial.println("leitura2");
  Serial.println(leitura2);


  mydata[0] = (leitura1 >> 16) & 0xFF;
  mydata[1] = (leitura1 >> 8) & 0xFF;
  mydata[2] = leitura1 & 0xFF;

  mydata[3] = (leitura2 >> 16) & 0xFF;
  mydata[4] = (leitura2 >> 8) & 0xFF;
  mydata[5] = leitura2 & 0xFF;

  
 // mydata[6] = (estado >> 8) & 0xFF;
  uint16_t auxestado = estado;
  mydata[6] = (auxestado >> 8) & 0xFF;
  mydata[7] = auxestado & 0xFF;

  Serial.println("envia_Dados");
  Serial.println("\n");
  int idteste = ((mydata[6] << 8)) + mydata[7];  
  Serial.println("decode estado");
  Serial.println(idteste);

};
void compara (float val1, float val2) {
  Serial.println("COMPARA");
  Serial.println(val1);
  Serial.println(val2);
  Serial.println(REFERENCIA);
  Serial.println(REFERENCIA2);

  if (val1 < val2 && (val2 > REFERENCIA && val2 > REFERENCIA2) )
  {
    sujo = 1;
  }else
  {
    sujo = 0;
  }
};

const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {2, 15,4},
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
      // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}


// *************************** CONFIGURAÇÕES DO PROGRAMA *****************************//
void setup() {
  Serial.begin(9600);                   // Inicia a comunicação serial
  Serial.begin(115200);  
  
  pinMode(servo, OUTPUT);
  mydata[0] = 0x00;

  ledcAttachPin(servo,9);
  ledcSetup(9,50,16);
  ledcWrite(9,4915);

   #ifdef VCC_ENABLES
   // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    LMIC_setLinkCheckMode(0);
    LMIC_setDrTxpow(DR_SF7,14);
    LMIC_selectSubBand(1);

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);

  // Cria uma tarefa que será executada na função gerenciaLeitura(), com  prioridade 1 e execução no núcleo 0
  xTaskCreatePinnedToCore(
    gerenciaLeitura,     /* Função da tarefa */
    "TaskGerenciaLeitura",   /* nome da tarefa */
    10000,               /* Tamanho (bytes) */
    NULL,               /* parâmetro da tarefa */
    1,                 /* prioridade da tarefa */
    &TaskGerenciaLeitura,        /* observa a tarefa criada */
    0);            /* tarefa alocada ao núcleo 0 */

  delay(500);

}//end setup

// *************************** DEFINIÇÃO DA TAREFA gerenciaLeitura  *****************************//
void gerenciaLeitura( void * pvParameters ) {
  for (;;) {                                                  // Cria um loop infinito, para a tarefa sempre ser executada quando estiver disponível.
    timer_val--;                           
  delay(1000);                          

  if(timer_val <= 0)                         
  {                                       
    timer_val=0;    

    if(primeira_leitura == 0){
    if(flag_start){
      REFERENCIA = medirLDR();
    }
      float leitura1 = medirLDR();
      grava_dado_nvs(leitura1);
      primeira_leitura = 1;
      timer_val = 15;
      Serial.println("PRIMEIRA LEITURA");
      Serial.println("\n");
    } else {
      realizaLimpeza();
      
      float leitura2 = medirLDR(); 
         if(flag_start){
      REFERENCIA2 = leitura2;
      flag_start = 0;
    }

      Serial.println(leitura2);
      Serial.println("SEGUNDA LEITURA");
      Serial.println("\n");
      float valorArmazenado = le_dado_nvs();

      compara(valorArmazenado, leitura2);
      // buscar leitura armazenada e comparar com leitura2
      //comparar leituras sujo = compara(val1, val2)
       
      if(sujo ==1){
        Serial.println("SUJO");
        Serial.println("\n");
        //reset de dados
        nvs_flash_erase_partition("nvs");
      } else {
        Serial.println("LIMPO");
        Serial.println("\n");
        
        //reset de dados
        nvs_flash_erase_partition("nvs"); 
      }
      // envia_Dados(valorArmazenado, leitura2, sujo);
        Serial.println("antes Leitura1");
       Serial.println(valorArmazenado);
      Serial.println("antes leitura2");
      Serial.println(leitura2);
       Serial.println("estado");
  Serial.println(sujo);
      envia_Dados(100.0*valorArmazenado, 100.0*leitura2, sujo);
      timer_val = 15; //somente setar novo tempo quando nescessario aguardar 15 min
      primeira_leitura = 0;
      sujo = 0;

    }
  } //end if !timer_val
    vTaskDelay(1000);
  }
}
// *************************** DEFINIÇÃO LOOP *****************************//
void loop() {
    os_runloop_once();
   
}//end loop