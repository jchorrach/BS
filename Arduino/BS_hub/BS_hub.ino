// BS_HUB
//
// Implementa las funcionalidades del hub de comunicaciones
// del proyecto BS
//
// 
// Autor: Juan Carlos Horrach Campins (jchorrach@gmail.com)
// Copyright (C) 2016  Juan Carlos Horrach Campins


#include <VirtualWire.h>
#include <SoftwareSerial.h>

const int led_pin = 13;
const int transmit_pin = 27;
const int receive_pin = 9;
const int transmit_en_pin = 29;
#define MAX_SENSORES 5
#define GSM_RX 2
#define GSM_TX 3


// Comunicación con módulo SIM900
SoftwareSerial gsm(GSM_RX, GSM_TX); // RX, TX
//


// Estructura  de datos de las sondas de un 
// módulo sensor
struct Sensor_STRUCT// Data Structure 
{
int    id;  // Identificador único del módulo sensor
long   batt;// mV de la bateria del sensor 
int    sen1;// Valor de la sonda #1
int    sen2;// Valor de la sonda #2
float  sen3;// Valor de la sonda #3 (Analógico)
};

Sensor_STRUCT sData = {0,0,0,0,0}; 

// Array con últimos valores recibidos de los sensores
Sensor_STRUCT sensores[MAX_SENSORES]; 

/****************************************
 * Setup Micro
 * 
 ****************************************/
void setup()
{
  Serial.begin(9600);	// Debugging
  while (!Serial) {
    ; // Espera a que se conecte. Se necesita sólo para USB nativo
  }
  
  Serial.println("setup"); // Debugging

  gsm.begin(19200); // Comunicación con móduloGSM SIM900

  // Inicializa el IO y el ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  vw_set_ptt_pin(transmit_en_pin);
  vw_set_ptt_inverted(true); // Requeriso para DR3100
  vw_setup(300);	 // Bits por sec

  vw_rx_start();       // Inicia PLL para recibir datos

  pinMode(led_pin, OUTPUT); // Debugging
  init_sensorDat;

  Serial.print("Inicializado!!\n\r"); // Debugging
  Serial.print(sensorJson()); // Debugging
  Serial.print("\n\r"); // Debugging
}

/****************************************
 *   Rutinas de manejo de datos de
 * los sensores
 * 
 ****************************************/
 
//   Inicializa array de ultimos valores recibidos
// de los sensores.
void init_sensorDat()
{
   for (int i=0;i<MAX_SENSORES;i++)
   {
    sensores[i] = (Sensor_STRUCT) {0,0,0,0,0};
   }
}

//   Inserta/Actualiza valores de un sensor. 
void merge_sensorDat(struct Sensor_STRUCT *sD)
{
    int i = 0;
    while (i<MAX_SENSORES && sensores[i].id!=0 && sensores[i].id!=sD->id)
    {
        i++;    
    }
    sensores[i].id  =sD->id;
    sensores[i].batt=sD->batt;
    sensores[i].sen1=sD->sen1;
    sensores[i].sen2=sD->sen2;
    sensores[i].sen3=sD->sen3;
}

// Crea JSON con los valores de los sensores
String sensorJson()
{
  String info;

  info += "\n\r{\"sensores\":[";
  
  for (int i=0;i<MAX_SENSORES;i++)
  {
    info.concat("{");
    info.concat("\"id\":");
    info.concat(String(sensores[i].id,DEC));
    info.concat(",\"batt\":");
    info.concat(String(sensores[i].batt,DEC));
    info.concat(",\"s1\":");
    info.concat(String(sensores[i].sen1,DEC));
    info.concat(",\"s2\":");
    info.concat(String(sensores[i].sen2,DEC));
    info.concat(",\"s3\":");
    info.concat(String(sensores[i].sen3,DEC));
    info.concat("},");  
  }
  info = info.substring(0,info.length()-1)+"]}";
  return info;
}

/****************************************
 *   Rutinas de manejo del módulo
 * GSM SIM900
 * 
 ****************************************/

// Gestiona envío comando AT al módulo
String gsm_cmd(String cmd, String fin = "\r\nOK\r\n", long timeout = 3000)
{
  
  long timeinit=millis();
  String res;
  char car;
  if (cmd=="^Z")
  {
    gsm.println((char) 26);
    Serial.println("Se ha enviado un ctrl+z"); // Debugging
  }
  else
    gsm.println(cmd);
  while (millis()-timeinit<timeout && !res.endsWith(fin) && !res.endsWith("\r\nERROR\r\n"))
  {
    if (gsm.available()) {
      car = gsm.read();
      int asc = car;
      Serial.print(asc, DEC);
      res += car;
    }
  }
  
  Serial.println(res); // Debugging
  
  return res;
}

void gsm_setup()
{
  String res;
  res = gsm_cmd("AT+CPIN?");
  Serial.println("---------"); // Debugging
  Serial.println(res);  
  Serial.println("---------"); // Debugging
  if (res.indexOf("+CPIN: READY")==-1)
  {
     res = gsm_cmd("AT+CPIN=1024");
  }
  //delay(1000);
  /*
  Serial.println(gsm_cmd("AT+CGATT=1"));
  Serial.println(gsm_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\""));
  //Serial.println(gsm_cmd("AT+SAPBR=3,1,\"APN\",\"telefonica.es\""));
  Serial.println(gsm_cmd("AT+SAPBR=1,1"));
  Serial.println(gsm_cmd("AT+SAPBR=2,1"));
  Serial.println(gsm_cmd("AT+HTTPINIT"));
  Serial.println(gsm_cmd("AT+HTTPPARA=\"CID\",1"));
  Serial.println(gsm_cmd("AT+HTTPPARA=\"URL\",\"http://www.google.com\""));
  Serial.println(gsm_cmd("AT+HTTPACTION=0","NULL"));
  Serial.println(gsm_cmd("AT+HTTPREAD"));
  Serial.println(gsm_cmd("AT+HTTPTERM"));
  Serial.println(gsm_cmd("AT+SAPBR=0,1"));
  */
  Serial.println(gsm_cmd("AT+CMGF=1"));
  
  Serial.println(gsm_cmd("AT+CMGS = \"+*********\"","> "));  
  res = "Test envio SMS";
  Serial.println(gsm_cmd(res,"> ",500));
  Serial.println(gsm_cmd("^Z","> "));
  //gsm.print((char)26);
  //gsm.println();
  gsm.flush();
  Serial.println("\r\nMensaje enviado\r\n");
  Serial.println(gsm_cmd("AT"));
}

void gsm_HTPP()
{
  String res;
  res = gsm_cmd("AT+CGATT=1");
  res = gsm_cmd("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
  //res = gsm_cmd("AT+SAPBR=3,1,\"APN\",\"telefonica.es\"");
  res = gsm_cmd("AT+SAPBR=1,1");
  res = gsm_cmd("AT+SAPBR=2,1");
  res = gsm_cmd("AT+HTTPINIT");
  res = gsm_cmd("AT+HTTPPARA=\"CID\",1");
  res = gsm_cmd("AT+HTTPPARA=\"URL\",\"http://www.google.com\"");
  res = gsm_cmd("AT+HTTPACTION=0","NULL");
  res = gsm_cmd("AT+HTTPREAD");
  res = gsm_cmd("AT+HTTPTERM");
  res = gsm_cmd("AT+SAPBR=0,1");
}

void gsm_SMS(String text)
{
  String res;
  res = gsm_cmd("AT+CMGF=1");
  
  res = gsm_cmd("AT+CMGS = \"+34639635751\"","> ");  
  
  res = gsm_cmd(text,"> ",1000);
  res = gsm_cmd("^Z","> ", 1000);
  gsm.flush();
  Serial.println("\r\nMensaje enviado\r\n"); // Debugging

}

void process_msg(struct Sensor_STRUCT *sD) // Debugging
{      
      Serial.print(F("\n\r===========================\n\r"));
      Serial.print(F("id : "));
      Serial.print( sD->id);
      Serial.print("\n\r");
      Serial.print(F("batt : "));
      Serial.print( sD->batt);
      Serial.print("\n\r");
      Serial.print(F("s1 : "));
      Serial.print( sD->sen1);
      Serial.print("\n\r");
      Serial.print(F("s2 : "));
      Serial.print( sD->sen2);
      Serial.print("\n\r");
      Serial.print(F("s3 : "));
      Serial.print( sD->sen3);
      Serial.print("\n\r");
      Serial.print(F("===========================\n\r"));
      merge_sensorDat(sD);
      Serial.print(F("\n\r"));
      Serial.print(F("+++++++++++++++++++++++++++++\n\r"));
      Serial.print(sensorJson());
      Serial.print(F("\n\r"));
}


void loop()
{

  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    if (buflen == sizeof(Sensor_STRUCT)) // Tamaño correcto de trama
    {
      memcpy( &sData, buf, sizeof(sData)); // Copiar datos en estructura sesnsor
      Serial.print(F("\n\rMensaje recibido!!\n\r")); // Debugging
      process_msg(&sData); 
gsm_setup();
    }
    else
    {
      Serial.print(F("The wrong amount of bytes was received !\n\r")); // Debugging
    }
  }

  if (gsm.available()) {
    Serial.write(gsm.read()); // Debugging
  }

  if (Serial.available()) { // Debugging
    gsm.write(Serial.read()); // Debugging
  }
  
}
