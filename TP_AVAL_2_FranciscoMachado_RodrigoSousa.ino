
/*

Nome ALUNO A- Francisco Machado
Nome ALUNO B- Rodrigo Sousa

IPLEIRIA - Instituto Politécnico de Leiria
ESTG - Escola Superior de Tecnologia e Gestão
EAU - Licenciatura em Engenharia Automóvel
SEEV - Sistemas Elétricos e Eletrónicos de Veículos

TP1: Pretende-se  neste  trabalho  prático  a  implementação  de um  algoritmo  que  permita  exemplificar um sistema de controlo de temperatura numa caravana que faz atuar a abertura e fecho das janelas e ar condicionados, utilizando um sistema operativo de tempo real FreeRTOS. Adicionalmente, existe um sistema de alarme.

LINK: https://youtu.be/4aqeeXKM7_I

*/

	//Declaração das bibliotecas
#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <DHT11.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

	//Declaração de informação fixa
#define MIN_ANG 50			//Angulo para o servo motor para a janela fechada
#define MAX_ANG 90			//Angulo para o servo motor para a janela fechada
#define STEP_ANG 1			//Compasso de transição de angulo para controlo de movimento do servo motot

#define MIN_TEMP 20			//Temperatura minima para acionamento do sistema
#define MAX_TEMP 21			//Temperatura maxima para acionamento do sistema

	//Declaração das Portas I/O
		//Pinos de RFID
#define SS_PIN 			5
#define RST_PIN 		4

/*
SDA 5
SCK 18
MOSI 23
MISO 19
RST 4
*/

#define PORTA_PIN 		12			//Acionamento da Porta da caravana
#define SERVO_PIN 		21			//Pino de sinal do servo
#define LED_PIN 		2			//LEDs de sinal (Alarme e trinco)
#define LED_SOF 		33			//LED da sofagem
#define LED_AC 			3			//LED do Ar Condicionado

#define AUT_MAN 		27			//Botão para modo automático
#define BUT_UP 			22			//Botão para fechar a janela
#define BUT_DOWN 		35			//Botão para abrir a janela

		//Pinos do Display
#define TFT_CS         15			//case select
#define TFT_RST        26			//reset
#define TFT_DC         32			//AO
#define TFT_MOSI       13			//Data = SDA
#define TFT_SCLK       14			//Clock = SCK

DHT11 dht11(25);			//Declaração do pino de sinal do sensor de temperatura e humidade
Servo myServo;
MFRC522 rfid(SS_PIN, RST_PIN);
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

		//Declaração das tarefas
static void IRAM_ATTR vInterruptPORTA_ABERTA( void );		//Interrupção dedicada para acionamneto do alame caso a caravana esteja trancada
static void IRAM_ATTR vInterruptAUT_MAN( void );			//Interrupção dedicada ao acionamento do modo automático
void SensorTemp(void *pvParameters); 						//Tarefa de controlo de temperatura e humidade
void ServoControl(void *pvParameters);						//Tarefa de controlo do acionamento do sistema de abertura e fecho da janela
void DisplayLCD(void *pvParameters);
void Manual(void *pvParameters);							//Tarefa que demonstra as infoemações no display
static void vAlarme( void *pvParameters );					//Tarefa de acionamento e quebra do sistema de alarme
static void vIdleRFID( void *pvParameters );				//Tarefa para leitura do cartão que tranca ou destranaca a caravan

		//Declaração dos Semaforos e Queues
SemaphoreHandle_t xBinarySemaphore_RFID, xBinarySemaphore_PORTA_A, xBinarySemaphore_AUT_MAN;
QueueHandle_t xQueue_TEMP, xQueue_HUM, xQueue_Janela, xQueue_AC, xQueue_Sofagem, xQueue_Trinco, xQueue_Alarme, xQueue_AUT_MAN;

void setup() {

    vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1);
    Serial.begin(115200);
    myServo.attach(SERVO_PIN);
    SPI.begin();
    rfid.PCD_Init();

    	//Iniciação dos semaforos
    vSemaphoreCreateBinary( xBinarySemaphore_RFID );
    vSemaphoreCreateBinary( xBinarySemaphore_PORTA_A );
    vSemaphoreCreateBinary( xBinarySemaphore_AUT_MAN );

    	//Iniciação das queues
    xQueue_TEMP = xQueueCreate(1, sizeof(int));
    xQueue_HUM = xQueueCreate(1, sizeof(int));
    xQueue_Janela = xQueueCreate(1, sizeof(int));
    xQueue_Trinco = xQueueCreate(1, sizeof(bool));
    xQueue_Alarme = xQueueCreate(1, sizeof(bool));
    xQueue_Sofagem = xQueueCreate(1, sizeof(bool));
    xQueue_AC = xQueueCreate(1, sizeof(bool));
    xQueue_AUT_MAN = xQueueCreate(1, sizeof(bool));

    	//Declaração de entradas e saidas
    pinMode(LED_AC, OUTPUT);
    pinMode(LED_SOF, OUTPUT);
    pinMode(LED_PIN, OUTPUT);
    pinMode(PORTA_PIN, INPUT_PULLUP);
    pinMode(AUT_MAN, INPUT_PULLUP);
    pinMode(BUT_UP, INPUT_PULLDOWN);
    pinMode(BUT_DOWN, INPUT_PULLDOWN);
    	//Iniciaão das interrupções
    attachInterrupt(digitalPinToInterrupt(PORTA_PIN), &vInterruptPORTA_ABERTA, RISING);
    attachInterrupt(digitalPinToInterrupt(AUT_MAN), &vInterruptAUT_MAN, FALLING);

    	//Criação das tarefas
    xTaskCreatePinnedToCore(SensorTemp, "DHT11", 1028, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(ServoControl, "ServoMotor", 4096, NULL, 4, NULL, 1);
    xTaskCreatePinnedToCore(vIdleRFID, "Handler", 1024, NULL, 1, NULL, 1);
    xTaskCreatePinnedToCore(vAlarme, "Alarme Caravana", 1024, NULL, 5, NULL, 1);
	xTaskCreatePinnedToCore(DisplayLCD, "ST7735", 4096, NULL, 2, NULL, 1);
	xTaskCreatePinnedToCore(Manual, "Auto/Manual", 2048, NULL, 4, NULL, 0);


}

void DisplayLCD(void *pvParameters) {
    int Temperatura = 0;
    int Humidade = 0;
    int sofagem = 0;
    bool alarme = false;
    bool Trico = false;
    bool Janela = false;
    int AC = 0;
    			//Inicialização das queues a 0 para evitar bugs no display
   // xQueueOverwrite(xQueue_TEMP, &Temperatura);
    xQueueOverwrite(xQueue_Sofagem, &sofagem);
	xQueueOverwrite(xQueue_AC, &AC);
	xQueueOverwrite(xQueue_Trinco, &Trico);
    xQueueOverwrite(xQueue_Alarme, &alarme);
    xQueueOverwrite(xQueue_Janela, &Janela);

    tft.initR(INITR_BLACKTAB);  					// Inicializa o display com o perfil correto
    tft.setRotation(1);  							// Define a rotação da tela (modo horizontal)
    tft.fillScreen(ST77XX_BLACK);

    // Mostra o título
    tft.setTextColor(ST77XX_WHITE, ST77XX_BLACK); 	// Cor do texto: branco, fundo: preto
    tft.setCursor(10, 20); 							// Define a posição do texto
    tft.setTextSize(2);
    tft.print("Caravana");
    tft.setCursor(10, 40);
    tft.print("Inteligente");

    // Mostra uma mensagem de introdução
    tft.setCursor(10, 70);
    tft.setTextSize(1);
    tft.print("Trabalho de SEEV");

    for (int x = -30; x < 160; x++) {
            // Apaga a caravana anterior
            tft.fillRect(x-1, 100, 35, 20, ST77XX_BLACK); // apaga o corpo retangular

            tft.fillRect(x, 100, 30, 15, ST77XX_GREEN);  // Corpo retangular

                // Desenha as rodas da caravana
            tft.fillCircle(x + 5, 115, 3, ST77XX_WHITE); // Roda de trás
            tft.fillCircle(x + 25, 115, 3, ST77XX_WHITE); // Roda da frente
            vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    tft.fillScreen(ST77XX_BLACK);
    tft.setRotation(2);

    for (;;) {


    	while(alarme){

    		xQueuePeek(xQueue_Alarme, &alarme, portMAX_DELAY);
    		tft.fillScreen(ST77XX_BLACK);
    		tft.setTextSize(3);
    		tft.setCursor(10, 70);
    		tft.setTextColor(ST77XX_WHITE);
    		tft.print("ALARME");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            tft.setCursor(10, 70);
    		tft.setTextColor(ST77XX_RED);
    		tft.print("ALARME");
            vTaskDelay(500 / portTICK_PERIOD_MS);
            tft.setCursor(10, 70);
            tft.setTextColor(ST77XX_BLACK);
            tft.print("ALARME");
            xQueuePeek(xQueue_Alarme, &alarme, portMAX_DELAY);
    	}

    	xQueuePeek(xQueue_Alarme, &alarme, portMAX_DELAY);

    	  // Cabeçalho
    	  tft.setTextColor(ST77XX_WHITE);
    	  tft.setTextSize(1);
    	  tft.setCursor(15, 10);
    	  tft.print("STATUS DO SISTEMA");
    	  tft.drawLine(0, 25, 160, 25, ST77XX_WHITE);


    	  // Seção de Temperatura e Umidade
    	  tft.setTextSize(1);
    	  tft.setCursor(10, 32);
    	  tft.print("Temp:");
    	  tft.setCursor(10, 47);
    	  tft.print("Humidade:");
    	  tft.drawLine(0, 60, 160, 60, ST77XX_WHITE);


    	  // Estados (Sistemas, Porta e Carro)
    	  tft.setCursor(10, 67);
    	  tft.print("Sofagem:");
    	  tft.setCursor(10, 84);
    	  tft.print("Ar Cond:");
    	  tft.drawLine(0, 98, 160, 98, ST77XX_WHITE);
    	  tft.setCursor(10, 103);
    	  tft.print("Porta:");
    	  tft.setCursor(10, 120);
    	  tft.print("Trinco:");
    	  tft.setCursor(10, 137);
    	  tft.print("Janela:");

    	  	  	  	  	  //Leitura do estado das variaveis
			bool Estado_porta = digitalRead(PORTA_PIN);

			xQueuePeek(xQueue_HUM, &Humidade, portMAX_DELAY);
			xQueuePeek(xQueue_Sofagem, &sofagem, portMAX_DELAY);
			xQueuePeek(xQueue_AC, &AC, portMAX_DELAY);
			xQueuePeek(xQueue_Trinco, &Trico, portMAX_DELAY);
			xQueuePeek(xQueue_Janela, &Janela, portMAX_DELAY);
			xQueuePeek(xQueue_TEMP, &Temperatura, portMAX_DELAY);

						//Apresentação dos dados relevantes
			  tft.fillRect(80, 32, 50, 10, ST77XX_BLACK);
			  tft.setCursor(80, 32);
			  tft.setTextColor(ST77XX_CYAN);
			  tft.print(Temperatura, 1);
			  tft.print(" C");

			  tft.fillRect(80, 47, 50, 10, ST77XX_BLACK);
			  tft.setCursor(80, 47);
			  tft.setTextColor(ST77XX_CYAN);
			  tft.print(Humidade, 1);
			  tft.print(" %");


			  tft.fillRect(70, 67, 50, 10, ST77XX_BLACK);
			  tft.setCursor(70, 67);
			  tft.setTextColor(sofagem ? ST77XX_GREEN : ST77XX_RED);
			  tft.print(sofagem ? "ON" : "OFF");

			  tft.fillRect(70, 84, 50, 10, ST77XX_BLACK);
			  tft.setCursor(70, 84);
			  tft.setTextColor(AC ? ST77XX_GREEN : ST77XX_RED);
			  tft.print(AC ? "ON" : "OFF");

			  tft.fillRect(60, 103, 80, 10, ST77XX_BLACK);
			  tft.setCursor(60, 103);
			  tft.setTextColor(Estado_porta ? ST77XX_GREEN : ST77XX_RED);
			  tft.print(Estado_porta ? "Aberta" : "Fechada");

			  tft.fillRect(60, 120, 80, 10, ST77XX_BLACK);
			  tft.setCursor(60, 120);
			  tft.setTextColor(Trico ? ST77XX_RED : ST77XX_GREEN);
			  tft.print(Trico ? "Trancado" : "Destrancado");

			  tft.fillRect(60, 137, 80, 10, ST77XX_BLACK);
			  tft.setCursor(60, 137);
			  tft.setTextColor(Janela ? ST77XX_GREEN : ST77XX_RED);
			  tft.print(Janela ? "Aberta" : "Fechada");


			  vTaskDelay(1000 / portTICK_PERIOD_MS);
			}

    }

static void vIdleRFID( void *pvParameters )
{

	  	bool Alarme = false;
	    bool Trico = false;

  for( ;; )
  {

    if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {			//Se deteta um cartão e lê o mesmo...

    	 Serial.print("Cartao:");
    	  String content= "";
    	  byte letter;
    	  for (byte i = 0; i < rfid.uid.size; i++)
    	  {
    	     Serial.print(rfid.uid.uidByte[i] < 0x10 ? " 0" : " ");				//Impressão do código cartão no Serial Monitor
    	     Serial.print(rfid.uid.uidByte[i], HEX);
    	     content.concat(String(rfid.uid.uidByte[i] < 0x10 ? " 0" : " "));	//Escreve na String "content" o código do cartão
    	     content.concat(String(rfid.uid.uidByte[i], HEX));
    	  }
    	  Serial.println();
    	  Serial.print("Message : ");
    	  content.toUpperCase();
    	  if (content.substring(1) == "30 98 66 3D") 							//Verifica a se é o cartão correto
    	  {


    		  xQueuePeek(xQueue_Alarme, &Alarme, 0 );							//Lê o estado da variavel Alarme
			  vTaskDelay(100 / portTICK_PERIOD_MS);
    		  if(Alarme){														//Verifica se o alarme está ativado

    			  Serial.println("Alarme desativado");
    			  Alarme = false;
    			  Trico = false;
        		  xQueueOverwrite(xQueue_Alarme, &Alarme);						//Atualiza o valor de alarme (desligado)
        		  xQueueOverwrite(xQueue_Trinco, &Trico);						//Atualiza o valor do trico (destrancada)
    			  digitalWrite(LED_PIN, HIGH);									//Sequencia luminosa para desativação do alarme
    			  vTaskDelay(200 / portTICK_PERIOD_MS);
    			  digitalWrite(LED_PIN, LOW);
    			  vTaskDelay(200 / portTICK_PERIOD_MS);
    			  digitalWrite(LED_PIN, HIGH);
    			  vTaskDelay(200 / portTICK_PERIOD_MS);
    			  digitalWrite(LED_PIN, LOW);

    		  }else{

    			  bool Estado_porta = digitalRead(PORTA_PIN);					//Verifica se a porta está aberta

    			  Serial.println("Estado_porta = ");
    			  Serial.print(Estado_porta);									//Diz em Serial Monitor se a porta está aberta ou fechada

				  if(Trico){													//Verifica se esta trancada

					  Trico = false;
					  xQueueOverwrite(xQueue_Trinco, &Trico);					//Destranca a caravana
					  Serial.println("*********Caravana Destrancada*********");
					  digitalWrite(LED_PIN, HIGH);								//Sequencia de luzes para quando a caravana é destrancada
					  vTaskDelay(500 / portTICK_PERIOD_MS);
					  digitalWrite(LED_PIN, LOW);
					  vTaskDelay(500 / portTICK_PERIOD_MS);
					  digitalWrite(LED_PIN, HIGH);
					  vTaskDelay(500 / portTICK_PERIOD_MS);
					  digitalWrite(LED_PIN, LOW);

				  }else{
					  if(!Estado_porta){										//Verifica se a porta esta fechada

						  Trico = true;
						  xQueueOverwrite(xQueue_Trinco, &Trico);				//Tranca a caravana
						  Serial.println("*********Caravana Trancada*********");
						  digitalWrite(LED_PIN, HIGH);							//Sequencia de luzes para quando a caravana é Trancada
						  vTaskDelay(3000 / portTICK_PERIOD_MS);
						  digitalWrite(LED_PIN, LOW);

					  }if(Estado_porta){										//Verifica se a porta está aberta
						  Serial.println("***********Nao trancou por Porta aberta***********");

							  digitalWrite(LED_PIN, HIGH);						//Sequencia de luzes quando a tentativa de trancar
							  vTaskDelay(100 / portTICK_PERIOD_MS);				//é falhada pela porta estar aberta
							  digitalWrite(LED_PIN, LOW);
							  vTaskDelay(100 / portTICK_PERIOD_MS);
							  digitalWrite(LED_PIN, HIGH);
							  vTaskDelay(100 / portTICK_PERIOD_MS);
							  digitalWrite(LED_PIN, LOW);
							  vTaskDelay(100 / portTICK_PERIOD_MS);
					  }
				  }

				  vTaskDelay(500 / portTICK_PERIOD_MS);
    		  }

    	 }else{																	//Caso encontre uma chave desconhecida não reage
    	    Serial.println("Chave Desconhecida");
			vTaskDelay(2000 / portTICK_PERIOD_MS);
    	 }
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);

  }
}
static void vAlarme( void *pvParameters )
{
  xSemaphoreTake( xBinarySemaphore_PORTA_A, 0);

  for( ;; )
  {
    xSemaphoreTake( xBinarySemaphore_PORTA_A, portMAX_DELAY );			//Recolha do semaforo
    Serial.println("Porta Aberta");
    bool Alarme = false;
    bool Trinco = false;
    xQueuePeek( xQueue_Trinco, &Trinco, 0 );
    Serial.print("Trinco = ");
    Serial.println(Trinco);

    if(Trinco){

		Alarme = true; 													// Transmite dados que avisa a ativação do alarme
    	xQueueOverwrite(xQueue_Alarme, &Alarme);
    	Serial.println("Caravana estava trancada");

    	while(true){													//Entrada em loop

		    xQueuePeek( xQueue_Alarme, &Alarme, 0 );					//Verifica o estado do alarme

			if(Alarme == 0){											//Caso o alarme seja desativado...

				Serial.println("Porta fechada");
				vTaskDelay(2000 / portTICK_PERIOD_MS);
				break;													//Sai do loop
			}else{
				digitalWrite(LED_PIN, HIGH);							//Sequencia lominosa de alarme
				vTaskDelay(500 / portTICK_PERIOD_MS);
				digitalWrite(LED_PIN, LOW);
				vTaskDelay(500 / portTICK_PERIOD_MS);
			}
			vTaskDelay(100 / portTICK_PERIOD_MS);
		}
    }else{																// Não reage pois não estava trancada
    	Serial.println("Caravana estava destrancada");
		vTaskDelay(100 / portTICK_PERIOD_MS);
    }

  }

}

void Manual(void *pvParameters){

	int Janela = 0;
	bool Aut_manual = false;
	bool Estado_UP = false;
	bool Estado_DOWN = false;
	bool Estado_AUT = false;
	bool SOF = false, AC = false;

	xQueueOverwrite(xQueue_AUT_MAN, &Aut_manual);
	xQueueOverwrite(xQueue_Janela, &Janela);


	for(;;){
		xQueuePeek(xQueue_AUT_MAN, &Aut_manual, portMAX_DELAY);			//Lê qual o modo atual
		Estado_AUT = digitalRead(AUT_MAN);
				if(!Aut_manual){										//Caso não esteja em modo automático

				digitalWrite(LED_SOF,LOW);						//Liga o LED a Sofagem
				xQueueOverwrite(xQueue_Sofagem, &SOF);			//Atualiza o estado da sofagem para ligado
				digitalWrite(LED_AC,LOW);							//Liga o LED do Ar Condicionado
				xQueueOverwrite(xQueue_AC, &AC);
				Estado_UP = digitalRead(BUT_UP);						//Lê o estdo do botão para fechar a janela
				Serial.print("ESTADO UP =");
				Serial.println(Estado_UP);

				Estado_DOWN = digitalRead(BUT_DOWN);					//Lê o estado do botão para abrir a janela
				Serial.print("ESTADO DOWN = ");
				Serial.println(Estado_DOWN);


				vTaskDelay(100 / portTICK_PERIOD_MS);
				xQueuePeek(xQueue_Janela, &Janela, portMAX_DELAY);		//lê o estado atual da janela

				if (Estado_DOWN && !Janela) {							//Se carregar no botão para abrir e a janela esta fechada
					myServo.write(MAX_ANG);								//Abre a janela
					vTaskDelay(50 / portTICK_PERIOD_MS);
					Serial.println("Janela abriu (Manual)");
					Janela = 1;
					xQueueOverwrite(xQueue_Janela, &Janela);			//Escreve na queue que a janela abriu (Janela = 1)
					vTaskDelay(500 / portTICK_PERIOD_MS);

				}
				if (Estado_UP && Janela) {								//Se carregar no botão para fechar e a janela esta aberta

					myServo.write(MIN_ANG);								//Fecha a janela
					vTaskDelay(50 / portTICK_PERIOD_MS);
					Serial.println("Janela fechou (Manual)");
					Janela = 0;
					xQueueOverwrite(xQueue_Janela, &Janela);			//Escreve na queue que a janela fechou (Janela = 0)
					vTaskDelay(500 / portTICK_PERIOD_MS);

				}else{
				vTaskDelay(500 / portTICK_PERIOD_MS);
				}

		}else{
			Serial.println("Esta no modo automatico");					//Não reage pois está em modo automático
			vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
	}
}
void ServoControl(void *pvParameters){
  int Janela = 0;
  int Temperatura = 0;
  int Alarme = false;
  int pos = 0;
  bool Sof = false, AC = false;
  bool Aut_manual = false;
  bool Estado_AUT = false;
	xQueueOverwrite(xQueue_Alarme, &Alarme);

	xSemaphoreTake( xBinarySemaphore_AUT_MAN, 0);					//Semaforo da interrupção do botão do modo automatico
    for(;;){
        xSemaphoreTake( xBinarySemaphore_AUT_MAN, portMAX_DELAY );

    	Estado_AUT = digitalRead(AUT_MAN);							//Lê estado do botão do modo automático

    	xQueuePeek(xQueue_TEMP, &Temperatura, portMAX_DELAY);		//Verifica o valor/estado da variavel inserida na queue
        xQueuePeek(xQueue_Alarme, &Alarme, portMAX_DELAY);			//Verifica o valor/estado da variavel inserida na queue
        xQueuePeek(xQueue_AUT_MAN, &Aut_manual, portMAX_DELAY);		//Verifica o valor/estado da variavel inserida na queue
        vTaskDelay(100 / portTICK_PERIOD_MS);

        if(Alarme){													//Em caso de Alarme
        	myServo.write(MIN_ANG);									//Fecha a janela
        	vTaskDelay(50 / portTICK_PERIOD_MS);
        	Janela = 1;
        	Serial.println("Janela Fechada");
        	xQueueOverwrite(xQueue_Janela, &Janela);				//Declara a janela fechada (Janela = 1) na queue

        }else{


    	    if(!Estado_AUT){										//Caso o butão esteja em modo automático
            	Serial.println("MODO AUTOMATICO");

    			Aut_manual = true;
    			xQueueOverwrite(xQueue_AUT_MAN, &Aut_manual);		//Atualiza o valor na queue para informar a ativação do modo automatico

    			while(!Estado_AUT){									//Enquanto o botão estiver em modo automático

    				xQueuePeek(xQueue_TEMP, &Temperatura, portMAX_DELAY); //Lê o valor de temperatura atualizado
    				vTaskDelay(100 / portTICK_PERIOD_MS);

    					Serial.println("Modo Automatico");

				if(Temperatura <= MIN_TEMP && !Janela){				//Caso esteja frio (valor menor que a temperatura minima) e a janela estiver aberta
				  digitalWrite(LED_SOF,HIGH);						//Liga o LED a Sofagem
				  Sof = 1;
				  xQueueOverwrite(xQueue_Sofagem, &Sof);			//Atualiza o estado da sofagem para ligado
				  digitalWrite(LED_AC,LOW);							//Liga o LED do Ar Condicionado
				  AC = 0;
				  xQueueOverwrite(xQueue_AC, &AC);					//Atualiza o estado do Ar Condicionado para desligado

				  for (pos = MAX_ANG; pos >= MIN_ANG; pos -= STEP_ANG) { //Movimento controlado do ServoMotor para fechar a janela
					  myServo.write(pos);
					  vTaskDelay(100 / portTICK_PERIOD_MS);
					  Serial.println(pos);

				  }
				  Janela = 1;
				  Serial.print("Janela Fechada");
				  xQueueOverwrite(xQueue_Janela, &Janela);			//Atualização da informação da janela para Fechada (Janela = 1)


			  }if(Temperatura >= MAX_TEMP && Janela){				//Caso esteja calor (valor superior que a temperatura máxima) e a janela estiver fechada
				  digitalWrite(LED_SOF,LOW);						//Desliga o LED da sofagem
				  Sof = 0;
				  xQueueOverwrite(xQueue_Sofagem, &Sof);			//Atualiza o estado da sofagem para desligado
				  digitalWrite(LED_AC,HIGH);						//Liga o LED do Ar Condicionado
				  AC = 1;
				  xQueueOverwrite(xQueue_AC, &AC);					//Atualiza o estado do Ar Condicionado para ligado

				  for (pos = MIN_ANG; pos <= MAX_ANG; pos += STEP_ANG) {	//Movimento controlado do ServoMotor para abrir a janela
					  myServo.write(pos);
					  vTaskDelay(100 / portTICK_PERIOD_MS);
					  Serial.println(pos);
				  }
				  Janela = 0;
				  Serial.print("Janela Aberta");
				  xQueueOverwrite(xQueue_Janela, &Janela);			//Atualização da informação da janela para Aberta (Janela = 0)


			  }
			  Estado_AUT = digitalRead(AUT_MAN);					//Lê estado do bão do modo automático
			  vTaskDelay(1000 / portTICK_PERIOD_MS);
		}
		Aut_manual = false;
		xQueueOverwrite(xQueue_AUT_MAN, &Aut_manual);		//Atualização de informação de mudança para modo manual na queue
		Serial.println("Fora do modo automatico");

		}
	vTaskDelay(1000 / portTICK_PERIOD_MS);

    }

}
}

void SensorTemp(void *pvParameters) {
    int temperatura = 0;
    int humidade = 0;

    for (;;) {
        int resultado = dht11.readTemperatureHumidity(temperatura, humidade);	// Leitura dos dados de Temperatura e humidade a partir do sensor e declaração desse valor nas devidas variaveis
        if (resultado == 0) {

        	Serial.print("Temperatura: ");
        	Serial.println(temperatura);
        	Serial.print("Humidade: ");
        	Serial.println(humidade);
        } else {
            Serial.println(DHT11::getErrorString(resultado)); //Escrira de um possivel erro encontrado
        }
        xQueueOverwrite(xQueue_HUM, &humidade);			//Atualização do valor de humidade na Queue xQueue_HUM
        xQueueOverwrite(xQueue_TEMP, &temperatura);		//Atualização do valor de temperatura na Queue xQueue_TEM
        vTaskDelay(3000 / portTICK_PERIOD_MS);			//Tempo necessário para a leitura correta do sensor
    }
}

static void  IRAM_ATTR  vInterruptPORTA_ABERTA( void ) 		// Interrupção da Porta
{

static signed portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( xBinarySemaphore_PORTA_A, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

  if( xHigherPriorityTaskWoken == pdTRUE )
  {

    portYIELD_FROM_ISR();

  }
}
static void  IRAM_ATTR  vInterruptAUT_MAN( void )		 // Interrupção do modo automático
{

static signed portBASE_TYPE xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR( xBinarySemaphore_AUT_MAN, (signed portBASE_TYPE*)&xHigherPriorityTaskWoken );

  if( xHigherPriorityTaskWoken == pdTRUE )
  {

    portYIELD_FROM_ISR();

  }
}

void loop() {
    vTaskDelete(NULL);
}
