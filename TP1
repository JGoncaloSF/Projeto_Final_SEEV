//Nome ALUNO A- João Freitas
//Nome ALUNO B- Tiago Vaz
//IPLEIRIA - Instituto Politécnico de Leiria
//ESTG - Escola Superior de Tecnologia e Gestão
//LEAU- Licenciatura em Engenharia Automóvel
//SEEV - Sistemas Elétricos e Eletrónicos de Veículos

//TP1: Pretende-se neste trabalho simular um sistema de atuação variável de uma válvula de admissão de um motor.

//Link para Youtube – https://www.youtube.com/shorts/2yR0Biyvl_E

#include "Arduino.h"
#include <stdio.h>
#include <ESP32Servo.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
//Bibliotecas para LCD:
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

//Definição dos pinos TFT:
#define TFT_CS 5
#define TFT_RST 0    //RESET
#define TFT_DC 17    //DC = A0
#define TFT_MOSI 23  //MOSI = SDA
#define TFT_SCLK 18  //Clock = SCK

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK,
TFT_RST);

//Definição dos pinos Encoder:
#define S1 4
#define S2 2

//Definição do pino Servo:
#define pinservo 16

//Definição de pino motor:
#define Pino_PWM 12

//Definição do pino sensor infra-vermelhos
#define Pino_Sensor 14

//Definição dos pinos LED:
#define LED1 13
#define LED2 27
#define LED3 26
#define LED4 25
#define LED5 33
#define LED6 32

//Criação das funções das tarefas:
void vTask_Encoder(void *pvParameters);
void vTask_Motor(void *pvParameters);
void vTask_Sensor(void *pvParameters);
void vTask_TFT(void *pvParameters);
void vTask_Servo(void *pvParameters);

//Criação de uma função de interrupção:
void Int_Contador(void);

//Declaração variável do semáfero:
SemaphoreHandle_t xCountingSemaphore;
SemaphoreHandle_t xMutex;

//Declaração de variáveis de Queues:
QueueHandle_t xQueueEncoder;
QueueHandle_t xQueueEncoder2;
QueueHandle_t xQueueEncoder3;
QueueHandle_t xQueueRPM;

//Declaração da variável servo:
Servo servo1;

//------------------------------------------------------------------------------------------------
void setup() {

	vTaskPrioritySet(NULL, configMAX_PRIORITIES - 1); //Define o setup como prioridade maxima

	Serial.begin(115200);

//Configuração dos pinos de saida do Encoder:
	pinMode(S1, INPUT);
	pinMode(S2, INPUT);

//Configuração do pino de entrada motor:
	pinMode(Pino_PWM, OUTPUT);

//Configuração do pino de entrada sensor infra-vermelho:
	pinMode(Pino_Sensor, INPUT);

//Configuração do pino Servo:
	pinMode(pinservo, OUTPUT);

//Configuração dos pinos LED:
	pinMode(LED1, OUTPUT);
	pinMode(LED2, OUTPUT);
	pinMode(LED3, OUTPUT);
	pinMode(LED4, OUTPUT);
	pinMode(LED5, OUTPUT);
	pinMode(LED6, OUTPUT);

//Criação Interrrupção:
	attachInterrupt(Pino_Sensor, Int_Contador, RISING);

//Criação de Queues:
	xQueueEncoder = xQueueCreate(2000, sizeof(int));
	xQueueEncoder2 = xQueueCreate(1, sizeof(int));
	xQueueEncoder3 = xQueueCreate(1, sizeof(int));
	xQueueRPM = xQueueCreate(3000, sizeof(int));

//Criação semáfero contador:
	xCountingSemaphore = xSemaphoreCreateCounting(4000, 0);

//Criação Mutex:
	xMutex = xSemaphoreCreateMutex();

//Criação das tarefas:
	if (xCountingSemaphore != NULL){
	xTaskCreatePinnedToCore(vTask_Encoder, "Task Encoder", 2000, NULL, 1,
	NULL, 1);
	xTaskCreatePinnedToCore(vTask_Motor, "Task Motor", 2000, NULL, 1, NULL, 1);

	xTaskCreatePinnedToCore(vTask_Sensor, "Task Sensor", 2000, NULL, 1,
	NULL, 1);

	xTaskCreatePinnedToCore(vTask_TFT, "Task TFT", 2000, NULL, 1,
	NULL, 1);

	xTaskCreatePinnedToCore(vTask_Servo, "Task Servo", 2000, NULL, 1,
	NULL, 1);
}
}

//----------------------------------------------------------------------------------------------
void vTask_Encoder(void *pvParameters) {
	int posicao = 1;
	int posicao2 = 1;
	int posicao3 = 1;
	int aState;
	int aLastState;

	// Lê o estado inicial da saida S1
	aLastState = digitalRead(S1);

	for (;;) {
		aState = digitalRead(S1); // lê o estado atual do rotary encoder
		// Se o estado atual for diferente do anterior, quer dizer que ocorreu um pulso
		if (aState != aLastState) {

			if (posicao >= 1) {

				if (posicao < 256) {

					if (digitalRead(S2) != aState) {
						posicao++;
						posicao2++;
						posicao3++;

					} else {
						posicao--;
						posicao2--;
						posicao3--;
					}

				} else {
					posicao--;
					posicao2--;
					posicao3--;
				}
			} else {
				posicao++;
				posicao2++;
				posicao3++;
			}

			xQueueSendToBack(xQueueEncoder, &posicao, 0);
			xQueueSendToBack(xQueueEncoder2, &posicao2, 0);
			xQueueSendToBack(xQueueEncoder3, &posicao3, 0);
		}
		aLastState = aState; // o último estado fica igual ao estado atual
	}
}
//-------------------------------------------------------------------------------------------
void vTask_Motor(void *pvParameters) {
	int ReceivedValue;
	portBASE_TYPE xStatusPWM;
	int pos = 0;

	for (;;) {
		//A tarefa pega no Mutex:
		xSemaphoreTake(xMutex, portMAX_DELAY);
		{
			//Recebe o valor da Queue Encoder
			xStatusPWM = xQueueReceive(xQueueEncoder, &ReceivedValue, 0);
		}
		//A tarefa devolve o Mutex depois de receber o valor da QueueEncoder:
		xSemaphoreGive(xMutex);

		if (xStatusPWM == pdPASS) {

			analogWrite(Pino_PWM, ReceivedValue);

		}

		if (ReceivedValue >= 2) {
			digitalWrite(LED1, HIGH);
		} else {
			digitalWrite(LED1, LOW);
		}

		if (ReceivedValue >= 50) {
			digitalWrite(LED2, HIGH);
		} else {
			digitalWrite(LED2, LOW);
		}

		if (ReceivedValue >= 100) {
			digitalWrite(LED3, HIGH);
		} else {
			digitalWrite(LED3, LOW);
		}

		if (ReceivedValue >= 150) {
			digitalWrite(LED4, HIGH);
		} else {
			digitalWrite(LED4, LOW);
		}

		if (ReceivedValue >= 200) {
			digitalWrite(LED5, HIGH);
		} else {
			digitalWrite(LED5, LOW);
		}

		if (ReceivedValue >= 255) {
			digitalWrite(LED6, HIGH);
		} else {
			digitalWrite(LED6, LOW);
		}

	}

}
//-------------------------------------------------------------------------------------------
void Int_Contador(void) {

	portBASE_TYPE xHigherPriorityTaskWoken;
	xHigherPriorityTaskWoken = pdFALSE;

	xSemaphoreGiveFromISR(xCountingSemaphore, &xHigherPriorityTaskWoken);

	if (xHigherPriorityTaskWoken == pdTRUE) {
		portYIELD_FROM_ISR();
	}

}
//--------------------------------------------------------------------------------------------
void vTask_Sensor(void *pvParameters) {

	int pulsos = 0;
	int rpm = 0;

	unsigned long timeold = 0;

	for (;;) {
		//Recebe a contagem dos pulsos feita pelo semáforo
		pulsos += uxSemaphoreGetCount(xCountingSemaphore);

		if (millis() - timeold >= 1000) {

			//Realização do cálculo dos RPM:
			rpm = (60 * 1000 / 4) / (millis() - timeold) * pulsos;
			timeold = millis();

			//Guarda os valores dos RPM na QueueRPM
			xQueueSendToBack(xQueueRPM, &rpm, 5);

			pulsos = 0;
		}

		xQueueReset(xCountingSemaphore);
	}

}
//------------------------------------------------------------------------------------------------
void vTask_TFT(void *pvParameters) {

	int Pedal;
	portBASE_TYPE xStatusPedal;
	long Display_RPM;

// Initialise the display 1.8" TFT screen:
	tft.initR(INITR_BLACKTAB);
	tft.setFont();
	tft.fillScreen(ST7735_BLACK);
	tft.setRotation(1);
// Desenhar/escrever conteúdos fixos:
	tft.setCursor(0, 0);
	tft.setTextColor(ST7735_BLUE);
	tft.setTextSize(4);
	tft.print("RPM:");
	tft.setCursor(0, 30);
	tft.setTextColor(ST7735_BLUE);
	tft.setTextSize(3);
	tft.print("Pedal:");
	if (Pedal == 256) {
		tft.setCursor(0, 55);
		tft.setTextColor(ST7735_RED);
		tft.setTextSize(2);
		tft.println("RPM Maximo!");
	}

	for (;;) {

		xStatusPedal = xQueueReceive(xQueueEncoder2, &Pedal, 0);

		xQueueReceive(xQueueRPM, &Display_RPM, 0);


		tft.setCursor(88, 4);
		tft.setTextColor(ST7735_WHITE);
		tft.setTextSize(3);
		tft.println(Display_RPM);

		tft.setCursor(88, 4);
		tft.setTextColor(ST7735_BLACK);
		tft.setTextSize(3);
		tft.println(Display_RPM);

		tft.setCursor(102, 31);
		tft.setTextColor(ST7735_WHITE);
		tft.setTextSize(3);
		tft.println(Pedal);

		tft.setCursor(102, 31);
		tft.setTextColor(ST7735_BLACK);
		tft.setTextSize(3);
		tft.println(Pedal);
		if (Pedal == 256) {
			tft.setCursor(0, 55);
			tft.setTextColor(ST7735_RED);
			tft.setTextSize(2);
			tft.println("RPM Maximo!");
			tft.setCursor(0, 55);
			tft.setTextColor(ST7735_BLACK);
			tft.setTextSize(2);
			tft.println("RPM Maximo!");

		}
	}
}
//------------------------------------------------------------------------------------------------------
void vTask_Servo(void *pvParameters) {

	int pos = 180;
	int Encoder;

	for (;;) {
		xQueueReceive(xQueueEncoder3, &Encoder, 0);

		if (Encoder >= 249) {
			servo1.attach(pinservo);

			for (pos = 0; pos <= 180; pos += 1) {

				servo1.write(pos);
				delay(1);
			}

			for (pos = 180; pos >= 0; pos -= 1) {

				servo1.write(pos);
				delay(1);

			}
		}
		servo1.detach();
	}

}

void loop() {

	vTaskDelete( NULL);
}
