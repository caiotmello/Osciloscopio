### Sumário
**[1.Considerações sobre os arquivos do Código Fonte](#SourceCode)**<br>
**[2.Configuração do CubeMX](#CubeMX)**<br>
**[3.Constantes, variáveis Globais e Tipos Importantes](#VarConstTypes)**<br>
**[4.Funções criadas do projeto (todas no main.c)](#Functions)**<br>

# 1.Considerações sobre os arquivos do Código Fonte <a name="SourceCode"></a>

A seguir, falamos de algumas considerações importantes sobre os arquivos *.h* e *.c* que são importantes para o correto funcionamento do projeto.  

## 1.1.Bibliotecas Importantes

Para o projeto, foram utilizadas as seguintes bibliotecas (já incluídas no código do GitHub):

|    Biblioteca           | Utilização                                                    |
| :---------------------: | :------------------------------------------------------------ |
| **fonts.h**             | Biblioteca de fontes para exibição de gráficos no display TFT |
| **tft.h**               | Biblioteca para escrita de valores no display TFT             |
| **user_setting.h**      | Biblioteca para escrita de valores no display TFT             |
| **functions.h**         | Biblioteca para escrita de valores no display TFT             |
| **arm_math.h**          | Para geração de séries de Fourier para análise de frequência  |
| **arm_const_structs.h** | Para geração de séries de Fourier para análise de frequência  |
| **math.h**              | Será necessário para trabalhar com valores numéricos          |
| **<string.h>**          | Biblioteca para trabalhar com string                          |

## 1.2.Folhas Importantes

Além disso, necessário também além das bibliotecas acima, os seguintes arquivos em *c* no projeto (eles já estão no projeto do Git, porém importante mencionar que precisam estar lá).

| Arquivo                 | Utilização                                                                                                                   |
| :---------------------: | :--------------------------------------------------------------------------------------------------------------------------- |
| **fonts.c**             | Arquivo de fontes para exibição de gráficos no display TFT                                                                   |
| **tft.c**               | Arquivo para escrita de valores no display TFT                                                                               |
| **imagens.c**           | Arquivo que contém as imagens que serão exibidas na tela do Osciloscopio para referencia dos eixos                           |
| **arm_fft_bin_data.c**  | Para geração de séries de Fourier para análise de frequência. Aqui fica o vetor que carrega as amostrar para exibição do FFT |

# 2.Configuração do CubeMX <a name="CubeMX"></a>

A configuração de pinos foi feita toda via CubeMX

![Configuração do CubeMX](Imagens/PrintIOC.jpeg)

Para isso foram utilizados os seguintes pinos e suas respectivas funções:

| Pino       | Port       | Obrigatório/Opcional      | Utilização       | Função Especial         | Comentários                                                                                                                                    |
| :--------: | :--------: |:------------------------: | :--------------: | :---------------------: | :--------------------------------------------------------------------------------------------------------------------------------------------- |  
| PA2        | A          | Opcional                  | RS232 TX         | USART                   | Pode ser ativado caso queira fazer a leitura de algum resultado pela Serial (USART)                                                            |
| PA3        | A          | Opcional                  | RS232 RX         | USART                   | Pode ser ativado caso queira fazer a leitura de algum resultado pela Serial (USART)                                                            |
| PA5        | A          | Opcional                  | LED Interno      | N/A                     | Pode ser usado para verificar alguma questão ou retorno para controle interno da placa. Por padrão, a placa do display TFT encobre esse LED.   |
| PA14       | A          | Obrigatório               |                  |                         |                                                                                                                                                |
| PA13       | A          | Obrigatório               |                  |                         |                                                                                                                                                |
| PB1        | B          | Obrigatório               | Modo DC          | ADC1 no Canal 9 (IN9)   | Utilização do Conversor AD para captura de tensões externas.                                                                                   |
| PB2        | B          | Obrigatório               | Timer            | TIM2 no Canal 4         | Utilização do Timer2 no Canal 4 para geração de uma função PWM. Para a primeira versão do projeto estamos utilizando um sinal de PWM interno.  |
| PB3        | B          | Obrigatório               |                  |                         |                                                                                                                                                |
| PC4        | C          | Obrigatório               | Botão Trigger    | EXTI4                   | Para o botão Hold funcionar o congelamento da tela, fizemos para o projeto através do uso de interrupção                                       |
| PC5        | C          | Obrigatório               | Escala V/DIV     | ADC2 no Canal 15 (IN15) | Usa para a escala V/DIV e sua exibição na tela do TFT                                                                                          |
| PC6        | C          | Obrigatório               | Botão Avança     |                         |                                                                                                                                                |
| PC8        | C          | Obrigatório               | Botão Recua      |                         |                                                                                                                                                |
| PC9        | C          | Obrigatório               | Botão Seleciona  |                         |                                                                                                                                                |
| PC13       | C          | Opcional                  | Botão Interno    | N/A                     | Pode ser usado para verificar alguma questão ou retorno para controle interno da placa. Por padrão, a placa do display TFT encobre esse botão. |
| PC14       | C          | Obrigatório               | Clock Interno    | RCC                     | Utilização do Clock Interno do microcontrolador                                                                                                |
| PC15       | C          | Obrigatório               | Clock Interno    | RCC                     | Utilização do Clock Interno do microcontrolador                                                                                                |
| PH1        | N/A        | Obrigatório               | Clock Interno    | RCC                     | Utilização do Clock Interno do microcontrolador                                                                                                |
| PH0        | N/A        | Obrigatório               | Clock Interno    | RCC                     | Utilização do Clock Interno do microcontrolador                                                                                                |

# 3.Constantes, variáveis Globais e Tipos Importantes <a name="VarConstTypes"></a>

## 3.1.Constantes

| Variável                        | Valor Padrão|  Comentário                                                                     |
| :-----------------------------: | :---------: |:------------------------------------------------------------------------------- |  
| **DEBUG**                       | 0           | Variável para auxiliar quando for necessário acionar algum debug                |
| **TEST_LENGTH_SAMPLES**         | 2048        | Usado pelas funções do FFT. Define a quantidade de amostras a capturar          |               
| **LCD_W**                       | 320         | Usado pelas funções do TFT. Define a largura do LCD em Pixels                   |
| **LCD_H**                       | 240         | Usado pelas funções do TFT. Define a altura do LCD em Pixels                    |
| **TIME_MAX**                    | 5           |                                                                                 |
| **SAMPLES**                     | 512         | Usado pelo FFT. Com isso, utilizamos para a exibição dos grafico do FFT na tela |
| **FFT_SIZE SAMPLES / 2**        |             | Usado pelo FFT. Com isso, utilizamos para a exibição dos grafico do FFT na tela |
| **DC_GND**                      | 0           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **FFT**                         | 1           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **T_DIV**                       | 2           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **HOLD_DC**                     | 3           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **HOLD_FFT**                    | 4           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **HOLD_TDIV**                   | 5           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **DC_GND_SCREEN**               | 1           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **FFT_SCREEN**                  | 2           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **FREQ_SCREEN**                 | 3           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **HOLD_SCREEN**                 | 4           | Para referenciar os estados da Máquina de Estado do Menu do FFT                 |
| **osciloscopio_320x240[76800]** | N/A         | Para exibição dos gráficos na tela do FFT                                       |
| **telaosc_320x240[76800]**      | N/A         | Para exibição dos gráficos na tela do FFT                                       |
 
## 3.2.Variáveis Globais

| Variável                             | Tipo               | Valor Padrão                                 | Comentário                                                                                            |
| :----------------------------------: | :----------------: | :------------------------------------------: |:----------------------------------------------------------------------------------------------------- |
| len                                  | int32_t            |  N/A                                         |                                                                                                       |
| size                                 | int32_t            |  N/A                                         |                                                                                                       |
| sBuffer[30]                          | char               |  N/A                                         | Usado como buffer para conversão                                                                      |
| iADCValues[320]                      | uint8_t            |  0                                           | Buffer para guardar as amostras do sinal analogico                                                    |
| iRefCH1                              |  uint16_t          |  120                                         | Posicao da referencia "0" no LCD                                                                      |
| mapTime[5][2]                        | uint16_t           | {{500,100},{250,50},{100,20},{50,10},{10,0}} | Matriz de 2X2 onde o primeiro valor é o tempo em us e o segundo valor é tempo para delay de amostras  |
| bModeDC                              | _Bool              | 1                                            | Variavel para indicar se o modo DC esta ativo                                                         |
| iTimeRef                             | uint8_t            | 4                                            |                                                                                                       |
| iModeFFT                             | uint8_t            | 0                                            |                                                                                                       |
| iVolts                               | uint8_t            | 2                                            |                                                                                                       |
| input_f32_10khz[TEST_LENGTH_SAMPLES] | float32_t          | Ver a Constante *TEST_LENGTH_SAMPLES*        | Buffer de Input e Output para o FFT                                                                   |
| output_buffer[TEST_LENGTH_SAMPLES/2] | static float32_t   | Ver a Constante *TEST_LENGTH_SAMPLES/2*      | Buffer de Input e Output para o FFT                                                                   |
| fim_amostragem                       | uint8_t            | 0                                            |                                                                                                       |
| Input[SAMPLES]                       | float32_t          | N/A                                          | Variáveis Globais para o FFT                                                                          |
| Output[FFT_SIZE]                     | float32_t          | N/A                                          | Variáveis Globais para o FFT                                                                          |
| bHold                                | _Bool              | 0                                            | Variáveis para trabalahar com a máquina de estado                                                     |
| bAvanca                              | _Bool              | 0                                            | Variáveis para trabalahar com a máquina de estado                                                     |
| bRecua                               | _Bool              | 0                                            | Variáveis para trabalahar com a máquina de estado                                                     |
| bSelect                              | _Bool              | 0                                            | Variáveis para trabalahar com a máquina de estado                                                     |
| iMenuSelection                       | uint8_t            | 1                                            |                                                                                                       |
| cState                               | uint8_t            | DC_GND                                       | O estado Inicial da Máquina de Estado.                                                                |

## 3.3.Tipos Importantes

### 3.3.1.State

Tipo que irá definir a estrutura da máquina de estado dos menus do FFT

```c

struct State {
	uint16_t Screen;		//Saída para o estado
	uint16_t wait;		//Tempo de espera do estado
	uint8_t next[8];	//Vetor de proximos estados
};

```

### 3.3.2.TipoS

Define a estrutura para o struct tipoS onde fica a definição completa da máquina de estado que vamos usar para navegar pelos menus e fazer as funções do osciloscopio.

```c

typedef const struct State tipoS;

//Estrutura de dados que corresponde ao diagrama de transição de estado
tipoS Fsm[6] = {
		[DC_GND] = {DC_GND_SCREEN,0,{DC_GND,HOLD_DC,FFT,HOLD_DC,T_DIV,HOLD_DC,DC_GND,HOLD_DC}},
		[FFT] = {FFT_SCREEN,0,{FFT,HOLD_FFT,T_DIV,HOLD_FFT,DC_GND,HOLD_FFT,FFT,HOLD_FFT}},
		[T_DIV] = {FREQ_SCREEN,0,{T_DIV,HOLD_TDIV,DC_GND,HOLD_TDIV,FFT,HOLD_TDIV,T_DIV,HOLD_TDIV}},
		[HOLD_DC] = {HOLD_SCREEN,0,{DC_GND,HOLD_DC,DC_GND,HOLD_DC,DC_GND,HOLD_DC,DC_GND,HOLD_DC}},
		[HOLD_FFT] = {HOLD_SCREEN,0,{FFT,HOLD_FFT,FFT,HOLD_FFT,FFT,HOLD_FFT,FFT,HOLD_FFT}},
		[HOLD_TDIV] = {HOLD_SCREEN,0,{T_DIV,HOLD_TDIV,T_DIV,HOLD_TDIV,T_DIV,HOLD_TDIV,T_DIV,HOLD_TDIV}},
};

```

# 4.Funções criadas do projeto (todas no main.c) <a name="Functions"></a>

## 4.1.USER_TFT_INIT

Usada para inicializar as funções do Display do TFT na rotação correta, com os timers necessários também. 
1. Parametros de Entrada: N/A
2. Parametros de Saída: N/A 

```c
void USER_TFT_Init(void)
{
	int16_t ID;

    //Sequência de inicialização do LCD
    tft_gpio_init(); 				//Inicializa os GPIOs do LCD (evita uso do CubeMX)
    HAL_TIM_Base_Start(&htim1); 		//Inicializa o Timer1 (base de tempo de us do LCD)
    ID = tft_readID(); 		//Lê o ID do LCD (poderia ser chamada pela inicialização do LCD)
    HAL_Delay(100);
    tft_init (ID); 			//Inicializa o LCD de acordo com seu ID
    setRotation(1); 			//Ajusta a orientação da tela
    fillScreen(BLACK); 		//Preenche a tela em uma só cor
}
```

## 4.2.USER_DrawBackground

Função Utilizada para o desenho da grade do osciloscopio na tela do TFT. Em uma primeira versão, usamos ela para fazer o desenho de cada botão e grade, e depois migramos para usar a função PlotaGrade que é chamada a partir dessa.
1. Parametros de Entrada: N/A
2. Parametros de Saída: N/A

```c
void USER_DrawBackground(void)
{
	fillScreen(BLACK);
	PlotaGrade(300, 200, 50, 10, 20, GREY);	// Plota a grade

	//Plota V/DIV
	//printnewtstr (10,18, WHITE, &mono12x7bold, 1, (uint8_t *)"V/DIV");

	//Plota menu DC
	//printnewtstr (15,237, WHITE, &mono12x7bold, 1, (uint8_t *)"DC");
	//printnewtstr_bc(15,237, BLACK,WHITE, &mono12x7bold, 1, (uint8_t *)" DC ");

	//Plota menu FFT
	//printnewtstr (150,237, WHITE, &mono12x7bold, 1, (uint8_t *)"FFT");

	//Plota menu t/DIV
	//printnewtstr (260,237, WHITE, &mono12x7bold, 1, (uint8_t *)"/DIV");

}
```

## 4.3.ModoDC

Função do modo DC do Osciloscópio, que Através do Menu Inicial iremos acessar
1. Parametros de Entrada: N/A
2. Parametros de Saída: N/A

```c
void ModoDC()
{
    uint8_t retornar = 0;
    uint16_t iSamplePos, iADC1;

	iSamplePos = 0;
	// AQUI VAI O CODIGO PARA FUNCIONAMENTO DO MODO AC DO OSCILOSCÓPIO//
	//Inicia a leitura  do conversor AD
	for(iSamplePos = 0; iSamplePos <= LCD_W; iSamplePos++)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
		{
			//Guarda o valor do conversor AD no Array
			iADC1 = HAL_ADC_GetValue(&hadc1);
			iADCValues[iSamplePos] = map(iADC1, 0, 4095, 0, 100);
			if(DEBUG)
			{
				size = sprintf(sBuffer,"ADC1 = %d\r\n",iADC1);
				HAL_UART_Transmit(&huart2, sBuffer, size, 10);
				size = sprintf(sBuffer,"iADCValues = %d\r\n",iADCValues[iSamplePos]);
				HAL_UART_Transmit(&huart2, sBuffer, size, 10);
			}

		}
		delayUs(mapTime[iTimeRef][1]);
	}

//	//Aqui é a condição de retorno/saída da tela.
//	if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_6))
//		retornar = 1;

	//Verifica se ja foi captada as 320 amostras
	if (iSamplePos >= LCD_W)
	{

		//Plota o sinal captado no LCD
		for(int x = 1; x <= LCD_W; x++)
		{
			drawLine(x-1, 120 - iADCValues[x-1], x, 120 - iADCValues[x], RED);
		}
		//memset(valores,0,320);
		iSamplePos = 0;
		HAL_Delay(500);

		//Limpa o sinal captado no LCD e printa a grade novamente
		for(int x = 1; x <= LCD_W; x++)
		{
			drawLine(x-1, 120 - iADCValues[x-1], x, 120 - iADCValues[x], BLACK);
		}
		PlotaGrade(300, 200, 50, 10, 20, GREY);
	}

}

```

## 4.4.ModoGND

Função para desenhar a referencia GND na tela
1. Parametros de Entrada: N/A
2. Parametros de Saída: N/A

```c
void ModoGND()
{
	drawLine ( 0,  iRefCH1,  LCD_W,  iRefCH1,  RED);
	HAL_Delay(500);
}
```

## 4.5.ModoFFT

Função para desenhar a referencia GND na tela
1. Parametros de Entrada: N/A
2. Parametros de Saída: N/A

```c
void ModoFFT(void)
{

	arm_cfft_radix4_instance_f32 S_CFFT;
	arm_rfft_instance_f32 S;
	uint16_t i;
    uint16_t y;
    uint16_t z;

	int32_t amp, escala, yini, yfinal, ycentro;
	char num[30];
	int32_t size;

    uint32_t fftSize = 1024;
    uint32_t ifftFlag = 0;
    uint32_t doBitReverse = 1;

	/* ----------------------------------------------------------------------
	* Max magnitude FFT Bin test
	* ------------------------------------------------------------------- */
	arm_status status;
	float32_t maxValue;

    /* Reference index at which max energy of bin ocuurs */
    uint32_t refIndex = 213;
    uint32_t  maxValueIndex = 0;

	//Verifica a amostragem do sinal terminou
    if(fim_amostragem == 1)
    {
    	fim_amostragem = 0;
    	if(DEBUG)
    	{
    		HAL_UART_Transmit(&huart2, "Fim da Amostragem\r\n", 21, 10);
    	}

		status = ARM_MATH_SUCCESS;

		//------------Teste LCD--------------
		//float32_t espelho_pre_processamento[320];
		//for(int i=0; i<320; i++)
		//  espelho_pre_processamento[i] = input_f32_10khz[i*2]; //Pula os zeros da parte imaginária

		/* Process the data through the CFFT/CIFFT module */
		arm_cfft_f32(&arm_cfft_sR_f32_len1024, input_f32_10khz, ifftFlag, doBitReverse);

		/* Process the data through the Complex Magnitude Module for calculating the magnitude at each bin */
		arm_cmplx_mag_f32(input_f32_10khz, output_buffer, fftSize);

		/* Calculates maxValue and returns corresponding BIN value */
		arm_max_f32(&output_buffer[1], fftSize-1, &maxValue, &maxValueIndex);

		/** Loop here if the signals fail the PASS check.
		 ** This denotes a test failure
		 ** ------------------------------------------------------------------- */
		if ( status != ARM_MATH_SUCCESS)
		{
			while (1);
		}

		//fs = 10.000, numero de amostras =1024, df = 10.000/1024 = 9,76
		//Detecta a frequencia principal
		maxValueIndex = maxValueIndex * 976 / 100;
		size = sprintf(sBuffer,"Frequencia principal = %d\r\n",maxValueIndex);
		HAL_UART_Transmit(&huart2, sBuffer, size, 10);

		//---------------Agora vai para o LCD----------------
		//Desenha o sinal no dominio da frequencia no LCD
		escala = 100;
		ycentro = 220;
		for(i=0; i<LCD_W; i++)
		{
			amp = (int32_t)(output_buffer[i]*escala/maxValue);
			if(DEBUG)
			{
				size = sprintf(num,"%d\r\n",amp);
				HAL_UART_Transmit(&huart2, num, size, 10);
			}

			if(amp>0) { yini = ycentro-amp; yfinal = ycentro; }
			else { yini = ycentro; yfinal = ycentro-amp; }

			fillRect(i, yini, 1, yfinal-yini+1, RED);
		}
		//Initialize ISR TIM6
		HAL_TIM_Base_Start_IT(&htim6);

		HAL_Delay(1000);

		//Limpa o sinal anterior
		for(i=0; i<LCD_W; i++)
		{
			amp = (int32_t)(output_buffer[i]*escala/maxValue);
			if(DEBUG)
			{
				size = sprintf(num,"%d\r\n",amp);
				HAL_UART_Transmit(&huart2, num, size, 10);
			}

			if(amp>0) { yini = ycentro-amp; yfinal = ycentro; }
			else { yini = ycentro; yfinal = ycentro-amp; }

			fillRect(i, yini, 1, yfinal-yini+1, BLACK);
		}

		//Plota a grade
		PlotaGrade(300, 200, 50, 10, 20, GREY);

    }


		//Geração de Sinal para teste( para utilizar, só descomentar esse e comentar o
		//laço for do  conversor AD acima)//

		/*
		for(i=0; i<512; ++i)
				{
					buffer_input[i] = (float32_t) 30*sin(2*PI*freq*i*dt);

					 ++freq;

						  if(freq > 200)
							freq = 1;
				}

		 */
		/////////////////////////////////////////////////////////////////////////////////
		// Plotagem do Sinal gerado para teste no dominio da frequencia para comparação//
		//(para utilizar, só descomentar , não esquecer de comentar o laço for do conversor AD//
		/*
		 for(i=0; i<255; ++i)
					 {
						drawLine(i + 32, (uint16_t)(buffer_input[i] + 50), i + 33, (uint16_t)(buffer_input[i+1] + 50), BLUE);

					  }
		 HAL_Delay(500);
		 */

}
```

## 4.6.updateMenu

Função responsavel por atualizar o menu do osciloscopio
1. Parametros de Entrada: iMenuSelect, variável que determina qual o menu que foi selecionado.
2. Parametros de Saída: N/A

```c
void updateMenu(uint8_t iMenuSelect)
{
	//Verifica se é modo DC ou GND
	if(bModeDC != true) sprintf(sBuffer,"GND");
		else sprintf(sBuffer,"DC ");

	if (iMenuSelect == DC_GND) printnewtstr_bc(15,237, BLACK,WHITE, &mono12x7bold, 1, sBuffer);
		else printnewtstr_bc (15,237, WHITE,BLACK, &mono12x7bold, 1, sBuffer);

	//Verifica se o Hold está pressionado
	if(bHold) printnewtstr_bc(260,15, BLACK,RED, &mono12x7bold, 1, (uint8_t *)"HOLD");
		else printnewtstr_bc(260,15, BLACK,BLACK, &mono12x7bold, 1, (uint8_t *)"    ");

	//Verifica se menu FFT está selecionado
	if (iMenuSelect == FFT)	printnewtstr_bc(150,237, BLACK,WHITE, &mono12x7bold, 1, "FFT");
		else printnewtstr_bc (150,237, WHITE,BLACK, &mono12x7bold, 1, "FFT");

	//Verifica se menu TDIV está selecionado
	if(iMenuSelect == T_DIV)
	{
		size = sprintf(sBuffer,"%dus ",mapTime[iTimeRef][0]);
		printnewtstr_bc(250,237, BLACK,WHITE, &mono12x7bold, 1, sBuffer);
	}
	else
	{
		size = sprintf(sBuffer,"%dus ",mapTime[iTimeRef][0]);
		printnewtstr_bc (250,237, WHITE,BLACK, &mono12x7bold, 1, sBuffer);
	}

	//Atualiza Volts/DIV
	size = sprintf(sBuffer,"%dV ",iVolts);
	printnewtstr (10,18, WHITE, &mono12x7bold, 1, sBuffer);

}
```

## 4.7.clickEventTreatment

Funcao para fazer o tratamento do click do botao select
1. Parametros de Entrada: iMenuSelect, variável que determina qual o menu que foi selecionado.
2. Parametros de Saída: N/A

```c
void clickEventTreatment(uint8_t iMenuSelect)
{
	if(iMenuSelect == DC_GND)
	{
		//altera entre modo GND e DC
		bModeDC = !bModeDC;
	}
	else if(iMenuSelect == T_DIV)
	{
		//altera tempo de amostragem do sinal
		iTimeRef++;
		if (iTimeRef >= TIME_MAX)
		{
			iTimeRef = 0;
		}

	}
	else
	{
		//Caso existir mais tratamentos de botões
	}

	bSelect = false;
}
```

## 4.8.HAL_GPIO_EXTI_Callback

Callback de tratamento de interrupcao EXTI
1. Parametros de Entrada: GPIO_PIN (Pino pressionado para verificar a interrupção)
2. Parametros de Saída: N/A

```c
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	//Verifica se a o botao hold foi pressionado
	if(GPIO_Pin == B_HOLD_Pin)
	{
		//HAL_Delay(20);
		if((HAL_GPIO_ReadPin(GPIOC, B_HOLD_Pin)== GPIO_PIN_SET))
		{
			bHold = !bHold;
		}
	}

	//Verifica se o botao select foi precionado
	if(GPIO_Pin == B_SELECT_Pin)
	{
		if((HAL_GPIO_ReadPin(GPIOC, B_SELECT_Pin)== GPIO_PIN_SET))
		{
			bSelect = true;
		}
	}
}
```

## 4.9.HAL_TIM_PeriodElapsedCallback

Callback das interrupcoes dos timers (TIM6) para amostragem do sinal de entrada para FFT
1. Parametros de Entrada: Ponteiro do Timer (*htim) a ser acionado como interrupção.
2. Parametros de Saída: N/A

```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static int32_t amostra = 0;

	//Ler um eixo do acelerometro
	if (amostra < TEST_LENGTH_SAMPLES/2)
	{
		HAL_ADC_Start(&hadc1);
		if(HAL_ADC_PollForConversion(&hadc1, 5) == HAL_OK)
		{
			input_f32_10khz[amostra*2] =  HAL_ADC_GetValue(&hadc1);
			input_f32_10khz[amostra*2 + 1] = 0;
			amostra++;
		}
	}
	else
	{
		fim_amostragem = 1;					//Sinaliza o fim do processo
		amostra = 0;
		HAL_TIM_Base_Stop_IT(&htim6);		//Interrompe a amostragem
	}
}
```
## 4.10.PlotaGrade

Funcao para plotar uma grade na tela LCD. Importante: o tamanho do quadro precisa ser um MDC entre largura e altura 
1. Parametros de Entrada: Tamanho da tela Largura e Altura (iLargura e iAltura), tamanho dos quadrados simetricos (tmQuadros), posicao X e Y do inicio da tela (posX e posY), cor da Linha (cCOLOR)   
2. Parametros de Saída: N/A

```c
void PlotaGrade(uint16_t iLargura, uint16_t iAltura, uint16_t tmQuadros, uint16_t posX, uint16_t posY, uint16_t cCOLOR)
{
	/** Define quantidade de quadros e tamanho */
	uint16_t qtL = (iAltura/tmQuadros); 	//qtd linhas
	uint16_t qtC = (iLargura/tmQuadros);	//qtd colunas
	uint16_t corT = YELLOW;					//cor dos tracos das subdivisoes
	int16_t posLV = (iLargura/2)+posX;		//posicao eixo central vertical
	int16_t posLH = (iAltura/2)+posY;		//posicao eixo central horizontal
	uint16_t i = 0;
	/** desenha linhas verticais */
	for(i=0;i<=qtC;i++)
	{
		drawFastVLine((i*tmQuadros+posX), posY, iAltura, cCOLOR);
	}
	/** desenha linhas horizontais */
	for(i=0;i<=qtL;i++)
	{
		drawFastHLine(posX, (i*tmQuadros+posY), iLargura, cCOLOR);
	}
	uint16_t fDX = 4;			//fator de divisao e multiplicacao (exibir tracos)
	uint16_t tmT = 6;			//tamanho do traco
	/** desenha tracos verticais */
	for(i=0;i<=(qtC*fDX);i++)
	{
		drawFastVLine((i*(tmQuadros/fDX)+posX), posLH-(tmT/2), tmT, corT);
	}
	/** desenha tracos horizontais */
	for(i=0;i<=(qtL*fDX);i++)
	{
		drawFastHLine(posLV-(tmT/2), (i*(tmQuadros/fDX)+posY), tmT, corT);
	}
}
```

## 4.11.MAP

Função MAP, portado do Arduino para o STM32

1. Parametros de Entrada:     
2. Parametros de Saída: 

```c
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
```

## 4.12.delayUs

Função para gerar Delay em microsegundos
1. Parametros de Entrada: quantidade de microsegundos a gerar (us)      
2. Parametros de Saída: N/A

```c
void delayUs (uint16_t us)
{
	__HAL_TIM_SET_COUNTER(&htim1,0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}
```