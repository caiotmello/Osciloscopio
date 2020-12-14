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

