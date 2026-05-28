/* ********************************************************************************************************************************* 
 * Microphone test - ADC in continuous mode and time-domain BP filtering 
 * Paulo Pedreiras, Pedro Fonecsa, Luis Moutinho 2026/Apr.
 * 
 * Tested:
 *  ESP32-C6 DevKitC-1
 * 
 * - Basic use of the ADC to get and process sound samples.
 * - Uses continuous mode ADC operation, to allow higher frequencies
 * - Signal is processed by a Band-Pass filter, in the time-domain, to identify defined frequencies 
 *  
 * Microphone is a MEMS Adafruit Silicon MEMS Microphone Breakout - SPW2430.
 *     Supplied with 3.3-5V, output at DC pin has a 0.7 V and a 100 mVpp "when talking near". 
 *      In my case I had around 1 V. So the attenuation cannot be 0 dB. 
 *      I have used 2.5 dB (vref/0.7), to get 1.3 to 1.5 volts for Vref+ and avoid saturation
 *      Check other mics to see if this is normal.  
 * 
 *  
 * Bibliography: 
 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/index.html
 *      https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/adc/adc_continuous.html 
 *      https://docs.espressif.com/projects/esp-dsp/en/latest/esp32/esp-dsp-library.html      
 * 
 * Based on the sample code  provided by EspressIF:
 *      https://github.com/espressif/esp-idf/tree/47faecc3/examples/peripherals/adc/continuous_read 
 * 
 * NOTE: must run idf.py add-dependency "espressif/esp-dsp" when creating a new project using dsp functionality
 ***********************************************************************************************************************************/ 

/* ********************************* 
 * Includes
 ***********************************/
#include <string.h>
#include <stdio.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"      // FreeRTOS includes
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h" // For ESP ADC
#include "esp_dsp.h"                // For ESP DSP functions, conv in the case
#include "esp_private/esp_clk.h"    // For ESP clock functions
#include "driver/gpio.h"            // Para o LED

/* ********************************
 * Global defines 
 **********************************/
#define MICEX_ADC_UNIT                    ADC_UNIT_1
#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5            // Use Vref/0.75, 1.3 ... 1.5 V
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH   // 12 bits resolution (maximum)

#define MICEX_ADC_FRAME_SIZE             512                           /* ADC frame size, in bytes */
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE)    /* Internal buffer, should an integer multiple of the frame size to avoid incomplete frames */
#define MICEX_ADC_SAMPLE_FREQ            (20 * 1000)                   /* Sample frequency, in Hz. Notice that there are lower and higher bounds*/

#define MICEX_SOUND_SAMPLES_BUF_SIZE     64 /* IMPORTANT: If FFT is to be used, must be must be a power of two */
                                              /* For time-domain conv. filters there is no such restriction */
                                              
#define MAX_FILT_IR_LEN                 200     /* Maximum IR filter length */

#define CONV_OUT_SIZE                    (2 * MICEX_SOUND_SAMPLES_BUF_SIZE  - 1) // Saida da convolução (N + M - 1)

#define LED_PIN 11 // Pino da placa 


/* Global variable declarations */
static adc_channel_t channel[1] = {ADC_CHANNEL_3};  // Mic on ADC channel 3
static TaskHandle_t s_task_handle;

static const char *TAG = "COFRE_P3";

/* ADC - Variables to hold data acquisition and parsing */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0}; // Buffer where the results of a continuous read are placed   
__attribute__((aligned(16))) adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES]; // Buffer where frame parsed data is placed 

// Buffers para os resuldados de cada tom 
__attribute__((aligned(16))) float output_tom0[CONV_OUT_SIZE];
__attribute__((aligned(16))) float output_tom1[CONV_OUT_SIZE];
__attribute__((aligned(16))) float output_tom2[CONV_OUT_SIZE];

/* FreeRTOS tasks and IPC */
#define PROCESSOR_TASK_STACK_SIZE       8192            // Accomodate calls to dsp functions, log, user vars, ...
#define PROCESSOR_TASK_PRIORITY	( tskIDLE_PRIORITY + 4 )
QueueHandle_t XQ;    /* Queue handle */

// Sequencias definidas para abrir e fechar o sistema
const int SEQ_ABRIR[4]  = {0, 1, 2, 0}; 
const int SEQ_FECHAR[4] = {0, 2, 1, 0};

// Filtros FIR com parâmetros: N=64, Fs=20000 
__attribute__((aligned(16))) const float filtro_tom0[64] = {-0.002957, -0.001096, 0.001190, 0.003740, 0.006166, 0.007811, 0.007872, 0.005671, 0.000995, -0.005627, -0.012849, -0.018711, -0.021115, -0.018471, -0.010311, 0.002326, 0.016920, 0.029937, 0.037693, 0.037372, 0.027932, 0.010612, -0.011152, -0.032425, -0.047951, -0.053511, -0.047094, -0.029550, -0.004522, 0.022366, 0.044859, 0.057632, 0.057632, 0.044859, 0.022366, -0.004522, -0.029550, -0.047094, -0.053511, -0.047951, -0.032425, -0.011152, 0.010612, 0.027932, 0.037372, 0.037693, 0.029937, 0.016920, 0.002326, -0.010311, -0.018471, -0.021115, -0.018711, -0.012849, -0.005627, 0.000995, 0.005671, 0.007872, 0.007811, 0.006166, 0.003740, 0.001190, -0.001096, -0.002957};

__attribute__((aligned(16))) const float filtro_tom1[64] = {-0.004522, -0.003509, -0.000777, 0.002959, 0.006273, 0.007264, 0.004444, -0.002054, -0.009784, -0.014606, -0.012696, -0.003050, 0.011039, 0.022780, 0.025040, 0.014433, -0.005929, -0.026946, -0.037560, -0.030558, -0.007081, 0.022866, 0.044702, 0.046506, 0.025546, -0.009498, -0.042168, -0.056184, -0.043767, -0.009983, 0.029356, 0.055337, 0.055337, 0.029356, -0.009983, -0.043767, -0.056184, -0.042168, -0.009498, 0.025546, 0.046506, 0.044702, 0.022866, -0.007081, -0.030558, -0.037560, -0.026946, -0.005929, 0.014433, 0.025040, 0.022780, 0.011039, -0.003050, -0.012696, -0.014606, -0.009784, -0.002054, 0.004444, 0.007264, 0.006273, 0.002959, -0.000777, -0.003509, -0.004522};

__attribute__((aligned(16))) const float filtro_tom2[64] = {-0.003088, -0.004642, -0.002621, 0.002124, 0.006399, 0.006169, -0.000086, -0.008669, -0.012097, -0.005167, 0.008956, 0.019059, 0.014427, -0.004599, -0.024053, -0.026304, -0.005733, 0.023651, 0.037407, 0.021010, -0.015614, -0.043510, -0.037795, 0.000156, 0.041258, 0.051286, 0.019730, -0.029675, -0.057050, -0.039101, 0.010808, 0.052746, 0.052746, 0.010808, -0.039101, -0.057050, -0.029675, 0.019730, 0.051286, 0.041258, 0.000156, -0.037795, -0.043510, -0.015614, 0.021010, 0.037407, 0.023651, -0.005733, -0.026304, -0.024053, -0.004599, 0.014427, 0.019059, 0.008956, -0.005167, -0.012097, -0.008669, -0.000086, 0.006169, 0.006399, 0.002124, -0.002621, -0.004642, -0.003088};


/* *************************************************************** 
 * Function prototypes 
 *****************************************************************/
/* Inits the ADC for continuous mode (channels, attenuation, frequency, handles, ...)*/
 static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
 /* Callback of ADC driver. Executed whenever a new frame is available */
static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
/* Task called to process one full buffer of data. A queue + blocking read is used for synchronization and data passing */
static void pv_processor_task(void *pvParam);

/******************************************************************* 
 * The main task 
 *******************************************************************/
void app_main(void)
{
    /* Variable declarations */
    esp_err_t ret;          // Generic return code variable
    esp_err_t parse_ret;    // return code of ADC frame parse function 
    uint32_t ret_num = 0;   // Length of bytes return by a read operation
    uint32_t sb_count = 0;   // For counting the number of acquired samples    
    uint32_t num_parsed_samples = 0;    // To count the number of parsed samples
    
    adc_continuous_evt_cbs_t cbs;   // Variable for setting callback type (internal poll full, or frame conversion completed)    
    adc_continuous_handle_t handle = NULL;  //Handle for ADC          

    float * sound_samp_buf_ADC;   // Buffer to hold sound samples. Sound buffers are float because conv() function requires float parameters - avoid conversions 
    
    /* Variable inits */
    memset(result, 0x00, MICEX_ADC_FRAME_SIZE); // Init frame buffer     
    sound_samp_buf_ADC = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);     

    s_task_handle = xTaskGetCurrentTaskHandle();    // Get handle of the current task

    cbs.on_conv_done = s_conv_done_cb;  // Callback called when one conversion frame is done     
    cbs.on_pool_ovf = NULL;          // Don't set callback for internbal buffer overflow         

    /* Set log level */
    /* Debug allow to see variable values */
    /* Info only shows the decision */
    /* Verbose shows a trace of calls an some additional vars*/
    esp_log_level_set(TAG,ESP_LOG_DEBUG);


    //  Configuração do GPIO do LED
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0); // Inicia desligado (somente para testar)
    
    
    /* Processor task and Queue inits */
    XQ=xQueueCreate(1, sizeof(float)*MICEX_SOUND_SAMPLES_BUF_SIZE); // Create queue to store one full sample period of sound
    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL );

    /* Init ADC */
    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle); // Call init function
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));   // Regiter callbacks
    ESP_ERROR_CHECK(adc_continuous_start(handle));                                  // Start the ADC

    /* Infinite loop - wait for data and process it */
    /* Synchronization with ADC is obtained via the ulTaskNotifyTake(pdTRUE, portMAX_DELAY); call */
    /*     that assures that processing does not proceed until a notification that a frame was acquired*/
    while (1) {        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // Wait for a new frame

        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK) {
                ESP_LOGV(TAG, "ret is %x, ret_num is %"PRIu32" bytes", ret, ret_num);                
                /* One frame received. Extract samples from frame and put them on sound sample buffer*/
                parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
                if (parse_ret == ESP_OK) {
                    
                    for (int i = 0; i < num_parsed_samples; i++) {
                        sound_samp_buf_ADC[sb_count] = (float) parsed_data[i].raw_data;                           
                        sb_count+=1;
                        if(sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) { // The sound buffer is full. Process it ... */
                            ESP_LOGD(TAG, "sound buffer acquired. Time to process ...\n");                
                            xQueueSend(XQ,(void *)sound_samp_buf_ADC,0);     // Places the sound buffer in the queue. If the queue is full skip it (ticksTo Wait set to 0)
                                                                        // The consumer/processing task is automatically waked if blocked in the Queue
                            sb_count = 0;
                        }
                    }

                } else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }
                /*                  
                 * To avoid a task watchdog timeout, add a delay here. 
                 */
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                //We try to read `EXAMPLE_READ_LEN` until API returns timeout, which means there's no available data
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}


/* **********************************************************************************************
 * Task activated when there is a full buffer of sound samples data available
 * The task reads a queue in blocking mode. This wait it awakes whenever the ADC processing code
 *      (the app_main taks in the case) delivers a new full buffer of data. 
 * Note that the use of a Queue and two separate buffers (ADC and processing) decouples the 
 *      acquisition from processing. I.e., processing can take as much time as needed without race conditions
 *      or any other sort of interference. The cost is overhead ...
 ************************************************************************************************/
void pv_processor_task(void *pvParam)
{
    /* Local vars, for auxiliary computations */    
    int n;        
    float * sound_samp_buf_proc;       // Buffer to hold sound samples. Buffers are float because conv() function requires float parameters - avoid conversions     
    
    /* Variable inits */
    sound_samp_buf_proc = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);         
    
    // Variáveis da Máquina de Estados (FSM)
    int tentativa[4];
    int contador = 0;
    
    // Variavel de controlo
    int ultimo_tom = -1;
    float threshold_pureza = 2.0; // Ajustar de acordo com ambiente
    
    
/* Infinite processing loop */
    for(;;) {
    
        /* Waits for new data */
        if(xQueueReceive(XQ, (void *)sound_samp_buf_proc, portMAX_DELAY) == pdTRUE) { // Reads a sound sample. Blocks if queue is empty.
            ESP_LOGV(TAG, "Process Task got a buffer!");
            

            // A) Pré-processamento
 				// 1.Componente DC 
            float comp_DC = 0;
            for(int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
                comp_DC += sound_samp_buf_proc[i];
            }
            comp_DC = comp_DC / (float)MICEX_SOUND_SAMPLES_BUF_SIZE;

				// 2.Remoção do DC e potencia total
	 		float P_total = 0;
            for(int i = 0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
                sound_samp_buf_proc[i] = (sound_samp_buf_proc[i] - comp_DC); 
                P_total +=  sound_samp_buf_proc[i] * sound_samp_buf_proc[i];
            }
            P_total = P_total / (float)MICEX_SOUND_SAMPLES_BUF_SIZE;
            
            // B) Filtragem FIR (Convolução) 
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom0, 64, output_tom0);
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom1, 64, output_tom1);
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom2, 64, output_tom2);

            // C. Cálculo da Energia (Soma dos quadrados) e potências
            float energia0 = 0, energia1 = 0, energia2 = 0, potencia0 = 0, potencia1 = 0, potencia2 = 0;
            
            for (int i = 0; i < CONV_OUT_SIZE; i++) {
                energia0 += output_tom0[i] * output_tom0[i];
                energia1 += output_tom1[i] * output_tom1[i];
                energia2 += output_tom2[i] * output_tom2[i];
            }
            potencia0 = energia0 / (float)CONV_OUT_SIZE;
            potencia1 = energia1 / (float)CONV_OUT_SIZE;
            potencia2 = energia2 / (float)CONV_OUT_SIZE;
            
            // D) Estimar ruido 
            float P_ruido = P_total - (potencia0 + potencia1 + potencia2);		// Filtro rejeita banda
           
            if (P_ruido < 0) P_ruido = 0;										// Evitar pequenos arredondamentos insignificantes
            
            // E) Logica de deteção dos tons
            	// 1. Determinação/seleção do tom em causa
            int tom_detetado = -1;
            float maior_potencia = 0;
            if (potencia0 > potencia1 && potencia0 > potencia2) {
                tom_detetado = 0;
                maior_potencia = potencia0;
                
            } else if (potencia1 > potencia0 && potencia1 > potencia2) {
                tom_detetado = 1;
                maior_potencia = potencia1;
                
            } else if (potencia2 > potencia0 && potencia2 > potencia1) {
                tom_detetado = 2;
                maior_potencia = potencia2;
            }
            
				// 2. Determinar se foi tom puro ou ruído
            if (tom_detetado != -1) {
                float racio_pureza = maior_potencia / (P_ruido + 0.0001);	// Adicionamos 0.0001 no denominador para evitar divisões por zero

                if (racio_pureza > threshold_pureza) {
                    // Mantém o tom_detetado
                } else {
                    // Falso positivo
                    tom_detetado = -1; 
                }
            }

            // F) Lógica da Máquina de Estados (FSM) 
            if (tom_detetado != -1) {	// Só entra aqui se um tom valido (0, 1 ou 2) for capturado
                if (tom_detetado != ultimo_tom) { 
                    ESP_LOGI(TAG, "Tom %d detetado! P0:%.1f | P1:%.1f | P2:%.1f | P_Ruido:%.2f", tom_detetado, potencia0, potencia1, potencia2, P_ruido);
                    
                    // Armazena o tom na sequência de tentativa
                    tentativa[contador] = tom_detetado;
                    contador++;
                    ultimo_tom = tom_detetado; // Bloqueio para evitar duplicação da mesma nota
                    
                    // Maquina de estados
                    if (contador == 4) {
                        // Estado: Verificar Sequência
                        if (memcmp(tentativa, SEQ_ABRIR, sizeof(SEQ_ABRIR)) == 0) {
                            ESP_LOGI(TAG, "Acesso Concedido: ABRIR");
                            gpio_set_level(LED_PIN, 1); 
                        } else if (memcmp(tentativa, SEQ_FECHAR, sizeof(SEQ_FECHAR)) == 0) {
                            ESP_LOGI(TAG, "Acesso Concedido: FECHAR");
                            gpio_set_level(LED_PIN, 0); 
                        } else {
                            // Estado: Sequência Errada
                            ESP_LOGW(TAG, "Sequência Errada! [ %d, %d, %d, %d ] -> LED a piscar...", tentativa[0], tentativa[1], tentativa[2], tentativa[3]);
                            for (int i = 0; i < 10; i++) { 
                                gpio_set_level(LED_PIN, 1);
                                vTaskDelay(pdMS_TO_TICKS(250));
                                gpio_set_level(LED_PIN, 0);
                                vTaskDelay(pdMS_TO_TICKS(250));
                        	}
                    	}
                        // Reset da FMS (IDLE)
                        contador = 0; 
                        ultimo_tom = -1; 
                	}
                    // Tempo minimo para limpeza do buffer acumulado (debounce)
                    vTaskDelay(pdMS_TO_TICKS(800));
                    xQueueReset(XQ); 
                }
            } else {
                ultimo_tom = -1;	// Limpeza do histórico no silêncio (para notas repetidas)
            }
        }
        xQueueReset(XQ);			// Limpeza da QUEUE
    }
}

/* ADC Callback - called when one frame was acquired */
static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    //Notify that ADC continuous driver has done enough number of conversions
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);

    return (mustYield == pdTRUE);
}

/* ADC init function */
static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle)
{
    adc_continuous_handle_t handle = NULL;

    adc_continuous_handle_cfg_t adc_config = {
        .max_store_buf_size = MICEX_ADC_BUF_SIZE,
        .conv_frame_size = MICEX_ADC_FRAME_SIZE,
    };
    ESP_ERROR_CHECK(adc_continuous_new_handle(&adc_config, &handle));

    adc_continuous_config_t dig_cfg = {
        .sample_freq_hz = MICEX_ADC_SAMPLE_FREQ,
        .conv_mode = MICEX_ADC_CONV_MODE,
    };

    adc_digi_pattern_config_t adc_pattern[SOC_ADC_PATT_LEN_MAX] = {0};
    dig_cfg.pattern_num = channel_num;
    for (int i = 0; i < channel_num; i++) {
        adc_pattern[i].atten = MICEX_ADC_ATTEN;
        adc_pattern[i].channel = channel[i] & 0x7;
        adc_pattern[i].unit = MICEX_ADC_UNIT;
        adc_pattern[i].bit_width = MICEX_ADC_BIT_WIDTH;

        ESP_LOGI(TAG, "adc_pattern[%d].atten is :%"PRIx8, i, adc_pattern[i].atten);
        ESP_LOGI(TAG, "adc_pattern[%d].channel is :%"PRIx8, i, adc_pattern[i].channel);
        ESP_LOGI(TAG, "adc_pattern[%d].unit is :%"PRIx8, i, adc_pattern[i].unit);
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}
