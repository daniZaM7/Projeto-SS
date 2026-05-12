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

/* ********************************
 * Global defines 
 **********************************/
#define MICEX_ADC_UNIT                    ADC_UNIT_1
#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5            // Use Vref/0.75, 1.3 ... 1.5 V
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH   // 12 bits resolution (maximum)

#define MICEX_ADC_FRAME_SIZE             512                           /* ADC frame size, in bytes */
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE)    /* Internal buffer, should an integer multiple of the frame size to avoid incomplete frames */
#define MICEX_ADC_SAMPLE_FREQ            8000                   /* Sample frequency, in Hz. Notice that there are lower and higher bounds*/

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048 /* IMPORTANT: If FFT is to be used, must be a power of two */
                                              /* For time-domain conv. filters there is no such restriction */
                                              
#define MAX_FILT_IR_LEN                 200     /* Maximum IR filter length */

/* Global variable declarations */
static adc_channel_t channel[1] = {ADC_CHANNEL_3};  // Mic on ADC channel 3
static TaskHandle_t s_task_handle;

static const char *TAG = "COFRE_P3";

/* ADC - Variables to hold data acquisition and parsing */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0}; // Buffer where the results of a continuous read are placed   
__attribute__((aligned(16))) adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES]; // Buffer where frame parsed data is placed 
__attribute__((aligned(16))) float output_tom0[CONV_OUT_SIZE];
__attribute__((aligned(16))) float output_tom1[CONV_OUT_SIZE];
__attribute__((aligned(16))) float output_tom2[CONV_OUT_SIZE];

/* FreeRTOS tasks and IPC */
#define PROCESSOR_TASK_STACK_SIZE       8192            // Accomodate calls to dsp functions, log, user vars, ...
#define PROCESSOR_TASK_PRIORITY	( tskIDLE_PRIORITY + 4 )
QueueHandle_t XQ;    /* Queue handle */

/* Impulse reponse filter and related variables */
__attribute__((aligned(16))) const float filtro_tom0[128] = {-0.000549, 0.000306, 0.000850, 0.000353, -0.000667, -0.000951, -0.000022, 0.001071, 0.000908, -0.000474, -0.001412, -0.000617, 0.001055, 0.001506, 0.000073, -0.001489, -0.001207, 0.000523, 0.001479, 0.000587, -0.000762, -0.000857, -0.000041, 0.000194, -0.000177, 0.000236, 0.001367, 0.000949, -0.001831, -0.003554, -0.000438, 0.005069, 0.005334, -0.002324, -0.009403, -0.005226, 0.007709, 0.013397, 0.001835, -0.015041, -0.015020, 0.005464, 0.022484, 0.012328, -0.015974, -0.027425, -0.004296, 0.027546, 0.027281, -0.008521, -0.037023, -0.020487, 0.023823, 0.041170, 0.007270, -0.038047, -0.037793, 0.010137, 0.047382, 0.026623, -0.027978, -0.049005, -0.009598, 0.042052, 0.042052, -0.009598, -0.049005, -0.027978, 0.026623, 0.047382, 0.010137, -0.037793, -0.038047, 0.007270, 0.041170, 0.023823, -0.020487, -0.037023, -0.008521, 0.027281, 0.027546, -0.004296, -0.027425, -0.015974, 0.012328, 0.022484, 0.005464, -0.015020, -0.015041, 0.001835, 0.013397, 0.007709, -0.005226, -0.009403, -0.002324, 0.005334, 0.005069, -0.000438, -0.003554, -0.001831, 0.000949, 0.001367, 0.000236, -0.000177, 0.000194, -0.000041, -0.000857, -0.000762, 0.000587, 0.001479, 0.000523, -0.001207, -0.001489, 0.000073, 0.001506, 0.001055, -0.000617, -0.001412, -0.000474, 0.000908, 0.001071, -0.000022, -0.000951, -0.000667, 0.000353, 0.000850, 0.000306, -0.000549};

__attribute__((aligned(16))) const float filtro_tom1[128] = {0.000655, 0.000315, -0.000829, -0.000052, 0.000952, -0.000285, -0.000985, 0.000684, 0.000880, -0.001096, -0.000600, 0.001436, 0.000151, -0.001601, 0.000394, 0.001509, -0.000898, -0.001148, 0.001184, 0.000613, -0.001101, -0.000116, 0.000594, -0.000058, 0.000225, -0.000374, -0.001058, 0.001575, 0.001450, -0.003483, -0.000892, 0.005745, -0.001026, -0.007722, 0.004449, 0.008592, -0.009122, -0.007535, 0.014336, 0.003964, -0.018998, 0.002249, 0.021811, -0.010628, -0.021556, 0.020070, 0.017405, -0.028977, -0.009190, 0.035533, -0.002440, -0.038079, 0.016044, 0.035489, -0.029586, -0.027479, 0.040800, 0.014752, -0.047630, 0.001065, 0.048660, -0.017683, -0.043431, 0.032569, 0.032569, -0.043431, -0.017683, 0.048660, 0.001065, -0.047630, 0.014752, 0.040800, -0.027479, -0.029586, 0.035489, 0.016044, -0.038079, -0.002440, 0.035533, -0.009190, -0.028977, 0.017405, 0.020070, -0.021556, -0.010628, 0.021811, 0.002249, -0.018998, 0.003964, 0.014336, -0.007535, -0.009122, 0.008592, 0.004449, -0.007722, -0.001026, 0.005745, -0.000892, -0.003483, 0.001450, 0.001575, -0.001058, -0.000374, 0.000225, -0.000058, 0.000594, -0.000116, -0.001101, 0.000613, 0.001184, -0.001148, -0.000898, 0.001509, 0.000394, -0.001601, 0.000151, 0.001436, -0.000600, -0.001096, 0.000880, 0.000684, -0.000985, -0.000285, 0.000952, -0.000052, -0.000829, 0.000315, 0.000655};

__attribute__((aligned(16))) const float filtro_tom2[128] = {0.000265, -0.000748, 0.000765, -0.000255, -0.000497, 0.001003, -0.000872, 0.000103, 0.000849, -0.001332, 0.000935, 0.000165, -0.001258, 0.001566, -0.000819, -0.000502, 0.001480, -0.001442, 0.000474, 0.000652, -0.001120, 0.000743, -0.000082, -0.000132, -0.000236, 0.000513, 0.000125, -0.001649, 0.002801, -0.001946, -0.001317, 0.005169, -0.006389, 0.002798, 0.004399, -0.010526, 0.010313, -0.002102, -0.009837, 0.017248, -0.013471, -0.001018, 0.017541, -0.024273, 0.014620, 0.006994, -0.026720, 0.030142, -0.012754, -0.015546, 0.035961, -0.033368, 0.007483, 0.025614, -0.043532, 0.032879, 0.000746, -0.035543, 0.047833, -0.028381, -0.010671, 0.043477, -0.047853, 0.020502, 0.020502, -0.047853, 0.043477, -0.010671, -0.028381, 0.047833, -0.035543, 0.000746, 0.032879, -0.043532, 0.025614, 0.007483, -0.033368, 0.035961, -0.015546, -0.012754, 0.030142, -0.026720, 0.006994, 0.014620, -0.024273, 0.017541, -0.001018, -0.013471, 0.017248, -0.009837, -0.002102, 0.010313, -0.010526, 0.004399, 0.002798, -0.006389, 0.005169, -0.001317, -0.001946, 0.002801, -0.001649, 0.000125, 0.000513, -0.000236, -0.000132, -0.000082, 0.000743, -0.001120, 0.000652, 0.000474, -0.001442, 0.001480, -0.000502, -0.000819, 0.001566, -0.001258, 0.000165, 0.000935, -0.001332, 0.000849, 0.000103, -0.000872, 0.001003, -0.000497, -0.000255, 0.000765, -0.000748, 0.000265};

#define LED_PIN 11 // Pino definido no enunciado

// Definição dos Estados da FSM
typedef enum {
    IDLE,
    AQUIRING,
    VALIDATING,
    ERROR_BLINK
} FSM_State_t;

// Sequências da Turma P3 (Exemplo baseado na lógica do enunciado)
const int SEQ_ABRIR[4]  = {0, 1, 2, 0}; 
const int SEQ_FECHAR[4] = {0, 2, 1, 0};

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
    
    
    static FSM_State_t estado_atual = IDLE;
    static int tentativa[4];
    static int contador = 0;
    

    /* Infinite processing loop */
    for(;;) {
        // 2. Aplicar Convolução para cada Tom
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom0, 128, output_tom0);
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom1, 128, output_tom1);
        dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom2, 128, output_tom2);

        // 3. Calcular a Energia de cada saída (Soma dos Quadrados)
        float energia0 = 0, energia1 = 0, energia2 = 0;
        for (int i = 0; i < CONV_OUT_SIZE; i++) {
            energia0 += output_tom0[i] * output_tom0[i];
            energia1 += output_tom1[i] * output_tom1[i];
            energia2 += output_tom2[i] * output_tom2[i];
        }

        float threshold = 1000000.0; //! Valor a ajustar experimentalmente
        int tom_detectado = -1;

        if (energia0 > energia1 && energia0 > energia2 && energia0 > threshold) tom_detectado = 0;
        else if (energia1 > energia0 && energia1 > energia2 && energia1 > threshold) tom_detectado = 1;
        else if (energia2 > energia0 && energia2 > energia1 && energia2 > threshold) tom_detectado = 2;

        if (tom_detectado != -1) {
            ESP_LOGI(TAG, "Tom Detectado: %d (E0: %.2f, E1: %.2f, E2: %.2f)", tom_detectado, energia0, energia1, energia2);
            
            switch (estado_atual) {
            case IDLE:
            case AQUIRING:
                tentativa[contador] = tom_detectado;
                contador++;
                ESP_LOGI(TAG, "Símbolo %d guardado: %d", contador, tom_detectado);
                
                if (contador == 4) {
                    estado_atual = VALIDATING;
                } else {
                    estado_atual = AQUIRING;
                }
                
                // Debounce: evitar ler o mesmo tom várias vezes seguidas
                vTaskDelay(pdMS_TO_TICKS(1000)); 
                break;

            default:
                break;
            }
        }
        // Processamento dos estados de saída
        if (estado_atual == VALIDATING) {
            if (memcmp(tentativa, SEQ_ABRIR, sizeof(SEQ_ABRIR)) == 0) {
                ESP_LOGI(TAG, "ACESSO CONCEDIDO: ABRIR");
                gpio_set_level(LED_PIN, 1); // Liga LED
                estado_atual = IDLE;
            } 
            else if (memcmp(tentativa, SEQ_FECHAR, sizeof(SEQ_FECHAR)) == 0) {
                ESP_LOGI(TAG, "ACESSO CONCEDIDO: FECHAR");
                gpio_set_level(LED_PIN, 0); // Desliga LED
                estado_atual = IDLE;
            } 
            else {
                ESP_LOGW(TAG, "SEQUÊNCIA ERRADA!");
                estado_atual = ERROR_BLINK;
            }
            contador = 0;
        }

        if (estado_atual == ERROR_BLINK) {
            // Requisito: LED a piscar durante 5 segundos
            for (int i = 0; i < 10; i++) {
                gpio_set_level(LED_PIN, 1);
                vTaskDelay(pdMS_TO_TICKS(250));
                gpio_set_level(LED_PIN, 0);
                vTaskDelay(pdMS_TO_TICKS(250));
            }
            estado_atual = IDLE;
        }
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