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
#include "freertos/FreeRTOS.h"      
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_adc/adc_continuous.h" 
#include "esp_dsp.h"                
#include "esp_private/esp_clk.h"    
#include "driver/gpio.h" // Adicionado para controlo do LED


/* ********************************
 * Global defines 
 **********************************/
#define MICEX_ADC_UNIT                    ADC_UNIT_1
#define MICEX_ADC_CONV_MODE               ADC_CONV_SINGLE_UNIT_1
#define MICEX_ADC_ATTEN                   ADC_ATTEN_DB_2_5            
#define MICEX_ADC_BIT_WIDTH               SOC_ADC_DIGI_MAX_BITWIDTH   

#define MICEX_ADC_FRAME_SIZE             512                           
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE)    
#define MICEX_ADC_SAMPLE_FREQ            8000 // Frequência de amostragem definida para os filtros

#define MICEX_SOUND_SAMPLES_BUF_SIZE     2048 
#define CONV_OUT_SIZE                    (MICEX_SOUND_SAMPLES_BUF_SIZE + 128 - 1) // Tamanho da saída da convolução (N + M - 1)

#define LED_PIN 11 // Pino definido no enunciado

/* Global variable declarations */
static adc_channel_t channel[1] = {ADC_CHANNEL_3};  // Microfone no ADC channel 3
static TaskHandle_t s_task_handle;

static const char *TAG = "COFRE_P3";

/* ADC - Variables to hold data acquisition and parsing */
__attribute__((aligned(16))) uint8_t result[MICEX_ADC_FRAME_SIZE] = {0};   
__attribute__((aligned(16))) adc_continuous_data_t parsed_data[MICEX_ADC_FRAME_SIZE / SOC_ADC_DIGI_RESULT_BYTES]; 

/* Buffers globais para o resultado da convolução */
__attribute__((aligned(16))) float output_tom0[CONV_OUT_SIZE];
__attribute__((aligned(16))) float output_tom1[CONV_OUT_SIZE];
__attribute__((aligned(16))) float output_tom2[CONV_OUT_SIZE];

/* FreeRTOS tasks and IPC */
#define PROCESSOR_TASK_STACK_SIZE       8192            
#define PROCESSOR_TASK_PRIORITY ( tskIDLE_PRIORITY + 4 )
QueueHandle_t XQ;    

/* Sequências de Acesso */
const int SEQ_ABRIR[4]  = {0, 1, 2, 0}; 
const int SEQ_FECHAR[4] = {0, 2, 1, 0};

/* Filtros FIR (N=128, Fs=8000) */
__attribute__((aligned(16))) const float filtro_tom0[128] = {0.000465, -0.000268, -0.000768, -0.000331, 0.000650, 0.000967, 0.000023, -0.001200, -0.001073, 0.000595, 0.001888, 0.000885, -0.001634, -0.002541, -0.000135, 0.003074, 0.002813, -0.001402, -0.004667, -0.002258, 0.003762, 0.005914, 0.000471, -0.006661, -0.006143, 0.002717, 0.009460, 0.004697, -0.007063, -0.011244, -0.001176, 0.011842, 0.011040, -0.004325, -0.015915, -0.008122, 0.011094, 0.017973, 0.002309, -0.017848, -0.016886, 0.005845, 0.022971, 0.012071, -0.015040, -0.024902, -0.003772, 0.023451, 0.022574, -0.006868, -0.029134, -0.015772, 0.017977, 0.030511, 0.005301, -0.027344, -0.026818, 0.007114, 0.032941, 0.018365, -0.019181, -0.033440, -0.006529, 0.028563, 0.028563, -0.006529, -0.033440, -0.019181, 0.018365, 0.032941, 0.007114, -0.026818, -0.027344, 0.005301, 0.030511, 0.017977, -0.015772, -0.029134, -0.006868, 0.022574, 0.023451, -0.003772, -0.024902, -0.015040, 0.012071, 0.022971, 0.005845, -0.016886, -0.017848, 0.002309, 0.017973, 0.011094, -0.008122, -0.015915, -0.004325, 0.011040, 0.011842, -0.001176, -0.011244, -0.007063, 0.004697, 0.009460, 0.002717, -0.006143, -0.006661, 0.000471, 0.005914, 0.003762, -0.002258, -0.004667, -0.001402, 0.002813, 0.003074, -0.000135, -0.002541, -0.001634, 0.000885, 0.001888, 0.000595, -0.001073, -0.001200, 0.000023, 0.000967, 0.000650, -0.000331, -0.000768, -0.000268, 0.000465};

__attribute__((aligned(16))) const float filtro_tom1[128] = {-0.000556, -0.000272, 0.000750, 0.000047, -0.000928, 0.000291, 0.001049, -0.000765, -0.001041, 0.001372, 0.000805, -0.002058, -0.000242, 0.002702, -0.000718, -0.003122, 0.002073, 0.003095, -0.003714, -0.002405, 0.005413, 0.000900, -0.006834, 0.001443, 0.007573, -0.004473, -0.007238, 0.007844, 0.005534, -0.011034, -0.002354, 0.013416, -0.002151, -0.014356, 0.007544, 0.013341, -0.013129, -0.010099, 0.018037, 0.004701, -0.021351, 0.002402, 0.022277, -0.010399, -0.020293, 0.018212, 0.015286, -0.024658, -0.007613, 0.028632, -0.001909, -0.029308, 0.012093, 0.026298, -0.021559, -0.019750, 0.028938, 0.010357, -0.033101, 0.000729, 0.033349, -0.012059, -0.029537, 0.022114, 0.022114, -0.029537, -0.012059, 0.033349, 0.000729, -0.033101, 0.010357, 0.028938, -0.019750, -0.021559, 0.026298, 0.012093, -0.029308, -0.001909, 0.028632, -0.007613, -0.024658, 0.015286, 0.018212, -0.020293, -0.010399, 0.022277, 0.002402, -0.021351, 0.004701, 0.018037, -0.010099, -0.013129, 0.013341, 0.007544, -0.014356, -0.002151, 0.013416, -0.002354, -0.011034, 0.005534, 0.007844, -0.007238, -0.004473, 0.007573, 0.001443, -0.006834, 0.000900, 0.005413, -0.002405, -0.003714, 0.003095, 0.002073, -0.003122, -0.000718, 0.002702, -0.000242, -0.002058, 0.000805, 0.001372, -0.001041, -0.000765, 0.001049, 0.000291, -0.000928, 0.000047, 0.000750, -0.000272, -0.000556};

__attribute__((aligned(16))) const float filtro_tom2[128] = {-0.000221, 0.000657, -0.000698, 0.000244, 0.000486, -0.001026, 0.000936, -0.000118, -0.001010, 0.001679, -0.001256, -0.000240, 0.001962, -0.002654, 0.001519, 0.001053, -0.003471, 0.003872, -0.001477, -0.002552, 0.005566, -0.005110, 0.000816, 0.004903, -0.008117, 0.006023, 0.000775, -0.008133, 0.010824, -0.006195, -0.003513, 0.012077, -0.013242, 0.005223, 0.007443, -0.016368, 0.014848, -0.002820, -0.012385, 0.020470, -0.015141, -0.001098, 0.017925, -0.023762, 0.013750, 0.006362, -0.023459, 0.025643, -0.010533, -0.012536, 0.028282, -0.025659, 0.005628, 0.018976, -0.031713, 0.023598, 0.000540, -0.024924, 0.033217, -0.019550, -0.007313, 0.029635, -0.032514, 0.013909, 0.013909, -0.032514, 0.029635, -0.007313, -0.019550, 0.033217, -0.024924, 0.000540, 0.023598, -0.031713, 0.018976, 0.005628, -0.025659, 0.028282, -0.012536, -0.010533, 0.025643, -0.023459, 0.006362, 0.013750, -0.023762, 0.017925, -0.001098, -0.015141, 0.020470, -0.012385, -0.002820, 0.014848, -0.016368, 0.007443, 0.005223, -0.013242, 0.012077, -0.003513, -0.006195, 0.010824, -0.008133, 0.000775, 0.006023, -0.008117, 0.004903, 0.000816, -0.005110, 0.005566, -0.002552, -0.001477, 0.003872, -0.003471, 0.001053, 0.001519, -0.002654, 0.001962, -0.000240, -0.001256, 0.001679, -0.001010, -0.000118, 0.000936, -0.001026, 0.000486, 0.000244, -0.000698, 0.000657, -0.000221};
/* *************************************************************** * Function prototypes 
 *****************************************************************/
 static void continuous_adc_init(adc_channel_t *channel, uint8_t channel_num, adc_continuous_handle_t *out_handle);
 static bool s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data);
 static void pv_processor_task(void *pvParam);

/******************************************************************* * The main task 
 *******************************************************************/
void app_main(void)
{
    esp_err_t ret;          
    esp_err_t parse_ret;    
    uint32_t ret_num = 0;   
    uint32_t sb_count = 0;      
    uint32_t num_parsed_samples = 0;    
    
    adc_continuous_evt_cbs_t cbs;      
    adc_continuous_handle_t handle = NULL;           

    float * sound_samp_buf_ADC;   
    
    memset(result, 0x00, MICEX_ADC_FRAME_SIZE);      
    sound_samp_buf_ADC = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);     

    s_task_handle = xTaskGetCurrentTaskHandle();    

    cbs.on_conv_done = s_conv_done_cb;      
    cbs.on_pool_ovf = NULL;                  

    esp_log_level_set(TAG,ESP_LOG_DEBUG);

    /* Configuração do GPIO para o LED (Requisito do Guião) */
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0); // Inicia desligado (porta fechada)

    XQ = xQueueCreate(1, sizeof(float)*MICEX_SOUND_SAMPLES_BUF_SIZE); 
    xTaskCreate(pv_processor_task, "Processor", PROCESSOR_TASK_STACK_SIZE, NULL, PROCESSOR_TASK_PRIORITY, NULL );

    continuous_adc_init(channel, sizeof(channel) / sizeof(adc_channel_t), &handle); 
    ESP_ERROR_CHECK(adc_continuous_register_event_callbacks(handle, &cbs, NULL));   
    ESP_ERROR_CHECK(adc_continuous_start(handle));                                  

    while (1) {        
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY); 

        while (1) {
            ret = adc_continuous_read(handle, result, MICEX_ADC_FRAME_SIZE, &ret_num, 0);
            if (ret == ESP_OK) {
                parse_ret = adc_continuous_parse_data(handle, result, ret_num, parsed_data, &num_parsed_samples);
                if (parse_ret == ESP_OK) {
                    
                    for (int i = 0; i < num_parsed_samples; i++) {
                        sound_samp_buf_ADC[sb_count] = (float) parsed_data[i].raw_data;                           
                        sb_count+=1;
                        if(sb_count == MICEX_SOUND_SAMPLES_BUF_SIZE) { 
                            xQueueSend(XQ,(void *)sound_samp_buf_ADC,0);     
                            sb_count = 0;
                        }
                    }

                } else {
                    ESP_LOGE(TAG, "Data parsing failed: %s", esp_err_to_name(parse_ret));
                }
                vTaskDelay(1);
            } else if (ret == ESP_ERR_TIMEOUT) {
                break;
            }
        }
    }

    ESP_ERROR_CHECK(adc_continuous_stop(handle));
    ESP_ERROR_CHECK(adc_continuous_deinit(handle));
}

/************************************************************************************************
 * Task activated when there is a full buffer of sound samples data available
 ************************************************************************************************/
void pv_processor_task(void *pvParam)
{
    float * sound_samp_buf_proc;           
    sound_samp_buf_proc = heap_caps_malloc(sizeof(float) * MICEX_SOUND_SAMPLES_BUF_SIZE, MALLOC_CAP_DMA);         
    
    // Variáveis da Máquina de Estados (FSM)
    int tentativa[4];
    int contador = 0;
    
    // NOVA VARIÁVEL: A "memória" do último som
    int ultimo_tom = -1; 
    
    // O teu threshold ajustado 
    float threshold = 500.0; 

    for(;;) {
        // Aguarda por novos dados do ADC
        if (xQueueReceive(XQ, (void *)sound_samp_buf_proc, portMAX_DELAY) == pdTRUE) {
            
            // ---------------------------------------------------------
            // >> ALTERAÇÃO 1: REMOVER A COMPONENTE DC (Centrar no zero)
            // ---------------------------------------------------------
            float media = 0;
            for(int i=0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
                media += sound_samp_buf_proc[i];
            }
            media = media / MICEX_SOUND_SAMPLES_BUF_SIZE;
            
            for(int i=0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) {
                sound_samp_buf_proc[i] -= media; // Puxa a onda para o zero!
            }
            // ---------------------------------------------------------

            /* 1. Aplicação dos Filtros FIR via Convolução */
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom0, 128, output_tom0);
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom1, 128, output_tom1);
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom2, 128, output_tom2);

            /* 2. Cálculo da Energia (Soma dos quadrados) */
            float energia0 = 0, energia1 = 0, energia2 = 0;
            for (int i = 0; i < CONV_OUT_SIZE; i++) {
                energia0 += output_tom0[i] * output_tom0[i];
                energia1 += output_tom1[i] * output_tom1[i];
                energia2 += output_tom2[i] * output_tom2[i];
            }

            // ---------------------------------------------------------
            // >> ALTERAÇÃO 2: IMPRIMIR AS ENERGIAS REAIS (Para descobrires o threshold)
            // ---------------------------------------------------------
            // Podes comentar esta linha mais tarde quando tudo funcionar
            ESP_LOGI(TAG, "Leitura: E0:%.0f | E1:%.0f | E2:%.0f", energia0, energia1, energia2);
            // ---------------------------------------------------------

            /* 3. Deteção do Tom */
            int tom_detetado = -1;
            if (energia0 > energia1 && energia0 > energia2 && energia0 > threshold) tom_detetado = 0;
            else if (energia1 > energia0 && energia1 > energia2 && energia1 > threshold) tom_detetado = 1;
            else if (energia2 > energia0 && energia2 > energia1 && energia2 > threshold) tom_detetado = 2;

            /* =========================================================
               4. LÓGICA DE "QUALQUER DURAÇÃO" (Edge Detection)
               ========================================================= */
            if (tom_detetado != -1) {
                // SÓ avança se o som for DIFERENTE do som que já estava a tocar
                if (tom_detetado != ultimo_tom) {
                    ultimo_tom = tom_detetado; // Guarda na memória
                    
                    ESP_LOGI(TAG, "NOVO Tom Registado: %d (Energia: E0:%.0f | E1:%.0f | E2:%.0f)", 
                             tom_detetado, energia0, energia1, energia2);
                    
                    tentativa[contador++] = tom_detetado;
                    
                    // Verifica se já temos 4 toques
                    if (contador == 4) {
                        if (memcmp(tentativa, SEQ_ABRIR, sizeof(SEQ_ABRIR)) == 0) {
                            ESP_LOGI(TAG, "Acesso Concedido: ABRIR");
                            gpio_set_level(LED_PIN, 1); 
                        } 
                        else if (memcmp(tentativa, SEQ_FECHAR, sizeof(SEQ_FECHAR)) == 0) {
                            ESP_LOGI(TAG, "Acesso Concedido: FECHAR");
                            gpio_set_level(LED_PIN, 0); 
                        } 
                        else {
                            ESP_LOGW(TAG, "Sequência Errada! A piscar LED...");
                            for (int i = 0; i < 10; i++) { 
                                gpio_set_level(LED_PIN, 1);
                                vTaskDelay(pdMS_TO_TICKS(150));
                                gpio_set_level(LED_PIN, 0);
                                vTaskDelay(pdMS_TO_TICKS(150));
                            }
                        }
                        contador = 0; // Prepara para nova tentativa
                    }
                }
            } else {
                // SE ESTIVER EM SILÊNCIO:
                // Apaga a memória. Assim, o utilizador pode repetir o mesmo Tom.
                ultimo_tom = -1;
            }
            
            // Limpa a fila de processamento para evitar acumulação de buffer antigo
            xQueueReset(XQ);
        }
    }
}

static bool IRAM_ATTR s_conv_done_cb(adc_continuous_handle_t handle, const adc_continuous_evt_data_t *edata, void *user_data)
{
    BaseType_t mustYield = pdFALSE;
    vTaskNotifyGiveFromISR(s_task_handle, &mustYield);
    return (mustYield == pdTRUE);
}

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
    }
    dig_cfg.adc_pattern = adc_pattern;
    ESP_ERROR_CHECK(adc_continuous_config(handle, &dig_cfg));

    *out_handle = handle;
}