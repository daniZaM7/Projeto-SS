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

#define MICEX_ADC_FRAME_SIZE             128                
#define MICEX_ADC_BUF_SIZE               (4 * MICEX_ADC_FRAME_SIZE)    
#define MICEX_ADC_SAMPLE_FREQ            20000 // Frequência de amostragem definida para os filtros

#define MICEX_SOUND_SAMPLES_BUF_SIZE     128
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

/* Filtros FIR (N=128, Fs=20000) */
__attribute__((aligned(16))) const float filtro_tom0[128] = {-0.000232, -0.001121, -0.001813, -0.002169, -0.002088, -0.001535, -0.000559, 0.000696, 0.001997, 0.003051, 0.003566, 0.003316, 0.002216, 0.000376, -0.001893, -0.004109, -0.005719, -0.006223, -0.005312, -0.002979, 0.000430, 0.004250, 0.007614, 0.009649, 0.009683, 0.007448, 0.003193, -0.002316, -0.007919, -0.012307, -0.014318, -0.013237, -0.009014, -0.002335, 0.005471, 0.012697, 0.017629, 0.018952, 0.016092, 0.009403, 0.000150, -0.009729, -0.018027, -0.022774, -0.022699, -0.017566, -0.008282, 0.003262, 0.014574, 0.023114, 0.026867, 0.024829, 0.017263, 0.005687, -0.007440, -0.019248, -0.027094, -0.029168, -0.024913, -0.015173, -0.002025, 0.011675, 0.022925, 0.029248, 0.029248, 0.022925, 0.011675, -0.002025, -0.015173, -0.024913, -0.029168, -0.027094, -0.019248, -0.007440, 0.005687, 0.017263, 0.024829, 0.026867, 0.023114, 0.014574, 0.003262, -0.008282, -0.017566, -0.022699, -0.022774, -0.018027, -0.009729, 0.000150, 0.009403, 0.016092, 0.018952, 0.017629, 0.012697, 0.005471, -0.002335, -0.009014, -0.013237, -0.014318, -0.012307, -0.007919, -0.002316, 0.003193, 0.007448, 0.009683, 0.009649, 0.007614, 0.004250, 0.000430, -0.002979, -0.005312, -0.006223, -0.005719, -0.004109, -0.001893, 0.000376, 0.002216, 0.003316, 0.003566, 0.003051, 0.001997, 0.000696, -0.000559, -0.001535, -0.002088, -0.002169, -0.001813, -0.001121, -0.000232};

__attribute__((aligned(16))) const float filtro_tom1[128] = {0.002000, 0.001701, 0.000605, -0.000849, -0.002034, -0.002372, -0.001590, 0.000103, 0.002025, 0.003258, 0.003055, 0.001254, -0.001523, -0.004032, -0.004933, -0.003473, 0.000017, 0.004094, 0.006767, 0.006440, 0.002809, -0.002783, -0.007787, -0.009590, -0.006846, -0.000339, 0.007144, 0.011985, 0.011463, 0.005238, -0.004216, -0.012562, -0.015576, -0.011274, -0.001104, 0.010467, 0.017906, 0.017264, 0.008243, -0.005384, -0.017337, -0.021732, -0.015984, -0.002262, 0.013284, 0.023295, 0.022704, 0.011300, -0.005949, -0.021064, -0.026774, -0.020033, -0.003633, 0.014951, 0.026989, 0.026635, 0.013780, -0.005768, -0.022929, -0.029608, -0.022543, -0.004912, 0.015113, 0.028184, 0.028184, 0.015113, -0.004912, -0.022543, -0.029608, -0.022929, -0.005768, 0.013780, 0.026635, 0.026989, 0.014951, -0.003633, -0.020033, -0.026774, -0.021064, -0.005949, 0.011300, 0.022704, 0.023295, 0.013284, -0.002262, -0.015984, -0.021732, -0.017337, -0.005384, 0.008243, 0.017264, 0.017906, 0.010467, -0.001104, -0.011274, -0.015576, -0.012562, -0.004216, 0.005238, 0.011463, 0.011985, 0.007144, -0.000339, -0.006846, -0.009590, -0.007787, -0.002783, 0.002809, 0.006440, 0.006767, 0.004094, 0.000017, -0.003473, -0.004933, -0.004032, -0.001523, 0.001254, 0.003055, 0.003258, 0.002025, 0.000103, -0.001590, -0.002372, -0.002034, -0.000849, 0.000605, 0.001701, 0.002000};

__attribute__((aligned(16))) const float filtro_tom2[128] = {-0.000671, 0.001131, 0.002102, 0.001440, -0.000465, -0.002206, -0.002330, -0.000495, 0.002067, 0.003298, 0.001902, -0.001397, -0.004095, -0.003739, -0.000079, 0.004304, 0.005737, 0.002469, -0.003463, -0.007376, -0.005587, 0.001236, 0.007994, 0.008911, 0.002410, -0.006964, -0.011638, -0.007097, 0.003909, 0.012857, 0.012028, 0.001111, -0.011786, -0.016115, -0.007509, 0.008022, 0.018207, 0.014218, -0.001727, -0.017380, -0.019875, -0.006312, 0.013212, 0.023109, 0.014777, -0.005961, -0.022871, -0.022060, -0.003409, 0.018727, 0.026605, 0.013369, -0.011013, -0.027268, -0.022120, 0.000833, 0.023601, 0.027971, 0.010135, -0.016002, -0.029713, -0.019966, 0.005661, 0.026901, 0.026901, 0.005661, -0.019966, -0.029713, -0.016002, 0.010135, 0.027971, 0.023601, 0.000833, -0.022120, -0.027268, -0.011013, 0.013369, 0.026605, 0.018727, -0.003409, -0.022060, -0.022871, -0.005961, 0.014777, 0.023109, 0.013212, -0.006312, -0.019875, -0.017380, -0.001727, 0.014218, 0.018207, 0.008022, -0.007509, -0.016115, -0.011786, 0.001111, 0.012028, 0.012857, 0.003909, -0.007097, -0.011638, -0.006964, 0.002410, 0.008911, 0.007994, 0.001236, -0.005587, -0.007376, -0.003463, 0.002469, 0.005737, 0.004304, -0.000079, -0.003739, -0.004095, -0.001397, 0.001902, 0.003298, 0.002067, -0.000495, -0.002330, -0.002206, -0.000465, 0.001440, 0.002102, 0.001131, -0.000671};
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
    
    // Variáveis da Máquina de Estados (Cofre)
    int tentativa[4];
    int contador = 0;
    int ultimo_tom = -1; 
    
    // Variáveis de Blindagem Temporal
    int tom_em_validacao = -1;
    int contagem_validacao = 0;
    
    // ==========================================
    // AFINAÇÕES DO ENGENHEIRO
    // ==========================================
    const int FRAMES_NECESSARIOS = 5; // Exige som puro durante ~16ms (5 * 3.2ms)
    float threshold = 50.0;          // Limiar de energia com buffer de 64
    float margem = 1.0;               // O vencedor tem de ser 20% mais forte que os outros
    // ==========================================

    for(;;) {
        // Aguarda por novos dados do ADC
        if (xQueueReceive(XQ, (void *)sound_samp_buf_proc, portMAX_DELAY) == pdTRUE) {
            
            // 1. Remover DC Offset (Centrar onda no zero)
            float media = 0;
            for(int i=0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) media += sound_samp_buf_proc[i];
            media = media / MICEX_SOUND_SAMPLES_BUF_SIZE;
            for(int i=0; i < MICEX_SOUND_SAMPLES_BUF_SIZE; i++) sound_samp_buf_proc[i] -= media;

            // 2. Aplicação dos Filtros FIR
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom0, 128, output_tom0);
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom1, 128, output_tom1);
            dsps_conv_f32(sound_samp_buf_proc, MICEX_SOUND_SAMPLES_BUF_SIZE, filtro_tom2, 128, output_tom2);

            // 3. Cálculo da Energia
            float energia0 = 0, energia1 = 0, energia2 = 0;
            for (int i = 0; i < CONV_OUT_SIZE; i++) {
                energia0 += output_tom0[i] * output_tom0[i];
                energia1 += output_tom1[i] * output_tom1[i];
                energia2 += output_tom2[i] * output_tom2[i];
            }

            // Opcional: Descomentar APENAS para calibrar, se comentado não bloqueia o terminal
            
            if (energia0 > 50.0 || energia1 > 50.0 || energia2 > 50.0) {
                ESP_LOGI(TAG, "Energia -> E0:%.0f | E1:%.0f | E2:%.0f", energia0, energia1, energia2);
            }
            

            // 4. Deteção Bruta (Teste de Pureza com Margem)
            int tom_candidato = -1;
            if (energia0 > (energia1 * margem) && energia0 > (energia2 * margem) && energia0 > threshold) tom_candidato = 0;
            else if (energia1 > (energia0 * margem) && energia1 > (energia2 * margem) && energia1 > threshold) tom_candidato = 1;
            else if (energia2 > (energia0 * margem) && energia2 > (energia1 * margem) && energia2 > threshold) tom_candidato = 2;

            // 5. Blindagem Temporal (Validação contínua)
            if (tom_candidato != -1) {
                if (tom_candidato == tom_em_validacao) {
                    contagem_validacao++;
                } else {
                    tom_em_validacao = tom_candidato;
                    contagem_validacao = 1;
                }

                // SÓ AVANÇA SE ATINGIR OS FRAMES EXIGIDOS
                if (contagem_validacao >= FRAMES_NECESSARIOS) {
                    
                    if (tom_candidato != ultimo_tom) {
                        ultimo_tom = tom_candidato; // Bloqueia repetições seguidas do mesmo tom
                        
                        ESP_LOGI(TAG, "+++ TOM VALIDADO E REGISTADO: %d +++", tom_candidato);
                        
                        tentativa[contador++] = tom_candidato;
                        
                        // Lógica da Password do Cofre
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
                                    gpio_set_level(LED_PIN, 1); vTaskDelay(pdMS_TO_TICKS(150));
                                    gpio_set_level(LED_PIN, 0); vTaskDelay(pdMS_TO_TICKS(150));
                                }
                            }
                            contador = 0;
                        }
                    }
                }
            } else {
                // Silêncio ou Ruído: Reset à validação temporal
                contagem_validacao = 0; 
                tom_em_validacao = -1;
                
                // Se o silêncio for genuíno, esquece o último tom para permitir repetições (ex: Tom 0 seguido de Tom 0)
                if (energia0 < (threshold/2) && energia1 < (threshold/2) && energia2 < (threshold/2)) {
                    ultimo_tom = -1;
                }
            }
            
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