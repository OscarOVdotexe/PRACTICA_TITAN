/*--------------------------PRACTICA PARA CONTROL DE MOTOR TITAN-----------------------*/
#include <stdio.h>
#include <math.h>
/*--------------------------lIBRERIAS DEL ESP-----------------------*/
#include "esp_log.h"
#include "esp_log_internal.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_spi_flash.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"
#include "freeRTOS/timers.h"

/*--------------------------APARTADO DE PARAMETROS-----------------------*/
#define ON 0
#define OFF 1
#define IZ 0
#define DER 1
////////////////////////////////////////////////////////////////////////////////////
#define PWM_GPIO_DER 18                  // GPIO de salida del PWM
#define PWM_GPIO_IZ 21                   // GPIO de salida del PWM
#define PWM_FREQUENCY 5000               // Frecuencia en Hz (5 kHz)
#define PWM_RESOLUTION LEDC_TIMER_12_BIT // Resolución del PWM (12 bits) son 4096 valores
////////////////////////////////////////////////////////////////////////////////////////////
#define boton_start_stop 4 // GPIO2 BOTÓN DE INICIO Y PARO
#define LimitSA 5          // GPIO15 LIMIT SWITCH ABIERTO
#define LimitSC 6          // GPIO13 LIMIT SWITCH CERRADO
//////////////////////////////////////////////////////////////////////////////////////////
/// ESTADOS DEFINIDOS
#define ESTADO_INICIO 0        // ESTADO INICIAL
#define ESTADO_CERRANDO 1      // ESTADO DE CERRANDO
#define ESTADO_CERRADO 2       // ESTADO DE CERRADO
#define ESTADO_ABRIENDO 3      // ESTADO DE ABRIENDO
#define ESTADO_ABIERTO 4       // ESTADO DE ABIERTO
#define ESTADO_STOP 5          //    ESTADO DE PARO para el motor
#define ESTADO_ERROR_HANDLER 6 // ESTADO DE MANEJO DE ERRORES

/*--------------------------ESTRUCTURAS-----------------------*/

struct global
{
    unsigned int BTN_SS;  // START/STOP BUTTON
    unsigned int LSA;     // limit switch de los portones Abierto
    unsigned int LSC;     // limit switch de los portones Cerrado
    unsigned int DATA_OK; // flag de datos correctos
} data;

struct global_MOTOR
{
    unsigned int DIRECCION; // direccion del motor
} motorcontrol;

/*--------------------------APARTADO DE VARIABLES GLOBALES-----------------------*/
int ESTADO_ACTUAL = ESTADO_INICIO;
int ESTADO_SIGUIENTE = ESTADO_INICIO;
int ESTADO_ANTERIOR = ESTADO_INICIO;

/*--------------------------PROTOTIPO DE FUNCIONES ESTADO-----------------------*/

void INICIALIZAR();
void CERRANDO();
void CERRADO();
void ABRIENDO();
void ABIERTO();
void STOP();
void handle_error();

/*--------------------------PROTOTIPO DE FUNCIONES DE CONFIGURACON-----------------------*/
esp_err_t timer_config();
esp_err_t adc_config();
esp_err_t gpio_configure();
esp_err_t PWM_CONFIG();
void vTimerCallback();
esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_cfg_t *init_config, adc_oneshot_unit_handle_t *ret_unit);
adc_oneshot_unit_handle_t adc1_handle;
adc_oneshot_chan_cfg_t config;
/*--------------------------EMPIEZA-----------------------*/

struct struct_adc // estructura de variables para el ADC
{
    long acc;         // acumulador de señal
    int n;            // contador de muestras
    int Vrms;         // valor eficaz
    int READ_ADC;     // VARIABLE LECTURA ADC
    int escalamiento; // relacion entre ADC y PWM

} variables_adc;

void vTimerCallback(TimerHandle_t pxTimer)
{
    adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &variables_adc.READ_ADC);
    variables_adc.READ_ADC = variables_adc.READ_ADC * variables_adc.READ_ADC;
    variables_adc.acc += variables_adc.READ_ADC;
    variables_adc.n++;

    if (variables_adc.n == 20)
    {
        variables_adc.Vrms = sqrt(variables_adc.acc) / variables_adc.n;
        variables_adc.acc = 0;
        variables_adc.n = 0;
        variables_adc.READ_ADC = 0;
    }

    variables_adc.escalamiento = (variables_adc.Vrms / 4095) * 4095;

    if (motorcontrol.DIRECCION == IZ && ESTADO_ACTUAL == CERRANDO && ESTADO_ACTUAL != STOP)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);

        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, variables_adc.escalamiento);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    else if (motorcontrol.DIRECCION == DER && ESTADO_ACTUAL == ABRIENDO && ESTADO_ACTUAL != STOP)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, variables_adc.escalamiento);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }
    else if (ESTADO_ACTUAL == STOP)
    {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    }

    //////////////////////////////////////
    return;
}

void app_main(void)
{

    esp_log_level_set("*", ESP_LOG_INFO);
    ESP_LOGI("MAIN", "INICIO DE PROGRAMA");
    gpio_configure();
    timer_config();
    adc_config();
    PWM_CONFIG();

    while (1)
    {
        if (ESTADO_ACTUAL == ESTADO_INICIO)
        {
            INICIALIZAR();
        }
        else if (ESTADO_ACTUAL == ESTADO_CERRANDO)
        {
            CERRANDO();
        }
        else if (ESTADO_ACTUAL == ESTADO_CERRADO)
        {
            CERRADO();
        }
        else if (ESTADO_ACTUAL == ESTADO_ABRIENDO)
        {
            ABRIENDO();
        }
        else if (ESTADO_ACTUAL == ESTADO_ABIERTO)
        {
            ABIERTO();
        }
        else if (ESTADO_ACTUAL == ESTADO_STOP)
        {
            STOP();
        }
        else if (ESTADO_ACTUAL == ESTADO_ERROR_HANDLER)
        {
            handle_error();
        }
    }
}

void INICIALIZAR()
{
    ////////////////////////////////////////
    // INICIALIZACION DE VARIABLES GLOBALES

    data.BTN_SS = gpio_get_level(boton_start_stop);
    data.LSA = gpio_get_level(LimitSA);
    data.LSC = gpio_get_level(LimitSC);
    data.DATA_OK = 0;
    ESTADO_ACTUAL = ESTADO_CERRANDO;
    ESTADO_SIGUIENTE = ESTADO_CERRANDO;
    ESTADO_ANTERIOR = ESTADO_INICIO;
    variables_adc.acc = 0;
    variables_adc.n = 0;
    variables_adc.READ_ADC = 0;
    variables_adc.Vrms = 0;
    variables_adc.escalamiento = 0;

    //////////////////////////////////////////
    // INICIALIZACION DE GPIO

    // Verificación de salidas y entradas.
    while (1)
    {

        if (data.LSA == ON)
        {
            // se pondra la direccion de giro y luego de 100 milisegundo volvera a cambiar el giro y cuando toque el limit switch otra vez se detiene
            // si pasan mas de 3 veces y no se activa el limit switch se activa el error handler
        }
        else if (data.LSC == ON)
        {
            // se pondra la direccion de giro y luego de 100 milisegundo volvera a cambiar el giro y cuando toque el limit switch otra vez se detiene
            // si pasan mas de 3 veces y no se activa el limit switch se activa el error handler
        }
    }

    return;
}

void CERRANDO()
{
    data.BTN_SS = OFF;
    motorcontrol.DIRECCION = IZ;
    ESTADO_ANTERIOR = ESTADO_INICIO;
    ESP_LOGI("CERRANDO", "Cerrando porton");
    while (data.BTN_SS == ON)
    {
        ESP_LOGI("CERRANDO", "NO PRESS BUTTON");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while (1)
    {
        data.BTN_SS = gpio_get_level(boton_start_stop); // se verifica si se presiona el boton de inicio
        data.LSA = gpio_get_level(LimitSA);             // se verifica si se activa el limit switch de abierto
        data.LSC = gpio_get_level(LimitSC);             //  se verifica si se activa el limit switch de cerrado

        if (data.LSC == ON)
        {
            ESTADO_ACTUAL = ESTADO_CERRADO;
            return;
        }
        else if (data.BTN_SS == ON)
        {
            ESTADO_ACTUAL = ESTADO_STOP;
            ESTADO_SIGUIENTE = ESTADO_CERRANDO;
            return;
        }
    }
}

void CERRADO()
{
    motorcontrol.DIRECCION = IZ;
    data.BTN_SS = OFF;
    ESP_LOGI("CERRADO", "Porton cerrado");
    while (data.BTN_SS == ON)
    {
        ESP_LOGI("CERRANDO", "NO PRESS BUTTON");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    while (1)
    {
        data.BTN_SS = gpio_get_level(boton_start_stop); // se verifica si se presiona el boton de inicio
        data.LSA = gpio_get_level(LimitSA);             // se verifica si se activa el limit switch de abierto
        data.LSC = gpio_get_level(LimitSC);             //  se verifica si se activa el limit switch de cerrado
        if (data.BTN_SS == ON)
        {
            ESTADO_ACTUAL = ESTADO_ABRIENDO;
            return;
        }
    }
}

void ABRIENDO()
{
    data.BTN_SS = OFF;
    motorcontrol.DIRECCION = DER;
    ESTADO_ANTERIOR = ESTADO_CERRADO;
    ESP_LOGI("ABRIENDO", "Abriendo porton");

    while (data.BTN_SS == ON)
    {
        ESP_LOGI("CERRANDO", "NO PRESS BUTTON");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while (1)
    {
        data.BTN_SS = gpio_get_level(boton_start_stop); // se verifica si se presiona el boton de inicio
        data.LSA = gpio_get_level(LimitSA);             // se verifica si se activa el limit switch de abierto
        data.LSC = gpio_get_level(LimitSC);             //  se verifica si se activa el limit switch de cerrado
        if (data.LSA == ON)
        {
            ESTADO_ACTUAL = ESTADO_ABIERTO;
            return;
        }
        else if (data.BTN_SS == ON)
        {
            ESTADO_ACTUAL = ESTADO_STOP;
            ESTADO_SIGUIENTE = ESTADO_ABRIENDO;
            return;
        }
    }
}

void ABIERTO()
{

    motorcontrol.DIRECCION = DER;
    data.BTN_SS = OFF;
    ESP_LOGI("ABIERTO", "Porton abierto");

    while (data.BTN_SS == ON)
    {
        ESP_LOGI("CERRANDO", "NO PRESS BUTTON");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

    while (1)
    {
        data.BTN_SS = gpio_get_level( boton_start_stop); // se verifica si se presiona el boton de inicio
        data.LSA = gpio_get_level( LimitSA);             // se verifica si se activa el limit switch de abierto
        data.LSC = gpio_get_level( LimitSC);             //  se verifica si se activa el limit switch de cerrado

        if (data.BTN_SS == ON)
        {
            ESTADO_ACTUAL = ESTADO_CERRANDO;
            return;
        }
    }
}

void STOP()
{
    data.BTN_SS = OFF;
    ESP_LOGI("STOP", "Esperando a que se presione el boton de inicio");
    while (data.BTN_SS == ON)
    {
        ESP_LOGI("CERRANDO", "NO PRESS BUTTON");
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    while (1)
    {
        data.BTN_SS = gpio_get_level( boton_start_stop); // se verifica si se presiona el boton de inicio

        if (data.BTN_SS == ON && ESTADO_ANTERIOR == ESTADO_CERRANDO)
        {
            ESTADO_ACTUAL = ESTADO_SIGUIENTE;
            return;
        }
        else if (data.BTN_SS == ON && ESTADO_ANTERIOR == ESTADO_ABRIENDO)
        {
            ESTADO_ACTUAL = ESTADO_SIGUIENTE;
            return;
        }
        else
        {
            ESTADO_ACTUAL = ESTADO_STOP;
            return;
        }
    }
}

void handle_error()
{
    data.BTN_SS = OFF;
    ESP_LOGE("ERROR_HANDLER", "Error en el sistema");
    while (1)
    {

        data.BTN_SS = gpio_get_level(boton_start_stop); // se verifica si se presiona el boton de inicio
        ///////////POSIBLES ERRORES
        ///////////1. NO SE ACTIVAN LOS LIMIT SWITCH
        ///////////2. NO SE ACTIVA EL BOTON DE INICIO
        ///////////3. NO SE ACTIVA EL BOTON DE STOP
        ///////////4. NO SE ACTIVA EL ADC
        ///////////5. NO SE ACTIVA EL PWM
        ///////////6. NO SE ACTIVA EL GPIO
        ///////////7. NO SE ACTIVA EL MOTOR
        ///////////SE DEBE DE ACTIVAR EL BOTON DE STOP PARA PODER CONTINUAR
        if (data.BTN_SS == ON)
        {
            ESTADO_ACTUAL = ESTADO_INICIO;
            return;
        }
    }
}



esp_err_t adc_config()
{
    
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_new_unit(&init_config1, &adc1_handle);

    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_12,
        .atten = ADC_ATTEN_DB_0,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config);

    return ESP_OK;
}

esp_err_t gpio_configure()
{
    //////////////////pin configuration/////////////////////
    gpio_set_direction( boton_start_stop,  GPIO_MODE_INPUT);
    gpio_set_direction( LimitSA,  GPIO_MODE_INPUT);
    gpio_set_direction( LimitSC,  GPIO_MODE_INPUT);

    return ESP_OK;
}

TimerHandle_t xTimers; // Variable que guarda los datos del timer.
int interval = 1;
int timerId = 1;

esp_err_t timer_config()
{
    xTimers = xTimerCreate("Timer",         // Just a text name, not used by the kernel.
                              pdMS_TO_TICKS(interval), // The timer period in ticks.
                              pdTRUE,          // The timers will auto-reload themselves when they expire.
                              (void *)timerId,       // Assign each timer a unique id equal to its array index.
                              vTimerCallback   // Each timer calls the same callback when it expires.
    );

    if (xTimers == NULL)
    {
        ESP_LOGE("TIMER", "No se pudo crear el timer");
    }
    else
    {
        // Start the timer.  No block time is specified, and even if one was
        // it would be ignored because the scheduler has not yet been
        // started.
        if (xTimerStart(xTimers, 0) != pdPASS)
        {
            ESP_LOGE("TIMER", "No se pudo iniciar el timer");
        }
    }

    return ESP_OK;
}

// Configuración del canal PWM
esp_err_t PWM_CONFIG()
{
    ledc_timer_config_t ledc_timer1 = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer1);

    ledc_timer_config_t ledc_timer2 = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQUENCY,
        .clk_cfg = LEDC_AUTO_CLK};
    ledc_timer_config(&ledc_timer2);

    // Configuración del canal PWM
    ledc_channel_config_t ledc_channel1 = {
        .gpio_num = PWM_GPIO_IZ,
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // INICIALIZA EN 0
        .hpoint = 0};

    ledc_channel_config(&ledc_channel1);

    ledc_channel_config_t ledc_channel2 = {
        .gpio_num = PWM_GPIO_DER,
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0, // INICIALIZA EN 0
        .hpoint = 0};

    ledc_channel_config(&ledc_channel2);

    return ESP_OK;
}

/*--------------------------FIN-----------------------*/