// Librerías originales
#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "driver/uart.h"
#include "esp_bt.h"
#include "nvs_flash.h"
#include "esp_bt_device.h"
#include "esp_gap_ble_api.h"
#include "esp_gattc_api.h"
#include "esp_gatt_defs.h"
#include "esp_bt_main.h"
#include "esp_system.h"
#include "esp_gatt_common_api.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"

// Librerías adicionales
#include <time.h>
#include <sys/time.h>
#include "unistd.h"
#include <stdint.h>

// Parámetros generales
#define GATTC_TAG                         "GATTC_SPP_DEMO"                                    // Nombre mensajes
#define PROFILE_NUM                       1                                                   // Número perfil
#define PROFILE_APP_ID                    0                                                   // id de la app del perfil
#define BT_BD_ADDR_STR                    "%02x:%02x:%02x:%02x:%02x:%02x"                     // cadena con la dirección
#define BT_BD_ADDR_HEX(addr)              addr[0],addr[1],addr[2],addr[3],addr[4],addr[5]     // dirección hexadecimal
#define ESP_GATT_SPP_SERVICE_UUID         0xABF0                                              // UUID
#define SCAN_ALL_THE_TIME                 0                                                   // Scan constante
#define INITIAL_SCAN_EVT_NUM_ESTIMATE     100                                                 // Tamaño inicial estimado del número de eventos de SCAN

// Instancia del perfil GATT
struct gattc_profile_inst {
    esp_gattc_cb_t gattc_cb;            // Callback
    uint16_t gattc_if;                  // Interfaz del Gatt client 
    uint16_t app_id;                    // ID de la app
    uint16_t conn_id;                   // ID de la conexión
    uint16_t service_start_handle;      // Handle del comienzo de servicios
    uint16_t service_end_handle;        // Handle del final de servicios
    uint16_t char_handle;               // Handle de características
    esp_bd_addr_t remote_bda;           // Dirección del dispositivo bluetooth remoto (bluetooth device address)
};

// Índices de eventos
enum{
    SPP_IDX_SVC,                // Índice de servicios

    SPP_IDX_SPP_DATA_RECV_VAL,  // Índice de valor recibido

    SPP_IDX_SPP_DATA_NTY_VAL,   // Índice de valor de notificación
    SPP_IDX_SPP_DATA_NTF_CFG,   // Índice de notificación de configuración 

    SPP_IDX_SPP_COMMAND_VAL,    // Ínfice de valor de comando

    SPP_IDX_SPP_STATUS_VAL,     // Índice de valor de estado
    SPP_IDX_SPP_STATUS_CFG,     // Índice de estado de configuración

    SPP_IDX_NB,                 // Índice NB
};

// Declaración de funciones estáticas
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);                                            // Callback Generic Access Profile
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);                  // Callback Perfil GATT
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);   // Gestor de Eventos del perfil GATT

// Un perfil basado en Gatt, un app_id y un gattc_if. Este array almacena el gattc_if devuelto por ESP_GATTS_REG_EVT
static struct gattc_profile_inst gl_profile_tab[PROFILE_NUM] = {       
    [PROFILE_APP_ID] = {                            
        .gattc_cb = gattc_profile_event_handler,    // Gestor de eventos del perfil gatt del cliente en el callback
        .gattc_if = ESP_GATT_IF_NONE,               // No obtener la interfaz del cliente GATT, así que la incial es ESP_GATT_IF_NONE */
    },      
};

// Parámetros del escaneo
static esp_ble_scan_params_t ble_scan_params = {            
    .scan_type              = BLE_SCAN_TYPE_ACTIVE,         // Tipo de escaneo ACTIVO
    .own_addr_type          = BLE_ADDR_TYPE_PUBLIC,         // Tipo de dirección propia PUBLICA
    .scan_filter_policy     = BLE_SCAN_FILTER_ALLOW_ALL,    // Política de filtro de escaneo PERMITIR TODO
    .scan_interval          = 0x50,                         // Intervalo de escaneo 0x50 ms
    .scan_window            = 0x30,                         // Ventana de escaneo 0x30 ms
    .scan_duplicate         = BLE_SCAN_DUPLICATE_DISABLE    // Duplicado de escaneo DESABILITADO
};

// Parámetros generales
static const char device_name[] = "ESP_SPP_SERVER";     // Nombre del dispositivo
static bool is_connect = false;                         // Estado de conexión
static uint16_t spp_conn_id = 0;                        // ID conexión
static uint16_t spp_mtu_size = 23000;                   // MTU
static uint16_t cmd = 0;                                // CMD
static uint16_t spp_srv_start_handle = 0;               // Handle comienzo de servicios
static uint16_t spp_srv_end_handle = 0;                 // Handle fin de servicios
static uint16_t spp_gattc_if = 0xff;                    // Interfaz del perfil GATTC
static char * notify_value_p = NULL;                    // Puntero al valor de la notificación
static int notify_value_offset = 0;                     // Offset del valor de la notificación
static int notify_value_count = 0;                      // Cuenta del valor de la notificación
static uint16_t count = SPP_IDX_NB;                     // Cuenta   
static esp_gattc_db_elem_t *db = NULL;                  // DB
static esp_ble_gap_cb_param_t scan_rst;                 // Reseteo del escaneo 
static QueueHandle_t cmd_reg_queue = NULL;              // Cola de registro de comandos

// UUID del servicio
static esp_bt_uuid_t spp_service_uuid = {               // UUID servicio SPP
    .len  = ESP_UUID_LEN_16,                            // Longitud
    .uuid = {.uuid16 = ESP_GATT_SPP_SERVICE_UUID,},     // UUID
};

// Número mínimo de campos
int16_t n_min = 4;

// Funciones para la gestión de Timestamps
uint32_t millis() {
    clock_t ticks = clock();
    uint32_t milliseconds = (ticks * 1000) / CLOCKS_PER_SEC;
    return milliseconds;
}

void set_time(){
   struct timeval tv;
   struct tm *timeinfo;
   time_t rawtime = time(NULL);
   timeinfo = localtime(&rawtime);
   timeinfo->tm_year = 123; timeinfo->tm_mon = 0; timeinfo->tm_mday = 1; 
   timeinfo->tm_hour = 0; timeinfo->tm_min = 0; timeinfo->tm_sec = 0;           
   time_t newtime = mktime(timeinfo); tv.tv_sec = newtime; tv.tv_usec = 0; 
   settimeofday(&tv, NULL);
}

__int64_t get_time(void){
   struct timeval tv_now;
   gettimeofday(&tv_now, NULL);
   int64_t time_us = (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
   return(time_us);                
}

void format_time(int64_t time_us){      
    time_t time_sec = time_us / 1000000L; 
    struct tm *timeinfo; timeinfo = localtime(&time_sec); char formatted_time[30];
    strftime(formatted_time, sizeof(formatted_time), "%H:%M:%S", timeinfo);
    printf("%s:%lld ", formatted_time, time_us % 1000000L); 
}

void time_delay(__int64_t sent_timestamp, __int64_t dev_current_time){
    __int64_t delay = dev_current_time - sent_timestamp;
    __int64_t delay_sec = delay / 1000000L;  

    int seconds = (int)(delay_sec % 60);
    int microseconds = (int)(delay % 1000000L);

    printf("%02d:%06d ", seconds, microseconds);
}


void msg_log (esp_ble_gattc_cb_param_t * p_data, int64_t CURRENT_TIME) {

    __int64_t SENT_TSTMP  = ((int64_t*)p_data->notify.value)[0];
    __int64_t PACK_N      = ((int64_t*)p_data->notify.value)[1];
    __int64_t PACK_SIZE   = ((int64_t*)p_data->notify.value)[2];
    
    int8_t n_payload = PACK_SIZE / 8;

    printf("RX ");
    format_time(SENT_TSTMP);
    format_time(CURRENT_TIME);
    printf("%lld ", PACK_N);
    time_delay(SENT_TSTMP, CURRENT_TIME);
    printf("%lld ", PACK_SIZE);

    __int64_t *PAYLOAD = (__int64_t*)malloc(sizeof(__int64_t) * (n_payload-n_min));
    for(int i = 0; i < n_payload - n_min ; i++){ 
        PAYLOAD[i] = ((int64_t*)p_data->notify.value)[n_min+i]; 
        printf("%d %lld ", i+1, PAYLOAD[i]);
    } ;
    printf("\n");

}

// Manejador de eventos de notificación
static void notify_event_handler(esp_ble_gattc_cb_param_t * p_data)
{
    uint8_t handle = 0;
    handle = p_data->notify.handle;
    int64_t current_time = get_time();

    if(db == NULL) {
        ESP_LOGE(GATTC_TAG, " %s db is NULL\n", __func__);
        return;
    }

    if(handle == db[SPP_IDX_SPP_DATA_NTY_VAL].attribute_handle) {
        msg_log(p_data, current_time);
    } else if(handle == ((db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle)) {
        esp_log_buffer_char(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
        //TODO:server notify status characteristic
    }else{
        esp_log_buffer_char(GATTC_TAG, (char *)p_data->notify.value, p_data->notify.value_len);
    }
}

// Callback de limpiado de la DataBase de servicios Gatt
static void free_gattc_srv_db(void)
{
    is_connect = false;         // Estado desconectado
    spp_gattc_if = 0xff;        // Interfaz ff
    spp_conn_id = 0;            // ID 0
    spp_mtu_size = 23000;       // MTU 
    cmd = 0;                    // CMD
    spp_srv_start_handle = 0;   // handle comienzo 
    spp_srv_end_handle = 0;     // handle final
    notify_value_p = NULL;      // puntero valor de notificación
    notify_value_offset = 0;    // offset del valor de la notificación
    notify_value_count = 0;     // cuenta del valor de la notificación
    if(db){                     // si db
        free(db);               // limpiar db
        db = NULL;              // db = NULL
    }
}

typedef struct {

    __int8_t* data;
    size_t size;
    size_t capacity;

} SCAN_EVT_VEC_t;

void initVector(SCAN_EVT_VEC_t* vec) {

    vec->data = (__int8_t*)malloc(sizeof(int) * INITIAL_SCAN_EVT_NUM_ESTIMATE);
    
    if (vec->data == NULL) {
        fprintf(stderr, "Error: No se pudo asignar memoria para el vector.\n");
        exit(1);
    }

    vec->size = 0;
    vec->capacity = INITIAL_SCAN_EVT_NUM_ESTIMATE;

}

void llenarVector(SCAN_EVT_VEC_t* vec, int value) {
    if (vec->size == vec->capacity) {
        // Si el vector se llena, aumenta la capacidad en 40
        vec->capacity += 5;
        vec->data = (__int8_t*)realloc(vec->data, sizeof(__int8_t) * vec->capacity);
        if (vec->data == NULL) {
            fprintf(stderr, "Error: No se pudo asignar memoria para el vector.\n");
            exit(1);
        }
    }
    vec->data[vec->size] = value;
    vec->size++;
}

void freeVector(SCAN_EVT_VEC_t* vec) {
    free(vec->data);
    vec->data = NULL;
    vec->size = 0;
    vec->capacity = 0;
}

__int32_t sr = 1;

SCAN_EVT_VEC_t maxint;
SCAN_EVT_VEC_t connint;
SCAN_EVT_VEC_t minint;
SCAN_EVT_VEC_t latenci;
SCAN_EVT_VEC_t timeaut;

// Callback de Generic Access Profile o GAP. GAP Define cómo se establece la conexión y gestionan las conexiones entre dispositivos BT. No hace falta ni mirarlo.
static void esp_gap_cb(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    uint8_t *adv_name = NULL;
    uint8_t adv_name_len = 0;
    esp_err_t err; 

    switch(event) {
    case ESP_GAP_BLE_SCAN_PARAM_SET_COMPLETE_EVT: {

        printf("PARAM SET COMPLETE\n\nMIN_IN           %lld\nMAX_IN           %lld\nLAT              %lld\nCONN_IN          %lld\nTOUT             %lld\n\n", (__int64_t)param->update_conn_params.min_int, (__int64_t)param->update_conn_params.max_int, (__int64_t)param->update_conn_params.latency, (__int64_t)param->update_conn_params.conn_int, (__int64_t)param->update_conn_params.timeout);
        usleep(3000000);

        if((err = param->scan_param_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Scan param set failed: %s", esp_err_to_name(err));
            break;
        }
        //the unit of the duration is second
        uint32_t duration = 0xFFFF;
        ESP_LOGI(GATTC_TAG, "Enable Ble Scan:during time %04" PRIx32 " minutes.",duration);
        esp_ble_gap_start_scanning(duration);
        break;
    }
    case ESP_GAP_BLE_SCAN_START_COMPLETE_EVT:
        //scan start complete event to indicate scan start successfully or failed
        if ((err = param->scan_start_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scan start failed: %s", esp_err_to_name(err));
            break;
        }

        printf("SCAN START COMPLETE\n\nMIN_IN           %lld\nMAX_IN           %lld\nLAT              %lld\nCONN_IN          %lld\nTOUT             %lld\n\n", (__int64_t)param->update_conn_params.min_int, (__int64_t)param->update_conn_params.max_int, (__int64_t)param->update_conn_params.latency, (__int64_t)param->update_conn_params.conn_int, (__int64_t)param->update_conn_params.timeout);
        usleep(3000000);

        ESP_LOGI(GATTC_TAG, "Scan start successed");
        break;
    case ESP_GAP_BLE_SCAN_STOP_COMPLETE_EVT:
        if ((err = param->scan_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTC_TAG, "Scan stop failed: %s", esp_err_to_name(err));
            break;
        }
        ESP_LOGI(GATTC_TAG, "Scan stop successed");
        if (is_connect == false) {
            ESP_LOGI(GATTC_TAG, "Connect to the remote device.");

            printf("SCAN STOP COMPLETE\n\nMIN_IN           %lld\nMAX_IN           %lld\nLAT              %lld\nCONN_IN          %lld\nTOUT             %lld\n\n", (__int64_t)param->update_conn_params.min_int, (__int64_t)param->update_conn_params.max_int, (__int64_t)param->update_conn_params.latency, (__int64_t)param->update_conn_params.conn_int, (__int64_t)param->update_conn_params.timeout);
            usleep(4000000);
            
            esp_ble_gattc_open(gl_profile_tab[PROFILE_APP_ID].gattc_if, scan_rst.scan_rst.bda, scan_rst.scan_rst.ble_addr_type, true);
        }
        break;
    case ESP_GAP_BLE_SCAN_RESULT_EVT: {
        esp_ble_gap_cb_param_t *scan_result = (esp_ble_gap_cb_param_t *)param;

        //**
        //printf("SCAN RESULT %ld\n\nMIN_IN           %lld\nMAX_IN           %lld\nLAT              %lld\nCONN_IN          %lld\nTOUT             %lld\n\n", sr, (__int64_t)param->update_conn_params.min_int, (__int64_t)param->update_conn_params.max_int, (__int64_t)param->update_conn_params.latency, (__int64_t)param->update_conn_params.conn_int, (__int64_t)param->update_conn_params.timeout);
        
        llenarVector(&minint, (__int8_t)param->update_conn_params.min_int);
        llenarVector(&maxint, (__int8_t)param->update_conn_params.max_int);
        llenarVector(&latenci, (__int8_t)param->update_conn_params.latency);
        llenarVector(&connint, (__int8_t)param->update_conn_params.conn_int);
        llenarVector(&timeaut, (__int8_t)param->update_conn_params.timeout);
        sr++;

        //usleep(500000);
        //**

        switch (scan_result->scan_rst.search_evt) {
        case ESP_GAP_SEARCH_INQ_RES_EVT:
            esp_log_buffer_hex(GATTC_TAG, scan_result->scan_rst.bda, 6);
            ESP_LOGI(GATTC_TAG, "Searched Adv Data Len %d, Scan Response Len %d", scan_result->scan_rst.adv_data_len, scan_result->scan_rst.scan_rsp_len);
            adv_name = esp_ble_resolve_adv_data(scan_result->scan_rst.ble_adv, ESP_BLE_AD_TYPE_NAME_CMPL, &adv_name_len);
            ESP_LOGI(GATTC_TAG, "Searched Device Name Len %d", adv_name_len);
            esp_log_buffer_char(GATTC_TAG, adv_name, adv_name_len);
            ESP_LOGI(GATTC_TAG, "\n");
            if (adv_name != NULL) {
                if ( strncmp((char *)adv_name, device_name, adv_name_len) == 0) {
                    memcpy(&(scan_rst), scan_result, sizeof(esp_ble_gap_cb_param_t));
                    esp_ble_gap_stop_scanning();
                }
            }
            break;
        case ESP_GAP_SEARCH_INQ_CMPL_EVT:
            break;
        default:
            break;
        }
        break;
    }
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if ((err = param->adv_stop_cmpl.status) != ESP_BT_STATUS_SUCCESS){
            ESP_LOGE(GATTC_TAG, "Adv stop failed: %s", esp_err_to_name(err));
        }else {
            ESP_LOGI(GATTC_TAG, "Stop adv successfully");
        }
        break;
    default:
        break;
    }
}

// Callback de eventos del perfil GATT del Cliente
static void esp_gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    //ESP_LOGI(GATTC_TAG, "EVT %d, gattc if %d", event, gattc_if);

    /* If event is register event, store the gattc_if for each profile */
    if (event == ESP_GATTC_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gattc_if = gattc_if;
        } else {
            ESP_LOGI(GATTC_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
            return;
        }
    }
    /* If the gattc_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gattc_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gattc_if == gl_profile_tab[idx].gattc_if) {
                if (gl_profile_tab[idx].gattc_cb) {
                    gl_profile_tab[idx].gattc_cb(event, gattc_if, param);
                }
            }
        }
    } while (0);
}

// Gestión y formato de datos
typedef struct{
    __int64_t *data;
    __uint8_t size;
} mateo_data_t;

// Valores del mensaje
int16_t payload = 0;
int32_t delay = 400;
int16_t n_paquete = 1;

// my_Task
void my_task(void *pvParameters){

    mateo_data_t data;

    int n_elements = n_min + payload;
    data.size = n_elements * sizeof(__int64_t);
    
    for (;;) {

        uint32_t inicio = millis();

        if ((data.size)&&(is_connect)) {

            data.data = (__int64_t*)malloc(sizeof(__int64_t) * data.size);

            if(data.data == NULL){
                ESP_LOGE(GATTC_TAG, "%s Fallo al crear data.data con malloc\n", __func__);
                break;
            }

            data.data[0] = get_time();
            data.data[1] = n_paquete; 
            data.data[2] = data.size;
            for (int i = n_min; i <= n_elements; i++) {data.data[i] = i-n_min+1;} 

            if(esp_ble_gattc_write_char(spp_gattc_if, spp_conn_id, spp_srv_start_handle + 2, data.size, (uint8_t*)data.data, ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE) == ESP_OK){
                printf("TX ");
                format_time(data.data[0]);
                printf("%lld\n", data.data[1]);
            }

            n_paquete++;
            free(data.data);
            
        }

        uint32_t retardo = millis() - inicio;

        if (retardo < delay){
            //printf("\n%lld\n", (__int64_t)(delay-retardo) / portTICK_PERIOD_MS);
            vTaskDelay((delay-retardo) / portTICK_PERIOD_MS);
        }
        else {
            ESP_LOGI(GATTC_TAG, "DELAY ENCONTRADO %ld", retardo);
        }

        //vTaskDelay(delay / portTICK_PERIOD_MS);
    }
    vTaskDelete(NULL);
}

// Inicializar tarea
static void my_task_init(void)
{
    xTaskCreate(my_task, "my_task", 2048, NULL, 8, NULL);
}

// Gestor o manejador de eventos dentro del perfil GATT del Cliente
static void gattc_profile_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param)
{
    esp_ble_gattc_cb_param_t *p_data = (esp_ble_gattc_cb_param_t *)param;

    switch (event) {
    case ESP_GATTC_REG_EVT:
        ESP_LOGI(GATTC_TAG, "REG EVT, set scan params");
        esp_ble_gap_set_scan_params(&ble_scan_params);
        break;
    case ESP_GATTC_CONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_CONNECT_EVT: conn_id=%d, gatt_if = %d", spp_conn_id, gattc_if);
        ESP_LOGI(GATTC_TAG, "REMOTE BDA:");
        esp_log_buffer_hex(GATTC_TAG, gl_profile_tab[PROFILE_APP_ID].remote_bda, sizeof(esp_bd_addr_t));
        spp_gattc_if = gattc_if;
        is_connect = true;
        spp_conn_id = p_data->connect.conn_id;
        memcpy(gl_profile_tab[PROFILE_APP_ID].remote_bda, p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
        esp_ble_gattc_search_service(spp_gattc_if, spp_conn_id, &spp_service_uuid);
        break;
    case ESP_GATTC_DISCONNECT_EVT:
        ESP_LOGI(GATTC_TAG, "disconnect");
        free_gattc_srv_db();
        esp_ble_gap_start_scanning(SCAN_ALL_THE_TIME);
        break;
    case ESP_GATTC_SEARCH_RES_EVT:
        ESP_LOGI(GATTC_TAG, "ESP_GATTC_SEARCH_RES_EVT: start_handle = %d, end_handle = %d, UUID:0x%04x",p_data->search_res.start_handle,p_data->search_res.end_handle,p_data->search_res.srvc_id.uuid.uuid.uuid16);
        spp_srv_start_handle = p_data->search_res.start_handle;
        spp_srv_end_handle = p_data->search_res.end_handle;
        break;
    case ESP_GATTC_SEARCH_CMPL_EVT:
        ESP_LOGI(GATTC_TAG, "SEARCH_CMPL: conn_id = %x, status %d", spp_conn_id, p_data->search_cmpl.status);
        esp_ble_gattc_send_mtu_req(gattc_if, spp_conn_id);
        break;
    case ESP_GATTC_REG_FOR_NOTIFY_EVT: {
        ESP_LOGI(GATTC_TAG,"Index = %d,status = %d,handle = %d\n",cmd, p_data->reg_for_notify.status, p_data->reg_for_notify.handle);
        if(p_data->reg_for_notify.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_REG_FOR_NOTIFY_EVT, status = %d", p_data->reg_for_notify.status);
            break;
        }
        uint16_t notify_en = 1;
        esp_ble_gattc_write_char_descr(
                spp_gattc_if,
                spp_conn_id,
                (db+cmd+1)->attribute_handle,
                sizeof(notify_en),
                (uint8_t *)&notify_en,
                ESP_GATT_WRITE_TYPE_RSP,
                ESP_GATT_AUTH_REQ_NONE);

        break;
    }
    case ESP_GATTC_NOTIFY_EVT:
        //ESP_LOGI(GATTC_TAG,"ESP_GATTC_NOTIFY_EVT\n");
        notify_event_handler(p_data);
        break;
    case ESP_GATTC_READ_CHAR_EVT:
        ESP_LOGI(GATTC_TAG,"ESP_GATTC_READ_CHAR_EVT\n");
        break;
    case ESP_GATTC_WRITE_CHAR_EVT:
        //ESP_LOGI(GATTC_TAG,"ESP_GATTC_WRITE_CHAR_EVT:status = %d,handle = %d", param->write.status, param->write.handle);
        if(param->write.status != ESP_GATT_OK){
            //ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_CHAR_EVT, error status = %d", p_data->write.status);
            break;
        }
        break;
    case ESP_GATTC_PREP_WRITE_EVT:
        break;
    case ESP_GATTC_EXEC_EVT:
        break;
    case ESP_GATTC_WRITE_DESCR_EVT:
        ESP_LOGI(GATTC_TAG,"ESP_GATTC_WRITE_DESCR_EVT: status =%d,handle = %d \n", p_data->write.status, p_data->write.handle);
        
        printf("MIN INT:\n");
        for (int i = 0; i < sr - 1; i++) {
        printf("%d ", minint.data[i]);
        }
        printf("\n");

        printf("MAX INT:\n");
        for (int i = 0; i < sr - 1; i++) {
        printf("%d ", maxint.data[i]);
        }
        printf("\n");

        printf("CONN INT:\n");
        for (int i = 0; i < sr - 1; i++) {
        printf("%d ", connint.data[i]);
        }
        printf("\n");

        printf("LATENCY:\n");
        for (int i = 0; i < sr - 1; i++) {
        printf("%d ", latenci.data[i]);
        }
        printf("\n");

        printf("TIMEOUT:\n");
        for (int i = 0; i < sr - 1; i++) {
        printf("%d ", timeaut.data[i]);
        }
        printf("\n");

        printf("%ld\n", sr - 1);
        //printf("%d, %d, %d, %d, %d \n", minint.size, maxint.size, connint.size, latenci.size, timeaut.size);

        if(p_data->write.status != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG, "ESP_GATTC_WRITE_DESCR_EVT, error status = %d", p_data->write.status);
            break;
        }
        switch(cmd){
        case SPP_IDX_SPP_DATA_NTY_VAL:
            cmd = SPP_IDX_SPP_STATUS_VAL;
            xQueueSend(cmd_reg_queue, &cmd,10/portTICK_PERIOD_MS);
            break;
        case SPP_IDX_SPP_STATUS_VAL:
        default:
            break;
        };
        break;
    case ESP_GATTC_CFG_MTU_EVT:
        if(p_data->cfg_mtu.status != ESP_OK){
            break;
        }
        ESP_LOGI(GATTC_TAG,"+MTU:%d\n", p_data->cfg_mtu.mtu);
        spp_mtu_size = p_data->cfg_mtu.mtu;

        db = (esp_gattc_db_elem_t *)malloc(count*sizeof(esp_gattc_db_elem_t));
        if(db == NULL){
            ESP_LOGE(GATTC_TAG,"%s:malloc db falied\n",__func__);
            break;
        }
        if(esp_ble_gattc_get_db(spp_gattc_if, spp_conn_id, spp_srv_start_handle, spp_srv_end_handle, db, &count) != ESP_GATT_OK){
            ESP_LOGE(GATTC_TAG,"%s:get db falied\n",__func__);
            break;
        }
        if(count != SPP_IDX_NB){
            ESP_LOGE(GATTC_TAG,"%s:get db count != SPP_IDX_NB, count = %d, SPP_IDX_NB = %d\n",__func__,count,SPP_IDX_NB);
            break;
        }
        for(int i = 0;i < SPP_IDX_NB;i++){
            switch((db+i)->type){
            case ESP_GATT_DB_PRIMARY_SERVICE:
                ESP_LOGI(GATTC_TAG,"attr_type = PRIMARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_SECONDARY_SERVICE:
                ESP_LOGI(GATTC_TAG,"attr_type = SECONDARY_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_CHARACTERISTIC:
                ESP_LOGI(GATTC_TAG,"attr_type = CHARACTERISTIC,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_DESCRIPTOR:
                ESP_LOGI(GATTC_TAG,"attr_type = DESCRIPTOR,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_INCLUDED_SERVICE:
                ESP_LOGI(GATTC_TAG,"attr_type = INCLUDED_SERVICE,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            case ESP_GATT_DB_ALL:
                ESP_LOGI(GATTC_TAG,"attr_type = ESP_GATT_DB_ALL,attribute_handle=%d,start_handle=%d,end_handle=%d,properties=0x%x,uuid=0x%04x\n",\
                        (db+i)->attribute_handle, (db+i)->start_handle, (db+i)->end_handle, (db+i)->properties, (db+i)->uuid.uuid.uuid16);
                break;
            default:
                break;
            }
        }
        cmd = SPP_IDX_SPP_DATA_NTY_VAL;
        xQueueSend(cmd_reg_queue, &cmd, 10/portTICK_PERIOD_MS);
        break;
    case ESP_GATTC_SRVC_CHG_EVT:
        break;
    default:
        break;
    }
}

void spp_client_reg_task(void* arg)
{
    uint16_t cmd_id;
    for(;;) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
        if(xQueueReceive(cmd_reg_queue, &cmd_id, portMAX_DELAY)) {
            if(db != NULL) {
                if(cmd_id == SPP_IDX_SPP_DATA_NTY_VAL){
                    ESP_LOGI(GATTC_TAG,"Index = %d,UUID = 0x%04x, handle = %d \n", cmd_id, (db+SPP_IDX_SPP_DATA_NTY_VAL)->uuid.uuid.uuid16, (db+SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_DATA_NTY_VAL)->attribute_handle);
                }else if(cmd_id == SPP_IDX_SPP_STATUS_VAL){
                    ESP_LOGI(GATTC_TAG,"Index = %d,UUID = 0x%04x, handle = %d \n", cmd_id, (db+SPP_IDX_SPP_STATUS_VAL)->uuid.uuid.uuid16, (db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                    esp_ble_gattc_register_for_notify(spp_gattc_if, gl_profile_tab[PROFILE_APP_ID].remote_bda, (db+SPP_IDX_SPP_STATUS_VAL)->attribute_handle);
                }
            }
        }
    }
}

void ble_client_appRegister(void)
{
    esp_err_t status;
    char err_msg[20];

    ESP_LOGI(GATTC_TAG, "register callback");

    //register the scan callback function to the gap module
    if ((status = esp_ble_gap_register_callback(esp_gap_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gap register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    //register the callback function to the gattc module
    if ((status = esp_ble_gattc_register_callback(esp_gattc_cb)) != ESP_OK) {
        ESP_LOGE(GATTC_TAG, "gattc register error: %s", esp_err_to_name_r(status, err_msg, sizeof(err_msg)));
        return;
    }
    esp_ble_gattc_app_register(PROFILE_APP_ID);

    // esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(23000);
    // if (local_mtu_ret){
    //     ESP_LOGE(GATTC_TAG, "set local  MTU failed: %s", esp_err_to_name_r(local_mtu_ret, err_msg, sizeof(err_msg)));
    // }

    cmd_reg_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(spp_client_reg_task, "spp_client_reg_task", 2048, NULL, 10, NULL);
}

void app_main(void)
{
    set_time();

    esp_err_t ret;

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    nvs_flash_init();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(GATTC_TAG, "%s init bluetooth\n", __func__);
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTC_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
        return;
    }

    ble_client_appRegister();
    my_task_init();
}
