#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "hardware/spi.h"

#include "Neo6M_UBXParser.h"
#include "mpu6050.h"
#include "navigation.h"
#include "kalman_stuff.h"
#include "PWMmotorDriver.h"
#include "ff.h"
#include "sd_card.h"
#include "LoRa.h"

#include "../ManualControl/project_assert.h"

FATFS fs;
FIL file;

// TARGET INFORMATION -------------------------------------------------------------------------------------------

struct TARGET {
    float deg;
    float arcmin;
    float arcsec;

    float deg_dec;
};
struct TARGET target_lat; // NORTH/SOUTH
struct TARGET target_lon;  // EAST/WEST

void setTarget() {
    target_lat.deg = 33.782032;
    target_lat.arcmin = 0.0;
    target_lat.arcsec = 0.0;
    target_lat.deg_dec = target_lat.deg + (float)(target_lat.arcmin * 1/60) + (float)(target_lat.arcsec * 1/3600);

    target_lon.deg = -84.407196;
    target_lon.arcmin = 0.0;
    target_lon.arcsec = 0.0;
    target_lon.deg_dec = target_lon.deg + (float)(target_lon.arcmin * 1/60) + (float)(target_lon.arcsec * 1/3600);

    // Latitude check (within -90 to 90)
    if (!c_assert(target_lat.deg_dec >= -90.0f && target_lat.deg_dec <= 90.0f)) return;
    // Longitude check (within -180 to 180)
    if (!c_assert(target_lon.deg_dec >= -180.0f && target_lon.deg_dec <= 180.0f)) return;
}
// --------------------------------------------------------------------------------------------------------------

#ifndef PI
#define PI 3.14159265359
#endif

#ifndef DEG_TO_RAD
#define DEG_TO_RAD (PI / 180.0f)
#endif

#ifndef RAD_TO_DEG
#define RAD_TO_DEG (180.0f / PI)
#endif

#define UART_ID uart1
#define BAUD_RATE 9600
#define UART_TX_PIN 4
#define UART_RX_PIN 5

#define i2c_dev i2c0
#define i2c_dev_SDA 20
#define i2c_dev_SCL 21
#define mpu_addr 0x68
#define imu_polling 100 // Hz

#define Vforward 0.95
#define KaP 0.7
#define KaD 0.1
#define stopping_dist 5 // m

#define m1_IN1 13
#define m1_IN2 15
#define m2_IN1 6
#define m2_IN2 7
#define min_DC 0.4
#define max_DC 1.0
#define find_heading_time (2 * 1e6) // us

#define servo_pin 0
#define servo_ticks 21

#define control_log_rate 2 // Hz

#define accel_fall_thres 0.4
#define fall_time_thres_us (0.5*1e6) // us
#define acc_landed_a 0.9
#define acc_landed_b 1.1
#define landed_time_thres_us (0.5 * 1e6) // us

#define CS_pin 9
#define RST_pin 8
#define IRQ_pin -1
#define SCK_pin 14
#define MOSI_pin 11
#define MISO_pin 12
#define Tx_interval (2 * 1e6) // us
#define LoRa_freq (915 * 1e6) // Hz
#define LoRa_sprd_factor 10 //12
#define LoRa_bw (125 * 1e3) //7.8 * 1e3
#define LoRa_cdng_rt 6//8
#define LoRa_tx_pwr 20
#define LoRa_use_PA_bst 1
#define LoRa_preamble_len 8 //12
#define LoRa_sync_wrd 0x12

// Servo PWM Struct -------------------------------------------------------------------------------
struct servoPWM {
    uint pin;
    uint slice;
    uint chan;
} parachute_servo;

// SPI Struct -------------------------------------------------------------------------------------
typedef struct {
    spi_inst_t *spi;
    uint cs_pin;
} pico_spi_context_t;
absolute_time_t lastTx;

// Motor Pin Structs ------------------------------------------------------------------------------
struct motor MotorRight;
struct motor MotorLeft;

// GPS Data Structs -------------------------------------------------------------------------------
struct posllhData posllh;
struct velnedData velned;
struct statusData status;

// IMU Vars ---------------------------------------------------------------------------------------
absolute_time_t last_imu_time;
float accel_data[3];
float gyro_data[3];
float temp_data;

// Fall tracking Vars ---------------------------------------------------------------------------------------
uint64_t falling_ts_candidate, landed_ts_candidate;
bool is_falling;

// Controller Vars --------------------------------------------------------------------------------
float u_r;
float u_l;
float last_alpha;
float alpha;
float Tp;
float Td;
float T;
float u_r_internal;
float u_l_internal;
absolute_time_t last_contr_call;
absolute_time_t last_log;
bool was_paused;

const char *hdr = "TOW,control_dt,heading,lat,lon,alt,dN,dE\r\n";

// Controller Funcs -------------------------------------------------------------------------------
static float shortestRotation(float x) {
    if (!c_assert(isfinite(x))) return 0.0f;
	return((x) > (PI) ? (x - 2 * PI): (x) < (-PI) ? (x + 2 * PI): (x));
}

static float constrainFloat(float x, float min, float max) {
    if (!c_assert(min <= max)) return min;
    if (!c_assert(isfinite(x))) return min;
	return ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)));
}

static float max(float a, float b) {
    if (!c_assert(isfinite(a))) return b;
    if (!c_assert(isfinite(b))) return a;
    return((a) < (b) ? (b): (a));
}

static float absFloat(float x) {
    if (!c_assert(isfinite(x))) return 0.0f;
    return((x) < (0.0) ? (x * -1.0) : (x));
}

static float boundTo2Pi(float x) {
    if (!c_assert(isfinite(x))) return 0.0f;
    return((x) > (2 * PI) ? (x - 2 * PI): (x) < (0.0f) ? (x + 2 * PI): (x));
}

float calcT(float target_heading, float current_heading) {
    float dt, de;

    dt = absolute_time_diff_us(last_contr_call, get_absolute_time());
    dt = (float)(dt * 1e-6);
   
    alpha = shortestRotation(target_heading - current_heading);
    de = alpha - last_alpha;

    Tp = KaP * alpha;
    Td = constrainFloat(KaD * de / dt, -2.0, 2.0);

    last_alpha = alpha;
    last_contr_call = get_absolute_time();

    return constrainFloat(Tp + Td, -0.5, 0.5);
}

void control(float target_heading, float current_heading) {
    if (!c_assert(isfinite(target_heading))) return 0.0f;
    if (!c_assert(isfinite(current_heading))) return 0.0f;

    T = calcT(target_heading, current_heading);
    u_l_internal = Vforward + T;
    u_r_internal = Vforward - T;

    u_l = constrainFloat(absFloat(u_l_internal), min_DC, max_DC);
    u_r = constrainFloat(absFloat(u_r_internal), min_DC, max_DC);

    printf("T=%f, u_l=%f, u_r=%f, dest_hdng=%f, cur_hdng=%f\n",
        T, u_l_internal, u_r_internal, target_heading, current_heading);

    setMotorPWM(&MotorRight, u_r, (u_r_internal) > (0.0) ? (FORWARD): (REVERSE));
    setMotorPWM(&MotorLeft, u_l, (u_l_internal) > (0.0) ? (FORWARD): (REVERSE));
}
// SPI Funcs --------------------------------------------------------------------------------------
void pico_spi_transfer(void* user, const uint8_t* tx, uint8_t* rx, size_t n) {
    if (!c_assert(user != NULL)) return;
    if (!c_assert(n > 0)) return;

    pico_spi_context_t* ctx = (pico_spi_context_t* )user;
    spi_inst_t* spi = ctx->spi;

    if (tx && rx) {
        spi_write_read_blocking(spi, tx, rx, n);
    }
    else if (tx) {
        spi_write_blocking(spi, tx, n);
    }
    else if (rx) {
        spi_read_blocking(spi, 0, rx, n);
    }
}

void pico_cs_set(void* user, bool active) {
    if (!c_assert(user != NULL)) return;
    pico_spi_context_t* ctx = (pico_spi_context_t* )user;
    gpio_put(ctx->cs_pin, active ? 0 : 1);
}

void pico_delay_ms(void* user, uint32_t ms) {
    sleep_ms(ms);
}

void pico_gpio_write(void* user, int pin, bool level) {
    if (!c_assert(pin >= 0 && pin <= 29)) return;
    gpio_put(pin, level);
}

bool pico_gpio_read(void* user, int pin) {
    if (!c_assert(pin >= 0 && pin <= 29)) return false;
    return gpio_get(pin);
}

void pico_spi_set_frequency(void* user, uint32_t hz) {
    if (!c_assert(user != NULL)) return;
    if (!c_assert(hz > 0)) return;
    pico_spi_context_t* ctx = (pico_spi_context_t* )user;
    spi_set_baudrate(ctx->spi, hz);
}

pico_spi_context_t spi_ctx = {
    .spi = spi1,
    .cs_pin = CS_pin
};

lora_hal_t pico_hal = {
    .spi_transfer = pico_spi_transfer,
    .cs_set = pico_cs_set,
    .delay_ms = pico_delay_ms,
    .gpio_write = pico_gpio_write,
    .gpio_read = pico_gpio_read,
    .spi_set_frequency = pico_spi_set_frequency,
    .user = (void* )&spi_ctx
};

LoRa* lora;

void LoRa_transmit(float lat, float lon) {
    if (!c_assert(lat >= -90.0f && lat <= 90.0f)) return;
    if (!c_assert(lon >= -180.0f && lon <= 180.0f)) return;
    char payload[30];
    snprintf(payload, sizeof(payload), "%f,%f", lat, lon);

    lora_begin_packet(lora, 0);
    lora_write(lora, (const uint8_t* )payload, strlen(payload));
    lora_end_packet(lora, false);

    printf("Transmitted '%s' though LoRa\n", payload);
}
// ------------------------------------------------------------------------------------------------

static void log_line(
    FIL *f, 
    unsigned long TOW, 
    float dt_log, 
    float heading,
    float dest_heading,
    float lat, 
    float lon, 
    float alt,
    float dN,
    float dE
    ) {

    if (!c_assert(f != NULL)) return;
    if (!c_assert(lat >= -90.0f && lat <= 90.0f)) return;

    char line[96];
    int len = snprintf(line, sizeof line, "%lu,%.2f,%.2f,%.2f,%3.7f,%3.7f,%.2f,%.2f,%.2f\r\n",
                       (unsigned long)TOW, dt_log, heading, dest_heading, lat, lon, alt, dN, dE);
    UINT bw;
    f_write(f, line, (UINT)len, &bw);
    f_sync(f);
}

void config_servo(struct servoPWM* servo) {
    if (!c_assert(servo != NULL)) return;
    // servo->pin is a uint so can't be negative
    if (!c_assert(servo->pin <= 29)) return;

    gpio_set_function(servo->pin, GPIO_FUNC_PWM);
    servo->slice = pwm_gpio_to_slice_num(servo->pin);
    pwm_set_wrap(servo->slice, 20000 - 1);
    pwm_set_clkdiv(servo->slice, 125);
    pwm_set_enabled(servo->slice, true);
    servo->chan = pwm_gpio_to_channel(servo->pin);
}

void set_servo(struct servoPWM* servo, int deg) {
    if (!c_assert(servo != NULL)) return;
    if (!c_assert(deg >= 0 && deg <= 180)) return;
    int flip_val = (int)(800 + 7.78 * deg);
    pwm_set_chan_level(servo->slice, servo->chan, flip_val);
}

void release_servo(struct servoPWM* servo) {
    if (!c_assert(servo != NULL)) return;
    if (!c_assert(servo_ticks > 0)) return;
    int tick_speed = 300;

    for (int i = 0; i < servo_ticks; i++) {
        int deg = 10 * i;
        set_servo(servo, 180 - deg);
        sleep_ms(tick_speed);
    }
}

bool landed() {
    if (!c_assert(isfinite(accel_data[0]))) return false;
    if (!c_assert(isfinite(accel_data[1]))) return false;

    float amag = sqrt(accel_data[0] * accel_data[0] + accel_data[1] * accel_data[1] + accel_data[2] * accel_data[2]);
    
    bool accel_freefall = amag < accel_fall_thres;
    bool accel_landed = acc_landed_a < amag && amag < acc_landed_b;

    if (!is_falling){
        if (accel_freefall) {
            if (falling_ts_candidate == 0) falling_ts_candidate = to_us_since_boot(get_absolute_time());
            else if (to_us_since_boot(get_absolute_time()) - falling_ts_candidate >= fall_time_thres_us) {
                is_falling = true;
            }
        }
        else {
            falling_ts_candidate = 0;
        }
        
    }
    else {
        if (accel_landed) {
            if (landed_ts_candidate == 0) landed_ts_candidate = to_us_since_boot(get_absolute_time());
            else if (to_us_since_boot(get_absolute_time()) - landed_ts_candidate > landed_time_thres_us) {
                return true;
            }
        }
        else {
            landed_ts_candidate = 0;
        }
    }

    return false;
}

bool repeating_imu_cb(__unused struct repeating_timer *t) {
    mpu6050_read(accel_data, gyro_data, &temp_data);
    last_imu_time = get_absolute_time();
    return true;
}

void findHeading() {
    if (!c_assert(find_heading_time > 0)) return;

    absolute_time_t start_time = get_absolute_time();
    absolute_time_t last_toggle = get_absolute_time();
    uint64_t toggle_time = 0.1 * 1e6; // us 
    int i = 0;
    float duty_cycle = 0.9;
    while (absolute_time_diff_us(start_time, get_absolute_time()) < find_heading_time) {
        if (absolute_time_diff_us(last_toggle, get_absolute_time()) >= toggle_time) {
            if (i) {
                setMotorPWM(&MotorRight, duty_cycle, FORWARD);
                setMotorPWM(&MotorLeft, duty_cycle, FORWARD);
            }
            else {
                setMotorPWM(&MotorLeft, duty_cycle, FORWARD);
                setMotorPWM(&MotorRight, duty_cycle, FORWARD);
            }
            i ^= 1;
            last_toggle = get_absolute_time();
        } 
    }
}

// everything below this point was pulled out of main() to reduce
// main size and make testing easier

static int init_uart(void) {
    printf("Initializing uart...\n");
    uart_init(UART_ID, BAUD_RATE);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    return 0;
}

static int init_imu_bus(void) {
    printf("Initializing mpu6050...\n");
    mpu6050_setdevparams(i2c_dev, mpu_addr);
    i2c_init(i2c_dev, 100 * 1000);
    gpio_set_function(i2c_dev_SDA, GPIO_FUNC_I2C);
    gpio_set_function(i2c_dev_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(i2c_dev_SDA);
    gpio_pull_up(i2c_dev_SCL);
    return 0;
}

static int init_lora(void) {
    printf("Setting up LoRa transmitter...\n");

    spi_init(spi1, 8 * 1e6);
    printf("Setting SPI data pins...\n");
    gpio_set_function(SCK_pin, GPIO_FUNC_SPI);
    gpio_set_function(MOSI_pin, GPIO_FUNC_SPI);
    gpio_set_function(MISO_pin, GPIO_FUNC_SPI);

    printf("Setting CS pin...\n");
    gpio_init(CS_pin);
    gpio_set_dir(CS_pin, GPIO_OUT);
    gpio_put(CS_pin, 1);

    printf("Setting RST pin...\n");
    gpio_init(RST_pin);
    gpio_set_dir(RST_pin, GPIO_OUT);
    gpio_put(RST_pin, 1);

    printf("Creating LoRa instance...\n");
    lora = lora_create(&pico_hal, CS_pin, RST_pin, IRQ_pin);
    if (!c_assert(lora != NULL)) return -1;

    printf("Configuring LoRa settings...\n");
    lora_set_spreading_factor(lora, LoRa_sprd_factor);
    lora_set_signal_bandwidth(lora, LoRa_bw);
    lora_set_coding_rate4(lora, LoRa_cdng_rt);
    lora_set_tx_power(lora, LoRa_tx_pwr, LoRa_use_PA_bst);
    lora_set_preamble_length(lora, LoRa_preamble_len);
    lora_set_sync_word(lora, LoRa_sync_wrd);

    printf("Initializing LoRa...\n");
    if (!c_assert(lora_begin(lora, LoRa_freq))) return -1;
    return 0;
}

static int init_imu_sensor(void) {
    printf("Waking mpu6050...\n");
    mpu6050_wake(true);
    mpu6050_ping();
    printf("Configuring mpu6050...\n");
    mpu6050_configure(3, 3);
    return 0;
}

static int init_motors(void) {
    printf("Configuring motor PWM...\n");
    addPins(&MotorRight, m1_IN1, m1_IN2);
    addPins(&MotorLeft, m2_IN1, m2_IN2);
    return 0;
}

static int init_servo(void) {
    printf("Configuring Servo PWM...\n");
    parachute_servo.pin = servo_pin;
    config_servo(&parachute_servo);
    return 0;
}

static int init_sd_logging(void) {
    printf("Mounting SD Card...\n");
    if (!c_assert(sd_init_driver())) return -1;
    if (!c_assert(f_mount(&fs, "0:", 1) == FR_OK)) return -1;

    printf("Opening log file...\n");
    FRESULT fr = f_open(&file, "CtrlLog.csv", FA_WRITE | FA_CREATE_ALWAYS);
    if (!c_assert(fr == FR_OK)) return -1;

    if (f_size(&file) == 0) {
        UINT bw;
        f_write(&file, hdr, (UINT)strlen(hdr), &bw);
        f_sync(&file);
    } else {
        printf("File seems to have data in it already!\n");
        return -1;
    }

    printf("Control log created.\n");
    return 0;
}

static void wait_for_gps_fix(void) {
    printf("Everything is configured!\n");
    printf("Checking GPS status now...\n");
    sleep_ms(1000);

    while (!posllhChanged() || !status.gpsFixOk) {
        while (uart_is_readable(UART_ID)) {
            addByte(uart_getc(UART_ID));
        }
        printf("[posllh: %d, fix: %d, fix type: %d] Waiting for GPS fix..\n",
               posllhChanged(), status.gpsFixOk, status.gpsFix);
        if (statusChanged()) { status = getSTATUS(); }
    }
    printf("GPS Fix good!\n");
}

int main()
{
// PRE LAUNCH CODE ----------------------------------------------------------------------------------------------
    stdio_init_all();
    sleep_ms(2000);

    printf("Hello! Starting configuration and pre flight checks!\n");
    sleep_ms(1000);

    setTarget();
    printf("Target location set!\n");
    sleep_ms(500);

    if (!c_assert(init_uart() == 0)) return 1;
    if (!c_assert(init_imu_bus() == 0)) return 1;
    if (!c_assert(init_lora() == 0)) return 1;
    if (!c_assert(init_imu_sensor() == 0)) return 1;

    printf("Starting mpu6050 timer and callback\n");
    struct repeating_timer imu_timer; 
    add_repeating_timer_us((int)(1e6 / imu_polling), repeating_imu_cb, NULL, &imu_timer);

    if (!c_assert(init_motors() == 0)) return 1;
    if (!c_assert(init_servo() == 0)) return 1;
    if (!c_assert(init_sd_logging() == 0)) return 1;

    wait_for_gps_fix();
// END PRE LAUNCH -----------------------------------------------------------------------------------------------
    printf("I am ready for launch!\n");
// LOAD INTO ROCKET ---------------------------------------------------------------------------------------------   
    float current_pos[3];
    float dN, dE;
    float velN, velE, velD;
    uint64_t last_velocity;
    float dt_vel;
    float est_heading;
    float destination[3];
    float dest_heading;

    falling_ts_candidate = 0;
    landed_ts_candidate = 0;
    is_falling = false;

    //while (!landed()) tight_loop_contents();
    //release_servo(&parachute_servo);

    posllh = getPOSLLH();
    set_ref_pos(posllh.lon, posllh.lat, posllh.height);
    computeNEDpos_deg(target_lon.deg_dec, target_lat.deg_dec, posllh.height, destination);
    //dest_heading = boundTo2Pi(azimuth_deg(posllh.lat, posllh.lon, target_lat.deg_dec, target_lon.deg_dec));

    //log_line(&file, posllh.iTOW, -1.0, -1.0, dest_heading * RAD_TO_DEG, posllh.lat, posllh.lon, posllh.hMSL, destination[0], destination[1]);

    LoRa_transmit(posllh.lat, posllh.lon);
    lastTx = get_absolute_time();

    //sleep_ms(10000); // give parachute time to detach

    sleep_ms(2000);

    findHeading();

// NAVIGATION TO TARGET -----------------------------------------------------------------------------------------
    while(true) {
        if (was_paused) {
            unpausePWM();
            was_paused = false;
        }

        while(uart_is_readable(UART_ID)) {
            addByte(uart_getc(UART_ID));
        }

        if (posllhChanged()) {
            posllh = getPOSLLH();
            computeNEDpos_deg(posllh.lon, posllh.lat, posllh.height, current_pos);

            z_X[0] = current_pos[0];
            z_Y[0] = current_pos[1];
            kalmanUpdate_X();
            kalmanUpdate_Y();

            dest_heading = boundTo2Pi(azimuth_deg(posllh.lat, posllh.lon, target_lat.deg_dec, target_lon.deg_dec));

            dN = destination[0] - xhat_X[0];
            dE = destination[1] - xhat_Y[0];

            //dN = destination[0] - current_pos[0];
            //dE = destination[1] - current_pos[1];
            //dest_heading = boundTo2Pi(atan2(dE, dN));

            //printf("Destination: [%f, %f, %f], Position: [%f, %f, %f], heading to destination: %f, current heading: %f\n", destination[0], destination[1], destination[2], xhat_X[0], xhat_Y[0], current_pos[2], dest_heading * RAD_TO_DEG, est_heading);
            
            if (sqrt(dN * dN + dE * dE) < stopping_dist) {
                pausePWM();
                printf("dN = %f, dE = %f\n", dN, dE);
                printf("Reached destination!\n");
                log_line(&file, posllh.iTOW, -1.0, est_heading * RAD_TO_DEG, dest_heading * RAD_TO_DEG, posllh.lat, posllh.lon, posllh.hMSL, dN, dE);
                f_close(&file);
                f_mount(NULL, "0:", 0);
                while (true) {
                    if (absolute_time_diff_us(lastTx, get_absolute_time()) >= Tx_interval) {
                        LoRa_transmit(posllh.lat, posllh.lon);
                        lastTx = get_absolute_time();
                    }   
                }
            }
        }

        if (absolute_time_diff_us(last_imu_time, get_absolute_time()) >= (int)(1e6/imu_polling)) {            
            float dt = (float)(absolute_time_diff_us(last_imu_time, get_absolute_time()) * 1e-6);
            kalmanPredict_heading(gyro_data[2], dt);

            est_heading = boundTo2Pi(xhat_heading[0] * DEG_TO_RAD);
            control(dest_heading, est_heading);
        }

        if (velnedChanged()) {
            velned = getVELNED();
            dt_vel = (float)(absolute_time_diff_us(last_velocity, get_absolute_time()) * 1e-6);
            //printf("Delta Lat = %f\n", lat_dest - posllh.lat);
            //printf("Meas X = %f, Est X = %f, Est velX bias = %f, velX = %f\n", current_pos[0], xhat_X[0], xhat_X[1], velned.velN);

            kalmanPredict_X(velned.velN, dt_vel);
            kalmanPredict_Y(velned.velE, dt_vel);
            last_velocity = get_absolute_time();

            z_heading[0] = velned.heading;
            //printf("VELNED Heading = %f\n", z_heading[0]);
            kalmanUpdate_heading();
            //printf("VELNED CHANGED\n");
        }

        //est_heading = boundTo2Pi(xhat_heading[0] * DEG_TO_RAD);
        //printf("\tEstimated heading = %f, xhat_heading = %f\n", est_heading, xhat_heading[0]);
        //control(dest_heading, est_heading);

        if (absolute_time_diff_us(last_log, get_absolute_time()) >= (int)(1e6/control_log_rate)) {           
            unsigned long timestamp = posllh.iTOW;
            float log_dt = (float)(absolute_time_diff_us(last_log, get_absolute_time()) * 1e-6);

            // wait until the first line of data is written to start logging
            if (f_size(&file) > strlen(hdr)) {
                log_line(&file, timestamp, log_dt, est_heading * RAD_TO_DEG, dest_heading * RAD_TO_DEG, posllh.lat, posllh.lon, posllh.hMSL, dN, dE);
            }

            last_log = get_absolute_time();
        }

        if (absolute_time_diff_us(lastTx, get_absolute_time()) >= Tx_interval) {
            LoRa_transmit(posllh.lat, posllh.lon);
            lastTx = get_absolute_time();
        }

        while (!status.gpsFixOk) {
            while(uart_is_readable(UART_ID)) {
                addByte(uart_getc(UART_ID));
            }

            if (!was_paused) {
                pausePWM();
                was_paused = true;
            }
        }
    }
}
