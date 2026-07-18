#include "wifi_connectivity.h"
#include "goalpost_mechanism.h"

#include <errno.h>
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "esp_err.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/ledc.h"

#include "lwip/inet.h"
#include "lwip/netdb.h"
#include "lwip/sockets.h"

/* TCP server settings */
#define PORT 30000
#define KEEPALIVE_IDLE 5
#define KEEPALIVE_INTERVAL 5
#define KEEPALIVE_COUNT 3

/* On-board/status LED */
#define BLINK_GPIO GPIO_NUM_15

/* LEDC PWM settings */
#define LEDC_MODE LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES LEDC_TIMER_10_BIT
#define LEDC_TIMER LEDC_TIMER_0
#define LEDC_FREQUENCY_HZ 10000U
#define LEDC_MAX_DUTY ((1U << 10U) - 1U)

/* DRV8833 input pins */
#define MOTOR_A_IN1_GPIO GPIO_NUM_14
#define MOTOR_A_IN2_GPIO GPIO_NUM_13
#define MOTOR_B_IN1_GPIO GPIO_NUM_21
#define MOTOR_B_IN2_GPIO GPIO_NUM_17

/* Two LEDC channels per motor: forward and reverse */
#define LEDC_CH_A_FWD LEDC_CHANNEL_0
#define LEDC_CH_A_REV LEDC_CHANNEL_1
#define LEDC_CH_B_FWD LEDC_CHANNEL_2
#define LEDC_CH_B_REV LEDC_CHANNEL_3

/*
 * Modern GPTimer configuration.
 *
 * A 1 MHz timer gives one timer count per microsecond. The motor command
 * transition lasts 500 ms, matching the previous legacy timer behavior.
 */
#define MOVEMENT_TIMER_RESOLUTION_HZ 1000000U
#define MOVEMENT_RAMP_DURATION_US 500000ULL
#define MOVEMENT_UPDATE_PERIOD_MS 1U

typedef struct
{
    bool forward;
    bool left;
    bool right;
    bool back;
} Movement;

typedef struct
{
    int8_t fullForward[2];
    int8_t fullBack[2];
    int8_t forwardLeft[2];
    int8_t forwardRight[2];
    int8_t backLeft[2];
    int8_t backRight[2];
    int8_t fullLeft[2];
    int8_t fullRight[2];
    int8_t stop[2];
} MoveTargets;

static const char *TAG = "MAIN";

/* Motor target values: index 0 = left motor, index 1 = right motor. */
static const MoveTargets moveTargets = {
    .fullForward = {95, -95},
    .fullBack = {-95, 95},
    .forwardLeft = {90, -100},
    .forwardRight = {100, -90},
    .backLeft = {-90, 100},
    .backRight = {-100, 90},
    .fullLeft = {-90, -90},
    .fullRight = {90, 90},
    .stop = {0, 0},
};

/*
 * currentDirection ranges from -100 to 100:
 *   negative = reverse
 *   positive = forward
 *   magnitude = requested motor power percentage
 */
static float currentDirection[2] = {0.0f, 0.0f};
static int8_t currentTargets[2] = {0, 0};
static float startTargets[2] = {0.0f, 0.0f};

static Movement movementState = {
    .forward = false,
    .left = false,
    .right = false,
    .back = false,
};

static Movement *moveStruct = &movementState;

static bool charging = false;
static bool inGame = false;
static bool resetting = false;

/* These flags are shared between FreeRTOS tasks. */
static volatile bool finishedMoving = false;
static volatile bool interruptMovement = false;

static TaskHandle_t doMovementHandle = NULL;
static SemaphoreHandle_t waitForData = NULL;

/* GPTimer state */
static gptimer_handle_t movementTimer = NULL;
static bool movementTimerRunning = false;

/* Function prototypes */
static void ledc_setup(void);
static void movement_timer_setup(void);
static void movement_timer_restart(void);
static void movement_timer_stop(void);
static void doBlink(void);
void move(void);

/* -------------------------------------------------------------------------- */
/* Modern GPTimer setup                                                        */
/* -------------------------------------------------------------------------- */

static void movement_timer_setup(void)
{
    const gptimer_config_t timerConfig = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = MOVEMENT_TIMER_RESOLUTION_HZ,
    };

    ESP_ERROR_CHECK(gptimer_new_timer(&timerConfig, &movementTimer));
    ESP_ERROR_CHECK(gptimer_enable(movementTimer));
}

static void movement_timer_restart(void)
{
    if (movementTimerRunning)
    {
        ESP_ERROR_CHECK(gptimer_stop(movementTimer));
        movementTimerRunning = false;
    }

    ESP_ERROR_CHECK(gptimer_set_raw_count(movementTimer, 0));
    ESP_ERROR_CHECK(gptimer_start(movementTimer));
    movementTimerRunning = true;
}

static void movement_timer_stop(void)
{
    if (movementTimerRunning)
    {
        ESP_ERROR_CHECK(gptimer_stop(movementTimer));
        movementTimerRunning = false;
    }
}

/* -------------------------------------------------------------------------- */
/* Movement command handling                                                   */
/* -------------------------------------------------------------------------- */

static float pwmFunction(uint8_t index, int32_t x)
{
    const float target = (float)currentTargets[index];
    const float start = startTargets[index];
    const float difference = target - start;

    if (fabsf(difference) < 0.001f)
    {
        return target;
    }

    const float amplitude = fabsf(difference);
    const float lowerValue = fminf(start, target);
    const float direction = (difference > 0.0f) ? 1.0f : -1.0f;
    const float slope = (1.0f / 2.5f) * direction;

    return amplitude / (1.0f + expf((-slope * (float)x) / 20.0f)) + lowerValue;
}

void setMoveStruct(char *buffer, int length)
{
    if (buffer == NULL || length < 0)
    {
        return;
    }

    moveStruct->forward = false;
    moveStruct->back = false;
    moveStruct->left = false;
    moveStruct->right = false;

    for (int i = 0; i < length; i++)
    {
        switch (buffer[i])
        {
        case 'u':
            moveStruct->forward = true;
            break;
        case 'd':
            moveStruct->back = true;
            break;
        case 'l':
            moveStruct->left = true;
            break;
        case 'r':
            moveStruct->right = true;
            break;
        default:
            break;
        }
    }

    if (moveStruct->forward && moveStruct->back)
    {
        moveStruct->forward = false;
        moveStruct->back = false;
    }

    if (moveStruct->left && moveStruct->right)
    {
        moveStruct->left = false;
        moveStruct->right = false;
    }
}

static void select_movement_targets(void)
{
    if (moveStruct->forward)
    {
        if (moveStruct->left)
        {
            currentTargets[0] = moveTargets.forwardLeft[0];
            currentTargets[1] = moveTargets.forwardLeft[1];
        }
        else if (moveStruct->right)
        {
            currentTargets[0] = moveTargets.forwardRight[0];
            currentTargets[1] = moveTargets.forwardRight[1];
        }
        else
        {
            currentTargets[0] = moveTargets.fullForward[0];
            currentTargets[1] = moveTargets.fullForward[1];
        }
    }
    else if (moveStruct->back)
    {
        if (moveStruct->left)
        {
            currentTargets[0] = moveTargets.backLeft[0];
            currentTargets[1] = moveTargets.backLeft[1];
        }
        else if (moveStruct->right)
        {
            currentTargets[0] = moveTargets.backRight[0];
            currentTargets[1] = moveTargets.backRight[1];
        }
        else
        {
            currentTargets[0] = moveTargets.fullBack[0];
            currentTargets[1] = moveTargets.fullBack[1];
        }
    }
    else if (moveStruct->left)
    {
        currentTargets[0] = moveTargets.fullLeft[0];
        currentTargets[1] = moveTargets.fullLeft[1];
    }
    else if (moveStruct->right)
    {
        currentTargets[0] = moveTargets.fullRight[0];
        currentTargets[1] = moveTargets.fullRight[1];
    }
    else
    {
        currentTargets[0] = moveTargets.stop[0];
        currentTargets[1] = moveTargets.stop[1];
    }
}

void beginMoving(void)
{
    startTargets[0] = currentDirection[0];
    startTargets[1] = currentDirection[1];

    select_movement_targets();
    movement_timer_restart();
}

void doMovement(void *pvParameters)
{
    (void)pvParameters;

    while (true)
    {
        interruptMovement = false;
        finishedMoving = false;

        beginMoving();

        while (true)
        {
            uint64_t elapsedUs = 0;
            ESP_ERROR_CHECK(gptimer_get_raw_count(movementTimer, &elapsedUs));

            if (elapsedUs >= MOVEMENT_RAMP_DURATION_US)
            {
                /*
                 * End at the exact target instead of stopping one iteration
                 * short of it.
                 */
                currentDirection[0] = (float)currentTargets[0];
                currentDirection[1] = (float)currentTargets[1];
                move();

                movement_timer_stop();
                break;
            }

            /*
             * Convert elapsed time from 0...500 ms into the original
             * sigmoid input range of approximately -250...250.
             */
            const int32_t curveX = (int32_t)(elapsedUs / 1000ULL) - 250;

            currentDirection[0] = pwmFunction(0, curveX);
            currentDirection[1] = pwmFunction(1, curveX);
            move();

            if (interruptMovement)
            {
                movement_timer_stop();
                break;
            }

            /*
             * A 1 ms delay prevents this task from monopolizing the CPU while
             * still providing smooth motor updates.
             */
            vTaskDelay(pdMS_TO_TICKS(MOVEMENT_UPDATE_PERIOD_MS));
        }

        if (interruptMovement)
        {
            /*
             * The sender gave the semaphore to wake this task, but an
             * interrupted ramp does not block on it. Consume that stale token.
             */
            (void)xSemaphoreTake(waitForData, 0);
            continue;
        }

        finishedMoving = true;
        ESP_LOGI("MOVEMENT", "Target reached; waiting for a new command");

        xSemaphoreTake(waitForData, portMAX_DELAY);

        ESP_LOGI("MOVEMENT", "New movement command received");
    }
}

/* -------------------------------------------------------------------------- */
/* TCP server                                                                  */
/* -------------------------------------------------------------------------- */

static bool send_all(int sock, const char *data, size_t length)
{
    size_t sentTotal = 0;

    while (sentTotal < length)
    {
        const int sent = send(sock, data + sentTotal, length - sentTotal, 0);
        if (sent < 0)
        {
            return false;
        }

        if (sent == 0)
        {
            errno = ECONNRESET;
            return false;
        }

        sentTotal += (size_t)sent;
    }

    return true;
}

static bool sendMessage(int sock, const char *message)
{
    if (message == NULL)
    {
        errno = EINVAL;
        return false;
    }

    return send_all(sock, message, strlen(message)) && send_all(sock, "|", 1);
}

static int receiveMessage(int sock, char *rxBuffer, size_t bufferSize)
{
    if (rxBuffer == NULL || bufferSize < 2)
    {
        errno = EINVAL;
        return -1;
    }

    size_t length = 0;

    while (length < bufferSize - 1)
    {
        char receivedChar = '\0';
        const int status = recv(sock, &receivedChar, 1, 0);

        if (status < 0)
        {
            return -1;
        }

        if (status == 0)
        {
            return 0;
        }

        if (receivedChar == '|')
        {
            rxBuffer[length] = '\0';
            return (int)length;
        }

        rxBuffer[length++] = receivedChar;
    }

    rxBuffer[bufferSize - 1] = '\0';
    errno = EMSGSIZE;
    return -1;
}

static void on_receive(int sock)
{
    char rxBuffer[128];
    static const char *SERVER_TAG = "SERVER_EVENT";

    while (true)
    {
        const int length = receiveMessage(sock, rxBuffer, sizeof(rxBuffer));

        if (length < 0)
        {
            ESP_LOGE(SERVER_TAG, "Receive failed: errno %d", errno);
            return;
        }

        if (length == 0)
        {
            ESP_LOGW(SERVER_TAG, "Connection closed");
            return;
        }

        if (length >= 10 && strncmp(rxBuffer, "readyCheck", 10) == 0)
        {
            ESP_LOGI("MESSAGE", "Ready-check message received");

            if (charging || resetting || inGame)
            {
                if (!sendMessage(sock, "not-ready"))
                {
                    ESP_LOGE(SERVER_TAG, "Failed to send not-ready response");
                    return;
                }
            }
            else
            {
                if (!sendMessage(sock, "ready"))
                {
                    ESP_LOGE(SERVER_TAG, "Failed to send ready response");
                    return;
                }

                inGame = true;

                if (doMovementHandle == NULL)
                {
                    const BaseType_t created = xTaskCreate(
                        doMovement,
                        "doMovement",
                        8192,
                        NULL,
                        3,
                        &doMovementHandle);

                    if (created != pdPASS)
                    {
                        doMovementHandle = NULL;
                        ESP_LOGE(SERVER_TAG, "Could not create movement task");
                        return;
                    }
                }
            }

            continue;
        }

        if (length >= 5 && strncmp(rxBuffer, "reset", 5) == 0)
        {
            inGame = false;
            setMoveStruct("z", 1);

            finishedMoving = false;
            interruptMovement = true;
            xSemaphoreGive(waitForData);
            return;
        }

        if (length >= 6 && strncmp(rxBuffer, "ignore", 6) == 0)
        {
            ESP_LOGI("MESSAGE", "Ignore message received");
            continue;
        }

        ESP_LOGI(SERVER_TAG, "Received %d bytes: %s", length, rxBuffer);

        setMoveStruct(rxBuffer, length);

        if (!finishedMoving)
        {
            interruptMovement = true;
        }

        xSemaphoreGive(waitForData);
    }
}

void taskServer(void *pvParameters)
{
    static const char *SERVER_TAG = "SERVER";
    const int addressFamily = (int)(intptr_t)pvParameters;

    char addressString[128] = {0};
    int ipProtocol = 0;

    const int keepAlive = 1;
    const int keepIdle = KEEPALIVE_IDLE;
    const int keepInterval = KEEPALIVE_INTERVAL;
    const int keepCount = KEEPALIVE_COUNT;

    struct sockaddr_storage destinationAddress;
    memset(&destinationAddress, 0, sizeof(destinationAddress));

    if (addressFamily == AF_INET)
    {
        struct sockaddr_in *destinationIpv4 =
            (struct sockaddr_in *)&destinationAddress;

        destinationIpv4->sin_addr.s_addr = htonl(INADDR_ANY);
        destinationIpv4->sin_family = AF_INET;
        destinationIpv4->sin_port = htons(PORT);
        ipProtocol = IPPROTO_IP;
    }
    else
    {
        ESP_LOGE(SERVER_TAG, "Unsupported address family: %d", addressFamily);
        vTaskDelete(NULL);
        return;
    }

    const int listenSocket = socket(addressFamily, SOCK_STREAM, ipProtocol);
    if (listenSocket < 0)
    {
        ESP_LOGE(SERVER_TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    const int reuseAddress = 1;
    (void)setsockopt(
        listenSocket,
        SOL_SOCKET,
        SO_REUSEADDR,
        &reuseAddress,
        sizeof(reuseAddress));

    ESP_LOGI(SERVER_TAG, "Socket created");

    int error = bind(
        listenSocket,
        (struct sockaddr *)&destinationAddress,
        sizeof(destinationAddress));

    if (error != 0)
    {
        ESP_LOGE(SERVER_TAG, "Socket bind failed: errno %d", errno);
        close(listenSocket);
        vTaskDelete(NULL);
        return;
    }

    error = listen(listenSocket, 1);
    if (error != 0)
    {
        ESP_LOGE(SERVER_TAG, "Socket listen failed: errno %d", errno);
        close(listenSocket);
        vTaskDelete(NULL);
        return;
    }

    while (true)
    {
        ESP_LOGI(SERVER_TAG, "Socket listening on port %d", PORT);

        struct sockaddr_storage sourceAddress;
        socklen_t addressLength = sizeof(sourceAddress);

        const int clientSocket = accept(
            listenSocket,
            (struct sockaddr *)&sourceAddress,
            &addressLength);

        if (clientSocket < 0)
        {
            ESP_LOGE(SERVER_TAG, "Accept failed: errno %d", errno);
            break;
        }

        (void)setsockopt(
            clientSocket,
            SOL_SOCKET,
            SO_KEEPALIVE,
            &keepAlive,
            sizeof(keepAlive));
        (void)setsockopt(
            clientSocket,
            IPPROTO_TCP,
            TCP_KEEPIDLE,
            &keepIdle,
            sizeof(keepIdle));
        (void)setsockopt(
            clientSocket,
            IPPROTO_TCP,
            TCP_KEEPINTVL,
            &keepInterval,
            sizeof(keepInterval));
        (void)setsockopt(
            clientSocket,
            IPPROTO_TCP,
            TCP_KEEPCNT,
            &keepCount,
            sizeof(keepCount));

        if (sourceAddress.ss_family == PF_INET)
        {
            inet_ntoa_r(
                ((struct sockaddr_in *)&sourceAddress)->sin_addr,
                addressString,
                sizeof(addressString));
        }

        ESP_LOGI(SERVER_TAG, "Accepted connection from %s", addressString);

        on_receive(clientSocket);

        shutdown(clientSocket, SHUT_RDWR);
        close(clientSocket);

        if (doMovementHandle != NULL)
        {
            while (!finishedMoving)
            {
                vTaskDelay(pdMS_TO_TICKS(10));
            }

            vTaskDelete(doMovementHandle);
            doMovementHandle = NULL;
        }
    }

    close(listenSocket);
    vTaskDelete(NULL);
}

/* -------------------------------------------------------------------------- */
/* LEDC motor control                                                          */
/* -------------------------------------------------------------------------- */

static uint32_t getRawDutyFromPercent(float dutyPercent)
{
    if (dutyPercent < 0.0f)
    {
        dutyPercent = 0.0f;
    }
    else if (dutyPercent > 100.0f)
    {
        dutyPercent = 100.0f;
    }

    return (uint32_t)lroundf(
        ((float)LEDC_MAX_DUTY * dutyPercent) / 100.0f);
}

static uint32_t getRawDutyFromBaseDirection(float direction)
{
    float magnitude = fabsf(direction);

    if (magnitude < 1.0f)
    {
        return 0;
    }

    if (magnitude > 100.0f)
    {
        magnitude = 100.0f;
    }

    return getRawDutyFromPercent(magnitude);
}

static void configure_ledc_channel(
    ledc_channel_t channel,
    gpio_num_t gpioNumber)
{
    const ledc_channel_config_t channelConfig = {
        .gpio_num = gpioNumber,
        .speed_mode = LEDC_MODE,
        .channel = channel,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .flags.output_invert = 0,
    };

    ESP_ERROR_CHECK(ledc_channel_config(&channelConfig));
}

static void set_ledc_duty(ledc_channel_t channel, uint32_t duty)
{
    ESP_ERROR_CHECK(ledc_set_duty(LEDC_MODE, channel, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_MODE, channel));
}

static void ledc_setup(void)
{
    const ledc_timer_config_t timerConfig = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
        .deconfigure = false,
    };

    ESP_ERROR_CHECK(ledc_timer_config(&timerConfig));

    configure_ledc_channel(LEDC_CH_A_FWD, MOTOR_A_IN1_GPIO);
    configure_ledc_channel(LEDC_CH_A_REV, MOTOR_A_IN2_GPIO);
    configure_ledc_channel(LEDC_CH_B_FWD, MOTOR_B_IN1_GPIO);
    configure_ledc_channel(LEDC_CH_B_REV, MOTOR_B_IN2_GPIO);

    set_ledc_duty(LEDC_CH_A_FWD, 0);
    set_ledc_duty(LEDC_CH_A_REV, 0);
    set_ledc_duty(LEDC_CH_B_FWD, 0);
    set_ledc_duty(LEDC_CH_B_REV, 0);
}

void move(void)
{
    const float leftDirection = currentDirection[0];
    const uint32_t leftDuty =
        getRawDutyFromBaseDirection(leftDirection);

    if (fabsf(leftDirection) < 1.0f)
    {
        set_ledc_duty(LEDC_CH_A_FWD, 0);
        set_ledc_duty(LEDC_CH_A_REV, 0);
    }
    else if (leftDirection > 0.0f)
    {
        set_ledc_duty(LEDC_CH_A_FWD, leftDuty);
        set_ledc_duty(LEDC_CH_A_REV, 0);
    }
    else
    {
        set_ledc_duty(LEDC_CH_A_FWD, 0);
        set_ledc_duty(LEDC_CH_A_REV, leftDuty);
    }

    const float rightDirection = currentDirection[1];
    const uint32_t rightDuty =
        getRawDutyFromBaseDirection(rightDirection);

    if (fabsf(rightDirection) < 1.0f)
    {
        set_ledc_duty(LEDC_CH_B_FWD, 0);
        set_ledc_duty(LEDC_CH_B_REV, 0);
    }
    else if (rightDirection > 0.0f)
    {
        set_ledc_duty(LEDC_CH_B_FWD, rightDuty);
        set_ledc_duty(LEDC_CH_B_REV, 0);
    }
    else
    {
        set_ledc_duty(LEDC_CH_B_FWD, 0);
        set_ledc_duty(LEDC_CH_B_REV, rightDuty);
    }
}

static void doBlink(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(BLINK_GPIO));
    ESP_ERROR_CHECK(gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_level(BLINK_GPIO, 1));
}

/* -------------------------------------------------------------------------- */
/* Current application entry point: PWM movement test without Wi-Fi            */
/* -------------------------------------------------------------------------- */

void app_main(void)
{
    doBlink();
    vTaskDelay(pdMS_TO_TICKS(500));

    waitForData = xSemaphoreCreateBinary();
    if (waitForData == NULL)
    {
        ESP_LOGE(TAG, "Could not create movement semaphore");
        return;
    }

    movement_timer_setup();
    ledc_setup();

    ESP_LOGI(TAG, "Starting PWM movement test without Wi-Fi");

    const BaseType_t created = xTaskCreate(
        doMovement,
        "doMovement",
        8192,
        NULL,
        3,
        &doMovementHandle);

    if (created != pdPASS)
    {
        doMovementHandle = NULL;
        ESP_LOGE(TAG, "Could not create movement task");
        return;
    }

    /*
     * Allow the movement task to complete its initial stop-to-stop ramp and
     * block on the semaphore.
     */
    vTaskDelay(pdMS_TO_TICKS(1000));

    static const char *commands[] = {
        "u",
        "d",
        "ul",
        "ur",
        "dl",
        "dr",
        "l",
        "r",
        "",
    };

    static const char *commandNames[] = {
        "Full forward",
        "Full back",
        "Forward-left",
        "Forward-right",
        "Back-left",
        "Back-right",
        "Left turn/spin",
        "Right turn/spin",
        "Stop",
    };

    const size_t commandCount = sizeof(commands) / sizeof(commands[0]);

    for (size_t i = 0; i < commandCount; i++)
    {
        const char *command = commands[i];
        const int commandLength = (int)strlen(command);

        setMoveStruct((char *)command, commandLength);

        if (!finishedMoving)
        {
            interruptMovement = true;
        }

        ESP_LOGI(
            TAG,
            "Command %u: %s (\"%s\")",
            (unsigned)(i + 1U),
            commandNames[i],
            command[0] != '\0' ? command : "none/stop");

        xSemaphoreGive(waitForData);

        /*
         * The ramp takes 500 ms. One second gives the robot time to reach and
         * hold each requested command briefly.
         */
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Command sequence complete; holding stop");

    while (true)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}