#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "shared_types.h"
#include "rpi_hal.h"

#define CMD_BUFFER 128

typedef enum
{
    T2_START,
    T2_MOVE_TO_OBS1,
    T2_CAPTURE1,
    T2_TURN1,
    T2_FOLLOW1,
    T2_MOVE_TO_OBS2,
    T2_CAPTURE2,
    T2_TURN2,
    T2_FOLLOW2,
    T2_RETURN,
    T2_PARK,
    T2_DONE
} Task2State;

static int cmd_id = 1;

static void send_motor_cmd(SharedAppContext *ctx,
                           const char *command,
                           int speed,
                           int value)
{
    char buffer[CMD_BUFFER];

    snprintf(buffer, sizeof(buffer),
             "%d/MOTOR/%s/%d/%d;",
             cmd_id++, command, speed, value);

    write(ctx->stm32_fd, buffer, strlen(buffer));
}

static void wait_for_ack(SharedAppContext *ctx)
{
    pthread_mutex_lock(&ctx->stm32_ack_mutex);
    pthread_cond_wait(&ctx->stm32_ack_cond, &ctx->stm32_ack_mutex);
    pthread_mutex_unlock(&ctx->stm32_ack_mutex);
}

static int request_image_capture(SharedAppContext *ctx)
{
    pthread_mutex_lock(&ctx->image_capture_mutex);

    ctx->last_image_capture_id++;

    pthread_cond_wait(&ctx->image_capture_cond,
                      &ctx->image_capture_mutex);

    int result = ctx->last_image_capture_id;

    pthread_mutex_unlock(&ctx->image_capture_mutex);

    return result;
}

void run_task2(SharedAppContext *ctx)
{
    Task2State state = T2_START;

    int arrow1 = 0;
    int arrow2 = 0;

    while (state != T2_DONE)
    {
        switch (state)
        {

        case T2_START:

            printf("TASK2 START\n");

            state = T2_MOVE_TO_OBS1;

            break;

        case T2_MOVE_TO_OBS1:

            send_motor_cmd(ctx, "FWD", 25, 350);
            wait_for_ack(ctx);

            state = T2_CAPTURE1;

            break;

        case T2_CAPTURE1:

            printf("CAPTURE OBSTACLE 1\n");

            arrow1 = request_image_capture(ctx);

            state = T2_TURN1;

            break;

        case T2_TURN1:

            if (arrow1 == 1)
            {
                send_motor_cmd(ctx, "TURNL", 25, 90);
            }
            else
            {
                send_motor_cmd(ctx, "TURNR", 25, 90);
            }

            wait_for_ack(ctx);

            state = T2_FOLLOW1;

            break;

        case T2_FOLLOW1:

            send_motor_cmd(ctx, "FWD", 25, 150);
            wait_for_ack(ctx);

            state = T2_MOVE_TO_OBS2;

            break;

        case T2_MOVE_TO_OBS2:

            send_motor_cmd(ctx, "FWD", 25, 250);
            wait_for_ack(ctx);

            state = T2_CAPTURE2;

            break;

        case T2_CAPTURE2:

            printf("CAPTURE OBSTACLE 2\n");

            arrow2 = request_image_capture(ctx);

            state = T2_TURN2;

            break;

        case T2_TURN2:

            if (arrow2 == 1)
            {
                send_motor_cmd(ctx, "TURNL", 25, 90);
            }
            else
            {
                send_motor_cmd(ctx, "TURNR", 25, 90);
            }

            wait_for_ack(ctx);

            state = T2_FOLLOW2;

            break;

        case T2_FOLLOW2:

            send_motor_cmd(ctx, "FWD", 25, 150);
            wait_for_ack(ctx);

            state = T2_RETURN;

            break;

        case T2_RETURN:

            send_motor_cmd(ctx, "TURNR", 25, 180);
            wait_for_ack(ctx);

            send_motor_cmd(ctx, "FWD", 25, 400);
            wait_for_ack(ctx);

            state = T2_PARK;

            break;

        case T2_PARK:

            send_motor_cmd(ctx, "TURNL", 25, 90);
            wait_for_ack(ctx);

            send_motor_cmd(ctx, "FWD", 25, 100);
            wait_for_ack(ctx);

            state = T2_DONE;

            break;

        default:
            state = T2_DONE;
            break;
        }
    }

    printf("TASK2 COMPLETE\n");
}