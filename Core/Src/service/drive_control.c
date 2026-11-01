#include "drive_control.h"

#include <math.h>
#include <stdio.h>

#define GAMEPAD_LY_CENTRE 128u
#define GAMEPAD_RX_CENTRE 127u

typedef struct {
    DriveSource source;
    DriveState state;
    DriveCommand gamepad_command;
    DriveCommand serial_command;
    uint32_t last_gamepad_frame_ms;
    uint32_t source_gate_ms;
    uint32_t last_tick_ms;
    bool gamepad_frame_seen;
    bool source_request_pending;
    DriveSource requested_source;
    bool arm_request_pending;
    bool arm_request_neutral;
    bool brake_request_pending;
    uint8_t serial_enable_last;           /* last en= code seen (DISABLE at boot) */
    bool serial_arm_request_pending;
    bool serial_arm_request_neutral;
    bool serial_disable_request_pending;
    volatile uint8_t estop_pending;
} DriveControlContext;

static DriveControlContext drive_control;

static bool TimestampAfter(uint32_t timestamp, uint32_t reference)
{
    return (int32_t)(timestamp - reference) > 0;
}

static bool TimedOut(uint32_t now_ms, uint32_t last_ms, uint32_t timeout_ms)
{
    return (uint32_t)(now_ms - last_ms) >= timeout_ms;
}

static void RequireFreshCommand(uint32_t now_ms)
{
    drive_control.source_gate_ms = now_ms;
}

static void EnterDisarmed(uint32_t now_ms)
{
    drive_control.state = DRIVE_STATE_DISARMED;
    RequireFreshCommand(now_ms);
}

static bool ActiveSourceHasFreshInput(void)
{
    if (drive_control.source == DRIVE_SOURCE_GAMEPAD)
    {
        return drive_control.gamepad_frame_seen &&
               TimestampAfter(drive_control.last_gamepad_frame_ms,
                              drive_control.source_gate_ms);
    }

    return drive_control.serial_command.valid &&
           TimestampAfter(drive_control.serial_command.received_ms,
                          drive_control.source_gate_ms);
}

static bool ActiveSourceTimedOut(uint32_t now_ms)
{
    if (!ActiveSourceHasFreshInput())
    {
        return false;
    }

    if (drive_control.source == DRIVE_SOURCE_GAMEPAD)
    {
        return TimedOut(now_ms, drive_control.last_gamepad_frame_ms,
                        DRIVE_GAMEPAD_TIMEOUT_MS);
    }

    return TimedOut(now_ms, drive_control.serial_command.received_ms,
                    DRIVE_SERIAL_TIMEOUT_MS);
}

void DriveControl_Init(uint32_t now_ms)
{
    drive_control = (DriveControlContext){
        .source = DRIVE_SOURCE_SERIAL,
        .state = DRIVE_STATE_DISARMED,
        .source_gate_ms = now_ms,
    };
}

void DriveControl_SubmitGamepadFrame(const GamepadFrame *frame)
{
    if (frame == 0 || !frame->valid)
    {
        return;
    }

    drive_control.last_gamepad_frame_ms = frame->received_ms;
    drive_control.gamepad_frame_seen = true;
}

void DriveControl_SubmitGamepadCommand(const DriveCommand *command)
{
    if (command == 0 || !command->valid)
    {
        return;
    }

    drive_control.gamepad_command = *command;
}

void DriveControl_SubmitSerialCommand(const DriveCommand *command)
{
    if (command == 0 || !command->valid)
    {
        return;
    }

    drive_control.serial_command = *command;
}

static bool SerialCommandIsNeutral(const DriveCommand *command)
{
    return fabsf(command->linear_mps) <= DRIVE_SERIAL_NEUTRAL_LINEAR_MPS &&
           fabsf(command->angular_radps) <= DRIVE_SERIAL_NEUTRAL_ANGULAR_RADPS;
}

void DriveControl_SubmitSerialEnable(uint8_t enable_code, const DriveCommand *command)
{
    if (command == 0 || !command->valid || enable_code == DRIVE_SERIAL_EN_ABSENT)
    {
        return;
    }

    if (enable_code == DRIVE_SERIAL_EN_ARM)
    {
        drive_control.serial_arm_request_pending = true;
        drive_control.serial_arm_request_neutral = SerialCommandIsNeutral(command);
    }
    else if (enable_code == DRIVE_SERIAL_EN_DISABLE &&
             drive_control.serial_enable_last != DRIVE_SERIAL_EN_DISABLE)
    {
        drive_control.serial_disable_request_pending = true;
    }

    drive_control.serial_enable_last = enable_code;
}

bool DriveControl_ParseSerialPacket(const char *text,
                                    uint32_t now_ms,
                                    DriveCommand *command,
                                    uint8_t *enable_code)
{
    float linear_mps;
    float angular_radps;
    int enable;
    char trailing;
    int matched;
    uint8_t code = DRIVE_SERIAL_EN_ABSENT;

    if (text == 0 || command == 0 || enable_code == 0)
    {
        return false;
    }

    /* New form first: exactly 3 conversions means nothing followed the frame. */
    matched = sscanf(text, "vcx=%f,wc=%f,en=%d %c",
                     &linear_mps, &angular_radps, &enable, &trailing);
    if (matched == 3)
    {
        if (enable < 0 || enable > (int)DRIVE_SERIAL_EN_ARM)
        {
            return false;
        }
        code = (uint8_t)enable;
    }
    else if (matched == 4)
    {
        return false; /* trailing garbage / merged frames */
    }
    else if (sscanf(text, "vcx=%f,wc=%f %c",
                    &linear_mps, &angular_radps, &trailing) != 2)
    {
        return false; /* legacy two-field form, no trailing data allowed */
    }

    if (!isfinite(linear_mps) || !isfinite(angular_radps))
    {
        return false;
    }

    command->valid = true;
    command->received_ms = now_ms;
    command->linear_mps = linear_mps;
    command->angular_radps = angular_radps;
    *enable_code = code;
    return true;
}

void DriveControl_RequestSource(DriveSource source, uint32_t requested_ms)
{
    (void)requested_ms;
    drive_control.requested_source = source;
    drive_control.source_request_pending = true;
}

bool DriveControl_IsGamepadNeutral(const GamepadFrame *frame)
{
    int32_t linear_delta;
    int32_t angular_delta;

    if (frame == 0 || !frame->valid)
    {
        return false;
    }

    linear_delta = (int32_t)frame->ly - (int32_t)GAMEPAD_LY_CENTRE;
    angular_delta = (int32_t)frame->rx - (int32_t)GAMEPAD_RX_CENTRE;

    if (linear_delta < 0)
    {
        linear_delta = -linear_delta;
    }
    if (angular_delta < 0)
    {
        angular_delta = -angular_delta;
    }

    return linear_delta <= (int32_t)DRIVE_AXIS_NEUTRAL_BAND &&
           angular_delta <= (int32_t)DRIVE_AXIS_NEUTRAL_BAND;
}

void DriveControl_RequestArm(const GamepadFrame *frame)
{
    if (frame == 0 || !frame->valid)
    {
        return;
    }

    drive_control.arm_request_neutral = DriveControl_IsGamepadNeutral(frame);
    drive_control.arm_request_pending = true;
}

void DriveControl_RequestBrake(uint32_t requested_ms)
{
    (void)requested_ms;
    drive_control.brake_request_pending = true;
}

void DriveControl_LatchEmergencyStopFromISR(void)
{
    drive_control.estop_pending = 1u;
}

void DriveControl_Tick(uint32_t now_ms, bool brake_wheels_stopped)
{
    bool serial_disable;

    drive_control.last_tick_ms = now_ms;

    if (drive_control.estop_pending != 0u)
    {
        drive_control.state = DRIVE_STATE_ESTOP;
    }

    if (drive_control.state == DRIVE_STATE_ESTOP)
    {
        drive_control.source_request_pending = false;
        drive_control.arm_request_pending = false;
        drive_control.brake_request_pending = false;
        drive_control.serial_arm_request_pending = false;
        drive_control.serial_disable_request_pending = false;
        return;
    }

    /* A serial Disable only counts while the serial source owns the robot, so
     * the PS2 controller (gamepad source) always overrides the phone. */
    serial_disable = drive_control.serial_disable_request_pending &&
                     drive_control.source == DRIVE_SOURCE_SERIAL &&
                     drive_control.state == DRIVE_STATE_ARMED;

    if (drive_control.state == DRIVE_STATE_BRAKING && brake_wheels_stopped)
    {
        EnterDisarmed(now_ms);
    }
    else if (drive_control.state != DRIVE_STATE_BRAKING &&
             (drive_control.brake_request_pending || serial_disable))
    {
        drive_control.state = DRIVE_STATE_BRAKING;
    }
    else
    {
        if (drive_control.state == DRIVE_STATE_DISARMED &&
            drive_control.arm_request_pending &&
            drive_control.arm_request_neutral)
        {
            drive_control.state = DRIVE_STATE_ARMED;
            RequireFreshCommand(now_ms);
        }
        if (drive_control.state != DRIVE_STATE_BRAKING &&
            drive_control.source_request_pending &&
            drive_control.requested_source != drive_control.source)
        {
            drive_control.source = drive_control.requested_source;
            RequireFreshCommand(now_ms);
        }
        /* Phone arm: evaluated after any PS2 source switch so a PS2 R2 takeover
         * in the same tick wins. */
        if (drive_control.state == DRIVE_STATE_DISARMED &&
            drive_control.source == DRIVE_SOURCE_SERIAL &&
            drive_control.serial_arm_request_pending &&
            drive_control.serial_arm_request_neutral)
        {
            drive_control.state = DRIVE_STATE_ARMED;
            RequireFreshCommand(now_ms);
        }
    }

    drive_control.source_request_pending = false;
    drive_control.arm_request_pending = false;
    drive_control.brake_request_pending = false;
    drive_control.serial_arm_request_pending = false;
    drive_control.serial_disable_request_pending = false;
}

DriveControlOutput DriveControl_GetOutput(void)
{
    const DriveCommand *command = 0;
    DriveControlOutput output = {
        .source = drive_control.source,
        .state = drive_control.state,
        .authorised = false,
        .linear_mps = 0.0f,
        .angular_radps = 0.0f,
    };

    if (drive_control.state != DRIVE_STATE_ARMED ||
        ActiveSourceTimedOut(drive_control.last_tick_ms))
    {
        return output;
    }

    if (drive_control.source == DRIVE_SOURCE_GAMEPAD)
    {
        command = &drive_control.gamepad_command;
    }
    else
    {
        command = &drive_control.serial_command;
    }

    if (command->valid && TimestampAfter(command->received_ms,
                                         drive_control.source_gate_ms))
    {
        output.authorised = true;
        output.linear_mps = command->linear_mps;
        output.angular_radps = command->angular_radps;
    }

    return output;
}

DriveSource DriveControl_GetSource(void)
{
    return drive_control.source;
}

DriveState DriveControl_GetState(void)
{
    return drive_control.state;
}

uint8_t DriveControl_GetCtrlMode(void)
{
    if (drive_control.state != DRIVE_STATE_ARMED)
    {
        return DRIVE_CTRL_MODE_DISABLED;
    }

    return (drive_control.source == DRIVE_SOURCE_GAMEPAD) ? DRIVE_CTRL_MODE_GAMEPAD
                                                          : DRIVE_CTRL_MODE_SERIAL;
}

float DriveControl_MapAxis(uint8_t value, uint8_t centre, float maximum)
{
    int32_t delta = (int32_t)value - (int32_t)centre;
    int32_t magnitude;
    int32_t span;

    if (delta >= -(int32_t)DRIVE_AXIS_NEUTRAL_BAND &&
        delta <= (int32_t)DRIVE_AXIS_NEUTRAL_BAND)
    {
        return 0.0f;
    }

    if (delta > 0)
    {
        magnitude = delta - (int32_t)DRIVE_AXIS_NEUTRAL_BAND;
        span = 255 - ((int32_t)centre + (int32_t)DRIVE_AXIS_NEUTRAL_BAND);
        return -maximum * ((float)magnitude / (float)span);
    }

    magnitude = -delta - (int32_t)DRIVE_AXIS_NEUTRAL_BAND;
    span = ((int32_t)centre - (int32_t)DRIVE_AXIS_NEUTRAL_BAND);
    return maximum * ((float)magnitude / (float)span);
}
