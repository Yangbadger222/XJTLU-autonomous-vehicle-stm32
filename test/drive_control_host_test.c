#include <assert.h>
#include <math.h>
#include <stdbool.h>

#include "drive_control.h"
#include "ps2_frame.h"

static GamepadFrame GamepadFrameAt(uint32_t timestamp, uint8_t ly, uint8_t rx)
{
    GamepadFrame frame = {
        .valid = true,
        .received_ms = timestamp,
        .buttons = UINT16_MAX,
        .ly = ly,
        .rx = rx,
    };
    return frame;
}

static DriveCommand CommandAt(uint32_t timestamp, float linear, float angular)
{
    DriveCommand command = {
        .valid = true,
        .received_ms = timestamp,
        .linear_mps = linear,
        .angular_radps = angular,
    };
    return command;
}

static void SubmitGamepad(uint32_t timestamp, float linear, float angular)
{
    GamepadFrame frame = GamepadFrameAt(timestamp, 128u, 127u);
    DriveCommand command = CommandAt(timestamp, linear, angular);

    DriveControl_SubmitGamepadFrame(&frame);
    DriveControl_SubmitGamepadCommand(&command);
}

static void ArmSerial(uint32_t timestamp)
{
    GamepadFrame frame = GamepadFrameAt(timestamp, 128u, 127u);
    DriveCommand command = CommandAt(timestamp, 0.4f, -0.2f);

    DriveControl_SubmitGamepadFrame(&frame);
    DriveControl_SubmitSerialCommand(&command);
    DriveControl_RequestArm(&frame);
    DriveControl_Tick(timestamp, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(!DriveControl_GetOutput().authorised);

    command.received_ms = timestamp + 1u;
    DriveControl_SubmitSerialCommand(&command);
    DriveControl_Tick(timestamp + 1u, false);
    assert(DriveControl_GetOutput().authorised);
}

static void TestInvalidGamepadCannotChangeSerial(void)
{
    DriveControl_Init(0u);
    ArmSerial(1u);

    /* Invalid gamepad data is deliberately never submitted to the arbiter. */
    DriveControl_Tick(50u, false);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_SERIAL);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetOutput().authorised);
}

static void TestSwitchNeedsPostSwitchCommand(void)
{
    DriveControlOutput output;

    DriveControl_Init(0u);
    ArmSerial(1u);
    SubmitGamepad(2u, 0.7f, 0.3f);

    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 10u);
    DriveControl_Tick(10u, false);
    output = DriveControl_GetOutput();
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(!output.authorised);
    assert(output.linear_mps == 0.0f);

    SubmitGamepad(11u, 0.7f, 0.3f);
    DriveControl_Tick(11u, false);
    output = DriveControl_GetOutput();
    assert(output.authorised);
    assert(fabsf(output.linear_mps - 0.7f) < 0.0001f);
    assert(fabsf(output.angular_radps - 0.3f) < 0.0001f);
}

static void TestArmAndSourceSwitchOrdering(void)
{
    GamepadFrame neutral = GamepadFrameAt(1u, 128u, 127u);

    DriveControl_Init(0u);
    SubmitGamepad(1u, 0.3f, 0.0f);
    DriveControl_RequestArm(&neutral);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 1u);
    DriveControl_Tick(1u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(!DriveControl_GetOutput().authorised);

    SubmitGamepad(2u, 0.3f, 0.0f);
    DriveControl_Tick(2u, false);
    assert(DriveControl_GetOutput().authorised);
}

static void TestTimeoutStopsMotionButKeepsControllerArmed(void)
{
    DriveControl_Init(0u);
    ArmSerial(1u);
    DriveControl_Tick(252u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(!DriveControl_GetOutput().authorised);

    DriveControl_SubmitSerialCommand(&(DriveCommand){
        .valid = true,
        .received_ms = 253u,
        .linear_mps = 0.4f,
        .angular_radps = -0.2f,
    });
    DriveControl_Tick(253u, false);
    assert(DriveControl_GetOutput().authorised);

    DriveControl_Init(0u);
    ArmSerial(1u);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 2u);
    DriveControl_Tick(2u, false);
    SubmitGamepad(3u, 0.5f, 0.0f);
    DriveControl_Tick(3u, false);
    SubmitGamepad(299u, 0.5f, 0.0f);
    DriveControl_Tick(300u, false);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetOutput().authorised);

    DriveControl_Tick(399u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(!DriveControl_GetOutput().authorised);
}

static void TestFreshCommandResumesWithoutAnotherArmRequest(void)
{
    DriveCommand command = CommandAt(253u, 0.4f, -0.2f);

    DriveControl_Init(0u);
    ArmSerial(1u);
    DriveControl_Tick(252u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(!DriveControl_GetOutput().authorised);

    DriveControl_SubmitSerialCommand(&command);
    DriveControl_Tick(253u, false);
    assert(DriveControl_GetOutput().authorised);
}

static void TestBrakingAndEstopRejectOtherRequests(void)
{
    GamepadFrame neutral = GamepadFrameAt(4u, 128u, 127u);

    DriveControl_Init(0u);
    ArmSerial(1u);
    /* PS2 B overrides even when the serial source (phone / ROS) owns the robot. */
    DriveControl_RequestBrake(2u);
    DriveControl_Tick(2u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);

    DriveControl_Init(0u);
    SubmitGamepad(1u, 0.2f, 0.0f);
    DriveControl_RequestArm(&(GamepadFrame){
        .valid = true,
        .received_ms = 1u,
        .ly = 144u,
        .rx = 127u,
    });
    DriveControl_Tick(1u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    DriveControl_Init(0u);
    SubmitGamepad(1u, 0.2f, 0.0f);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 2u);
    DriveControl_Tick(2u, false);
    SubmitGamepad(3u, 0.2f, 0.0f);
    DriveControl_RequestArm(&neutral);
    DriveControl_Tick(4u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);

    DriveControl_RequestBrake(5u);
    DriveControl_Tick(5u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);
    assert(!DriveControl_GetOutput().authorised);

    DriveControl_RequestSource(DRIVE_SOURCE_SERIAL, 6u);
    DriveControl_RequestArm(&neutral);
    DriveControl_RequestBrake(6u);
    DriveControl_Tick(6u, false);
    assert(DriveControl_GetSource() == DRIVE_SOURCE_GAMEPAD);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);

    DriveControl_Tick(7u, true);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    GamepadFrame brake_arm_frame = GamepadFrameAt(8u, 128u, 127u);
    DriveControl_SubmitGamepadFrame(&brake_arm_frame);
    DriveControl_RequestArm(&brake_arm_frame);
    DriveControl_Tick(8u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(!DriveControl_GetOutput().authorised);

    SubmitGamepad(9u, 0.2f, 0.0f);
    DriveControl_Tick(9u, false);
    assert(DriveControl_GetOutput().authorised);

    DriveControl_LatchEmergencyStopFromISR();
    DriveControl_RequestSource(DRIVE_SOURCE_SERIAL, 10u);
    DriveControl_RequestArm(&brake_arm_frame);
    DriveControl_Tick(10u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_ESTOP);
    assert(!DriveControl_GetOutput().authorised);
}

static void TestNeutralBandAndAxisMapping(void)
{
    GamepadFrame neutral = GamepadFrameAt(1u, 143u, 112u);
    GamepadFrame outside = GamepadFrameAt(1u, 144u, 127u);

    assert(DriveControl_IsGamepadNeutral(&neutral));
    assert(!DriveControl_IsGamepadNeutral(&outside));
    assert(DriveControl_MapAxis(128u, 128u, 2.0f) == 0.0f);
    assert(DriveControl_MapAxis(143u, 128u, 2.0f) == 0.0f);
    assert(fabsf(DriveControl_MapAxis(0u, 128u, 2.0f) - 2.0f) < 0.0001f);
    assert(fabsf(DriveControl_MapAxis(255u, 128u, 2.0f) + 2.0f) < 0.0001f);
}

static void TestPs2FrameValidationAndEdges(void)
{
    uint8_t response[PS2_FRAME_LENGTH] = {0};
    GamepadFrame frame;
    uint16_t pressed_buttons;

    response[1] = 0x73u;
    response[2] = 0x5Au;
    response[3] = 0xFFu;
    response[4] = 0xFEu; /* L2 is pressed in the active-low bitmap. */
    response[5] = 0x00u;
    response[6] = 0x80u;
    response[7] = 0xFFu;
    response[8] = 0x00u;

    assert(PS2_DecodeGamepadFrame(response,
                                  sizeof(response),
                                  10u,
                                  0u,
                                  &frame,
                                  &pressed_buttons));
    assert(frame.buttons == 0xFEFFu);
    assert((frame.buttons_pressed & GAMEPAD_BUTTON_L2) != 0u);
    assert(frame.lx == 0xFFu);
    assert(frame.ly == 0x00u);
    assert(frame.rx == 0x00u);
    assert(frame.ry == 0x80u);

    response[4] = 0xFFu;
    assert(PS2_DecodeGamepadFrame(response,
                                  sizeof(response),
                                  11u,
                                  pressed_buttons,
                                  &frame,
                                  &pressed_buttons));
    assert((frame.buttons_released & GAMEPAD_BUTTON_L2) != 0u);

    assert(!PS2_DecodeGamepadFrame(response,
                                   PS2_FRAME_LENGTH - 1u,
                                   12u,
                                   pressed_buttons,
                                   &frame,
                                   &pressed_buttons));
    response[1] = 0x41u;
    assert(!PS2_DecodeGamepadFrame(response,
                                   sizeof(response),
                                   12u,
                                   pressed_buttons,
                                   &frame,
                                   &pressed_buttons));
    response[1] = 0x73u;
    response[2] = 0x00u;
    assert(!PS2_DecodeGamepadFrame(response,
                                   sizeof(response),
                                   12u,
                                   pressed_buttons,
                                   &frame,
                                   &pressed_buttons));
}

static void PhoneArm(uint32_t timestamp)
{
    DriveCommand zero = CommandAt(timestamp, 0.0f, 0.0f);

    DriveControl_SubmitSerialCommand(&zero);
    DriveControl_SubmitSerialEnable(DRIVE_SERIAL_EN_ARM, &zero);
    DriveControl_Tick(timestamp, false);
}

static void PhoneCode(uint8_t code, uint32_t timestamp, float linear)
{
    DriveCommand command = CommandAt(timestamp, linear, 0.0f);

    DriveControl_SubmitSerialCommand(&command);
    DriveControl_SubmitSerialEnable(code, &command);
    DriveControl_Tick(timestamp, false);
}

static void TestParseSerialPacket(void)
{
    DriveCommand command;
    uint8_t code;

    assert(DriveControl_ParseSerialPacket("vcx=0.500,wc=-0.200,en=2\n", 7u, &command, &code));
    assert(code == DRIVE_SERIAL_EN_ARM && command.valid && command.received_ms == 7u);
    assert(fabsf(command.linear_mps - 0.5f) < 0.0001f);
    assert(fabsf(command.angular_radps + 0.2f) < 0.0001f);

    assert(DriveControl_ParseSerialPacket("vcx=0.000,wc=0.000,en=0\n", 1u, &command, &code));
    assert(code == DRIVE_SERIAL_EN_DISABLE);
    assert(DriveControl_ParseSerialPacket("vcx=0.1,wc=0.2,en=1", 1u, &command, &code));
    assert(code == DRIVE_SERIAL_EN_NORMAL);
    assert(DriveControl_ParseSerialPacket("vcx=0.1,wc=0.2,en=1\r\n", 1u, &command, &code));
    assert(code == DRIVE_SERIAL_EN_NORMAL);

    /* Legacy two-field form still works and carries no enable code. */
    assert(DriveControl_ParseSerialPacket("vcx=0.500,wc=-0.200\n", 1u, &command, &code));
    assert(code == DRIVE_SERIAL_EN_ABSENT);

    /* The exact frame that the old parser silently dropped. */
    assert(DriveControl_ParseSerialPacket("vcx=0.250,wc=0.000,en=1\n", 1u, &command, &code));

    assert(!DriveControl_ParseSerialPacket("vcx=0.5,wc=0.1,en=3\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("vcx=0.5,wc=0.1,en=-1\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("vcx=0.5,wc=0.1,en=\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("vcx=0.5,wc=0.1,x\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("vcx=0.5,wc=0.1\nvcx=0.5,wc=0.1\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket(
        "vcx=0.5,wc=0.1,en=1\nvcx=0.5,wc=0.1,en=1\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("vcx=nan,wc=0.1,en=1\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("vcx=inf,wc=0.1\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("hello\n", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket("", 1u, &command, &code));
    assert(!DriveControl_ParseSerialPacket(0, 1u, &command, &code));
}

static void TestPhoneArmDrivesWithoutPs2(void)
{
    DriveControl_Init(0u);
    assert(DriveControl_GetCtrlMode() == DRIVE_CTRL_MODE_DISABLED);

    /* Disarmed: phone commands are ignored. */
    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 1u, 0.3f);
    assert(!DriveControl_GetOutput().authorised);

    PhoneArm(2u);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    assert(DriveControl_GetCtrlMode() == DRIVE_CTRL_MODE_SERIAL);
    assert(!DriveControl_GetOutput().authorised); /* needs a post-arm command */

    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 3u, 0.3f);
    assert(DriveControl_GetOutput().authorised);
    assert(fabsf(DriveControl_GetOutput().linear_mps - 0.3f) < 0.0001f);
}

static void TestPhoneArmGuards(void)
{
    DriveCommand moving = CommandAt(1u, 0.3f, 0.0f);

    /* A non-neutral arm request is refused. */
    DriveControl_Init(0u);
    DriveControl_SubmitSerialCommand(&moving);
    DriveControl_SubmitSerialEnable(DRIVE_SERIAL_EN_ARM, &moving);
    DriveControl_Tick(1u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    /* en=1 (what ROS sends) and legacy frames never arm. */
    DriveControl_Init(0u);
    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 1u, 0.0f);
    PhoneCode(DRIVE_SERIAL_EN_ABSENT, 2u, 0.0f);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    /* PS2 owns the robot (gamepad source): the phone cannot arm. */
    DriveControl_Init(0u);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 1u);
    DriveControl_Tick(1u, false);
    PhoneArm(2u);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    /* A PS2 R2 takeover in the same tick as the phone arm wins. */
    DriveControl_Init(0u);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 1u);
    DriveControl_SubmitSerialEnable(DRIVE_SERIAL_EN_ARM, &(DriveCommand){
        .valid = true, .received_ms = 1u});
    DriveControl_Tick(1u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);

    /* A latched e-stop cannot be cleared by the phone. */
    DriveControl_Init(0u);
    DriveControl_LatchEmergencyStopFromISR();
    PhoneArm(2u);
    assert(DriveControl_GetState() == DRIVE_STATE_ESTOP);
}

static void TestPhoneDisableBrakesAndIsEdgeTriggered(void)
{
    DriveControl_Init(0u);
    PhoneArm(1u);
    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 2u, 0.3f);
    assert(DriveControl_GetOutput().authorised);

    /* Disable -> active brake (BRAKING), then disarmed once the wheels stop. */
    PhoneCode(DRIVE_SERIAL_EN_DISABLE, 3u, 0.0f);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);
    assert(!DriveControl_GetOutput().authorised);
    DriveControl_Tick(4u, true);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);
    assert(DriveControl_GetCtrlMode() == DRIVE_CTRL_MODE_DISABLED);

    /* en=1 after a disable does not re-arm; only a new en=2 does. */
    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 5u, 0.0f);
    assert(DriveControl_GetState() == DRIVE_STATE_DISARMED);
    PhoneArm(6u);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);

    /* A robot armed from the PS2 controller is not braked by an en=0 heartbeat
     * (no falling edge), but is by a real enabled -> disabled transition. */
    DriveControl_Init(0u);
    ArmSerial(1u);
    PhoneCode(DRIVE_SERIAL_EN_DISABLE, 5u, 0.0f);
    PhoneCode(DRIVE_SERIAL_EN_DISABLE, 6u, 0.0f);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 7u, 0.0f);
    PhoneCode(DRIVE_SERIAL_EN_DISABLE, 8u, 0.0f);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);
}

static void TestPs2OverridesPhone(void)
{
    /* R2 hands control to the PS2 controller; phone Disable is then ignored. */
    DriveControl_Init(0u);
    PhoneArm(1u);
    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 2u, 0.2f);
    DriveControl_RequestSource(DRIVE_SOURCE_GAMEPAD, 3u);
    DriveControl_Tick(3u, false);
    assert(DriveControl_GetCtrlMode() == DRIVE_CTRL_MODE_GAMEPAD);
    assert(!DriveControl_GetOutput().authorised); /* needs a fresh PS2 command */
    PhoneCode(DRIVE_SERIAL_EN_DISABLE, 4u, 0.0f);
    assert(DriveControl_GetState() == DRIVE_STATE_ARMED);
    SubmitGamepad(5u, 0.4f, 0.0f);
    DriveControl_Tick(5u, false);
    assert(DriveControl_GetOutput().authorised);
    assert(fabsf(DriveControl_GetOutput().linear_mps - 0.4f) < 0.0001f);

    /* PS2 B brakes a phone-driven robot. */
    DriveControl_Init(0u);
    PhoneArm(1u);
    PhoneCode(DRIVE_SERIAL_EN_NORMAL, 2u, 0.2f);
    DriveControl_RequestBrake(3u);
    DriveControl_Tick(3u, false);
    assert(DriveControl_GetState() == DRIVE_STATE_BRAKING);
}

int main(void)
{
    TestInvalidGamepadCannotChangeSerial();
    TestSwitchNeedsPostSwitchCommand();
    TestArmAndSourceSwitchOrdering();
    TestTimeoutStopsMotionButKeepsControllerArmed();
    TestFreshCommandResumesWithoutAnotherArmRequest();
    TestBrakingAndEstopRejectOtherRequests();
    TestNeutralBandAndAxisMapping();
    TestPs2FrameValidationAndEdges();
    TestParseSerialPacket();
    TestPhoneArmDrivesWithoutPs2();
    TestPhoneArmGuards();
    TestPhoneDisableBrakesAndIsEdgeTriggered();
    TestPs2OverridesPhone();
    return 0;
}
