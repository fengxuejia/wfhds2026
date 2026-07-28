#include "line_follow.h"

/* Speeds and turn differentials are in mm/s. A positive differential slows
 * the left wheel and accelerates the right wheel, producing a left turn. */
#define TURN_90_DIFF 120.0f
#define TURN_MAX_DIFF 90.0f
#define TURN_MID_DIFF 50.0f
#define TURN_MIN_DIFF 30.0f
#define BASE_SPEED 250.0f
#define TURN_SLOWDOWN_THRESHOLD 80.0f
#define SHARP_TURN_BASE_SPEED 140.0f
/* Maximum differential change accepted from one sensor frame to the next. */
#define TURN_STEP_LIMIT 35.0f

#define RIGHT_ANGLE_MAX_TICKS 4000U

typedef enum
{
    STATE_STRAIGHT_1 = 0x060, // 000001100000
    STATE_STRAIGHT_2 = 0x040, // 000001000000
    STATE_STRAIGHT_3 = 0x020, // 000000100000

    STATE_LEFT_SMALL_1 = 0x0C0, // 000011000000
    STATE_LEFT_SMALL_2 = 0x100, // 000100000000
    STATE_LEFT_SMALL_3 = 0x200, // 001000000000
    STATE_LEFT_SMALL_4 = 0x600, // 011000000000
    STATE_LEFT_SMALL_5 = 0x1C0, // 000111000000
    STATE_LEFT_SMALL_6 = 0x380, // 001110000000

    STATE_LEFT_MID_1 = 0xC00, // 110000000000
    STATE_LEFT_BIG_1 = 0x800, // 100000000000

    STATE_LEFT_90_1 = 0xE00, // 111000000000
    STATE_LEFT_90_2 = 0xF00, // 111100000000
    STATE_LEFT_90_3 = 0xF80, // 111110000000
    STATE_LEFT_90_4 = 0xFC0, // 111111000000
    STATE_LEFT_90_5 = 0xFE0, // 111111100000
    STATE_LEFT_90_6 = 0xFF0, // 111111110000
    STATE_LEFT_90_7 = 0xFF8, // 111111111000
    STATE_LEFT_90_8 = 0xFFC, // 111111111100
    STATE_LEFT_90_9 = 0x7C0, // 011111000000

    STATE_RIGHT_SMALL_1 = 0x010, // 000000010000
    STATE_RIGHT_SMALL_2 = 0x030, // 000000110000
    STATE_RIGHT_SMALL_3 = 0x018, // 000000011000
    STATE_RIGHT_SMALL_4 = 0x00C, // 000000001100
    STATE_RIGHT_SMALL_5 = 0x038, // 000000111000
    STATE_RIGHT_SMALL_6 = 0x01C, // 000000011100

    STATE_RIGHT_MID_1 = 0x006, // 000000000110
    STATE_RIGHT_BIG_1 = 0x003, // 000000000011

    STATE_RIGHT_90_1 = 0x007, // 000000000111
    STATE_RIGHT_90_2 = 0x00F, // 000000001111
    STATE_RIGHT_90_3 = 0x01F, // 000000011111
    STATE_RIGHT_90_4 = 0x03F, // 000000111111
    STATE_RIGHT_90_5 = 0x07F, // 000001111111
    STATE_RIGHT_90_6 = 0x0FF, // 000011111111
    STATE_RIGHT_90_7 = 0x1FF, // 000111111111
    STATE_RIGHT_90_8 = 0x3FF, // 001111111111

    STATE_LOST = 0x000 // 000000000000
} SensorState_t;

static float line_error;
static float turn_diff;
static float base_speed;
static bool line_valid;
static SensorState_t last_state;
static uint16_t right_angle_ticks;
static int8_t right_angle_direction;
static bool right_angle_event;
static float previous_turn_diff;

static float absolute_float(float value)
{
    return value < 0.0f ? -value : value;
}

static float sensor_turn(uint16_t pattern)
{
    static const int8_t weights[LINE_SENSOR_COUNT] =
        { 6, 5, 4, 3, 2, 1, -1, -2, -3, -4, -5, -6 };
    int32_t weighted_sum = 0;
    uint8_t active_count = 0U;

    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        if ((pattern & (uint16_t)(1U << (11U - i))) != 0U)
        {
            weighted_sum += weights[i];
            active_count++;
        }
    }

    if (active_count == 0U)
    {
        return 0.0f;
    }

    /* Positive values steer left; negative values steer right. */
    return (float)weighted_sum * 15.0f / (float)active_count;
}

static uint16_t pack_sensor_state(
    const uint8_t sensors[LINE_SENSOR_COUNT])
{
    uint16_t state = 0U;

    /* Bit 11 is sensor 0 and bit 0 is sensor 11. */
    for (uint8_t i = 0U; i < LINE_SENSOR_COUNT; i++)
    {
        state |= (uint16_t)((sensors[i] & 1U) << (11U - i));
    }
    return state;
}

static bool is_left_90_state(SensorState_t state)
{
    return state == STATE_LEFT_90_1 || state == STATE_LEFT_90_2 ||
           state == STATE_LEFT_90_3 || state == STATE_LEFT_90_4 ||
           state == STATE_LEFT_90_5 || state == STATE_LEFT_90_6 ||
           state == STATE_LEFT_90_7 || state == STATE_LEFT_90_8 ||
           state == STATE_LEFT_90_9;
}

static bool is_right_90_state(SensorState_t state)
{
    return state == STATE_RIGHT_90_1 || state == STATE_RIGHT_90_2 ||
           state == STATE_RIGHT_90_3 || state == STATE_RIGHT_90_4 ||
           state == STATE_RIGHT_90_5 || state == STATE_RIGHT_90_6 ||
           state == STATE_RIGHT_90_7 || state == STATE_RIGHT_90_8;
}

static SensorState_t classify_state(const uint8_t sensors[LINE_SENSOR_COUNT])
{
    uint16_t pattern = pack_sensor_state(sensors);
    return pattern == STATE_LOST ? STATE_LOST : (SensorState_t)pattern;
}

static void update_right_angle(SensorState_t raw_state)
{
    bool left_candidate = is_left_90_state(raw_state);
    bool right_candidate = is_right_90_state(raw_state);

    if (right_angle_ticks == 0U)
    {
        if (left_candidate || right_candidate)
        {
            /* Use the same sign convention as turn_diff: positive is left. */
            right_angle_direction = left_candidate ? 1 : -1;
            right_angle_ticks = 1U;
            right_angle_event = true;
        }
    }
    else if ((!left_candidate && !right_candidate) ||
             right_angle_ticks >= RIGHT_ANGLE_MAX_TICKS)
    {
        right_angle_ticks = 0U;
        right_angle_direction = 0;
    }
    else
    {
        right_angle_ticks++;
    }
}

static float state_turn(SensorState_t state, uint16_t pattern)
{
    switch (state)
    {
    case STATE_LEFT_90_1:
    case STATE_LEFT_90_2:
    case STATE_LEFT_90_3:
    case STATE_LEFT_90_4:
    case STATE_LEFT_90_5:
    case STATE_LEFT_90_6:
    case STATE_LEFT_90_7:
    case STATE_LEFT_90_8:
    case STATE_LEFT_90_9:
        return TURN_90_DIFF;
    case STATE_LEFT_BIG_1:
        return TURN_MAX_DIFF;
    case STATE_LEFT_MID_1:
        return TURN_MID_DIFF;
    case STATE_LEFT_SMALL_1:
    case STATE_LEFT_SMALL_2:
    case STATE_LEFT_SMALL_3:
    case STATE_LEFT_SMALL_4:
    case STATE_LEFT_SMALL_5:
    case STATE_LEFT_SMALL_6:
        return TURN_MIN_DIFF;
    case STATE_RIGHT_90_1:
    case STATE_RIGHT_90_2:
    case STATE_RIGHT_90_3:
    case STATE_RIGHT_90_4:
    case STATE_RIGHT_90_5:
    case STATE_RIGHT_90_6:
    case STATE_RIGHT_90_7:
    case STATE_RIGHT_90_8:
        return -TURN_90_DIFF;
    case STATE_RIGHT_BIG_1:
        return -TURN_MAX_DIFF;
    case STATE_RIGHT_MID_1:
        return -TURN_MID_DIFF;
    case STATE_RIGHT_SMALL_1:
    case STATE_RIGHT_SMALL_2:
    case STATE_RIGHT_SMALL_3:
    case STATE_RIGHT_SMALL_4:
    case STATE_RIGHT_SMALL_5:
    case STATE_RIGHT_SMALL_6:
        return -TURN_MIN_DIFF;
    case STATE_STRAIGHT_1:
    case STATE_STRAIGHT_2:
    case STATE_STRAIGHT_3:
        return 0.0f;
    case STATE_LOST:
        if (last_state == STATE_LEFT_SMALL_1 ||
            last_state == STATE_LEFT_SMALL_2 ||
            last_state == STATE_LEFT_SMALL_3 ||
            last_state == STATE_LEFT_SMALL_4 ||
            last_state == STATE_LEFT_SMALL_5 ||
            last_state == STATE_LEFT_SMALL_6 ||
            last_state == STATE_LEFT_MID_1)
        {
            return TURN_MID_DIFF;
        }
        if (last_state == STATE_RIGHT_SMALL_1 ||
            last_state == STATE_RIGHT_SMALL_2 ||
            last_state == STATE_RIGHT_SMALL_3 ||
            last_state == STATE_RIGHT_SMALL_4 ||
            last_state == STATE_RIGHT_SMALL_5 ||
            last_state == STATE_RIGHT_SMALL_6 ||
            last_state == STATE_RIGHT_MID_1)
        {
            return -TURN_MID_DIFF;
        }
        if (last_state == STATE_LEFT_BIG_1)
        {
            return TURN_MAX_DIFF;
        }
        if (last_state == STATE_RIGHT_BIG_1)
        {
            return -TURN_MAX_DIFF;
        }
        if (is_left_90_state(last_state))
        {
            return TURN_90_DIFF;
        }
        if (is_right_90_state(last_state))
        {
            return -TURN_90_DIFF;
        }
        /* Keep searching in the last observed direction for unlisted patterns. */
        return previous_turn_diff;
    default:
        return sensor_turn(pattern);
    }
}

void LineFollow_Init(void)
{
    line_error = 0.0f;
    turn_diff = 0.0f;
    base_speed = BASE_SPEED;
    line_valid = false;
    last_state = STATE_STRAIGHT_1;
    right_angle_ticks = 0U;
    right_angle_direction = 0;
    right_angle_event = false;
    previous_turn_diff = 0.0f;
}

void LineFollow_Update(const uint8_t sensors[LINE_SENSOR_COUNT])
{
    SensorState_t raw_state = classify_state(sensors);
    uint16_t pattern = pack_sensor_state(sensors);
    line_error = sensor_turn(pattern);

    /* Preserve the latest valid direction for lost-line recovery, including
     * the right-angle states handled by the special turn branch below. */
    if (raw_state != STATE_LOST)
    {
        last_state = raw_state;
    }

    update_right_angle(raw_state);

    if (right_angle_ticks != 0U)
    {
        line_valid = true;
        turn_diff = (float)right_angle_direction * TURN_90_DIFF;
    }
    else
    {
        line_valid = raw_state != STATE_LOST;
        /* Use calibrated states where available and weighted position for
         * sensor patterns that are not explicitly listed. */
        turn_diff = state_turn(raw_state, pattern);
    }

    /* Limit one-frame changes to prevent a corner transition from causing
     * an immediate full correction in the opposite direction. */
    if (turn_diff > previous_turn_diff + TURN_STEP_LIMIT)
    {
        turn_diff = previous_turn_diff + TURN_STEP_LIMIT;
    }
    else if (turn_diff < previous_turn_diff - TURN_STEP_LIMIT)
    {
        turn_diff = previous_turn_diff - TURN_STEP_LIMIT;
    }
    previous_turn_diff = turn_diff;

    float turn_magnitude = absolute_float(turn_diff);
    if (raw_state != STATE_LOST && turn_magnitude < TURN_SLOWDOWN_THRESHOLD)
    {
        /* Reduce common speed progressively as the requested turn tightens. */
        base_speed = BASE_SPEED -
                     (BASE_SPEED * turn_magnitude /
                      (TURN_SLOWDOWN_THRESHOLD * 2.0f));
    }
    else
    {
        /* Search for a lost line and negotiate sharp turns at low speed. */
        base_speed = SHARP_TURN_BASE_SPEED;
    }
}

float LineFollow_GetError(void)
{
    return line_error;
}

bool LineFollow_IsValid(void)
{
    return line_valid;
}

float LineFollow_GetTurnDiff(void)
{
    return turn_diff;
}

float LineFollow_GetBaseSpeed(void)
{
    return base_speed;
}

bool LineFollow_ConsumeRightAngleEvent(void)
{
    bool event = right_angle_event;

    right_angle_event = false;
    return event;
}
