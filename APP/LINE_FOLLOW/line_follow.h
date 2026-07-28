#ifndef APP_LINE_FOLLOW_H
#define APP_LINE_FOLLOW_H

#include <stdbool.h>
#include <stdint.h>

#define LINE_SENSOR_COUNT 12U

void LineFollow_Init(void);
void LineFollow_Update(const uint8_t sensors[LINE_SENSOR_COUNT]);
float LineFollow_GetError(void);
bool LineFollow_IsValid(void);
/* Positive differential requests a left turn; return value is mm/s. */
float LineFollow_GetTurnDiff(void);
/* Common forward speed before applying the turn differential, in mm/s. */
float LineFollow_GetBaseSpeed(void);
bool LineFollow_ConsumeRightAngleEvent(void);

#endif
