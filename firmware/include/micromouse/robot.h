#pragma once

#include <micromouse/feedback_topic.h>

/**
 * This file provides compatibility with Robot class functionality for C
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

void Robot_Init(void);
void Robot_Periodic(void);
void Robot_OnConnect(void);
void Robot_OnDisconnect(void);
void Robot_PublishPeriodicFeedback(void);
void Robot_PublishExtraFeedback(void);

void Robot_ReportError(void);

// Delegates newly received feedback data to the appropriate subsystem.
//
// This function should be called by platform-specific code controlling the
// underlying hardware receiving feedback topics.
void Robot_DelegateReceivedFeedback(uint8_t topic, uint8_t* data);

#define ROBOT_UPDATE_PERIOD_MS 20
#define ROBOT_UPDATE_PERIOD_S (ROBOT_UPDATE_PERIOD_MS / 1000.f)

#define ROBOT_PUBLISH_FEEDBACK_PERIOD_MS 100
#define ROBOT_PUBLISH_FEEDBACK_PERIOD_S \
  (ROBOT_PUBLISH_FEEDBACK_PERIOD_MS / 1000.f)

#ifdef __cplusplus
}
#endif
