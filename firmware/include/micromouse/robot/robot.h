#pragma once

#include <micromouse/feedback/feedback_topic.h>

/**
 * This file provides compatibility with Robot class functionality for C
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

// These functions must be called by each platform's code to operate the MicroMouse.

void Robot_Init(void);
void Robot_Periodic(void);
void Robot_OnConnect(void);
void Robot_OnDisconnect(void);
void Robot_PublishPeriodicFeedback(void);

void Robot_ReportError(void);

/**
 * Delegates received feedback data to the appropriate subsystem. The feedback data provided must be the
 * correct size for the given topic.
 *
 * @param topic Feedback topic
 * @param data Pointer to feedback data for the given topic, must be correct size
 */
void Robot_DelegateReceivedFeedback(uint8_t topic, uint8_t* data);

// Period to call Robot_Periodic()
#define ROBOT_UPDATE_PERIOD_MS 2
#define ROBOT_UPDATE_PERIOD_S (ROBOT_UPDATE_PERIOD_MS / 1000.f)

// Period to call Robot_PublishPeriodicFeedback()
#define ROBOT_PUBLISH_FEEDBACK_PERIOD_MS 100
#define ROBOT_PUBLISH_FEEDBACK_PERIOD_S (ROBOT_PUBLISH_FEEDBACK_PERIOD_MS / 1000.f)

#ifdef __cplusplus
}

#include <units/time.h>

#define ROBOT_UPDATE_PERIOD units::second_t{ROBOT_UPDATE_PERIOD_S}
#define ROBOT_PUBLISH_FEEDBACK_PERIOD units::second_t{ROBOT_PUBLISH_FEEDBACK_PERIOD_S}

#endif
