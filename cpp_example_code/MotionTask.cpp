#include "MotionTask.h"
#include "TagBoard.h"

#define TASK_PERIOD				50

const uint8_t SUBSCRIBED_EVENTS[] = { EV_START_MOVING };

MotionTask::MotionTask(uint32_t id, const char *name, SystemService &systemService, Board &board)
	: BoardTask(id, name, systemService, board, MOTION_EVENT, SUBSCRIBED_EVENTS, sizeof(SUBSCRIBED_EVENTS) / sizeof(SUBSCRIBED_EVENTS[0]), TASK_PERIOD, osPriorityRealtime, configMINIMAL_STACK_SIZE * 2)
{
	motionSensor = new MotionSensor(((TagBoard *)&board)->GetAccelerometer(), *((TagBoard *)&board)->GetTrembler(), *((TagBoard *)&board)->GetExternalMessageSender());
}

void MotionTask::TaskInit(void)
{
	motionSensor->InitMotionSensor();
}

void MotionTask::TaskLoop(void)
{
	motionSensor->MotionSensorUpdate();
}

MotionTask::~MotionTask()
{
	delete motionSensor;
}