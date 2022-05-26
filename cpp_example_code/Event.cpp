#include "Event.h"
#include "math.h"

Event Event::instance;

Event::Event()
{
	GetEventArrayId = [](uint32_t listenerID) {return (uint8_t)log2(listenerID);};
	
//	Event::GetInstance().AddTaskEvents();
	
	//Event lambdas to simplify event processing
//	uint32_t &taskIdRef = taskId;
	
	osSemaphoreDef_t semaphoreDef;	
	eventSemaphore = osSemaphoreCreate(&semaphoreDef, 1); 
	osSemaphoreRelease(eventSemaphore);	
}
/*******************************************************************************
	 Function         : GetInstance
	 Description      : Gets instance of Event singleton
	 Parameters       : None
	 Return           : Instance of  Event
*******************************************************************************/
Event & Event::GetInstance(void)
{
	return instance;
}

/*******************************************************************************
 Function         : AddTaskEvents 
 Description      : Adds Events for the task that it listening to
 Parameters       : - listenerID - Id of the task
					- eventsIds - array of events
					- numOfEvents - size of events array
 Return           : None
 Globals Accessed : Yes
 *******************************************************************************/
void Event::AddTaskEvents(uint32_t listenerID, uint8_t const * const eventsIds, uint8_t numOfEvents)
{
	uint8_t id = GetEventArrayId(listenerID);
	
	if (id < TASKS_NUMBER)
	{
		if (taskEvents[id] != NULL)
			delete taskEvents[id];
			
		if (numOfEvents > 0 && eventsIds != NULL)
			taskEvents[id] = new osEvents(eventsIds, numOfEvents);
	}	
}	

/*******************************************************************************
 Function         : SetEvent 
 Description      : Set event fr all tasks who subscribed to it
 Parameters       : - listenerID - Id of the task
					- eventsId - Event Id
 Return           : None
 Globals Accessed : Yes
 *******************************************************************************/
void Event::SetEvent(uint32_t listenerID, uint8_t eventId)
{
	if (osSemaphoreWait(eventSemaphore, osWaitForever) == osOK)
	{			
		for (auto &te : taskEvents)
			if (te != NULL)
				te->SetEvent(eventId);
			
		osSemaphoreRelease(eventSemaphore);	
	}
}

/*******************************************************************************
 Function         : GetEvent 
 Description      : Get Event for the task if subscibed
 Parameters       : - listenerID - Id of the task
					- eventsId - Event Id
					- shouldClear - true if event needs to be clearaed after reading
 Return           : true if event is set
 Globals Accessed : Yes
 *******************************************************************************/
bool Event::CheckEvent(uint32_t listenerID, uint8_t eventId, bool shouldClear)
{
	uint8_t id = GetEventArrayId(listenerID);
	if (id <= TASKS_NUMBER)
		return taskEvents[id]->CheckEvent(eventId, shouldClear);
	
	return false;
}

/*******************************************************************************
 Function         : ClearEvent 
 Description      : Clears Event for the task if subscibed
 Parameters       : - listenerID - Id of the task
					- eventsId - Event Id					
 Return           : None
 Globals Accessed : Yes
 *******************************************************************************/
void Event::ClearEvent(uint32_t listenerID, uint8_t eventId)
{
	uint8_t id = GetEventArrayId(listenerID);
	if (id < TASKS_NUMBER)	
		return taskEvents[id]->ClearEvent(eventId);
}	
	
Event::~Event()
{
	osSemaphoreDelete(eventSemaphore);
}