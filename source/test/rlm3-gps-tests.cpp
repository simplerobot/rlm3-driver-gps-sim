#include "Test.hpp"
#include "rlm3-gps.h"
#include "rlm3-task.h"


static volatile size_t g_pulse_count = 0;
static volatile RLM3_Task g_client_thread = nullptr;

extern void RLM3_GPS_PulseCallback()
{
	g_pulse_count++;
	RLM3_GiveFromISR(g_client_thread);
}

TEST_CASE(RLM3_GPS_Init_HappyCase)
{
	ASSERT(!RLM3_GPS_IsInit());
	RLM3_GPS_Init();
	ASSERT(RLM3_GPS_IsInit());
}

TEST_CASE(RLM3_GPS_Init_DoubleInit)
{
	RLM3_GPS_Init();
	ASSERT_ASSERTS(RLM3_GPS_Init());
}

TEST_CASE(RLM3_GPS_Deinit_HappyCase)
{
	RLM3_GPS_Init();
	ASSERT(RLM3_GPS_IsInit());
	RLM3_GPS_Deinit();
	ASSERT(!RLM3_GPS_IsInit());
}

TEST_CASE(RLM3_GPS_Deinit_NotInitalized)
{
	ASSERT_ASSERTS(RLM3_GPS_Deinit());
}

TEST_CASE(RLM3_GPS_GetNextMessage_HappyCase)
{
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02 = {};
	RLM3_GPS_SET_MESSAGE_SIZE(message02);
	message02.message_type = RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION;
	message02.software_type = 1;
	SIM_GPS_Write((RLM3_GPS_MESSAGE*)&message02);
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* result = RLM3_GPS_GetNextMessage(1000);

	ASSERT(result != nullptr);
	ASSERT(result->payload_length == 2);
	ASSERT(result->message_type == 2);
	ASSERT(((RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION*)result)->software_type == 1);
}

TEST_CASE(RLM3_GPS_GetNextMessage_NotInitalized)
{
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02 = {};
	RLM3_GPS_SET_MESSAGE_SIZE(message02);
	message02.message_type = RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION;
	message02.software_type = 1;
	SIM_GPS_Write((RLM3_GPS_MESSAGE*)&message02);

	ASSERT_ASSERTS(RLM3_GPS_GetNextMessage(1000));
}

TEST_CASE(RLM3_GPS_GetNextMessage_NoMessage)
{
	RLM3_GPS_Init();

	const RLM3_GPS_MESSAGE* result = RLM3_GPS_GetNextMessage(1000);

	ASSERT(result == nullptr);
}

TEST_CASE(RLM3_GPS_SendMessage_HappyCase)
{
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02a = {};
	RLM3_GPS_SET_MESSAGE_SIZE(message02a);
	message02a.message_type = RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION;
	message02a.software_type = 1;
	SIM_GPS_Read((RLM3_GPS_MESSAGE*)&message02a, true);
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02b = message02a;
	RLM3_GPS_Init();

	ASSERT(RLM3_GPS_SendMessage((RLM3_GPS_MESSAGE*)&message02b));
}

TEST_CASE(RLM3_GPS_SendMessage_Fails)
{
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02a = {};
	RLM3_GPS_SET_MESSAGE_SIZE(message02a);
	message02a.message_type = RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION;
	message02a.software_type = 1;
	SIM_GPS_Read((RLM3_GPS_MESSAGE*)&message02a, false);
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02b = message02a;
	RLM3_GPS_Init();

	ASSERT(!RLM3_GPS_SendMessage((RLM3_GPS_MESSAGE*)&message02b));
}

TEST_CASE(RLM3_GPS_SendMessage_Missing)
{
	RLM3_GPS_MESSAGE_02_QUERY_SOFTWARE_VERSION message02 = {};
	RLM3_GPS_SET_MESSAGE_SIZE(message02);
	message02.message_type = RLM3_GPS_MESSAGE_TYPE_02_QUERY_SOFTWARE_VERSION;
	message02.software_type = 1;
	RLM3_GPS_Init();

	ASSERT_ASSERTS(RLM3_GPS_SendMessage((RLM3_GPS_MESSAGE*)&message02));
}

TEST_CASE(RLM3_GPS_SendMessage_NULL)
{
	RLM3_GPS_Init();

	ASSERT_ASSERTS(RLM3_GPS_SendMessage(nullptr));
}

TEST_CASE(RLM3_GPS_PulseCallback_HappyCase)
{
	g_pulse_count = 0;
	g_client_thread = RLM3_GetCurrentTask();
	SIM_GPS_Pulse();

	ASSERT(g_pulse_count == 0);
	RLM3_Take();
	ASSERT(g_pulse_count == 1);
}
