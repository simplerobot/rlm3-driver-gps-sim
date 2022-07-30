#include "rlm3-gps.h"
#include "Test.hpp"
#include <vector>
#include <queue>
#include "rlm3-sim.hpp"


static bool g_is_initialized = false;
static std::queue<std::vector<uint8_t>> g_write_messages;
static std::queue<std::pair<std::vector<uint8_t>, bool>> g_read_messages;
static std::vector<uint8_t> g_current_message;


static std::vector<uint8_t> ToBytes(const RLM3_GPS_MESSAGE* message)
{
	ASSERT(message != nullptr);
	size_t size = 2 + message->payload_length;
	std::vector<uint8_t> message_raw((uint8_t*)message, (uint8_t*)message + size);
	return message_raw;
}

extern void RLM3_GPS_Init()
{
	ASSERT(!g_is_initialized);
	g_is_initialized = true;
}

extern void RLM3_GPS_Deinit()
{
	ASSERT(g_is_initialized);
	g_is_initialized = false;
}

extern bool RLM3_GPS_IsInit()
{
	return g_is_initialized;
}

extern const RLM3_GPS_MESSAGE* RLM3_GPS_GetNextMessage(size_t timeout_ms)
{
	ASSERT(g_is_initialized);
	if (g_write_messages.empty())
		return nullptr;
	g_current_message = g_write_messages.front();
	g_write_messages.pop();
	return (RLM3_GPS_MESSAGE*)g_current_message.data();
}

extern bool RLM3_GPS_SendMessage(const RLM3_GPS_MESSAGE* message)
{
	ASSERT(g_is_initialized);
	ASSERT(!g_read_messages.empty());
	ASSERT(g_read_messages.front().first == ToBytes(message));
	bool result = g_read_messages.front().second;
	g_read_messages.pop();
	return result;
}

extern __attribute((weak)) void RLM3_GPS_PulseCallback()
{
}

extern __attribute((weak)) void RLM3_GPS_ErrorCallback()
{
}

extern void SIM_GPS_Write(const RLM3_GPS_MESSAGE* message)
{
	g_write_messages.push(ToBytes(message));
}

extern void SIM_GPS_Read(const RLM3_GPS_MESSAGE* message, bool result)
{
	g_read_messages.emplace(ToBytes(message), result);
}

extern void SIM_GPS_Pulse()
{
	SIM_AddInterrupt([]() {
		RLM3_GPS_PulseCallback();
	});
}

TEST_SETUP(SIM_GPS_Initialization)
{
	g_is_initialized = false;
	while (!g_write_messages.empty())
		g_write_messages.pop();
	while (!g_read_messages.empty())
		g_read_messages.pop();
	g_current_message.clear();
}
