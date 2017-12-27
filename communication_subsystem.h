#ifndef __COMMUNICATION_SUBSYSTEM_H__
#define __COMMUNICATION_SUBSYSTEM_H__

// CSS - Communication SubSystem

// Error codes
namespace CSS {

	const uint32_t NO_ERROR			= 0x00;
	const uint32_t DESYNC			= 0x01;
	const uint32_t CONNECTION_LOST	= 0x02;
}

// External interface
namespace CSS {

	void initialize();
	void process();
	uint32_t get_status();
}

extern TXRX::control_data_t g_cp;
extern TXRX::state_data_t g_sp;

#endif /* __COMMUNICATION_SUBSYSTEM_H__ */