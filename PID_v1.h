#ifndef PID_v1_h
#define PID_v1_h

class PID
{
protected:
	float m_output;
	float m_Kp;
	float m_Ki;
	float m_Kd;

	float m_integral;
	float m_last_error;

	uint32_t m_last_time;
	uint32_t m_interval_us;
public:
	PID();
	void set_inteval(uint32_t interval_us);
	float calculation(float current, float dest);
	void set_factors(float P, float I, float D);
};
#endif

