#include "Arduino.h"
#include "CONFIG.h"		
#include "PID_v1.h"

PID::PID() {

	m_output = 0;
	m_Kp = 0;
	m_Ki = 0;
	m_Kd = 0;

	m_integral = 0;
	m_last_error = 0;

	m_last_time = 0;
	m_interval_us = 2500;
}

void PID::set_inteval(uint32_t interval_us) {
	m_interval_us = interval_us;
}

float PID::calculation(float current, float dest) {

   uint32_t current_time = micros();

   if (current_time - m_last_time >= m_interval_us) {
	  
	   // Calculate error
	   float error = dest - current;
	   
	   // Calculate PID
	   float P = m_Kp * error;
	   m_integral += m_Ki * error;
	   float D = m_Kd * (error - m_last_error) / (m_interval_us / 1000.0);
	  
	   // Update variables
	   m_last_error = error;
	   m_last_time = current_time;

	   // Check output range
	   m_output = P + m_integral + D;
	   m_output = constrain(m_output, FCS_PID_MIN_LEVEL, FCS_PID_MAX_LEVEL);
   }

   return m_output;
}

void PID::set_factors(float Kp, float Ki, float Kd) {

	m_Kp = Kp;
	m_Ki = Ki;
	m_Kd = Kd;

	if (m_Ki == 0)
		m_integral = 0;
}