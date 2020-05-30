
/******************* TRABAJO OBLIGATORIO 1 IRIN ******************/
/*****************************************************************/

// GRUPO 1:

// JAVIER LÓPEZ INIESTA DÍAZ DEL CAMPO
// JORQUE QUIJORNA SANTOS
// JORGE ROMEO TERCIADO


#ifndef SUBSUMPTIONGARBAGECONTROLLER_H_
#define SUBSUMPTIONGARBAGECONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:

    CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_wrtie_to_file);
    ~CIri1Controller();
    void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
		/* ROBOT */
  		CEpuck* m_pcEpuck;
   
	 	/* SENSORS */
		CWheelsActuator* m_acWheels;
    		CEpuckProximitySensor* m_seProx;
		CRealLightSensor* m_seLight;
		CRealBlueLightSensor* m_seBlueLight;
		CRealRedLightSensor* m_seRedLight;
		CContactSensor* m_seContact;
		CGroundSensor* m_seGround;
		CGroundMemorySensor* m_seGroundMemory;
		CBatterySensor* m_seBattery;  
		CBlueBatterySensor* m_seBlueBattery;  
		CRedBatterySensor* m_seRedBattery;  
		CEncoderSensor* m_seEncoder;  
		CCompassSensor* m_seCompass;  

		/* Global Variables */
		double 		m_fLeftSpeed;
		double 		m_fRightSpeed;
		double**	m_fActivationTable;
		int 		m_nWriteToFile;
		double 		m_fTime;
    		double 		fBattToServeInhibitor;
	 	double		TurnOffBlueLight;

		/* Functions */

		void ExecuteBehaviors ( void );
		void Coordinator ( void );

		void ObstacleAvoidance ( unsigned int un_priority );
                void Navigate ( unsigned int un_priority );
		void GoLoad ( unsigned int un_priority );
                void Serve ( unsigned int un_priority );
                void GoToBar ( unsigned int un_priority );
};

#endif
