
/******************* TRABAJO OBLIGATORIO 1 IRIN ******************/
/*****************************************************************/

// GRUPO 1:

// JAVIER LÓPEZ INIESTA DÍAZ DEL CAMPO
// JORQUE QUIJORNA SANTOS
// JORGE ROMEO TERCIADO

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

#include <math.h>
#ifndef MPI
#define M_PI 3.14159265358979323846
#endif

/******************** Simulator ****************/
/******************** Sensors ******************/
#include "epuckproximitysensor.h"
#include "contactsensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"


/******************************************************************************/
/******************************************************************************/

extern gsl_rng* rng;
extern long int rngSeed;
/******************************************************************************/
/******************************************************************************/

using namespace std;
/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS		 5

#define AVOID_PRIORITY 		 0
#define RELOAD_PRIORITY 	 1
#define SERVE_PRIORITY		 2
#define GOTOBAR_PRIORITY         3
#define NAVIGATE_PRIORITY  	 4


/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.6

/* Threshold to define the battery discharged */
#define BATTERY_THRESHOLD 0.15

/* Threshold to reduce the velocity in the restaurant */
#define SPEED_THRESHOLD 0.6


#define SPEED 1000

/******************************************************************************/
/******************************************************************************/
CIri1Controller::CIri1Controller (const char* pch_name, CEpuck* pc_epuck, int n_write_to_file) : CController (pch_name, pc_epuck)

{
	m_nWriteToFile = n_write_to_file;
	
	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator*) m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor*) m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor*) m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set contact Sensor */
	m_seContact = (CContactSensor*) m_pcEpuck->GetSensor (SENSOR_CONTACT);
	/* Set ground Sensor */
	m_seGround = (CGroundSensor*) m_pcEpuck->GetSensor (SENSOR_GROUND);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor*) m_pcEpuck->GetSensor (SENSOR_GROUND_MEMORY);
	/* Set battery Sensor */
	m_seBattery = (CBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BATTERY);
	/* Set blue battery Sensor */
	m_seBlueBattery = (CBlueBatterySensor*) m_pcEpuck->GetSensor (SENSOR_BLUE_BATTERY);
	/* Set red battery Sensor */
	m_seRedBattery = (CRedBatterySensor*) m_pcEpuck->GetSensor (SENSOR_RED_BATTERY);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor*) m_pcEpuck->GetSensor (SENSOR_ENCODER);
  	m_seEncoder->InitEncoderSensor(m_pcEpuck);
	/* Set compass Sensor */
	m_seCompass = (CCompassSensor*) m_pcEpuck->GetSensor (SENSOR_COMPASS);


  	fBattToServeInhibitor = 1.0;
	TurnOffBlueLight == 0.0;

	/* Initilize Variables */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;

	m_fActivationTable = new double* [BEHAVIORS];
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i] = new double[3];
	}

}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		delete [] m_fActivationTable;
	}
}


/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

	if (m_nWriteToFile ) 
	{
	/* INIT: WRITE TO FILES */
	/* Write robot position and orientation */
		FILE* filePosition = fopen("outputFiles/robotPosition", "a");
		fprintf(filePosition,"%2.4f %2.4f %2.4f %2.4f\n", m_fTime, m_pcEpuck->GetPosition().x, m_pcEpuck->GetPosition().y, m_pcEpuck->GetRotation());
		fclose(filePosition);

		/* Write robot wheels speed */
		FILE* fileWheels = fopen("outputFiles/robotWheels", "a");
		fprintf(fileWheels,"%2.4f %2.4f %2.4f \n", m_fTime, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileWheels);
		/* END WRITE TO FILES */
	}

}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors ( void )
{
	for ( int i = 0 ; i < BEHAVIORS ; i++ )
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Release Inhibitors */
	fBattToServeInhibitor = 1.0;
	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(	LED_COLOR_BLACK);
	
	ObstacleAvoidance ( AVOID_PRIORITY );   
        GoLoad ( RELOAD_PRIORITY );
        Serve ( SERVE_PRIORITY );
        GoToBar ( GOTOBAR_PRIORITY );
        Navigate ( NAVIGATE_PRIORITY );


}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator ( void )
{
	int nBehavior;
	for ( nBehavior = 0 ; nBehavior < BEHAVIORS ; nBehavior++ )
	{
		if ( m_fActivationTable[nBehavior][2] == 1.0 )
		{
			break;
		}
	}

	m_fLeftSpeed = m_fActivationTable[nBehavior][0];
	m_fRightSpeed = m_fActivationTable[nBehavior][1];
	
  printf("%d %2.4f %2.4f \n", nBehavior, m_fLeftSpeed, m_fRightSpeed);
 

	
  if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write coordinator ouputs */
		FILE* fileOutput = fopen("outputFiles/coordinatorOutput", "a");
		fprintf(fileOutput,"%2.4f %d %2.4f %2.4f \n", m_fTime, nBehavior, m_fLeftSpeed, m_fRightSpeed);
		fclose(fileOutput);

		if (nBehavior == 0){
			FILE* fileOutput = fopen("outputFiles/behaviorOutput", "a");
			fprintf(fileOutput,"%2.4f 1 0 0 0 0\n", m_fTime);
			fclose(fileOutput);
		}
		else if(nBehavior == 1){
			FILE* fileOutput = fopen("outputFiles/behaviorOutput", "a");
			fprintf(fileOutput,"%2.4f 0 1 0 0 0\n", m_fTime);
			fclose(fileOutput);
		}
		else if(nBehavior == 2){
			FILE* fileOutput = fopen("outputFiles/behaviorOutput", "a");
			fprintf(fileOutput,"%2.4f 0 0 1 0 0\n", m_fTime);
			fclose(fileOutput);
		}
		else if(nBehavior == 3){
			FILE* fileOutput = fopen("outputFiles/behaviorOutput", "a");
			fprintf(fileOutput,"%2.4f 0 0 0 1 0\n", m_fTime);
			fclose(fileOutput);
		} else{
			FILE* fileOutput = fopen("outputFiles/behaviorOutput", "a");
			fprintf(fileOutput,"%2.4f 0 0 0 0 1\n", m_fTime);
			fclose(fileOutput);
		}
		/* END WRITE TO FILES */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance ( unsigned int un_priority )
{
	
	/* Leer Sensores de Proximidad */
	double* prox = m_seProx->GetSensorReading(m_pcEpuck);

	double fMaxProx = 0.0;
	const double* proxDirections = m_seProx->GetSensorDirections();

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for ( int i = 0 ; i < m_seProx->GetNumberOfInputs() ; i ++ )
	{
		vRepelent.x += prox[i] * cos ( proxDirections[i] );
		vRepelent.y += prox[i] * sin ( proxDirections[i] );

		if ( prox[i] > fMaxProx )
			fMaxProx = prox[i];
	}
	
	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while ( fRepelent > M_PI ) fRepelent -= 2 * M_PI;
	while ( fRepelent < -M_PI ) fRepelent += 2 * M_PI;

	/* If above a threshold */
	if ( fMaxProx > PROXIMITY_THRESHOLD )
	{
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);
		printf("Obstacle detected\n");

		double fCLinear = 1.0;
		double fCAngular = 1.0;
		double fC1 = SPEED / M_PI;
		
		/* Calc Linear Speed */
		double fVLinear = SPEED * fCLinear * ( cos ( fRepelent / 2) );

		/*Calc Angular Speed */
		double fVAngular = fRepelent;

		m_fActivationTable[un_priority][0] = fVLinear - fC1 * fVAngular;
		m_fActivationTable[un_priority][1] = fVLinear + fC1 * fVAngular;
		m_fActivationTable[un_priority][2] = 1.0;
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate ( unsigned int un_priority )
{
	/* Leer Sensores de Luz */
        double* red_light = m_seRedLight->GetSensorReading(m_pcEpuck);
        double fTotalRedLight = 0.0;

	for ( int i = 0 ; i < m_seRedLight->GetNumberOfInputs() ; i ++ )
	{
                fTotalRedLight += red_light[i];
	}
	
        if (fTotalRedLight >= SPEED_THRESHOLD)
        {
	        m_fActivationTable[un_priority][0] = SPEED - 300;
                m_fActivationTable[un_priority][1] = SPEED - 300;
        }

        else
        {
                m_fActivationTable[un_priority][0] = SPEED;
                m_fActivationTable[un_priority][1] = SPEED;
        }
         
        m_fActivationTable[un_priority][2] = 1.0;

	if (m_nWriteToFile ) 
	{
		/* INIT: WRITE TO FILES */
		/* Write level of competence ouputs */
		FILE* fileOutput = fopen("outputFiles/navigateOutput", "a");
		if(fTotalRedLight >= SPEED_THRESHOLD){		
			fprintf(fileOutput,"%2.4f 1 0 %2.4f %2.4f %2.4f\n", m_fTime, m_fActivationTable[un_priority][2],m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
			fclose(fileOutput);
		}
		else {		
			fprintf(fileOutput,"%2.4f 0 1 %2.4f %2.4f %2.4f\n", m_fTime, m_fActivationTable[un_priority][2], m_fActivationTable[un_priority][0], m_fActivationTable[un_priority][1]);
			fclose(fileOutput);
		} 

		/* END WRITE TO FILES */
	}

}
		
/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoLoad ( unsigned int un_priority )
{
	/* Leer Battery Sensores */
	double* battery = m_seBattery->GetSensorReading(m_pcEpuck);
		
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	printf("Battery: %2f \n",battery[0]);

	if ( battery[0] < BATTERY_THRESHOLD )
	{
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);
		printf("Low Battery Level\n");

		fBattToServeInhibitor = 0.0;

		/* If not pointing to the light */
			if ( ( light[0] * light[7] == 0.0 ) )
			{
				m_fActivationTable[un_priority][2] = 1.0;

				double lightLeft = light[0] + light[1] + light[2] + light[3];
				double lightRight = light[4] + light[5] + light[6] + light[7];

				if ( lightLeft > lightRight )
				{
					m_fActivationTable[un_priority][0] = -SPEED;
					m_fActivationTable[un_priority][1] = SPEED;
				}
				else
				{
					m_fActivationTable[un_priority][0] = SPEED;
					m_fActivationTable[un_priority][1] = -SPEED;
				}
			}
		
	}
	

	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/batteryOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f \n", m_fTime, battery[0]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Serve ( unsigned int un_priority )
{
	/* Leer Sensores de Suelo Memory */
	double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	
	/* Leer Sensores de Luz */
	double* bluelight = m_seBlueLight->GetSensorReading(m_pcEpuck);

	/* If with a virtual puck */
	if ( ( groundMemory[0] * fBattToServeInhibitor ) == 1.0 )
	{


		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);

		TurnOffBlueLight = 1.0;

		/* Go to the blue light */
		if ( ( bluelight[0] * bluelight[7] == 0.0 ) )
			{
			m_fActivationTable[un_priority][2] = 1.0;
				double bluelightLeft = bluelight[0] + bluelight[1] + bluelight[2] + bluelight[3];
				double bluelightRight = bluelight[4] + bluelight[5] + bluelight[6] + bluelight[7];


				if ( bluelightLeft > bluelightRight )
				{
					m_fActivationTable[un_priority][0] = -SPEED;
					m_fActivationTable[un_priority][1] = SPEED;
				}
				else
				{
					m_fActivationTable[un_priority][0] = SPEED;
					m_fActivationTable[un_priority][1] = -SPEED;
				}
			}
	}

	if (m_nWriteToFile ) 
	{
		/* INIT WRITE TO FILE */
		FILE* fileOutput = fopen("outputFiles/serveOutput", "a");
		fprintf(fileOutput, "%2.4f %2.4f %2.4f\n", m_fTime, fBattToServeInhibitor, groundMemory[0]);
		fclose(fileOutput);
		/* END WRITE TO FILE */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoToBar ( unsigned int un_priority )
{
	/* Leer Battery Sensores */

        double* groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
		
	/* Leer Sensores de Luz */
	double* light = m_seLight->GetSensorReading(m_pcEpuck);
	double* red_light = m_seRedLight->GetSensorReading(m_pcEpuck);
        double fTotalRedLight = 0.0;

	for ( int i = 0 ; i < m_seLight->GetNumberOfInputs() ; i ++ )
	{
                fTotalRedLight += red_light[i];
	}


		/* If not pointing to the light */
			
	if ( ((groundMemory[0] * fBattToServeInhibitor) != 1.0) && (fTotalRedLight >= SPEED_THRESHOLD))
	{			


			if (TurnOffBlueLight == 1.0){
				m_seBlueLight -> SwitchNearestLight(0);
				TurnOffBlueLight = 0.0;
				printf("Customer served \n");
			}

                        if ( ( light[0] * light[7] == 0.0 ) )
			{
				m_fActivationTable[un_priority][2] = 1.0;

				double lightLeft = light[0] + light[1] + light[2] + light[3];
				double lightRight = light[4] + light[5] + light[6] + light[7];

				if ( lightLeft > lightRight )
				{
					m_fActivationTable[un_priority][0] = -SPEED;
					m_fActivationTable[un_priority][1] = SPEED;
				}
				else
				{
					m_fActivationTable[un_priority][0] = SPEED;
					m_fActivationTable[un_priority][1] = -SPEED;
				}
			}
	  }

	printf("----------------- \n");	

}

