
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

/******************** Simulator ****************/
#include <vector>
#include "simmath.h"
#include "random.h"
#include "programmedarena.h"

#include "iri1exp.h"

/******************** Sensors ******************/
#include "contactsensor.h"
#include "epuckproximitysensor.h"
#include "reallightsensor.h"
#include "realbluelightsensor.h"
#include "realredlightsensor.h"
#include "groundsensor.h"
#include "groundmemorysensor.h"
#include "batterysensor.h"
#include "bluebatterysensor.h"
#include "redbatterysensor.h"
#include "encodersensor.h"
#include "encodersensor.h"
#include "compasssensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controllers **************/
#include "iri1controller.h"

using namespace std;

/*Create Arena */
static char* pchHeightMap = 

"%%%%%%%%%%%%%%%%%%%%"
"%%%########%#######%"
"%%%########%#######%"
"%%%#####%##%#######%"
"%%%##%##%##%#######%"
"%%%##%##%##%#######%"
"%%%##%##%##%#######%"
"%%%##%##%##%#######%"
"%%%##%##%%%%%######%"
"%%%##%#############%"
"%####%#############%"
"%%%##%%%%##%%######%"
"%%%##%#####%#######%"
"%%%##%#####%#######%"
"%%%##%##%%%%#######%"
"%%%##%##%##%#######%"
"%%%##%##%##%#######%"
"%%%########%#######%"
"%%%########%#######%"
"%%%%%%%%%%%%%%%%%%%%";


extern gsl_rng* rng;
extern long int rngSeed;

/*******************************************************************************/
///*******************************************************************************/
//
CIri1Exp::CIri1Exp(const char* pch_name, const char* paramsFile) :
	CExperiment(pch_name, COLLISION_MODEL_SIMPLE, COLLISION_HANDLER_POSITION)
{

	m_fLightSensorRange = 1.0; //1 meter
	m_fBlueLightSensorRange = 1.0; //1 meter
	
	/* If there is not a parameter file input get default values*/
	if (paramsFile == NULL )
	{
		m_nRobotsNumber = 1;
		SetNumberOfEpucks(m_nRobotsNumber);
		m_pcvRobotPositions = new dVector2[m_nRobotsNumber];
		m_fRobotOrientations = new double[m_nRobotsNumber];
		for ( int i = 0 ; i < m_nRobotsNumber ; i++)
		{
			m_pcvRobotPositions[i].x 	= 0.0;
			m_pcvRobotPositions[i].y 	= 0.0;
			m_fRobotOrientations[i] 	= 0.0;
		}
		m_nRunTime = 10000;


		m_nLightObjectNumber = 0;
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
	
		m_nBlueLightObjectNumber = 0;
		m_pcvBlueLightObjects = new dVector2[m_nBlueLightObjectNumber];
		
		m_nNumberOfGroundArea = 0;
		m_vGroundAreaCenter = new dVector2[m_nNumberOfGroundArea];
		m_fGroundAreaExternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaInternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaColor = new double[m_nNumberOfGroundArea];
	}
	/* Else, extract info from the file */
	else
	{
		ifstream pfile(paramsFile);
		if(!pfile) {
      cerr << "Can't find parameters file " << endl;
			exit(0);
		}	

		/* EXTRA */
		
		/* Get number of robots */
		m_nRobotsNumber = getInt('=',pfile);
		/* Set numer of robots */
		SetNumberOfEpucks(m_nRobotsNumber);
		/* For each robot get position and orientation */
		m_pcvRobotPositions = new dVector2[m_nRobotsNumber];
		m_fRobotOrientations = new double[m_nRobotsNumber];
		for ( int i = 0 ; i < m_nRobotsNumber ; i++)
		{
			m_pcvRobotPositions[i].x 	= getDouble('=',pfile);
			m_pcvRobotPositions[i].y 	= getDouble('=',pfile);
			m_fRobotOrientations[i] 	= getDouble('=',pfile);
		}
		
		/* Get write to file flag */
		m_nWriteToFile 	= getInt('=',pfile);
		m_nRunTime = getInt('=',pfile);

		/* ENVIRONMENT */
		
		/* Lights */
		/* Get Light Objects Number */
		m_nLightObjectNumber = getInt('=',pfile);
		/* Create Objects */
		m_pcvLightObjects = new dVector2[m_nLightObjectNumber];
		for ( int i = 0 ; i < m_nLightObjectNumber; i++){
			/* Get X position */
			m_pcvLightObjects[i].x = getDouble('=',pfile);
			/* Get Y Position */
			m_pcvLightObjects[i].y = getDouble('=',pfile);
		}
		
		/* Blue Lights */
		/* Get Blue Light Objects Number */
		m_nBlueLightObjectNumber = getInt('=',pfile);
		/* Create Objects */
		m_pcvBlueLightObjects = new dVector2[m_nBlueLightObjectNumber];
		for ( int i = 0 ; i < m_nBlueLightObjectNumber; i++){
			/* Get X position */
			m_pcvBlueLightObjects[i].x = getDouble('=',pfile);
			/* Get Y Position */
			m_pcvBlueLightObjects[i].y = getDouble('=',pfile);
		}
		
		/* Red Lights */
		/* Get Red Light Objects Number */
		m_nRedLightObjectNumber = getInt('=',pfile);
		/* Create Objects */
		m_pcvRedLightObjects = new dVector2[m_nRedLightObjectNumber];
		for ( int i = 0 ; i < m_nRedLightObjectNumber; i++){
			/* Get X position */
			m_pcvRedLightObjects[i].x = getDouble('=',pfile);
			/* Get Y Position */
			m_pcvRedLightObjects[i].y = getDouble('=',pfile);
		}

		/* Ground Areas */
		/* Get GroundArea Objects */
		m_nNumberOfGroundArea = getInt('=',pfile);
		m_vGroundAreaCenter = new dVector2[m_nNumberOfGroundArea];
		m_fGroundAreaExternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaInternalRadius = new double[m_nNumberOfGroundArea];
		m_fGroundAreaColor = new double[m_nNumberOfGroundArea];

		for ( int i = 0 ; i < m_nNumberOfGroundArea ; i++)
		{
			m_vGroundAreaCenter[i].x = getDouble('=',pfile);
			m_vGroundAreaCenter[i].y = getDouble('=',pfile);
			m_fGroundAreaInternalRadius[i] = 0.0;
			m_fGroundAreaExternalRadius[i] = getDouble('=',pfile);
			m_fGroundAreaColor[i] = getDouble('=',pfile);

		}
		
		/* SENSORS */
		/* Get Light Range */
		m_fLightSensorRange = getDouble('=',pfile); 
		
		/* Get Blue Light Range */
		m_fBlueLightSensorRange = getDouble('=',pfile); 
		
		/* Get Red Light Range */
		m_fRedLightSensorRange = getDouble('=',pfile); 
		
		/* Get Battery load range */
		m_fBatterySensorRange = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fBatteryChargeCoef = getDouble('=',pfile);
		/* Get batttery charge coef */
		m_fBatteryDischargeCoef = getDouble('=',pfile);
		
		/* Get Blue Battery load range */
		m_fBlueBatterySensorRange = getDouble('=',pfile);
		/* Get Blue batttery charge coef */
		m_fBlueBatteryChargeCoef = getDouble('=',pfile);
		/* Get Blue batttery charge coef */
		m_fBlueBatteryDischargeCoef = getDouble('=',pfile);
		
		/* Get Red Battery load range */
		m_fRedBatterySensorRange = getDouble('=',pfile);
		/* Get Red batttery charge coef */
		m_fRedBatteryChargeCoef = getDouble('=',pfile);
		/* Get Red batttery charge coef */
		m_fRedBatteryDischargeCoef = getDouble('=',pfile);
		
    		/* Get Encoder Sensor Error */
		m_fEncoderSensorError = getDouble('=',pfile);
	}
}

/******************************************************************************/
/******************************************************************************/

CIri1Exp::~CIri1Exp ( void )
{
	delete [] m_pcvLightObjects;
	delete [] m_pcvBlueLightObjects;
	delete [] m_pcvRedLightObjects;
	delete [] m_vGroundAreaCenter;
	delete [] m_fGroundAreaExternalRadius;
	delete [] m_fGroundAreaInternalRadius;
	delete [] m_fGroundAreaColor;
}

	/******************************************************************************/
/******************************************************************************/
CArena* CIri1Exp::CreateArena()
{

	/* Create Arena */
	CArena* pcArena = NULL;
	pcArena = new CProgrammedArena("CProgrammedArena", 20, 20, 3.0, 3.0);
	((CProgrammedArena*)pcArena)->SetHeightPixelsFromChars(pchHeightMap, ' ', '#', '%');

	/* Create and add Light Object */
	char pchTemp[128];
	CLightObject* pcLightObject = NULL;
	for( int i = 0 ; i < m_nLightObjectNumber ; i++){
		sprintf(pchTemp, "LightObject%d", i);
		CLightObject* pcLightObject = new CLightObject (pchTemp);
		pcLightObject->SetCenter(m_pcvLightObjects[i]);
		pcArena->AddLightObject(pcLightObject);
	}

	/* Create and add Blue Light Object */
	CBlueLightObject* pcBlueLightObject = NULL;
	for( int i = 0 ; i < m_nBlueLightObjectNumber ; i++){
		sprintf(pchTemp, "BlueLightObject%d", i);
		CBlueLightObject* pcBlueLightObject = new CBlueLightObject (pchTemp);
		pcBlueLightObject->SetCenter(m_pcvBlueLightObjects[i]);
		pcArena->AddBlueLightObject(pcBlueLightObject);
	}
	
	/* Create and add Red Light Object */
	CRedLightObject* pcRedLightObject = NULL;
	for( int i = 0 ; i < m_nRedLightObjectNumber ; i++){
		sprintf(pchTemp, "RedLightObject%d", i);
		CRedLightObject* pcRedLightObject = new CRedLightObject (pchTemp);
		pcRedLightObject->SetCenter(m_pcvRedLightObjects[i]);
		pcArena->AddRedLightObject(pcRedLightObject);
	}
	
	/* Create GroundArea */
	char sGroundAreaName[100]="epuck";


	for ( int i = 0 ; i < m_nNumberOfGroundArea ; i++)
	{
		//Create GroundArea
		sprintf(sGroundAreaName,"groundArea%d",i);
		CGroundArea* groundArea = new CGroundArea(sGroundAreaName);
		groundArea->SetCenter(m_vGroundAreaCenter[i]);
		groundArea->SetExtRadius(m_fGroundAreaExternalRadius[i]);
		groundArea->SetIntRadius(m_fGroundAreaInternalRadius[i]);
		groundArea->SetColor(m_fGroundAreaColor[i]);
		groundArea->SetHeight(0.20);
		pcArena->AddGroundArea(groundArea);
	}

	return pcArena;
}

/******************************************************************************/
/******************************************************************************/

void CIri1Exp::AddActuators(CEpuck* pc_epuck)
{
	/* Create and Add Wheels */
	char pchTemp[128];
	sprintf(pchTemp, "actuators_%s", pc_epuck->GetName());
	CActuator* pcActuator = NULL;
	pcActuator = new CWheelsActuator(pchTemp, pc_epuck);
	pc_epuck->AddActuator(pcActuator);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Exp::AddSensors(CEpuck* pc_epuck)
{
	//
	/* Create and add Proximity Sensor */
	CSensor* pcProxSensor = NULL;
	pcProxSensor = new CEpuckProximitySensor(252);
	pc_epuck->AddSensor(pcProxSensor);

	//Light Sensor
	CSensor* pcLightSensor = NULL;
	pcLightSensor = new CRealLightSensor("Light Sensor", m_fLightSensorRange);
	pc_epuck->AddSensor(pcLightSensor);
	
	//Blue Light Sensor
	CSensor* pcBlueLightSensor = NULL;
	pcBlueLightSensor = new CRealBlueLightSensor("Blue Light Sensor", m_fBlueLightSensorRange);
	pc_epuck->AddSensor(pcBlueLightSensor);
	
	//Red Light Sensor
	CSensor* pcRedLightSensor = NULL;
	pcRedLightSensor = new CRealRedLightSensor("Red Light Sensor", m_fRedLightSensorRange);
	pc_epuck->AddSensor(pcRedLightSensor);
	
	//Contact Sensor
	CSensor* pcContactSensor = NULL;
	pcContactSensor = new CContactSensor("Contact Sensor");
	pc_epuck->AddSensor(pcContactSensor);
	
	//Ground Sensor
	CSensor* pcGroundSensor = NULL;
	pcGroundSensor = new CGroundSensor("Ground Sensor");
	pc_epuck->AddSensor(pcGroundSensor);
	
	//Ground Memory Sensor
	CSensor* pcGroundMemorySensor = NULL;
	pcGroundMemorySensor = new CGroundMemorySensor("Ground Memory Sensor");
	pc_epuck->AddSensor(pcGroundMemorySensor);
	
	//Battery Sensor
	CSensor* pcBatterySensor = NULL;
	pcBatterySensor = new CBatterySensor("Battery Sensor", m_fBatterySensorRange, m_fBatteryChargeCoef, m_fBatteryDischargeCoef);
	pc_epuck->AddSensor(pcBatterySensor);
	
	//Blue Battery Sensor
	CSensor* pcBlueBatterySensor = NULL;
	pcBlueBatterySensor = new CBlueBatterySensor("Battery Sensor", m_fBlueBatterySensorRange, m_fBlueBatteryChargeCoef, m_fBlueBatteryDischargeCoef);
	pc_epuck->AddSensor(pcBlueBatterySensor);
	
	//Red Battery Sensor
	CSensor* pcRedBatterySensor = NULL;
	pcRedBatterySensor = new CRedBatterySensor("Battery Sensor", m_fRedBatterySensorRange, m_fRedBatteryChargeCoef, m_fRedBatteryDischargeCoef);
	pc_epuck->AddSensor(pcRedBatterySensor);
	
  //Encoder Sensor 
  CSensor* pcEncoderSensor = NULL;
  pcEncoderSensor = new CEncoderSensor("Encoder Sensor", (CArena*) m_pcSimulator->GetArena(), m_fEncoderSensorError, pc_epuck->GetPosition().x, pc_epuck->GetPosition().y);
  pc_epuck->AddSensor(pcEncoderSensor);
  
  //Compass Sensor
  CSensor* pcCompassSensor = NULL;
  pcCompassSensor = new CCompassSensor("compass", (CArena*) m_pcSimulator->GetArena());
  pc_epuck->AddSensor(pcCompassSensor);

}

/******************************************************************************/
/******************************************************************************/

void CIri1Exp::SetController(CEpuck* pc_epuck)
{
	char pchTemp[128];
	sprintf(pchTemp, "Iri1");
	CController* pcController = new CIri1Controller(pchTemp, pc_epuck, m_nWriteToFile);
	pc_epuck->SetControllerType( CONTROLLER_IRI1 );
	pc_epuck->SetController(pcController);

}

/******************************************************************************/
/******************************************************************************/

void CIri1Exp::CreateAndAddEpucks(CSimulator* pc_simulator)
{
	/* Create and add epucks */
	char label[100] = "epuck";    
	for (int i = 0; i < m_nRobotsNumber; i++)
	{
		sprintf(label, "epuck%0.4d", i);
		CEpuck* pcEpuck = CreateEpuck(label, m_pcvRobotPositions[i].x, m_pcvRobotPositions[i].y, m_fRobotOrientations[i]);
		pc_simulator->AddEpuck(pcEpuck);
		pc_simulator->SetTimeLimit(m_nRunTime);
	}

	Reset();
}


/******************************************************************************/
/******************************************************************************/

void CIri1Exp::Reset ( void )
{
}
