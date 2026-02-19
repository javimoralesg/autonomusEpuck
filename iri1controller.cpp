/******************* INCLUDES ******************/
/***********************************************/

/******************** General ******************/
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <sys/time.h>
#include <iostream>

#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <cstdlib>
#include <cstdio>

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
#include "encodersensor.h"

/******************** Actuators ****************/
#include "wheelsactuator.h"

/******************** Controller **************/
#include "iri1controller.h"

/******************************************************************************/
/******************************************************************************/

/******************** External Variables ******/
#include "../externalV.h"

bool red = true;

bool yellow = true;

/******************************************************************************/
/******************************************************************************/

extern gsl_rng *rng;
extern long int rngSeed;

const int mapGridX = 20;
const int mapGridY = 20;
double mapLengthX = 4.0;
double mapLengthY = 4.0;
int robotStartGridX = 18;
int robotStartGridY = 1;

const int n = mapGridX; // horizontal size of the map
const int m = mapGridY; // vertical size size of the map
static int map[n][m];
static int onlineMap[n][m];
static int closed_nodes_map[n][m]; // map of closed (tried-out) nodes
static int open_nodes_map[n][m];   // map of open (not-yet-tried) nodes
static int dir_map[n][m];		   // map of directions
const int dir = 8;				   // number of possible directions to go at any position
static int dx[dir] = {1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir] = {0, 1, 1, 1, 0, -1, -1, -1};

#define ERROR_DIRECTION 0.05
#define ERROR_POSITION 0.02

/******************************************************************************/
/******************************************************************************/

using namespace std;

/******************************************************************************/
/******************************************************************************/

#define BEHAVIORS 8

#define AVOID_OBSTACLE_PRIORITY 0
#define AVOID_FIRE_PRIORITY 1
#define EXTINGUISH_FIRE_PRIORITY 2
#define SEARCH_EXTINGUISHER_PRIORITY 3
#define RESCUE_PRIORITY 4
#define SEARCH_STUDENT_PRIORITY 5
#define NAVIGATE_PRIORITY 6
#define GO_GOAL_PRIORITY 7

/* Threshold to avoid obstacles */
#define PROXIMITY_THRESHOLD 0.6
/* Threshols to detect fire */
#define FIRE_THRESHOLD 0.57
/* Threshols to detect extinguisher */
#define EXTINGUISHER_THRESHOLD 0.9

#define SPEED 500

#define NO_OBSTACLE 0
#define OBSTACLE 1
#define START 2
#define PATH 3
#define END 4
#define NEST 5
#define PREY 6

/******************************************************************************/
/******************************************************************************/

class node
{
	// current position
	int xPos;
	int yPos;
	// total distance already travelled to reach the node
	int level;
	// priority=level+remaining distance estimate
	int priority; // smaller: higher priority

public:
	node(int xp, int yp, int d, int p)
	{
		xPos = xp;
		yPos = yp;
		level = d;
		priority = p;
	}

	int getxPos() const { return xPos; }
	int getyPos() const { return yPos; }
	int getLevel() const { return level; }
	int getPriority() const { return priority; }

	void updatePriority(const int &xDest, const int &yDest)
	{
		priority = level + estimate(xDest, yDest) * 10; // A*
	}

	// give better priority to going strait instead of diagonally
	void nextLevel(const int &i) // i: direction
	{
		level += (dir == 8 ? (i % 2 == 0 ? 10 : 14) : 10);
	}

	// Estimation function for the remaining distance to the goal.
	const int &estimate(const int &xDest, const int &yDest) const
	{
		static int xd, yd, d;
		xd = xDest - xPos;
		yd = yDest - yPos;

		// Euclidian Distance
		d = static_cast<int>(sqrt(xd * xd + yd * yd));

		return (d);
	}
};

/******************************************************************************/
/******************************************************************************/

// Determine priority (in the priority queue)
bool operator<(const node &a, const node &b)
{
	return a.getPriority() > b.getPriority();
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::CIri1Controller(const char *pch_name, CEpuck *pc_epuck, int n_write_to_file) : CController(pch_name, pc_epuck)

{
	/* Set Write to File */
	m_nWriteToFile = n_write_to_file;

	/* Set epuck */
	m_pcEpuck = pc_epuck;
	/* Set Wheels */
	m_acWheels = (CWheelsActuator *)m_pcEpuck->GetActuator(ACTUATOR_WHEELS);
	/* Set Prox Sensor */
	m_seProx = (CEpuckProximitySensor *)m_pcEpuck->GetSensor(SENSOR_PROXIMITY);
	/* Set light Sensor */
	m_seLight = (CRealLightSensor *)m_pcEpuck->GetSensor(SENSOR_REAL_LIGHT);
	/* Set Blue light Sensor */
	m_seBlueLight = (CRealBlueLightSensor *)m_pcEpuck->GetSensor(SENSOR_REAL_BLUE_LIGHT);
	/* Set Red light Sensor */
	m_seRedLight = (CRealRedLightSensor *)m_pcEpuck->GetSensor(SENSOR_REAL_RED_LIGHT);
	/* Set ground memory Sensor */
	m_seGroundMemory = (CGroundMemorySensor *)m_pcEpuck->GetSensor(SENSOR_GROUND_MEMORY);
	/* Set encoder Sensor */
	m_seEncoder = (CEncoderSensor *)m_pcEpuck->GetSensor(SENSOR_ENCODER);
	m_seEncoder->InitEncoderSensor(m_pcEpuck);

	/* Initialize Motor Variables */
	m_fLeftSpeed = 0.0;
	m_fRightSpeed = 0.0;

	/* Initialize Inhibitors */
	fFireInhibitor = 1.0;
	fExtinguisherInhibitor = 1.0;
	fGoalToInhibitor = 1.0;

	/* Initialize Counters */
	counterSearchStudent = 0;
	counterFire = 0;

	/* Initialize Activation Table */
	m_fActivationTable = new double *[BEHAVIORS];
	for (int i = 0; i < BEHAVIORS; i++)
	{
		m_fActivationTable[i] = new double[3];
	}

	/* Odometry */
	m_nState = 0;
	m_nPathPlanningStops = 0;
	m_fOrientation = 0.0;
	m_vPosition.x = 0.0;
	m_vPosition.y = 0.0;

	/* Set Actual Position to robot Start Grid */
	m_nRobotActualGridX = robotStartGridX;
	m_nRobotActualGridY = robotStartGridY;

	/* Init onlineMap */
	for (int y = 0; y < m; y++)
		for (int x = 0; x < n; x++)
			onlineMap[x][y] = OBSTACLE;

	/* DEBUG */
	PrintMap(&onlineMap[0][0]);
	/* DEBUG */

	/* Initialize status of foraging */
	m_nRescueStatus = 0;

	/* Initialize Nest/Prey variables */
	m_nNestGridX = 0;
	m_nNestGridY = 0;
	m_nPreyGridX = 0;
	m_nPreyGridY = 0;
	m_nPreyFound = 0;
	m_nNestFound = 0;

	/* Initialize PathPlanning Flag*/
	m_nPathPlanningDone = 0;
}

/******************************************************************************/
/******************************************************************************/

CIri1Controller::~CIri1Controller()
{
	for (int i = 0; i < BEHAVIORS; i++)
	{
		delete[] m_fActivationTable;
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SimulationStep(unsigned n_step_number, double f_time, double f_step_interval)
{
	/* Move time to global variable, so it can be used by the bahaviors to write to files*/
	m_fTime = f_time;

	/* Move step number to global variable, so it can be used by the bahaviors to write to files*/
	m_nStepNumber = n_step_number;

	/* Execute the levels of competence */
	ExecuteBehaviors();

	/* Execute Coordinator */
	Coordinator();

	/* Set Speed to wheels */
	m_acWheels->SetSpeed(m_fLeftSpeed, m_fRightSpeed);

	/* Counter to reactivate the fire */
	if (red == false)
	{
		counterFire++;
		if (counterFire > 3500)
		{
			red = true;
			counterFire = 0;
		}
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExecuteBehaviors()
{
	for (int i = 0; i < BEHAVIORS; i++)
	{
		m_fActivationTable[i][2] = 0.0;
	}

	/* Set Leds to BLACK */
	m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);

	/* Read Sensors */
	/* Proximity */
	double *prox = m_seProx->GetSensorReading(m_pcEpuck);
	const double *proxDirections = m_seProx->GetSensorDirections();
	int proxInputs = m_seProx->GetNumberOfInputs();
	/* Yellow Light */
	double *light = m_seLight->GetSensorReading(m_pcEpuck);
	const double *lightDirections = m_seLight->GetSensorDirections();
	int lightInputs = m_seLight->GetNumberOfInputs();
	/* Blue Light */
	double *blueLight = m_seBlueLight->GetSensorReading(m_pcEpuck);
	const double *blueLightDirections = m_seBlueLight->GetSensorDirections();
	int blueLightInputs = m_seBlueLight->GetNumberOfInputs();
	/* Red Light */
	double *redLight = m_seRedLight->GetSensorReading(m_pcEpuck);
	const double *redLightDirections = m_seRedLight->GetSensorDirections();
	int redLightInputs = m_seRedLight->GetNumberOfInputs();
	/* Ground Memory */
	double *groundMemory = m_seGroundMemory->GetSensorReading(m_pcEpuck);
	/* Encoder */
	double *encoder = m_seEncoder->GetSensorReading(m_pcEpuck);

	/* Execute Behaviors */
	ObstacleAvoidance(AVOID_OBSTACLE_PRIORITY, prox, proxDirections, proxInputs);
	FireAvoidance(AVOID_FIRE_PRIORITY, redLight, redLightDirections, redLightInputs);

	ExtinguishFire(EXTINGUISH_FIRE_PRIORITY, redLight, redLightDirections, redLightInputs);
	SearchExtinguisher(SEARCH_EXTINGUISHER_PRIORITY, blueLight, blueLightDirections, blueLightInputs);

	Rescue(RESCUE_PRIORITY, groundMemory[0], light, lightDirections, lightInputs);
	SearchStudent(SEARCH_STUDENT_PRIORITY, groundMemory[0], light, lightDirections, lightInputs);

	ComputeActualCell(GO_GOAL_PRIORITY, groundMemory[0], encoder);
	PathPlanning(GO_GOAL_PRIORITY);
	GoGoal(GO_GOAL_PRIORITY, groundMemory[0]);

	Navigate(NAVIGATE_PRIORITY);

	// /* Write Files */
	// /* Proximity Max Sensor*/
	// double maxProx = 0.0;
	// for (int i = 0; i < proxInputs; i++)
	// {
	// 	if (prox[i] > maxProx)
	// 		maxProx = prox[i];
	// }
	// FILE *fileOutput = fopen("outputFiles/proximityMax.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, maxProx);
	// fclose(fileOutput);

	// /* Light Max Sensor*/
	// double maxLight = 0.0;
	// for (int i = 0; i < lightInputs; i++)
	// {
	// 	if (light[i] > maxLight)
	// 		maxLight = light[i];
	// }
	// fileOutput = fopen("outputFiles/lightMax.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, maxLight);
	// fclose(fileOutput);

	// /* Blue Light Max Sensor*/
	// double maxBlueLight = 0.0;
	// for (int i = 0; i < blueLightInputs; i++)
	// {
	// 	if (blueLight[i] > maxBlueLight)
	// 		maxBlueLight = blueLight[i];
	// }
	// fileOutput = fopen("outputFiles/blueLightMax.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, maxBlueLight);
	// fclose(fileOutput);

	// /* Red Light Max Sensor*/
	// double maxRedLight = 0.0;
	// for (int i = 0; i < redLightInputs; i++)
	// {
	// 	if (redLight[i] > maxRedLight)
	// 		maxRedLight = redLight[i];
	// }
	// fileOutput = fopen("outputFiles/redLightMax.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, maxRedLight);
	// fclose(fileOutput);

	// /* Ground Memory Sensor*/
	// fileOutput = fopen("outputFiles/groundMemory.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, groundMemory[0]);
	// fclose(fileOutput);

	// /* Ligth Object */
	// fileOutput = fopen("outputFiles/lightObject.txt", "a");
	// fprintf(fileOutput, "%u %d \n", m_nStepNumber, yellow);
	// fclose(fileOutput);

	// /* Red Light Object */
	// fileOutput = fopen("outputFiles/redLightObject.txt", "a");
	// fprintf(fileOutput, "%u %d \n", m_nStepNumber, red);
	// fclose(fileOutput);

	// /* Goal To Inhibitor*/
	// fileOutput = fopen("outputFiles/goalToInhibitor.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, fGoalToInhibitor);
	// fclose(fileOutput);

	// /* Fire Inhibitor */
	// fileOutput = fopen("outputFiles/fireInhibitor.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, fFireInhibitor);
	// fclose(fileOutput);

	// /* Extinguisher Inhibitor */
	// fileOutput = fopen("outputFiles/extinguisherInhibitor.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, fExtinguisherInhibitor);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Coordinator(void)
{
	/* Create counter for behaviors */
	int nBehavior;
	/* Create angle of movement */
	double fAngle = 0.0;
	/* Create vector of movement */
	dVector2 vAngle;
	vAngle.x = 0.0;
	vAngle.y = 0.0;

	/* For every Behavior */
	for (nBehavior = 0; nBehavior < BEHAVIORS; nBehavior++)
	{
		/* If behavior is active */
		if (m_fActivationTable[nBehavior][2] == 1.0)
		{
			/* DEBUG */
			printf("Behavior %d: %2f\n", nBehavior, m_fActivationTable[nBehavior][0]);
			/* DEBUG */
			vAngle.x += m_fActivationTable[nBehavior][1] * cos(m_fActivationTable[nBehavior][0]);
			vAngle.y += m_fActivationTable[nBehavior][1] * sin(m_fActivationTable[nBehavior][0]);
		}
	}

	/* Calc angle of movement */
	fAngle = atan2(vAngle.y, vAngle.x);
	/* DEBUG */
	printf("fAngle: %2f\n", fAngle);
	printf("\n");
	/* DEBUG */

	/* Normalize fAngle */
	while (fAngle > M_PI)
		fAngle -= 2 * M_PI;
	while (fAngle < -M_PI)
		fAngle += 2 * M_PI;

	/* Based on the angle, calc wheels movements */
	double fCLinear = 1.0;
	double fCAngular = 1.0;
	double fC1 = SPEED / M_PI;

	/* Calc Linear Speed */
	double fVLinear = SPEED * fCLinear * (cos(fAngle / 2));

	/*Calc Angular Speed */
	double fVAngular = fAngle;

	m_fLeftSpeed = fVLinear - fC1 * fVAngular;
	m_fRightSpeed = fVLinear + fC1 * fVAngular;
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ObstacleAvoidance(unsigned int un_priority, double *prox_, const double *proxDirections_, int proxInputs_)
{
	double fMaxProx = 0.0;

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for (int i = 0; i < proxInputs_; i++)
	{
		vRepelent.x += prox_[i] * cos(proxDirections_[i]);
		vRepelent.y += prox_[i] * sin(proxDirections_[i]);

		if (prox_[i] > fMaxProx)
			fMaxProx = prox_[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while (fRepelent > M_PI)
		fRepelent -= 2 * M_PI;
	while (fRepelent < -M_PI)
		fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = 1.0;

	/* If above a threshold */
	if (fMaxProx > PROXIMITY_THRESHOLD)
	{
		/* Set Leds to GREEN */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_GREEN);

		/* Mark Behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/avoidObstacle.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::FireAvoidance(unsigned int un_priority, double *redLight_, const double *redLightDirections_, int redLightInputs_)
{
	double fMaxRedLight = 0.0;

	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for (int i = 0; i < redLightInputs_; i++)
	{
		vRepelent.x += redLight_[i] * cos(redLightDirections_[i]);
		vRepelent.y += redLight_[i] * sin(redLightDirections_[i]);

		if (redLight_[i] > fMaxRedLight)
			fMaxRedLight = redLight_[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;
	/* Normalize angle */
	while (fRepelent > M_PI)
		fRepelent -= 2 * M_PI;
	while (fRepelent < -M_PI)
		fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = 1.0;

	if (fMaxRedLight >= (FIRE_THRESHOLD + 0.01))
	{
		/* Set Leds to RED */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_RED);

		/* Mark Behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/avoidFire.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ExtinguishFire(unsigned int un_priority, double *redLight_, const double *redLightDirections_, int redLightInputs_)
{
	double fMaxRedLight = 0.0;

	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for (int i = 0; i < redLightInputs_; i++)
	{
		vRepelent.x += redLight_[i] * cos(redLightDirections_[i]);
		vRepelent.y += redLight_[i] * sin(redLightDirections_[i]);

		if (redLight_[i] > fMaxRedLight)
			fMaxRedLight = redLight_[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);

	/* Normalize angle */
	while (fRepelent > M_PI)
		fRepelent -= 2 * M_PI;
	while (fRepelent < -M_PI)
		fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = fMaxRedLight;

	if ((fMaxRedLight >= FIRE_THRESHOLD) && red)
	{
		if (fExtinguisherInhibitor == 1.0)
		{
			fFireInhibitor = 0.0;
		}
		else
		{
			fFireInhibitor = 1.0;
			fExtinguisherInhibitor = 1.0;
			red = false;
		}
	}

	if ((!fFireInhibitor * !fExtinguisherInhibitor) == 1.0)
	{
		/* Set Leds to BLUE */
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLUE);

		/* Mark Behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;
	}

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/extinguishFire.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SearchExtinguisher(unsigned int un_priority, double *blueLight_, const double *blueLightDirections_, int blueLightInputs_)
{
	double fMaxBlueLight = 0.0;

	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for (int i = 0; i < blueLightInputs_; i++)
	{
		vRepelent.x += blueLight_[i] * cos(blueLightDirections_[i]);
		vRepelent.y += blueLight_[i] * sin(blueLightDirections_[i]);

		if (blueLight_[i] > fMaxBlueLight)
			fMaxBlueLight = blueLight_[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);

	/* Normalize angle */
	while (fRepelent > M_PI)
		fRepelent -= 2 * M_PI;
	while (fRepelent < -M_PI)
		fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = fMaxBlueLight;

	/* If with a virtual puck */
	if ((!fFireInhibitor * fExtinguisherInhibitor) == 1.0)
	{
		/* Mark Behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;

		if (fMaxBlueLight >= EXTINGUISHER_THRESHOLD)
		{
			fExtinguisherInhibitor = 0.0;
		}
	}

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/searchExtinguisher.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Rescue(unsigned int un_priority, double groundMemory_, double *light_, const double *lightDirections_, int lightInputs_)
{
	double fMaxLight = 0.0;

	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for (int i = 0; i < lightInputs_; i++)
	{
		vRepelent.x += light_[i] * cos(lightDirections_[i]);
		vRepelent.y += light_[i] * sin(lightDirections_[i]);

		if (light_[i] > fMaxLight)
			fMaxLight = light_[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);
	/* Create repelent angle */
	fRepelent -= M_PI;

	/* Normalize angle */
	while (fRepelent > M_PI)
		fRepelent -= 2 * M_PI;
	while (fRepelent < -M_PI)
		fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = 1 - fMaxLight;

	/* If with a virtual puck */
	if ((groundMemory_ * fFireInhibitor * fGoalToInhibitor) == 1.0)
	{
		yellow = true;

		/* Set Leds to YELLOW*/
		m_pcEpuck->SetAllColoredLeds(LED_COLOR_YELLOW);

		/* Mark Behavior as active */
		m_fActivationTable[un_priority][2] = 1.0;

		// /* Write File */
		// FILE *fileOutput = fopen("outputFiles/rescuePosition.txt", "a");
		// fprintf(fileOutput, "%2f,%2f\n", m_vPosition.x, m_vPosition.y);
		// fclose(fileOutput);
	}

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/rescue.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::SearchStudent(unsigned int un_priority, double groundMemory_, double *light_, const double *lightDirections_, int lightInputs_)
{
	double fMaxLight = 0.0;

	/* We call vRepelent to go similar to Obstacle Avoidance, although it is an aproaching vector */
	dVector2 vRepelent;
	vRepelent.x = 0.0;
	vRepelent.y = 0.0;

	/* Calc vector Sum */
	for (int i = 0; i < lightInputs_; i++)
	{
		vRepelent.x += light_[i] * cos(lightDirections_[i]);
		vRepelent.y += light_[i] * sin(lightDirections_[i]);

		if (light_[i] > fMaxLight)
			fMaxLight = light_[i];
	}

	/* Calc pointing angle */
	float fRepelent = atan2(vRepelent.y, vRepelent.x);

	/* Normalize angle */
	while (fRepelent > M_PI)
		fRepelent -= 2 * M_PI;
	while (fRepelent < -M_PI)
		fRepelent += 2 * M_PI;

	m_fActivationTable[un_priority][0] = fRepelent;
	m_fActivationTable[un_priority][1] = fMaxLight;

	if ((!groundMemory_ * fFireInhibitor * fGoalToInhibitor) == 1.0)
	{
		m_fActivationTable[un_priority][2] = 1.0;

		/* Flicker the Ligth */
		counterSearchStudent++;
		if (counterSearchStudent <= 500)
		{
			yellow = false;
		}
		else
		{
			yellow = true;
		}
		if (counterSearchStudent > 900)
		{
			counterSearchStudent = 0;
		}

		// /* Write File */
		// FILE *fileOutput = fopen("outputFiles/searchStudentPosition.txt", "a");
		// fprintf(fileOutput, "%2f,%2f\n", m_vPosition.x, m_vPosition.y);
		// fclose(fileOutput);
	}

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/searchStudent.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::ComputeActualCell(unsigned int un_priority, double groundMemory_, double *encoder_)
{
	CalcPositionAndOrientation(encoder_);

	/* Calc increment of position, correlating grid and metrics */
	double fXmov = mapLengthX / ((double)mapGridX);
	double fYmov = mapLengthY / ((double)mapGridY);

	/* Compute X grid */
	double tmp = m_vPosition.x;
	tmp += robotStartGridX * fXmov + 0.5 * fXmov;
	m_nRobotActualGridX = (int)(tmp / fXmov);

	/* Compute Y grid */
	tmp = -m_vPosition.y;
	tmp += robotStartGridY * fYmov + 0.5 * fYmov;
	m_nRobotActualGridY = (int)(tmp / fYmov);

	/* Update no-obstacles on map */
	if (onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != NEST &&
		onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] != PREY)
		onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = NO_OBSTACLE;

	/* If looking for nest and arrived to nest */
	if (m_nRescueStatus == 1 && groundMemory_ == 0)
	{
		/* update forage status */
		m_nRescueStatus = 0;
		/* Asumme Path Planning is done */
		m_nPathPlanningDone = 0;
		/* Restart PathPlanning state */
		m_nState = 0;
		/* Mark nest on map */
		onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = NEST;
		/* Flag that nest was found */
		m_nNestFound = 1;
		/* Update nest grid */
		m_nNestGridX = m_nRobotActualGridX;
		m_nNestGridY = m_nRobotActualGridY;
		/* DEBUG */
		PrintMap(&onlineMap[0][0]);
		/* DEBUG */
	} // end looking for nest

	/* If looking for prey and prey graspped */
	else if (m_nRescueStatus == 0 && groundMemory_ == 1)
	{
		/* Update forage Status */
		m_nRescueStatus = 1;
		/* Asumme Path Planning is done */
		m_nPathPlanningDone = 0;
		/* Restart PathPlanning state */
		m_nState = 0;
		/* Mark prey on map */
		onlineMap[m_nRobotActualGridX][m_nRobotActualGridY] = PREY;
		/* Flag that nest was found */
		m_nPreyFound = 1;
		/* Update nest grid */
		m_nPreyGridX = m_nRobotActualGridX;
		m_nPreyGridY = m_nRobotActualGridY;
		/* DEBUG */
		PrintMap(&onlineMap[0][0]);
		/* DEBUG */
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PathPlanning(unsigned int un_priority)
{
	/* Clear Map */
	for (int y = 0; y < m; y++)
		for (int x = 0; x < n; x++)
			map[x][y] = NO_OBSTACLE;

	if (m_nNestFound == 1 && m_nPreyFound == 1 && m_nPathPlanningDone == 0)
	{
		m_nPathPlanningStops = 0;
		m_fActivationTable[un_priority][2] = 1;

		/* Obtain start and end desired position */
		int xA, yA, xB, yB;
		if (m_nRescueStatus == 1)
		{
			xA = m_nRobotActualGridX;
			yA = m_nRobotActualGridY;
			xB = m_nNestGridX;
			yB = m_nNestGridY;
		}
		else
		{
			xA = m_nRobotActualGridX;
			yA = m_nRobotActualGridY;
			xB = m_nPreyGridX;
			yB = m_nPreyGridY;
		}

		/* DEBUG */
		printf("START: %d, %d - END: %d, %d\n", xA, yA, xB, yB);
		/* DEBUG */

		/* Obtain Map */
		for (int y = 0; y < m; y++)
			for (int x = 0; x < n; x++)
				if (onlineMap[x][y] != NO_OBSTACLE && onlineMap[x][y] != NEST && onlineMap[x][y] != PREY)
					map[x][y] = OBSTACLE;

		/* Obtain optimal path */
		string route = pathFind(xA, yA, xB, yB);
		/* DEBUG */
		if (route == "")
			cout << "An empty route generated!" << endl;
		cout << "Route:" << route << endl;
		printf("route Length: %d\n", route.length());
		/* DEBUG */

		/* Obtain number of changing directions */
		for (int i = 1; i < route.length(); i++)
			if (route[i - 1] != route[i])
				m_nPathPlanningStops++;

		/* Add last movement */
		m_nPathPlanningStops++;
		/* DEBUG */
		printf("STOPS: %d\n", m_nPathPlanningStops);
		/* DEBUG */

		/* Define vector of desired positions. One for each changing direction */
		m_vPositionsPlanning = new dVector2[m_nPathPlanningStops];

		/* Calc increment of position, correlating grid and metrics */
		double fXmov = mapLengthX / mapGridX;
		double fYmov = mapLengthY / mapGridY;

		/* Get actual position */
		dVector2 actualPos;
		actualPos.x = m_nRobotActualGridX * fXmov;
		actualPos.y = m_nRobotActualGridY * fYmov;

		/* Fill vector of desired positions */
		int stop = 0;
		int counter = 0;
		/* Check the route and obtain the positions*/
		for (int i = 1; i < route.length(); i++)
		{
			/* For every position in route, increment countr */
			counter++;
			/* If a direction changed */
			if ((route[i - 1] != route[i]))
			{
				/* Obtain the direction char */
				char c;
				c = route.at(i - 1);

				/* Calc the new stop according to actual position and increment based on the grid */
				m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov * dx[atoi(&c)];
				m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov * dy[atoi(&c)];

				/* Update position for next stop */
				actualPos.x = m_vPositionsPlanning[stop].x;
				actualPos.y = m_vPositionsPlanning[stop].y;

				/* Increment stop */
				stop++;
				/* reset counter */
				counter = 0;
			}

			/* If we are in the last update, calc last movement */
			if (i == (route.length() - 1))
			{
				/* Increment counter */
				counter++;
				/* Obtain the direction char */
				char c;
				c = route.at(i);

				/* Calc the new stop according to actual position and increment based on the grid */
				m_vPositionsPlanning[stop].x = actualPos.x + counter * fXmov * dx[atoi(&c)];
				m_vPositionsPlanning[stop].y = actualPos.y + counter * fYmov * dy[atoi(&c)];

				/* Update position for next stop */
				actualPos.x = m_vPositionsPlanning[stop].x;
				actualPos.y = m_vPositionsPlanning[stop].y;

				/* Increment stop */
				stop++;
				/* reset counter */
				counter = 0;
			}
		}

		/* DEBUG */
		if (route.length() > 0)
		{
			int j;
			char c;
			int x = xA;
			int y = yA;
			map[x][y] = START;
			for (int i = 0; i < route.length(); i++)
			{
				c = route.at(i);
				j = atoi(&c);
				x = x + dx[j];
				y = y + dy[j];
				map[x][y] = PATH;
			}
			map[x][y] = END;

			PrintMap(&onlineMap[0][0]);
			printf("\n\n");
			PrintMap(&map[0][0]);
		}
		/* END DEBUG */

		/* Convert to simulator coordinates */
		for (int i = 0; i < m_nPathPlanningStops; i++)
		{
			m_vPositionsPlanning[i].x -= (mapGridX * fXmov) / 2;
			m_vPositionsPlanning[i].y -= (mapGridY * fYmov) / 2;
			m_vPositionsPlanning[i].y = -m_vPositionsPlanning[i].y;
		}

		/* Convert to robot coordinates. FAKE!!.
		 * Notice we are only working with initial orientation = 0.0 */
		for (int i = 0; i < m_nPathPlanningStops; i++)
		{
			/* Traslation */
			m_vPositionsPlanning[i].x -= ((robotStartGridX * fXmov) - (mapGridX * fXmov) / 2);
			m_vPositionsPlanning[i].y += ((robotStartGridY * fYmov) - (mapGridY * fYmov) / 2);
		}

		m_nPathPlanningDone = 1;
	}
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::GoGoal(unsigned int un_priority, double groundMemory_)
{
	if (((m_nNestFound * fFireInhibitor) == 1) && ((m_nPreyFound * fFireInhibitor) == 1))
	{
		yellow = false;

		if (groundMemory_ == 1.0)
		{
			/* Set Leds to RED */
			m_pcEpuck->SetAllColoredLeds(LED_COLOR_YELLOW);
		}
		else
		{
			/* Set Leds to RED */
			m_pcEpuck->SetAllColoredLeds(LED_COLOR_BLACK);
		}

		/* If something not found at the end of planning, reset plans */
		if (m_nState >= m_nPathPlanningStops)
		{
			printf(" --------------- LOST!!!!!!!! --------------\n");
			m_nNestFound = 0;
			m_nPreyFound = 0;
			m_nState = 0;
			return;
		}

		/* DEBUG */
		printf("PlanningX: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].x, m_vPosition.x);
		printf("PlanningY: %2f, Actual: %2f\n", m_vPositionsPlanning[m_nState].y, m_vPosition.y);
		/* DEBUG */

		double fX = (m_vPositionsPlanning[m_nState].x - m_vPosition.x);
		double fY = (m_vPositionsPlanning[m_nState].y - m_vPosition.y);
		double fGoalDirection = 0;

		/* If on Goal, return 1 */
		if ((fabs(fX) <= ERROR_POSITION) && (fabs(fY) <= ERROR_POSITION))
		{
			fGoalToInhibitor = 1.0;
			m_nState++;
		}
		else
		{
			/* Enable Inhibitor to Rescue */
			fGoalToInhibitor = 0.0;
		}

		fGoalDirection = atan2(fY, fX);

		/* Translate fGoalDirection into local coordinates */
		fGoalDirection -= m_fOrientation;
		/* Normalize Direction */
		while (fGoalDirection > M_PI)
			fGoalDirection -= 2 * M_PI;
		while (fGoalDirection < -M_PI)
			fGoalDirection += 2 * M_PI;

		m_fActivationTable[un_priority][0] = fGoalDirection;
		m_fActivationTable[un_priority][1] = 1;
		m_fActivationTable[un_priority][2] = 1;

		// /* Write File */
		// FILE *fileOutput = fopen("outputFiles/GoGoalPosition.txt", "a");
		// fprintf(fileOutput, "%2f,%2f\n", m_vPosition.x, m_vPosition.y);
		// fclose(fileOutput);
	}

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/goGoal.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::Navigate(unsigned int un_priority)
{
	/* Direction Angle 0.0 and always active. We set its vector intensity to 0.5 if used */
	m_fActivationTable[un_priority][0] = 0.0;
	m_fActivationTable[un_priority][1] = 0.1;
	m_fActivationTable[un_priority][2] = 1.0;

	// /* Write File */
	// FILE *fileOutput = fopen("outputFiles/navigate.txt", "a");
	// fprintf(fileOutput, "%u %2.4f \n", m_nStepNumber, m_fActivationTable[un_priority][2]);
	// fclose(fileOutput);
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::CalcPositionAndOrientation(double *f_encoder)
{
	/* Remake kinematic equations */
	double fIncU = (f_encoder[0] + f_encoder[1]) / 2;
	double fIncTetha = (f_encoder[1] - f_encoder[0]) / CEpuck::WHEELS_DISTANCE;

	/* Substitute arc by chord, take care of 0 division */
	if (fIncTetha != 0.0)
		fIncU = ((f_encoder[0] / fIncTetha) + (CEpuck::WHEELS_DISTANCE / 2)) * 2.0 * sin(fIncTetha / 2.0);

	/* Update new Position */
	m_vPosition.x += fIncU * cos(m_fOrientation + fIncTetha / 2);
	m_vPosition.y += fIncU * sin(m_fOrientation + fIncTetha / 2);

	/* Update new Orientation */
	m_fOrientation += fIncTetha;

	/* Normalize Orientation */
	while (m_fOrientation < 0)
		m_fOrientation += 2 * M_PI;
	while (m_fOrientation > 2 * M_PI)
		m_fOrientation -= 2 * M_PI;
}

/******************************************************************************/
/******************************************************************************/

// A-star algorithm.
// The route returned is a string of direction digits.
string CIri1Controller::pathFind(const int &xStart, const int &yStart, const int &xFinish, const int &yFinish)
{
	static priority_queue<node> pq[2]; // list of open (not-yet-tried) nodes
	static int pqi;					   // pq index
	static node *n0;
	static node *m0;
	static int i, j, x, y, xdx, ydy;
	static char c;
	pqi = 0;

	// reset the node maps
	for (y = 0; y < m; y++)
	{
		for (x = 0; x < n; x++)
		{
			closed_nodes_map[x][y] = 0;
			open_nodes_map[x][y] = 0;
		}
	}

	// create the start node and push into list of open nodes
	n0 = new node(xStart, yStart, 0, 0);
	n0->updatePriority(xFinish, yFinish);
	pq[pqi].push(*n0);
	// open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

	// A* search
	while (!pq[pqi].empty())
	{
		// get the current node w/ the highest priority from the list of open nodes
		n0 = new node(pq[pqi].top().getxPos(), pq[pqi].top().getyPos(), pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

		x = n0->getxPos();
		y = n0->getyPos();

		pq[pqi].pop(); // remove the node from the open list
		open_nodes_map[x][y] = 0;
		// mark it on the closed nodes map
		closed_nodes_map[x][y] = 1;

		// quit searching when the goal state is reached
		if (x == xFinish && y == yFinish)
		{
			// generate the path from finish to start by following the directions
			string path = "";
			while (!(x == xStart && y == yStart))
			{
				j = dir_map[x][y];
				c = '0' + (j + dir / 2) % dir;
				path = c + path;
				x += dx[j];
				y += dy[j];
			}

			// garbage collection
			delete n0;
			// empty the leftover nodes
			while (!pq[pqi].empty())
				pq[pqi].pop();
			return path;
		}

		// generate moves (child nodes) in all possible directions
		for (i = 0; i < dir; i++)
		{
			xdx = x + dx[i];
			ydy = y + dy[i];

			if (!(xdx < 0 || xdx > n - 1 || ydy < 0 || ydy > m - 1 || map[xdx][ydy] == 1 || closed_nodes_map[xdx][ydy] == 1))
			{
				// generate a child node
				m0 = new node(xdx, ydy, n0->getLevel(), n0->getPriority());
				m0->nextLevel(i);
				m0->updatePriority(xFinish, yFinish);

				// if it is not in the open list then add into that
				if (open_nodes_map[xdx][ydy] == 0)
				{
					open_nodes_map[xdx][ydy] = m0->getPriority();
					pq[pqi].push(*m0);
					// mark its parent node direction
					dir_map[xdx][ydy] = (i + dir / 2) % dir;
				}
				else if (open_nodes_map[xdx][ydy] > m0->getPriority())
				{
					// update the priority info
					open_nodes_map[xdx][ydy] = m0->getPriority();
					// update the parent direction info
					dir_map[xdx][ydy] = (i + dir / 2) % dir;

					// replace the node by emptying one pq to the other one except the node to be replaced will be ignored and the new node will be pushed in instead
					while (!(pq[pqi].top().getxPos() == xdx && pq[pqi].top().getyPos() == ydy))
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pq[pqi].pop(); // remove the wanted node

					// empty the larger size pq to the smaller one
					if (pq[pqi].size() > pq[1 - pqi].size())
						pqi = 1 - pqi;
					while (!pq[pqi].empty())
					{
						pq[1 - pqi].push(pq[pqi].top());
						pq[pqi].pop();
					}
					pqi = 1 - pqi;
					pq[pqi].push(*m0); // add the better node instead
				}
				else
					delete m0; // garbage collection
			}
		}
		delete n0; // garbage collection
	}
	return ""; // no route found
}

/******************************************************************************/
/******************************************************************************/

void CIri1Controller::PrintMap(int *print_map)
{
	for (int x = 0; x < n; x++)
	{
		for (int y = 0; y < m; y++)
		{
			if (print_map[y * n + x] == 0)
				cout << ".";
			else if (print_map[y * n + x] == 1)
				cout << "O"; // obstacle
			else if (print_map[y * n + x] == 2)
				cout << "S"; // start
			else if (print_map[y * n + x] == 3)
				cout << "R"; // route
			else if (print_map[y * n + x] == 4)
				cout << "F"; // finish
			else if (print_map[y * n + x] == 5)
				cout << "N"; // finish
			else if (print_map[y * n + x] == 6)
				cout << "P"; // finish
		}
		cout << endl;
	}
}
