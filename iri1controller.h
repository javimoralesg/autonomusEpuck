#ifndef IRI1CONTROLLER_H_
#define IRI1CONTROLLER_H_

/******************************************************************************/
/******************************************************************************/

#include "controller.h"

/******************************************************************************/
/******************************************************************************/

class CIri1Controller : public CController
{
public:
	CIri1Controller(const char *pch_name, CEpuck *pc_epuck, int n_wrtie_to_file);
	~CIri1Controller();
	void SimulationStep(unsigned n_step_number, double f_time, double f_step_interval);

private:
	/* ROBOT */
	CEpuck *m_pcEpuck;

	/* SENSORS */
	CWheelsActuator *m_acWheels;
	CEpuckProximitySensor *m_seProx;
	CRealLightSensor *m_seLight;
	CRealBlueLightSensor *m_seBlueLight;
	CRealRedLightSensor *m_seRedLight;
	CGroundMemorySensor *m_seGroundMemory;
	CEncoderSensor *m_seEncoder;

	/* Global Variables */
	double m_fLeftSpeed;
	double m_fRightSpeed;
	double **m_fActivationTable;
	int m_nWriteToFile;
	double m_fTime;
	unsigned m_nStepNumber;

	/* Counters */
	int counterSearchStudent;
	int counterFire;

	/* Inhibitors */
	double fGoalToInhibitor;
	double fFireInhibitor;
	double fExtinguisherInhibitor;

	/* Odometry */
	float m_fOrientation;
	dVector2 m_vPosition;
	int m_nState;
	dVector2 *m_vPositionsPlanning;
	int m_nPathPlanningStops;
	int m_nRobotActualGridX;
	int m_nRobotActualGridY;

	int m_nRescueStatus;

	int m_nNestFound;
	int m_nNestGridX;
	int m_nNestGridY;

	int m_nPreyFound;
	int m_nPreyGridX;
	int m_nPreyGridY;

	int m_nPathPlanningDone;

	/* Functions */
	void ExecuteBehaviors(void);
	void Coordinator(void);

	void CalcPositionAndOrientation(double *f_encoder);
	string pathFind(const int &xStart, const int &yStart, const int &xFinish, const int &yFinish);

	void PrintMap(int *print_map);

	/* Behaviors */
	void ObstacleAvoidance(unsigned int un_priority, double *prox_, const double *proxDirections_, int proxInputs_);
	void FireAvoidance(unsigned int un_priority, double *redLight_, const double *redLightDirections_, int redLightInputs_);

	void ExtinguishFire(unsigned int un_priority, double *redLight_, const double *redLightDirections_, int redLightInputs_);
	void SearchExtinguisher(unsigned int un_priority, double *blueLight_, const double *blueLightDirections_, int blueLightInputs_);

	void Rescue(unsigned int un_priority, double groundMemory_, double *light_, const double *lightDirections_, int lightInputs_);
	void SearchStudent(unsigned int un_priority, double groundMemory_, double *light_, const double *lightDirections_, int lightInputs_);

	void ComputeActualCell(unsigned int un_priority, double groundMemory_, double *encoder_);
	void PathPlanning(unsigned int un_priority);
	void GoGoal(unsigned int un_priority, double groundMemory_);

	void Navigate(unsigned int un_priority);
};

#endif
