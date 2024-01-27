#ifndef MASSSPRINGSYSTEMSIMULATOR_h
#define MASSSPRINGSYSTEMSIMULATOR_h
#include "Simulator.h"

// Do Not Change
#define EULER 0
#define LEAPFROG 1
#define MIDPOINT 2
// Do Not Change

enum integrator
{
	euler = EULER,
	leapfrog = LEAPFROG,
	midpoint = MIDPOINT,
	rk4,
	none
};

class MassPoint {
public:
	MassPoint(const Vec3& position, const Vec3& velocity, const float mass, const bool is_fixed)
		: position(position),
		velocity(velocity),
		is_fixed(is_fixed)
	{
	}

	Vec3 position;
	Vec3 velocity;
	Vec3 force;

	bool is_fixed;
};

std::ostream& operator<<(std::ostream& os, const MassPoint& p);

class Spring {
public:
	Spring(const int mass_point_1, const int mass_point_2, const float initial_length)
		: mass_point_1(mass_point_1),
		mass_point_2(mass_point_2),
		initial_length(initial_length)
	{
	}

	int mass_point_1;
	int mass_point_2;

	float initial_length;
};

std::ostream& operator<<(std::ostream& os, const Spring& p);

class MassSpringSystemSimulator :public Simulator {
public:
	// Construtors
	MassSpringSystemSimulator();

	// UI Functions
	const char* getTestCasesStr();
	void initUI(DrawingUtilitiesClass* DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void externalForcesCalculations(float timeElapsed);
	void simulateTimestep(float time_step);
	void onClick(int x, int y);
	void onMouse(int x, int y);

	// Specific Functions
	void setMass(float mass);
	void setSize(float size);
	void setStiffness(float stiffness);
	void setDampingFactor(float damping);
	int addMassPoint(Vec3 position, Vec3 velocity, bool isFixed);
	void addSpring(int masspoint1, int masspoint2, float initialLength);
	void addSpring(int p0, int p1);
	int getNumberOfMassPoints();
	int getNumberOfSprings();
	Vec3 getPositionOfMassPoint(int index);
	Vec3 getVelocityOfMassPoint(int index);
	void setVelocityOfMassPoint(int index, Vec3 v);
	Mat4 getMassPointToWorld(int index);
	void applyExternalForce(Vec3 force);

	// Do Not Change
	void setIntegrator(int integrator) {
		m_iIntegrator = integrator;
	}
	std::vector<MassPoint> mass_points;
	float m_fMass;

private:
	// Data Attributes
	float m_sphereSize;
	Mat4 m_sphereScale;
	float m_fStiffness;
	float m_fDamping;
	int m_iIntegrator;

	float timestep_override;

	// UI Attributes
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;

	std::vector<Spring> springs;

	uint16_t numHorizontalPoints = 10;
	uint16_t numVerticalPoints = 10;

	float gravity = 0;

	std::vector<int> movablePoints;
	void initSystem();
	void calculateForcesForPoints(std::vector<MassPoint>& points);

	void stepEuler(float time_step);
	void stepLeapfrog(float time_step);
	void stepMidpoint(float time_step);
	void stepRK4(float time_step);
	float getPointDistance(int p0, int p1) const;
	Vec3 getForce(const MassPoint&) const;
};
#endif