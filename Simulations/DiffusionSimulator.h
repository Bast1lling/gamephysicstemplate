#ifndef DIFFUSIONSIMULATOR_h
#define DIFFUSIONSIMULATOR_h

#include "Simulator.h"
#include "vectorbase.h"

class Grid {
	// to be implemented
	public:
		Grid(double xMin, double xMax, double yMin, double yMax, int width, int height);
		double get(int i, int j);
		void set(int i, int j, double val);
		void setAll(std::vector<double> x);

		void adjustGrid(int new_width, int new_height);

		int width;
		int height;
		double xDelta;
		double yDelta;
		double *T;

	private:
		double f_0(double i, double j);
		void printVector(double* b);

		double xMin;
		double yMin;
		double xMax;
		double yMax;
};



class DiffusionSimulator:public Simulator{
public:
	// Construtors
	DiffusionSimulator();

	// Functions
	const char * getTestCasesStr();
	void initUI(DrawingUtilitiesClass * DUC);
	void reset();
	void drawFrame(ID3D11DeviceContext* pd3dImmediateContext);
	void notifyCaseChanged(int testCase);
	void simulateTimestep(float timeStep);
	void externalForcesCalculations(float timeElapsed) {};
	void onClick(int x, int y);
	void onMouse(int x, int y);
	// Specific Functions
	void drawObjects();

	// Feel free to change the signature of these functions, add arguments, etc.
	void diffuseTemperatureExplicit(float timeElapsed);
	void diffuseTemperatureImplicit(float timeElapsed);

	int desiredWidth;
	int desiredHeight;

	/*
	static void TW_CALL WidthSetCallback(const void* value, void* clientData) {
		DiffusionSimulator* sim = static_cast<DiffusionSimulator*>(clientData);
		int new_width = *static_cast<const int*>(value);
		sim->T = Grid(-0.5, 0.5, -0.5, 0.5, new_width, sim->T.height);
	};
	static void TW_CALL WidthGetCallback(void* value, void* clientData) {
		DiffusionSimulator* sim = static_cast<DiffusionSimulator*>(clientData);
		*static_cast<int*>(value) = sim->T.width;
	};
	*/

private:

	// Functions	
	void printMatrix(double* A);
	void createGrid(Grid old_grid);

	// Attributes
	Vec3  m_vfMovableObjectPos;
	Vec3  m_vfMovableObjectFinalPos;
	Vec3  m_vfRotate;
	Point2D m_mouse;
	Point2D m_trackmouse;
	Point2D m_oldtrackmouse;
	Grid T;
	double alpha;
	double xMin;
	double xMax;
	double yMin;
	double yMax;
	double time;
};

#endif