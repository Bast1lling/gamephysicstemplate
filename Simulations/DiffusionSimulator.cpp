#include "DiffusionSimulator.h"
#include "pcgsolver.h"
#include <cstdlib>
#include <ctime>
using namespace std;


DiffusionSimulator::DiffusionSimulator()
	: alpha(0.03), xMin(-0.5), xMax(0.5), yMin(-0.5), yMax(0.5),
	desiredHeight(128), desiredWidth(128), T(Grid(-0.5, 0.5, -0.5, 0.5, 128, 128))
{
	m_iTestCase = 0;
	m_vfMovableObjectPos = Vec3();
	m_vfMovableObjectFinalPos = Vec3();
	m_vfRotate = Vec3();
	time = 0;
	//rest to be implemented
	//initSimpleDemo();
}

const char * DiffusionSimulator::getTestCasesStr(){
	return "Explicit_solver, Implicit_solver";
}

void DiffusionSimulator::reset(){
		m_mouse.x = m_mouse.y = 0;
		m_trackmouse.x = m_trackmouse.y = 0;
		m_oldtrackmouse.x = m_oldtrackmouse.y = 0;
		this->T = Grid(-0.5, 0.5, -0.5, 0.5, desiredHeight, desiredHeight);
}

void DiffusionSimulator::initUI(DrawingUtilitiesClass * DUC)
{
	this->DUC = DUC;
	// to be implemented
	TwAddVarRW(DUC->g_pTweakBar, "Alpha", TW_TYPE_FLOAT, &alpha, "min=0 step=0.01");
	TwAddVarRW(DUC->g_pTweakBar, "Grid width", TW_TYPE_INT32, &desiredWidth, "min=2 max=128 step=1");
	TwAddVarRW(DUC->g_pTweakBar, "Grid height", TW_TYPE_INT32, &desiredHeight, "min=2 max=128 step=1");
	//TwAddVarCB(DUC->g_pTweakBar, "Grid width", TW_TYPE_INT32, DiffusionSimulator::WidthSetCallback, DiffusionSimulator::WidthGetCallback, this, "");
}

void DiffusionSimulator::notifyCaseChanged(int testCase)
{
	m_iTestCase = testCase;
	m_vfMovableObjectPos = Vec3(0, 0, 0);
	m_vfRotate = Vec3(0, 0, 0);
	this->T = Grid(-0.5, 0.5, -0.5, 0.5, desiredWidth, desiredHeight);
	//
	// to be implemented
	//
	switch (m_iTestCase)
	{
	case 0:
		cout << "Explicit solver!\n";
		break;
	case 1:
		cout << "Implicit solver!\n";
		break;
	default:
		cout << "Empty Test!\n";
		break;
	}
}

void DiffusionSimulator::diffuseTemperatureExplicit(float timeElapsed) {
	// to be implemented
	int arraySize = desiredHeight*desiredWidth;
	double* T_new = new double[arraySize];
	for(int i = 0; i< desiredWidth; i++){
		for(int j = 0; j< desiredHeight; j++){
			double T_center = T.get(i,j);
			double T_east = T.get(i+1,j);
			double T_west = T.get(i-1,j);
			double T_south = T.get(i,j+1);
			double T_north = T.get(i,j-1);
			double x = (T_west - 2*T_center + T_east)/(T.xDelta*T.xDelta);
			double y = (T_north - 2*T_center + T_south)/(T.yDelta*T.yDelta);
			double T_center_new = T_center + alpha*timeElapsed*(x+y);
			T_new[i*desiredWidth + j] = T_center_new;
		}
	}

	for (int i = 0; i < desiredWidth; i++) {
		for (int j = 0; j < desiredHeight; j++) {
			T.set(i,j,T_new[i*desiredWidth + j]);
		}
	}
}


void DiffusionSimulator::diffuseTemperatureImplicit(float timeElapsed) {

	double lambda = alpha*timeElapsed/(T.xDelta*T.xDelta);
	const int N = T.width * T.height;

	SparseMatrix<Real> A(N);
	std::vector<Real> b(N);
	std::vector<Real> x(N);

	Real pcg_target_residual = 1e-05;
	Real pcg_max_iterations = 1000;
	Real ret_pcg_residual = 1e10;
	int  ret_pcg_iterations = -1;
	SparsePCGSolver<Real> solver;
	solver.set_solver_parameters(pcg_target_residual, pcg_max_iterations, 0.97, 0.25);

	//for every pixel fill A, b and x
	for (int i = 0; i < N; i++) {
		x[i] = 0;
		b[i] = T.T[i];
		//set diagonal
		A.set_element(i, i, 1 + 4*lambda);
		//set neighbors
		//if we are not on the left border
		if (i % T.width != 0) {
			A.set_element(i, i - 1, -lambda);
		}
		//if we are not on the right border
		if (i % T.width != T.width - 1) {
			A.set_element(i, i + 1, -lambda);
		}
		//if we are not on the top border
		if (i >= T.width) {
			A.set_element(i, i - T.width, -lambda);
		}
		//if we are not on the bottom border
		if (i < N - T.width) {
			A.set_element(i, i + T.width, -lambda);
		}
	}
	/*//iterate over all entrie of A
	for (int i = 0; i < N; i++) {
		for (int j = 0; j < N; j++) {
			std::cout << A(i, j) << " ";
		}
		std::cout << " = " << b[i] << "\n";
	}
	*/
	solver.solve(A, b, x, ret_pcg_residual, ret_pcg_iterations, 0);
	T.setAll(x);
}

void Grid::printVector(double* b)
{
	//print contents of b
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			std::cout << b[i*width + j] << " ";
		}
		std::cout << "\n";
	}
}

void DiffusionSimulator::printMatrix(double* A)
{
}

void DiffusionSimulator::createGrid(Grid old_grid)
{	delete[] T.T;
	T = Grid(-0.5, 0.5, -0.5, 0.5, desiredWidth, desiredHeight);
	//for every pixel
	for (int i = 0; i < desiredWidth; i++) {
		for (int j = 0; j < desiredHeight; j++) {
			//if we are in the old grid
			if (i < old_grid.width && j < old_grid.height) {
				T.set(i,j,old_grid.get(i,j));
			}
			else {
				T.set(i,j,0);
			}
		}
	}
}



void DiffusionSimulator::simulateTimestep(float timeStep)
{
	if (desiredWidth != T.width || desiredHeight != T.height) {
		//T.adjustGrid(desiredWidth, desiredHeight);
		createGrid(T);
	}
	// update current setup for each frame
	switch (m_iTestCase)
	{
	case 0:
		// feel free to change the signature of this function
		diffuseTemperatureExplicit(timeStep);
		break;
	case 1:
		// feel free to change the signature of this function
		diffuseTemperatureImplicit(timeStep);
		break;
	}
}

void DiffusionSimulator::drawObjects()
{
	// to be implemented
	//visualization
	//draw the grid
	for(int i = 0; i< desiredWidth; i++){
		for(int j = 0; j< desiredHeight; j++){
			Vec3 pos = Vec3(xMin + i*T.xDelta, yMin + j*T.yDelta, 0);
			double size = 0.01;
			double heat = T.get(i,j);
			Vec3 color = Vec3();
			if(heat > 0)
				color = Vec3(heat, heat, heat);
			else
				color = Vec3(heat,0,0);

			DUC->setUpLighting(Vec3(), 0.4*Vec3(1,1,1), 100, 0.6*color);
			DUC->drawSphere(pos, Vec3(T.xDelta/2, T.yDelta/2, size));
		}
	}
}


void DiffusionSimulator::drawFrame(ID3D11DeviceContext* pd3dImmediateContext)
{
	drawObjects();
}

void DiffusionSimulator::onClick(int x, int y)
{
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

void DiffusionSimulator::onMouse(int x, int y)
{
	m_oldtrackmouse.x = x;
	m_oldtrackmouse.y = y;
	m_trackmouse.x = x;
	m_trackmouse.y = y;
}

Grid::Grid(double xMin, double xMax, double yMin, double yMax, int width, int height): xMin(xMin), yMin(yMin), xMax(xMax), yMax(yMax), width(width), height(height)
{
	xDelta = (xMax - xMin)/width;
	yDelta = (yMax - yMin)/height;
	T = new double[width*height];
	std::srand(std::time(0));
	//X
	for (int i = 0; i < width; i++) {
		//Y
		for (int j = 0; j < height; j++) {
			T[i*width + j] = f_0(xMin + i*xDelta, yMin + j*yDelta);
		}
	}
}

double Grid::get(int i, int j)
{
	if(i < 0 || i >= width || j < 0 || j >= height)
		return 0;
	//printVector(T);
	return T[i*width + j];
}

void Grid::set(int i, int j, double val)
{
	if(i < 0 || i >= width || j < 0 || j >= height)
		return;

	T[i*width + j] = val;
}

void Grid::setAll(std::vector<double> x)
{
	for (int i = 0; i < width; i++) {
		for (int j = 0; j < height; j++) {
			T[i*width + j] = x[i*width + j];
		}
	}
}

void Grid::adjustGrid(int new_width, int new_height)
{
	width = new_width;
	height = new_height;
	xDelta = (xMax - xMin) / width;
	yDelta = (yMax - yMin) / height;
	/*
	//X
	for (int i = 0; i < width; i++) {
		//Y
		for (int j = 0; j < height; j++) {
			T[i * width + j] = f_0(xMin + i * xDelta, yMin + j * yDelta);
		}
	}
	*/
}

double Grid::f_0(double i, double j)
{
	//f0(x) = sin (πx/L)
	double maxX = xMin + width*xDelta;
	double maxY = yMin + height*yDelta;
	double s = sin(M_PI*i/maxX);
	double c = cos(M_PI*j/maxY);
	//return 2*(s+c) - 1;
	/*return sin(M_PI * i / maxX);
	std::mt19937 eng;
	std::uniform_real_distribution<float> randCol(-1.0f, 1.0f);
	return randCol(eng);  */ 

	return (static_cast<double>(std::rand()) / RAND_MAX) * 2 - 1;

}
