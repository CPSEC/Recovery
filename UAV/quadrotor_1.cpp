#include "../flowstar-template/Continuous.h"

using namespace flowstar;
using namespace std;


int main()
{
	unsigned int numVars = 13;
	intervalNumPrecision = 100;

	int x1_id = stateVars.declareVar("x1");
	int x2_id = stateVars.declareVar("x2");
	int x3_id = stateVars.declareVar("x3");
	int x4_id = stateVars.declareVar("x4");
	int x5_id = stateVars.declareVar("x5");
	int x6_id = stateVars.declareVar("x6");
	int x7_id = stateVars.declareVar("x7");
	int x8_id = stateVars.declareVar("x8");
	int x9_id = stateVars.declareVar("x9");
	int x10_id = stateVars.declareVar("x10");
	int x11_id = stateVars.declareVar("x11");
	int x12_id = stateVars.declareVar("x12");
	int t_id = stateVars.declareVar("t");



	// define the dynamics
	Expression_AST<Real> ode_expression_x1("cos(x8)*cos(x9)*x4 + (sin(x7)*sin(x8)*cos(x9) - cos(x7)*sin(x9))*x5 + (cos(x7)*sin(x8)*cos(x9) + sin(x7)*sin(x9))*x6");
	Expression_AST<Real> ode_expression_x2("cos(x8)*sin(x9)*x4 + (sin(x7)*sin(x8)*sin(x9) + cos(x7)*cos(x9))*x5 + (cos(x7)*sin(x8)*sin(x9) - sin(x7)*cos(x9))*x6");
	Expression_AST<Real> ode_expression_x3("sin(x8)*x4 - sin(x7)*cos(x8)*x5 - cos(x7)*cos(x8)*x6");
	Expression_AST<Real> ode_expression_x4("x12*x5 - x11*x6 - 9.81*sin(x8)");
	Expression_AST<Real> ode_expression_x5("x10*x6 - x12*x4 + 9.81*cos(x8)*sin(x7)");
	Expression_AST<Real> ode_expression_x6("x11*x4 - x10*x5 + 9.81*cos(x8)*cos(x7) - 9.81 + 7.14285714285714*(x3 - 1) - 2.14285714285714*x6");
	Expression_AST<Real> ode_expression_x7("x10 + (sin(x7)*(sin(x8)/cos(x8)))*x11 + (cos(x7)*(sin(x8)/cos(x8)))*x12");
	Expression_AST<Real> ode_expression_x8("cos(x7)*x11 - sin(x7)*x12");
	Expression_AST<Real> ode_expression_x9("(sin(x7)/cos(x8))*x11 + (cos(x7)/cos(x8))*x12");
	Expression_AST<Real> ode_expression_x10("-0.92592592592593*x11*x12 - 18.51851851851852*(x7 + x10)");
	Expression_AST<Real> ode_expression_x11("0.92592592592593*x10*x12 - 18.51851851851852*(x8 + x11)");
	Expression_AST<Real> ode_expression_x12("0");
	Expression_AST<Real> ode_expression_t("1");



	vector<Expression_AST<Real> > ode_rhs(numVars);
	ode_rhs[x1_id] = ode_expression_x1;
	ode_rhs[x2_id] = ode_expression_x2;
	ode_rhs[x3_id] = ode_expression_x3;
	ode_rhs[x4_id] = ode_expression_x4;
	ode_rhs[x5_id] = ode_expression_x5;
	ode_rhs[x6_id] = ode_expression_x6;
	ode_rhs[x7_id] = ode_expression_x7;
	ode_rhs[x8_id] = ode_expression_x8;
	ode_rhs[x9_id] = ode_expression_x9;
	ode_rhs[x10_id] = ode_expression_x10;
	ode_rhs[x11_id] = ode_expression_x11;
	ode_rhs[x12_id] = ode_expression_x12;
	ode_rhs[t_id] = ode_expression_t;



	Deterministic_Continuous_Dynamics dynamics(ode_rhs);



	// set the reachability parameters
	Computational_Setting setting;

	// set the stepsize and the order
	setting.setFixedStepsize(0.025, 3, 8);			// adaptive orders

	// set the time horizon
	setting.setTime(5);

	// set the cutoff threshold
	setting.setCutoffThreshold(1e-6);

	// set the queue size for the symbolic remainder, it is 0 if symbolic remainder is not used
	setting.setQueueSize(0);

	// print out the computation steps
	setting.printOn();

	// set up the remainder estimation
	Interval I(-1e-3, 1e-3);
	vector<Interval> remainder_estimation(numVars, I);
	setting.setRemainderEstimation(remainder_estimation);

	// call this function when all of the parameters are defined
	setting.prepare();

	double w = 0.1;

	// define the initial set which is a box
	Interval init_x1(-w, w), init_x2(-w, w), init_x3(-w, w), init_x4(-w, w),
			init_x5(-w, w), init_x6(-w, w), init_x7, init_x8, init_x9, init_x10, init_x11,
			init_x12, init_t;

	vector<Interval> initial_box(numVars);
	initial_box[x1_id] = init_x1;
	initial_box[x2_id] = init_x2;
	initial_box[x3_id] = init_x3;
	initial_box[x4_id] = init_x4;
	initial_box[x5_id] = init_x5;
	initial_box[x6_id] = init_x6;
	initial_box[x7_id] = init_x7;
	initial_box[x8_id] = init_x8;
	initial_box[x9_id] = init_x9;
	initial_box[x10_id] = init_x10;
	initial_box[x11_id] = init_x11;
	initial_box[x12_id] = init_x12;
	initial_box[t_id] = init_t;


	Flowpipe initialSet(initial_box);


	// the unsafe set
	vector<Constraint> unsafeSet;


	/*
	 * The structure of the class Result_of_Reachability is defined as below:
	 * nonlinear_flowpipes: the list of computed flowpipes
	 * tmv_flowpipes: translation of the flowpipes, they will be used for further analysis
	 * fp_end_of_time: the flowpipe at the time T
	 */
	Result_of_Reachability result;

	// run the reachability computation
	clock_t begin, end;
	begin = clock();

	dynamics.reach(result, setting, initialSet, unsafeSet);

	end = clock();
	printf("time cost: %lf\n", (double)(end - begin) / CLOCKS_PER_SEC);

/*
	switch(result.status)
	{
	case COMPLETED_SAFE:
		printf("Safe\n");
		break;
	case COMPLETED_UNSAFE:
		printf("Unsafe\n");
		break;
	case COMPLETED_UNKNOWN:
		printf("Unknown\n");
		break;
	default:
		printf("Fail to compute flowpipes.\n");
	}

	std::vector<Interval> box;
	result.fp_end_of_time.intEval(box, 8, setting.tm_setting.cutoff_threshold);
	printf("Width of x4 at t = 20: %e\n", box[x4_id].width());
*/


	result.transformToTaylorModels(setting);

	Plot_Setting plot_setting;
	plot_setting.printOn();
	plot_setting.setOutputDims(t_id, x3_id);

	plot_setting.plot_2D_interval_MATLAB("quadrotor_1", result);

	return 0;
}

