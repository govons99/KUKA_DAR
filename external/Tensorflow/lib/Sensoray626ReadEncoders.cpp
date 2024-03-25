#include <windows.h>
#include <stdio.h>

#include "Win626.h"
//#include "C:/Users/valerio/Desktop/pendubot_nmpc_export/dq1_8_dq2_8/acado_common.h"
//#include "C:/Users/valerio/Desktop/pendubot_nmpc_export/dq1_8_dq2_8/acado_auxiliary_functions.h"
//#include "C:/Users/valerio/Desktop/pendubot_nmpc_export/nuovi/dq1_7.8_dq2_6_soft/acado_common.h"
//#include "C:/Users/valerio/Desktop/pendubot_nmpc_export/nuovi/dq1_7.8_dq2_6_soft/acado_auxiliary_functions.h"
#include "C:/Users/valerio/Desktop/pendubot_nmpc_export/nuovi/dq1_7_8_soft_dq2_5_soft/acado_common.h"
#include "C:/Users/valerio/Desktop/pendubot_nmpc_export/nuovi/dq1_7_8_soft_dq2_5_soft/acado_auxiliary_functions.h"

#include <tensorflow/c/c_api.h>

#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>
#include <iomanip>
#include <cstring>
#include <cmath>
#include <cstdlib>
#include <chrono>
#include <windows.h>
#include <stdio.h>
#include "Win626.h"
#include <thread>
using namespace std;

/* Some convenient definitions. */
#define NX          ACADO_NX  /* Number of differential state variables.  */
#define NXA         ACADO_NXA /* Number of algebraic variables. */
#define NU          ACADO_NU  /* Number of control inputs. */
#define NOD         ACADO_NOD /* Number of online data values. */
#define NY          ACADO_NY  /* Number of measurements/references on nodes 0..N - 1. */
#define NYN         ACADO_NYN /* Number of measurements/references on node N. */
#define N           ACADO_N   /* Number of intervals in the horizon. */
#define NUM_STEPS   6000      /* Number of real-time iterations. */

# define pi           3.14159265358979323846

#define DAC_VSCALAR 819.2
#define kt 0.0742
#define kupm 6

bool RL = true;

bool emergency_stop(double q1, double q2, double dq1, double dq2) {
	bool activate = 0;
	if (q1 > 2.3) {
		activate = 1;
	}
	if (q1 < -5.0) {
		activate = 1;
	}
	if (dq1 > 15 || dq1 < -15) {
		activate = 1;
	}
	if (dq2 > 25 || dq2 < -25) {
		activate = 1;
	}
	return activate;
}

long long MAXIMUM = 0;

double counter2deg(int counter, double max_counter) {
	if (counter > 600000)
		counter -= 16777215;
	double angle = counter * max_counter;
	return angle;
}

//net_utility_function
void free_buffer(void* data, size_t length) {
	free(data);
}
TF_Buffer* read_file(const char* file) {
	FILE *f = fopen(file, "rb");
	fseek(f, 0, SEEK_END);
	long fsize = ftell(f);
	fseek(f, 0, SEEK_SET);  //same as rewind(f);

	void* data = malloc(fsize);
	fread(data, fsize, 1, f);
	fclose(f);

	TF_Buffer* buf = TF_NewBuffer();
	buf->data = data;
	buf->length = fsize;
	buf->data_deallocator = free_buffer;
	return buf;
}
static void Deallocator(void* data, size_t length, void* arg) {
}


double filter_angle(std::vector <double> q_samples, double Ts, double q_actual, double iteration, std::vector <double> dq_samples, int what_angle,bool balancing) {

	double dq = 0;

	//if (iteration >= 90) {
	if (!balancing) {
		if (iteration >= 90) {
			dq = 1.573*dq_samples[4] - 0.6188*dq_samples[3] + 22.65*q_samples[4] - 22.65*q_samples[3];
			//dq = ((25. / 12.)*q_samples[4] - 4 * q_samples[3] + 3 * q_samples[2] - (4. / 3.)*q_samples[1] + 0.25*q_samples[0]) / Ts;
		}
		else {
			dq = (q_samples[4] - q_samples[3]) / Ts;
		}
	}
	else {
		//1ms
		dq = 1.774*dq_samples[4] - 0.7866*dq_samples[3] + 12.77*q_samples[4] - 12.77*q_samples[3];
		//0.5ms
		//dq = 1.884*dq_samples[4] - 0.8869*dq_samples[3] + 6.781*q_samples[4] - 6.781*q_samples[3];
		//0.2ms
		//dq = 1.953*dq_samples[4] - 0.9531*dq_samples[3] + 2.812*q_samples[4] - 2.812*q_samples[3];

		//dq = (q_samples[4] - q_samples[3]) / 0.0005;

	}

	/*if (what_angle == 2) {
		if (iteration >= 1000)
			dq = (q_samples[4] - q_samples[3]) / Ts;
	}*/
	//if (iteration >= 2000)
	//	dq = ((25. / 12.)*q_samples[4] - 4 * q_samples[3] + 3 * q_samples[2] - (4. / 3.)*q_samples[1] + 0.25*q_samples[0]) / Ts;
	
	//if (what_angle == 1 && abs(dq) >= 5.5 && abs(dq) <= 9.0) {
		//dq = 1.573*dq_samples[4] - 0.6188*dq_samples[3] + 22.65*q_samples[4] - 22.65*q_samples[3];
		//if (abs(dq) >= 7.0 && abs(dq) <= 9.0)
		//	dq = -7.0;
	//}
	
	return dq;
}

double mean(std::vector <double> q_samples) {
	double m = 0;
	for (int d = 0; d <= 199; d++) {
		m += q_samples[d];
	}
	m = m / 200;
	return m;
}


double setDACvoltage(HBD hbd, double tau) {
	// Convert NMPC tau into volts
	//tau = 0.0;
	double volts = tau / (kt*kupm);
	volts = volts - 0.0085;
	//std::cout << volts << std::endl;
	// Make adjustments to prevent conversion errors.
	if (volts > 10.0) volts = 10.0;
	else if (volts < -10.0) volts = -10.0;
	// Program new DAC setpoint.
	S626_WriteDAC(hbd, 0, (SHORT)(volts * DAC_VSCALAR));
	S626_WriteDAC(hbd, 1, 0);
	S626_WriteDAC(hbd, 2, 0);
	S626_WriteDAC(hbd, 3, 0);

	return volts;
}

void setOuputZero(HBD hbd) {
	//S626_WriteDAC(hbd, 0, 0);
	double volts = -0.0085;
	S626_WriteDAC(hbd, 0, (SHORT)(volts * DAC_VSCALAR));
	S626_WriteDAC(hbd, 1, 0);
	S626_WriteDAC(hbd, 2, 0);
	S626_WriteDAC(hbd, 3, 0);
}


/* Global variables used by the solver. */
ACADOvariables acadoVariables;
ACADOworkspace acadoWorkspace;


/* A template for testing the solver. */
int main()
{
	SetPriorityClass(GetCurrentProcess(), REALTIME_PRIORITY_CLASS);
	SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_TIME_CRITICAL);
	/* Some temporary variables. */
	unsigned i, j, iter;
	int failure;
	double volts;
	DWORD version;
	// Encoder 1 and 2
	WORD counter_q1 = CNTR_0A;
	WORD counter_q2 = CNTR_1A;

	// Link to S626.DLL.
	S626_DLLOpen();

	// Declare Model 626 board to driver and launch the interrupt thread.
	HBD hdb = 0;
	S626_OpenBoard(hdb, 0, 0, 0);

	// Reset all solver memory
	memset(&acadoWorkspace, 0, sizeof(acadoWorkspace));
	memset(&acadoVariables, 0, sizeof(acadoVariables));


	/* Initialize the solver. */
	acado_initializeSolver();

	/* MPC: initialize the current state feedback. */
#if ACADO_INITIAL_STATE_FIXED
	double x0[] = { -pi, 0.0, 0.0, 0.0 };
	for (i = 0; i < NX; ++i) acadoVariables.x0[i] = x0[i];
#endif


	/* Logger initialization */
	vector< vector< double > > log;

	log.resize(NUM_STEPS);
	for (i = 0; i < log.size(); ++i)
		log[i].resize(12, 0.0);

	/* Prepare first step */
	acado_preparationStep();



	if (S626_GetErrors(hdb)) {
		printf("ERROR: problem opening board.\n");
		return 0;
	}
	else
	{
		S626_WriteDAC(hdb, 0, 0); //8192 correspond to 10 Volts
		S626_WriteDAC(hdb, 1, 0);
		S626_WriteDAC(hdb, 2, 0);
		S626_WriteDAC(hdb, 3, 0);
		S626_CounterModeSet(0, counter_q1,
			(LOADSRC_INDX << BF_LOADSRC) | // Index causes preload.
			(INDXSRC_HARD << BF_INDXSRC) | // Hardware index is enabled.
			(INDXPOL_POS << BF_INDXPOL) | // Active high index.
			(CLKSRC_COUNTER << BF_CLKSRC) | // Operating mode is Counter.
			(CLKPOL_POS << BF_CLKPOL) | // Active high clock.
			(CLKMULT_4X << BF_CLKMULT) | // Clock multiplier is 4x.
			(CLKENAB_ALWAYS << BF_CLKENAB)); // Counting is always enabled.

		// Initialize preload value to zero so that the counter core will be set
		// to zero upon the occurance of an Index.
		S626_CounterPreload(hdb, counter_q1, 0);

		// Enable latching of accumulated counts on demand. This assumes that
		// there is no conflict with the latch source used by paired counter 2B.
		S626_CounterLatchSourceSet(hdb, counter_q1, LATCHSRC_AB_READ);

		// Reset encoder
		S626_CounterSoftIndex(hdb, counter_q1);

		// Enable the counter to generate interrupt requests upon index.
		S626_CounterIntSourceSet(hdb, counter_q1, INTSRC_INDX);

		S626_CounterModeSet(hdb, counter_q2,
			(LOADSRC_INDX << BF_LOADSRC) | // Index causes preload.
			(INDXSRC_HARD << BF_INDXSRC) | // Hardware index is enabled.
			(INDXPOL_POS << BF_INDXPOL) | // Active high index.
			(CLKSRC_COUNTER << BF_CLKSRC) | // Operating mode is Counter.
			(CLKPOL_POS << BF_CLKPOL) | // Active high clock.
			(CLKMULT_4X << BF_CLKMULT) | // Clock multiplier is 4x.
			(CLKENAB_ALWAYS << BF_CLKENAB)); // Counting is always enabled.

		// Initialize preload value to zero so that the counter core will be set
		// to zero upon the occurance of an Index.
		S626_CounterPreload(hdb, counter_q2, 0);

		// Enable latching of accumulated counts on demand. This assumes that
		// there is no conflict with the latch source used by paired counter 2B.
		S626_CounterLatchSourceSet(hdb, counter_q2, LATCHSRC_AB_READ);

		// Reset encoder
		S626_CounterSoftIndex(hdb, counter_q2);

		// Enable the counter to generate interrupt requests upon index.
		S626_CounterIntSourceSet(hdb, counter_q2, INTSRC_INDX);
	}

/* The "real-time iterations" loop. */
double q1, q2, dq1, dq2, Ts;
Ts = 0.002;
//net_stuff
TF_Session * session;
TF_Graph* graph;
TF_Status* status;
std::vector<TF_Output> inputs;
std::vector<TF_Output> outputs;
TF_Buffer* graph_def = read_file("C:/Users/valerio/Desktop/guided_mpc/dependencies/neural_network_policy/froze_graph_lowvel_07_2.pb");
graph = TF_NewGraph();
status = TF_NewStatus();
TF_ImportGraphDefOptions* opts = TF_NewImportGraphDefOptions();
TF_GraphImportGraphDef(graph, graph_def, opts, status);
TF_DeleteImportGraphDefOptions(opts);
if (TF_GetCode(status) != TF_OK) {
	fprintf(stderr, "ERROR: Unable to import graph %s", TF_Message(status));
}
fprintf(stdout, "Successfully imported graph..\n");
fprintf(stdout, "Running session...\n");
TF_SessionOptions * options = TF_NewSessionOptions();
session = TF_NewSession(graph, options, status);
inputs.push_back({ TF_GraphOperationByName(graph, "obs0"), 0 });
outputs.push_back({ TF_GraphOperationByName(graph, "actor/Tanh"), 0 });
const int num_bytes_in = 4 * sizeof(float);
const int num_bytes_out = 1 * sizeof(float);
int64_t in_dims[] = { 1,4 };
int64_t out_dims[] = { 1,1 };

//INITIALIZATION
float values[1 * 4] = { 0,0, 0,0 };
std::vector<TF_Output> inputs_trial;
std::vector<TF_Tensor*> input_values_trial;
inputs_trial.push_back({ TF_GraphOperationByName(graph, "obs0"), 0 });
input_values_trial.push_back(TF_NewTensor(TF_FLOAT, in_dims, 2, values, num_bytes_in, &Deallocator, 0));
std::vector<TF_Output> outputs_trial;
outputs_trial.push_back({ TF_GraphOperationByName(graph, "actor/Tanh"), 0 });
std::vector<TF_Tensor*> output_values_trial;
output_values_trial.push_back(TF_AllocateTensor(TF_FLOAT, out_dims, 2, num_bytes_out));
TF_SessionRun(session, nullptr,
	&inputs[0], &input_values_trial[0], 1,
	&outputs_trial[0], &output_values_trial[0], 1,
	nullptr, 0, nullptr, status);
const auto data = static_cast<float*>(TF_TensorData(input_values_trial.at(0)));
const auto tau_data = static_cast<float*>(TF_TensorData(output_values_trial.at(0)));


std::vector <double> trajectories;
//trajectories.resize(N*NY + 5);
trajectories.resize(N*NY + 6);
ACADOworkspace net_integration;
ACADOworkspace mpc_integration;
std::vector<double> q1_samples;
q1_samples.resize(5);
for (i = 0; i <= 4; i++) q1_samples[i] = 0;
std::vector<double> q2_samples;
q2_samples.resize(5);
for (i = 0; i <= 4; i++) q2_samples[i] = 0;
mpc_integration.state[0] = -pi;
net_integration.state[0] = -pi;
auto start_zero = std::chrono::high_resolution_clock::now();

//RTI MPC
dq1 = 0;
dq2 = 0;
std::vector<double> dq1_filter_samples;
dq1_filter_samples.resize(5);
for (i = 0; i <= 4; i++) dq1_filter_samples[i] = 0;
std::vector<double> dq2_filters_samples;
dq2_filters_samples.resize(5);
for (i = 0; i <= 4; i++) dq2_filters_samples[i] = 0;


std::cout << "START" << std::endl;

bool balancing = false;
bool balancing2 = false;
float dq1_notBounded = 0;
float dq2_notBounded = 0;

std::vector<double> q1_mean;
q1_mean.resize(200);
std::vector<double> q2_mean;
q2_mean.resize(200);
double offset_q1 = 0;
double offset_q2 = 0;

for (iter = 0; iter < NUM_STEPS; ++iter)
{
	auto start = std::chrono::high_resolution_clock::now();
	auto time_incremental = chrono::duration_cast<chrono::microseconds>(start - start_zero).count();
	double timestamp = (double)time_incremental / 1e6;
	q1 = S626_CounterReadLatch(hdb, counter_q1);
	q2 = S626_CounterReadLatch(hdb, counter_q2);

	q1 = counter2deg(q1, -pi / 4096);
	q2 = -counter2deg(q2, -2 * pi / 4096);
	q1_samples[0] = q1_samples[1];
	q1_samples[1] = q1_samples[2];
	q1_samples[2] = q1_samples[3];
	q1_samples[3] = q1_samples[4];
	q1_samples[4] = q1;
	q2_samples[0] = q2_samples[1];
	q2_samples[1] = q2_samples[2];
	q2_samples[2] = q2_samples[3];
	q2_samples[3] = q2_samples[4];
	q2_samples[4] = q2;

	dq1 = filter_angle(q1_samples, Ts, q1, iter, dq1_filter_samples, 1, balancing);
	dq2 = filter_angle(q2_samples, Ts, q2, iter, dq2_filters_samples, 2, balancing);







	dq1_filter_samples[0] = dq1_filter_samples[1];
	dq1_filter_samples[1] = dq1_filter_samples[2];
	dq1_filter_samples[2] = dq1_filter_samples[3];
	dq1_filter_samples[3] = dq1_filter_samples[4];
	dq1_filter_samples[4] = dq1;
	dq2_filters_samples[0] = dq2_filters_samples[1];
	dq2_filters_samples[1] = dq2_filters_samples[2];
	dq2_filters_samples[2] = dq2_filters_samples[3];
	dq2_filters_samples[3] = dq2_filters_samples[4];
	dq2_filters_samples[4] = dq2;
	//std::cout <<"state" << q1 << " " << q2 << " " << dq1 << " " << dq2 << std::endl;

	dq1_notBounded = dq1;
	dq2_notBounded = dq2;

	

	/*if (!RL) {
		if (dq1 > 8)
			dq1 = 8;
		if (dq1 < -8)
			dq1 = -8;
		if (dq2 > 8)
			dq2 = 8;
		if (dq2 < -8)
			dq2 = -8;
	}*/

	bool emergency = emergency_stop(q1, q2, dq1, dq2);
	
	//bool emergency = emergency_stop(q1, q2, dq1, dq2);
	if (emergency) {
		setOuputZero(hdb);
		cout << "failed!" << endl;
		cout << "iter " << iter << endl;
		std::cout << q1 << " " << q2 << " " << dq1 << " " << dq2 << std::endl;
		std::cout << q1 << " " << q2 << " " << dq1 << " " << dq2 << std::endl;
		//return EXIT_FAILURE;
		break;
	}
	
	//update x0
	mpc_integration.state[0] = -pi - q1;
	mpc_integration.state[1] = q2;
	mpc_integration.state[2] = -dq1;
	mpc_integration.state[3] = dq2;
	
	net_integration.state[1] = q2;
	net_integration.state[2] = -dq1;
	net_integration.state[3] = dq2;

	double action = 0.0;
	if (!balancing) {
		for (unsigned int k = 0; k < N + 1; k++) {
			//std::cout << "Number: " << k << std::endl;
			net_integration.state[0] = mpc_integration.state[0] + pi;
			if (net_integration.state[0] > pi)
				net_integration.state[0] = net_integration.state[0] - pi * 2;
			if (net_integration.state[0] < -pi)
				net_integration.state[0] = net_integration.state[0] + pi * 2;

			float values[1 * 4] = { net_integration.state[0],net_integration.state[1], net_integration.state[2],net_integration.state[3] };
			std::vector<TF_Output> inputs;
			std::vector<TF_Tensor*> input_values;
			inputs.push_back({ TF_GraphOperationByName(graph, "obs0"), 0 });
			input_values.push_back(TF_NewTensor(TF_FLOAT, in_dims, 2, values, num_bytes_in, &Deallocator, 0));
			std::vector<TF_Output> outputs;
			outputs.push_back({ TF_GraphOperationByName(graph, "actor/Tanh"), 0 });
			std::vector<TF_Tensor*> output_values;
			output_values.push_back(TF_AllocateTensor(TF_FLOAT, out_dims, 2, num_bytes_out));

			TF_SessionRun(session, nullptr,
				&inputs[0], &input_values[0], 1,
				&outputs[0], &output_values[0], 1,
				nullptr, 0, nullptr, status);
			const auto data = static_cast<float*>(TF_TensorData(input_values.at(0)));
			const auto tau_data = static_cast<float*>(TF_TensorData(output_values.at(0)));
			//double action = ((double)tau_data[0])*0.4;
			action = ((double)tau_data[0])*0.4;
			if (RL)
				break;

			mpc_integration.state[28] = action;

			acado_integrate(mpc_integration.state, 1);
			net_integration.state[0] = mpc_integration.state[0];
			net_integration.state[1] = mpc_integration.state[1];
			net_integration.state[2] = mpc_integration.state[2];
			net_integration.state[3] = mpc_integration.state[3];

			//std::cout << "q1" << net_integration.state[0] << std::endl;

			trajectories[k * 6 + 0] = mpc_integration.state[0];
			trajectories[k * 6 + 1] = mpc_integration.state[1];
			trajectories[k * 6 + 2] = mpc_integration.state[2];
			trajectories[k * 6 + 3] = mpc_integration.state[3];
			trajectories[k * 6 + 4] = 0.0;
			trajectories[k * 6 + 5] = 0.0;
		}
	}



	
	//while (true)
	//{
	//	auto actual_loop = std::chrono::high_resolution_clock::now();
	//	auto elapsed = chrono::duration_cast<chrono::microseconds>(actual_loop - start).count();
	//	if (elapsed > 1999)
	//		break;
	//}
	//continue;

	mpc_integration.state[0] = -pi - q1;
	mpc_integration.state[1] = q2;
	mpc_integration.state[2] = -dq1;
	mpc_integration.state[3] = dq2;

	if ((fabs(-pi - q1) < 0.6 && fabs(q2) < 0.4 && fabs(-dq1) < 6 && fabs(dq2) < 6) || balancing) {
		balancing = true;
		if (iter > 2000 && iter < 2200) {
			q1_mean[iter - 2001] = -pi - q1;
			q2_mean[iter - 2001] = q2;
		}
		if (iter == 2205) {
			offset_q1 = mean(q1_mean)*0;
			offset_q2 = mean(q2_mean)*0;
		}
		//action = -4.947*(-pi - q1) + (-4.205*q2) + (-0.848*-dq1) + (-0.478*dq2);
		//action = -5.4483*(-pi - q1) + (-4.3042*q2) + (-0.9788*-dq1) + (-0.4730*dq2);
		//action = -11.0939*(-pi - q1) + (-10.6951*q2) + (-1.9565*-dq1) + (-1.2633*dq2);
		//action = -3.3612*(-pi - q1) + (-3.0378*q2) + (-0.5850*-dq1) + (-0.3443*dq2);
		
		//good
		action = -5.9900*(-pi - q1 - offset_q1) + (-5.4311*(q2 - offset_q2)) + (-1.0472*-dq1) + (-0.6253*dq2);

		//action = -10.7818*(-pi - q1 - offset_q1) + (-10.2525*(q2 - offset_q2)) + (-1.8564*-dq1) + (-1.2331*dq2);
		
		
		//action = -7.8215*(-pi - q1 - offset_q1) + (-6.9873*(q2 - offset_q2)) + (-1.3627*-dq1) + (-0.8078*dq2);
		/*if (action > 0.2)
			action = 0.2;
		if (action < -0.2)
			action = -0.2;*/
		if (action > 0.2)
			action = 0.2;
		if (action < -0.2)
			action = -0.2;

		volts = setDACvoltage(hdb, -action);
	}

	if (RL && !balancing)
		volts = setDACvoltage(hdb, action);

	/* ---------------------------------- */
	// ACADO
	/* ---------------------------------- */
	/* Get current state from sensors */

	//std::cout << q2 << std::endl;
	//std::cout << "initial state MPC" << std::endl;
	double total_acado = 0;
	if (!RL) {
		if (!balancing) {
			for (i = 0; i < NX; ++i) {
				acadoVariables.x0[i] = mpc_integration.state[i];
			}

			for (i = 0; i < N; ++i) {
				for (j = 0; j < NY; ++j) {
					acadoVariables.y[i*NY + j] = trajectories[i*NY + j];
				}
			}

			for (i = 0; i < NYN; ++i) {
				acadoVariables.yN[i] = trajectories[N*NY + i];
			}

			auto start_step = std::chrono::high_resolution_clock::now();
			failure = acado_feedbackStep();
			auto finish_step = std::chrono::high_resolution_clock::now();
			auto time_step = chrono::duration_cast<chrono::microseconds>(finish_step - start_step).count();
			double Ts_step = (double)time_step / 1e3;

			/* For debugging */
			if (failure)
			{
				setOuputZero(hdb);
				cout << "Solver failed MPC!" << endl;
				std::cout << "iteration " << iter << std::endl;
				std::cout << q1 << " " << q2 << " " << -dq1 << " " << dq2 << std::endl;
				std::cout << mpc_integration.state[0] << " " << mpc_integration.state[1] << " " << mpc_integration.state[2] << " " << mpc_integration.state[3] << std::endl;
				volts = 0.0;
				break;
			}
			else {
				//std::cout << "torque" << acadoVariables.u[0] << std::endl;
				volts = setDACvoltage(hdb, acadoVariables.u[0]);
			}

			/* Prepare for the next step. */
			auto start_preparation = std::chrono::high_resolution_clock::now();
			acado_preparationStep();
			auto finish_prepation = std::chrono::high_resolution_clock::now();
			auto time_preparation = chrono::duration_cast<chrono::microseconds>(finish_prepation - start_preparation).count();
			double Ts_preparation = (double)time_preparation / 1e3;

			total_acado = Ts_step + Ts_preparation;

		}
	}

	
	log[iter][1] = q1;
	log[iter][2] = q2;
	log[iter][3] = dq1_notBounded;
	log[iter][4] = dq2_notBounded;
	log[iter][5] = mpc_integration.state[0];
	log[iter][6] = mpc_integration.state[1];
	log[iter][7] = mpc_integration.state[2];
	log[iter][8] = mpc_integration.state[3];
	log[iter][9] = acadoVariables.u[0];
	//log[iter][10] = action;
	//log[iter][11] = timestamp;
	log[iter][10] = balancing;
	log[iter][11] = offset_q2;

	auto finish = std::chrono::high_resolution_clock::now();
	auto time = chrono::duration_cast<chrono::microseconds>(finish - start).count();
	double Ts_fake = (double)time / 1e3;
	int wait_time = int(Ts_fake);

	while (true)
	{
		auto actual_loop = std::chrono::high_resolution_clock::now();
		auto elapsed = chrono::duration_cast<chrono::microseconds>(actual_loop - start).count();
		if(!balancing)
			if (elapsed > 1999)
				break;
		if(balancing)
			if (elapsed > 999)
				break;
	}
	
	auto finish_total = std::chrono::high_resolution_clock::now();
	auto time_total = chrono::duration_cast<chrono::microseconds>(finish_total - start).count();
	double Ts_actual = (double)time_total / 1e6;
	Ts = Ts_actual;
	log[iter][0] = Ts_actual;
	}

	setOuputZero(hdb);

	/* Save log into a file */
	ofstream dataLog("./data_log.txt");
	if (dataLog.is_open())
	{
		for (i = 0; i < log.size(); i++)
		{
			dataLog << log[i][0] << " ";
			dataLog << log[i][1] << " ";
			dataLog << log[i][2] << " ";
			dataLog << log[i][3] << " ";
			dataLog << log[i][4] << " ";
			dataLog << log[i][5] << " ";
			dataLog << log[i][6] << " ";
			dataLog << log[i][7] << " ";
			dataLog << log[i][8] << " ";
			dataLog << log[i][9] << " ";
			dataLog << log[i][10] << " ";
			dataLog << log[i][11] << " ";
			dataLog << endl;
		}

		dataLog.close();
	}
	else
	{
		cout << "Log file could not be opened" << endl;

		return 1;
	}

	// Disconnect from the 626 board.
	S626_CloseBoard(0);
	// Disconnect from S626.DLL.
	S626_DLLClose();
	return 0;
}
