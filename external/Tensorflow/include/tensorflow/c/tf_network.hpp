#ifndef TF_NETWORK_
#define TF_NETWORK_

#include<tensorflow/c/utils.hpp>
#include<tensorflow/c/c_api.h>
#include<tensorflow/c/c_api_experimental.h>
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
#include <stdio.h>
#include <thread>
#include <string.h>





class tf_network
{
    public:
        tf_network(char * net_path, char * in_name, char * out_name)
        {   
            graph = TF_NewGraph();
            status = TF_NewStatus();
            graph_def = read_file(net_path);
            opts = TF_NewImportGraphDefOptions();
            TF_GraphImportGraphDef(graph, graph_def, opts, status);
            TF_DeleteImportGraphDefOptions(opts);
            if (TF_GetCode(status) != TF_OK)
            {
	            fprintf(stderr, "ERROR: Unable to import graph %s", TF_Message(status));
            }
            
            std::cout << "Successfully imported graph" << "\n";
            std::cout << "Running session" << "\n";
            options = TF_NewSessionOptions();
            session = TF_NewSession(graph, options, status);

            input_name = (char *) malloc(strlen(in_name) * sizeof(char));
            output_name = (char *) malloc(strlen(out_name) * sizeof(char));
            strcpy(input_name, in_name);
            strcpy(output_name, out_name);
            
        };
        
        Network_Output predict(Network_Input input);

        protected:

            void EigToArray(Network_Input IN,float *OUT);
            Network_Output ArrayToEig(float *IN);

            TF_Buffer* read_file(const char* file);
            TF_Session * session;
            TF_Graph   * graph;
            TF_Status  * status;
            TF_Buffer  * graph_def;
            TF_ImportGraphDefOptions* opts;
            TF_SessionOptions * options;
            const int num_bytes_in = INPUT_SIZE * sizeof(float);
            const int num_bytes_out = OUTPUT_SIZE * sizeof(float);
            int64_t in_dims[2] = { 1,INPUT_SIZE };
            int64_t out_dims[2] = { 1,OUTPUT_SIZE };

            char * input_name;
            char * output_name;
};

#endif /* TF_NETWORK_*/

void free_buffer(void* data, size_t length);
static void Deallocator(void* data, size_t length, void* arg);