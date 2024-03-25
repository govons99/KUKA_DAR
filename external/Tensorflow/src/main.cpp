
#include<tensorflow/c/tf_network.hpp>
int main(int argc, char *argv[])
{
    //char net_path[] = "/home/kuka_linux/Desktop/Kuka_Controller/external/Tensorflow/models/linear/net1.pb";
    char net_path[] = "/home/kuka_linux/Desktop/Kuka_Controller/external/Tensorflow/models/linear/net2.pb";
    char in_name[] = "KerasInput_input_26";
    char out_name[] = "KerasOutput_25/BiasAdd";
    std::cout << "hello world  tensorflow in C" << "\n";   
    tf_network prova(net_path,in_name, out_name);
    float values1[] = {0,0,0};
    float values2[] = {1,1,1};
    float values3[] = {2,2,2};
    float output[1];

    prova.predict(values1,output);
    std::cout << output[0] << "\n";
    prova.predict(values2,output);
    std::cout << output[0] << "\n";
    prova.predict(values3,output);
    std::cout << output[0] << "\n";
}