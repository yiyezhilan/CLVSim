// =============================================================================
// Authors: Qingning Lan
// =============================================================================
//
// A MLP implement with C++ and json
// The json is converted from onnx with name "coefficient" and "intercepts"
// The MLP is used to predict the torque with input of {throttle, speed}
// However, the throttle is in [0,1] and the torque and speed are motor's real value, without declearaion
// So, there are 80 times difference between this torque and the torque to input the wheel shaft
//
// =============================================================================

#ifndef MLP_H
#define MLP_H

#include <iostream>
#include <vector>
#include <fstream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

class MLP {
  public:
    MLP(const std::string& model_file);
    std::vector<double> predict(const std::vector<double>& input);

  private:
    json model_data;
    std::vector<double> coefficient;
    std::vector<double> biases;
    double rows;
    double cols;

    // ReLU¼¤»îº¯Êý
    double relu(double x) { return (x > 0) ? x : 0; }

    std::vector<double> dot(const std::vector<double>& matrix, const std::vector<double>& vec, int rows, int cols);
};

#endif
