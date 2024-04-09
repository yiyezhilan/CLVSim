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

#include "MLP.h"

MLP::MLP(const std::string& model_file) {
    std::ifstream file(model_file);
    //if (!file.is_open()) {
    //    std::cerr << "Can not open file：" << model_file << std::endl;
    //    exit(EXIT_FAILURE);
    //}
    file >> model_data;
    rows = 0;
    cols = 0;
}
// MLP forward spread function
std::vector<double> MLP::predict(const std::vector<double>& input) {
    std::vector<double> x = input;
    for (auto& layer : model_data["layers"]) {
        if (layer["name"] == "coefficient") {
            coefficient = layer["data"].get<std::vector<double>>();
            rows = layer["shape"][1];
            cols = layer["shape"][0];
        } else if (layer["name"] == "intercepts") {
            biases = layer["data"].get<std::vector<double>>();
            x = dot(coefficient, x, rows, cols);
            // std::cout << x[0] << std::endl;
            for (size_t i = 0; i < x.size(); ++i) {
                x[i] += biases[i];
                if (layer["shape"][1] != 1) {  // The last layer doesn't use ReLU
                    x[i] = relu(x[i]);
                }
            }
        }
    }
    return x;
}

std::vector<double> MLP::dot(const std::vector<double>& matrix, const std::vector<double>& vec, int rows, int cols) {
    std::vector<double> result(rows, 0.0);
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols; ++j) {
            result[i] += matrix[j * rows + i] * vec[j];
        }
    }
    return result;
}
