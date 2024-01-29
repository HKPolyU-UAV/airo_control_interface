
#include <cmath> 
#include <iostream>
int main() {
     const double K1 = 0.06092;
     const double K2 = 1.843;
     const double K3 = 0.7826;
     const double voltage_2 = 14.8;

     double thrust = (sqrt(((0.72 * 15)/ 3) / (K1 * pow(voltage_2, K2)) + pow(((1 - K3) / (2 * sqrt(K3))), 2)) - ((1 - K3) / (2 * sqrt(K3)))) / sqrt(K3);
     std::cout << thrust << std::endl;

     return 0;
}
