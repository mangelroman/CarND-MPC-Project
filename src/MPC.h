#ifndef MPC_H
#define MPC_H

#include <vector>

class MPC {
  public:
    MPC();
    virtual ~MPC();

    void load_config(const std::string filename);

    std::vector<double> delay(
      const std::vector<double> &state,
      const std::vector<double> &actuators,
      double latency
    );

    // Solve the model given an initial state and polynomial coefficients.
    // Return the first actuatotions.
    bool solve(const std::vector<double> &state, const std::vector<double> &coeffs);

    std::vector<double> ptsx;
    std::vector<double> ptsy;
    double steering_value;
    double throttle_value;
};

#endif /* MPC_H */
