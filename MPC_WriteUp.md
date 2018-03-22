# Model Predictive Control (MPC) Project


## The model

### Input from simulator
The simulator inputs the following parameters

    double px = j[1]["x"];
    double py = j[1]["y"];
    double psi = j[1]["psi"];
    double v = j[1]["speed"];
    // initial steering / throttle

The actuator values are as well derived from the simulator by
	
    double steer_value = j[1]["steering_angle"];
    double throttle_value = j[1]["throttle"];

### Polynomial fitting (to waypoints)
First the input from the simulator is transformed into car coordinates (from map coordinates).
These transformed coordinates are processed with the polyfit function.

    auto coeffs = polyfit(rdpts_x_eigen, rdpts_y_eigen, 3);
The function return coefficients of a 3rd order polynomial function.
The cte is derived by evaluation the polynom returned by polyfit at 0.
The epsi value is derived by getting the negative atan of the first order coefficient of the 3rd order polynomial (as it was taught in the lessons).

    double cte = polyeval(coeffs, 0);
    double epsi = -atan(coeffs[1]);


### Initial state
The initial state is set by:

    Eigen::VectorXd state(6);
    state << 0, 0, 0, v, cte, epsi;

### cost function

To calculate the cost caused by driving aside of the trajectory the following equations are used:

    // part based on reference state
    for (unsigned int x = 0; x < N; x++){
      fg[0] += w_cte * CppAD::pow(vars[cte_start + x], 2);
      fg[0] += w_epsi * CppAD::pow(vars[epsi_start + x], 2);
      fg[0] += w_v * CppAD::pow(vars[v_start + x] - v_ref, 2);
    }

These equation sum of the cost caused by errors from the reference state or the reference trajectory e.g. by driving not in the middle of the lane or having a divergent heading angle than expected.

    // Minimize the use of n_actuators
    for (unsigned int x = 0; x < N - 1; x++){
      fg[0] += w_delta * CppAD::pow(vars[delta_start + x], 2);
      fg[0] += w_a * CppAD::pow(vars[a_start + x], 2);
    }

These cost functions handle the actuators. These are necessary to not change actuators state too abrupt.

    // Minimize the value gap between sequential actuatotions
    for (unsigned int x = 0; x < N - 2; x++){
      fg[0] += d_w_delta * CppAD::pow(vars[delta_start + x + 1] - vars[delta_start + x], 2);
      fg[0] += d_w_a * CppAD::pow(vars[a_start + x + 1] - vars[a_start + x], 2);
    }
The last block of cost functions take into account smooth changes between subsequent states. Considering these will lead to a smoother driving behaviour in general e.g. by cornering.

### cost function weights

As shown in the cost functions above, each cost function is weighted by a specific parameter. The weights rate the different parts according to their importance for a stable driving behaviour.

    // MPC weights
    double w_cte = 4000;
    double w_epsi = 4000;
    double w_v = 1;
    double w_delta = 5;
    double w_a = 5;
    double d_w_delta = 500;
    double d_w_a = 10;
It can be seen that the weights of the error values cte and epsi are quite high compared to the other ones. This consideres their importance.

### Update equations
The state of the MPC is updated by the following questions of the vehicle model used in the lessons.

      AD<double> f0 = coeffs[3] * CppAD::pow(x0,3) + coeffs[2] * CppAD::pow(x0,2) + coeffs[1] * x0 + coeffs[0];
      AD<double> psides0 = CppAD::atan(3 * coeffs[3] * CppAD::pow(x0,2) + 2 * coeffs[2] * x0 + coeffs[1]);
      // Model equations
      fg[2 + x_start + i] = x1 - (x0 + v0 * CppAD::cos(psi0) * dt);
      fg[2 + y_start + i] = y1 - (y0 + v0 * CppAD::sin(psi0) * dt);
      fg[2 + psi_start + i] = psi1 - (psi0 - (v0/Lf) * delta0  * dt);
      fg[2 + v_start + i] = v1 - (v0 + a0 * dt);
      fg[2 + cte_start + i] = cte1 - ((f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
      fg[2 + epsi_start + i] = epsi1 - ((psi0 - psides0) + (v0/Lf) * delta0 * dt);



### Prediction horizon

The right values for the prediction horizon 

    N = number of time steps
    dt = elapsed time between time steps
    
were determined by trial and error. I started with a large prediction horizon with 

    N = 20 and dt = 0.05
but these values were way too high. The driving behaviour was very unstable.
Next I significantly reduced the number of time steps to

    N = 5.
It was shown that this value was quite too slow, the fitting if a polynom failed and led to unstable driving behaviour as well.
At the end I came up with the following values that showed a good performance:

    N = 10; dt = 0.1;


### MPC with latency

To implement the latency of 100ms to the model I chose the following approach. I set the initial state of the px and py coordinates derived by the simulator to a predicted state in 100ms. Therefore I used the equations of the Vehicle Model from the lessons:

    xt+1 = xt + v * cos(psi) * dt
    yt+1 = yt + V * sin(psi) * dt
	psit+1 = psit + v / Lf * delta * dt
	v = v + a * dt

implemented in the project code it looks like this:

    px += v * cos(psi) * latency;
    py += v * sin(psi) * latency;
    psi -= v * steer_value / Lf * latency;
    v += throttle_value * latency;


## Simulator performance

The MPC showed a satisfying performance in the simulator's autonomous mode. AT the beginning the MPC oscialltes a little bit due to the augmented latency and the data basis to fit the polynom.
I recorded two videos, 


- one with a reference speed of v_ref = 80
- and one with a reference speed of v_ref = 45

you can choose which one suits you more. :-)

Video with v_ref = 45:
[https://youtu.be/1h1x8WfjaV4](https://youtu.be/1h1x8WfjaV4)

Video with v_ref = 80:
[https://youtu.be/TvhvohVnV-Q](https://youtu.be/TvhvohVnV-Q)