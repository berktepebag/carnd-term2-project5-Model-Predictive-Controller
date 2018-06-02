# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## The model in detail. The state, actuators and update equations:

In the main.cpp:

1. Shift the x,y coordinates from simulator to car coordinates. Then find a 3rd degree polynomial according to x,y points. 

  ``` coeffs = polyfit(ptsx_transform,ptsy_transform,3); ```

2. Find the current state of the car considering latency.

  ```
  double calculate_latency_state(double x,double y, double psi, double v
    , double steer_value, double throttle_value, Eigen::VectorXd coeffs, double latency){
      //Adding latency to current state, so we are looking at the 100 milisecond into the future.
    double dt = latency/1000; //milisecond to second
    x += x + v*cos(psi)*dt;  
    psi -= steer_value/Lf*dt;
    v += throttle_value*dt;  
    double epsi = psi - atan(coeffs[1] + 2 * x * coeffs[2] + 3 * coeffs[3] * pow(x,2));
    double cte = polyeval(coeffs,0) + v*sin(epsi)*dt;
    //
    return v,cte,epsi;}
  ```

### Timestep Length and Elapsed Duration (N & dt) The reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values:

T should be as large as possible, while dt should be as small as possible. In the case of driving a car, T should be a few seconds, at most.(Lesson 19.5)

In main.cpp I have calculated average speed of the car for 1 turn around the track.

### Previous values tried.

| **N**| **dt** | **Average speed** | **Notes** |
| :-------------:|:-------------:|:-------------:|:-------------:|
| 10 | 0.2 | 42.8102 | *Finished without problem* |
| 20 | 0.2 | 45.8678 | *Finished without problem* |
| 30 | 0.2 | | *Crashed at first corner* |
| 20 | 0.15 | 50.7844 |*Went out of road a bit*| 
| 20 | 0.10 | | *Car started oscillation before first corner, crashed* |
| 20 | 0.12 | | *Car started oscillation at first corner, crashed at bridge* |
| 20 | 0.13 | | *Car started oscillation at first corner, crashed at bridge* |
| 20 | 0.14 | | *Car started oscillation at second corner, crashed before third* |
| 10 | 0.15 | 48.685| *Average speed decreased* |
| 25 | 0.15 | |*Crashed before first corner*| 
| 20 | 0.16 | 50.307|*Went out of road a bit*| 
| 20 | 0.17 | 48.967|*Went out of road a bit*| 
| 20 | 0.18 | 48.853|*Went out of road a bit*| 
| 20 | 0.19 | 50.716|*Crashed at first corner*| 
| 20 | 0.20 |  | *Crashed at third corner? Retrying* |
| 20 | 0.20 | 45.259 | *Finished without problem*|
| 20 | 0.20 | 48.038 | *Crashed at third corner!* |
| 15 | 0.20 | 45.151 | *Finished without problem* |
| 19 | 0.20 | 46.955 | *Finished without problem* |
| 19 | 0.15 | 50.915 | *Went out of road a bit* |
| 18 | 0.18 | 47.791 | *Went out of road a bit* |
| 19 | 0.19 | 43.095 | *Crashed before first corner, suddenly?* |
| 20 | 0.20 | 45.037 | *First turn, Finished without problem* |
| 20 | 0.20 | 48.088 | *Second turn, Finished without problem* |

