// Mars lander simulator
// Version 1.11
// Mechanical simulation functions
// Gabor Csanyi and Andrew Gee, August 2019

// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation, to make use of it
// for non-commercial purposes, provided that (a) its original authorship
// is acknowledged and (b) no modified versions of the source code are
// published. Restriction (b) is designed to protect the integrity of the
// exercise for future generations of students. The authors would be happy
// to receive any suggested modifications by private correspondence to
// ahg@eng.cam.ac.uk and gc121@eng.cam.ac.uk.

#include "lander.h"

void autopilot_hover(void)
{
    double F_eq, k_p, k_d;
    F_eq = GRAVITY * MARS_MASS * 200 / pow((500 + MARS_RADIUS), 2);
    k_p = 0.0910;
    k_d = 0.178;
    throttle = F_eq / MAX_THRUST + k_p * (500 - (position.abs() - MARS_RADIUS)) - k_d * velocity * position.norm();
    cout << k_p * (500 - (position.abs() - MARS_RADIUS)) << endl;
}

void autopilot (void)
// Minimise fuel usage = Minimise energy from thrust = Maximise energy loss from air resistance
{
    double delta, P;
    double expected_throttle;

    delta = (GRAVITY * MARS_MASS * mass / position.abs2()) / MAX_THRUST;

    set_attitude_to_velocity();
    switch (autopilot_status)
    {
    case IDLE: // Initial state
        if (position.abs() < MARS_RADIUS + EXOSPHERE && safe_to_deploy_parachute()) {
            parachute_status = DEPLOYED;
            autopilot_status = MAX_SPEED;
        }

        throttle = 0;
        break;
    case MAX_SPEED: // Maintain MAX_PARACHUTE_SPEED until drag on chute = MAX_PARACHUTE_DRAG.
        if (position.abs() < MARS_RADIUS + MAX_DRAG_HEIGHT && speed_or_throttle_after_delay(true) < 0) autopilot_status = FREE_FALL;

        P = -0.001 * (MAX_PARACHUTE_SPEED + speed_or_throttle_after_delay(true));
        // Make more allowances when there is delay as predicted speed gets more inaccurate the further ahead we attempt to predict.
        // Make more allowances when there is lag as it takes longer to get to intended throttle value.
        if (P < -0.005 - pow(ENGINE_LAG, 1.4) * 0.0015 - ENGINE_DELAY * 0.00005) throttle = 0;
        else if (P >= 1 - delta) throttle = 1;
        else throttle = delta + P;
        break;
    case FREE_FALL: // Parachute will do all the job as we enter the atmosphere.
        if (will_land_in_full_thrust()) autopilot_status = FULL_THRUST;

        throttle = 0;
        break;
    case FULL_THRUST: // Decelerate full thrust to MAX_IMPACT_DESCENT_RATE.
        if (speed_or_throttle_after_delay(true) > -MAX_IMPACT_DESCENT_RATE - ENGINE_LAG - ENGINE_DELAY / 10) autopilot_status = LAND;

        throttle = 1;
        break;
    case LAND: // Maintain MAX_IMPACT_DESCENT_RATE.
        P = calc_land_P();
        if (P < -delta) expected_throttle = 0;
        else if (P >= 1 - delta) expected_throttle = 1;
        else expected_throttle = delta + P;

        // To get expected throttle in the next step we must aim a bit further ahead
        // so that lagged throttle will land we want it to.
        // However, throttle cannot be set to more than 1 or less than 0, making it difficult to land under high lag.
        throttle = (expected_throttle - speed_or_throttle_after_delay(false) * k) / (1 - k);
        break;
    default:
        break;
    }
}

// Calculate P during LAND phase.
double calc_land_P(void)
{
    double e;
    // Aim for 0.8 m/s
    // Not 0.9 because speed will easily get over 1 m/s and crash
    // Not 0.7 because: 
    //                  #1. It is slower and consume more fuel
    //                  #2. The lander will not be able to control throttle in time under high lag, 
    //                      causing the lander to climb up which costs huge amount of fuel to fix.
    // For unknown reasons the lander starts to believe that it is always too slow starting from 20s lag.
    // An offset term is added to fix this.
	if (ENGINE_DELAY < 1)
		e = -(MAX_IMPACT_DESCENT_RATE - 0.2 - pow(ENGINE_DELAY, 0.1) / 7 + speed_or_throttle_after_delay(true));
	else
		e = -(MAX_IMPACT_DESCENT_RATE - 0.2 + speed_or_throttle_after_delay(true));
    
    if (abs(e) > 0.2) {
        // If we are far away from target speed, we must get to it quickly - no time to dilly-dally!
        // Under high delay and lag we need to tune this down a bit to prevent overshooting
        if (ENGINE_DELAY < 1)
        {
			if (ENGINE_LAG < 1) return 0.4 * e;
			else return 0.4 * pow(ENGINE_LAG, -0.8) * e;
        }
        else
        {
			if (ENGINE_LAG < 1) return 0.4 * pow(ENGINE_DELAY, -0.3) * e + pow(ENGINE_DELAY, 0.1) / 34;
			else return 0.4 * pow(ENGINE_LAG, -0.8) * e;
        }
    }
    // If we are close to target speed, we maintain current speed with very small adjustments.
    else return 0.001 * e;
}

// Controls transition from FREE_FALL to FULL_THRUST
bool will_land_in_full_thrust(void)
{
    double x, _x, x_, v, m, g, d, a, th;

    x = position.abs();
    _x = previous_position.abs();
    v = velocity * position.norm();
    m = mass;
    th = 0;

    // Predict position after lander comes to a stop with throttle set to 1, and assuming open parachute.
    // Uses Verlet.
    for (double t = 0; v < 0; t = t + delta_t)
    {
        m -= delta_t * (FUEL_RATE_AT_MAX_THRUST * th) * FUEL_DENSITY;
        g = -GRAVITY * MARS_MASS * m / pow(x, 2);
        d = -0.5 * atmospheric_density(vector3d(x, 0, 0)) * (DRAG_COEF_LANDER * AREA_LANDER + DRAG_COEF_CHUTE * AREA_CHUTE) * v * abs(v);

        // + delta_t to predict one step ahead or lander will crash.
        // Without 1.0000000000001 lander crashes at 1.5s delay due to rounding errors.
        if (t <= ENGINE_DELAY * 1.0000000000001 + delta_t) th = 0;
        else th = k * th + (1.0 - k) * 1;
        a = (g + d + MAX_THRUST * th) / m;

        x_ = 2 * x - _x + pow(delta_t, 2) * a;
        v = (x_ - _x) / (2 * delta_t);
        _x = x;
        x = x_;
    }

    // If altitude is less than 0 then transition to MAX_THRUST
    // An offset term for lag and delay is included because prediction gets more inaccurate as it predicts further ahead,
    // and tends to start full thrust earlier than it should.
    if (x < MARS_RADIUS - pow(ENGINE_LAG, 1.9) / 19 - pow(ENGINE_DELAY, 1.7) / 10.3) return true;
    return false;
}

// Predict speed and throttle after delay assuming open parachute
// Uses Verlet.
// Return speed or throttle depending on parameter
double speed_or_throttle_after_delay(bool returnSpeed)
{
    double x, _x, x_, v, m, g, d, th, a;

    x = position.abs();
    _x = previous_position.abs();
    v = velocity * position.norm();
    m = mass;
    th = thrust_wrt_world().abs() / MAX_THRUST;
    // using get functions for throttle_buffer_length, throttle_buffer_pointer, and throttle_buffer avoids unsafe operations with them
    for (int i = 0; i < get_throttle_buffer_length(); i++)
    {
        m -= delta_t * (FUEL_RATE_AT_MAX_THRUST * th) * FUEL_DENSITY;
        g = -GRAVITY * MARS_MASS * m / pow(x, 2);
        d = -0.5 * atmospheric_density(vector3d(x, 0, 0)) * (DRAG_COEF_LANDER * AREA_LANDER + DRAG_COEF_CHUTE * AREA_CHUTE) * v * abs(v);
        th = k * th + (1.0 - k) * get_throttle_buffer()[(get_throttle_buffer_pointer() + i) % get_throttle_buffer_length()];

        a = (g + d + MAX_THRUST * th) / m;

        x_ = 2 * x - _x + pow(delta_t, 2) * a;
        v = (x_ - _x) / (2 * delta_t);
        _x = x;
        x = x_;
    }

    if (returnSpeed) return v;
    else return th;
}

void numerical_dynamics(void)
// This is the function that performs the numerical integration to update the
// lander's pose. The time step is delta_t (global variable).
{
    vector3d new_position, gravity, drag, thrust, acceleration;

    mass = UNLOADED_LANDER_MASS + fuel * FUEL_CAPACITY * FUEL_DENSITY;

    gravity = -GRAVITY * MARS_MASS * mass * position.norm() / position.abs2();

    if (parachute_status == DEPLOYED) {
        drag = -0.5 * atmospheric_density(position) * (DRAG_COEF_LANDER * AREA_LANDER + DRAG_COEF_CHUTE * AREA_CHUTE) * velocity.abs2() * velocity.norm();
    }
    else {
        drag = -0.5 * atmospheric_density(position) * DRAG_COEF_LANDER * AREA_LANDER * velocity.abs2() * velocity.norm();
    }

    thrust = thrust_wrt_world() + 100.0 * cos(0.1 * simulation_time) * position.norm();

    acceleration = (gravity + drag + thrust) / mass;

    if (simulation_time == 0.0) {
        //do an Euler update for the first iteration
        new_position = position + delta_t * velocity; //Euler update, using position and velocity
        velocity = velocity + delta_t * acceleration; //Euler update, using acceleration
    }
    else {
        //do a Verlet update on all subsequent iterations
        new_position = 2 * position - previous_position + pow(delta_t, 2) * acceleration; //Verlet update, using position and previous_position
        velocity = (new_position - previous_position) / (2 * delta_t); //Verlet update, using new_position and position
    }

    previous_position = position;
    position = new_position;

    // Here we can apply an autopilot to adjust the thrust, parachute and attitude
    if (autopilot_enabled) autopilot_hover();

    // Here we can apply 3-axis stabilization to ensure the base is always pointing downwards
    if (stabilized_attitude) attitude_stabilization();

    // Write to file
    if (fout) fout << simulation_time << ' ' << position.abs() - MARS_RADIUS << ' ' << velocity.abs() << endl;
    else cout << "Could not open trajectory file for writing" << endl;
}

void initialize_simulation (void)
// Lander pose initialization - selects one of 10 possible scenarios
{
    // Write the trajectories to file
    fout.close();
    fout.open("trajectories.txt");

    // The parameters to set are:
    // position - in Cartesian planetary coordinate system (m)
    // velocity - in Cartesian planetary coordinate system (m/s)
    // orientation - in lander coordinate system (xyz Euler angles, degrees)
    // delta_t - the simulation time step
    // boolean state variables - parachute_status, stabilized_attitude, autopilot_enabled
    // scenario_description - a descriptive string for the help screen

    scenario_description[0] = "circular orbit";
    scenario_description[1] = "descent from 10km";
    scenario_description[2] = "elliptical orbit, thrust changes orbital plane";
    scenario_description[3] = "polar launch at escape velocity (but drag prevents escape)";
    scenario_description[4] = "elliptical orbit that clips the atmosphere and decays";
    scenario_description[5] = "descent from 200km";
    scenario_description[6] = "areostationary orbit";
    scenario_description[7] = "";
    scenario_description[8] = "";
    scenario_description[9] = "";

    switch (scenario) {

    case 0:
        // a circular equatorial orbit
        position = vector3d(1.2 * MARS_RADIUS, 0.0, 0.0);
        velocity = vector3d(0.0, -3247.087385863725, 0.0);
        orientation = vector3d(0.0, 90.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        break;

    case 1:
        // a descent from rest at 10km altitude
        position = vector3d(0.0, -(MARS_RADIUS + 10000.0), 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = false;
        break;

    case 2:
        // an elliptical polar orbit
        position = vector3d(0.0, 0.0, 1.2 * MARS_RADIUS);
        velocity = vector3d(3500.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        break;

    case 3:
        // polar surface launch at escape velocity (but drag prevents escape)
        position = vector3d(0.0, 0.0, MARS_RADIUS + LANDER_SIZE / 2.0);
        velocity = vector3d(0.0, 0.0, 5027.0);
        orientation = vector3d(0.0, 0.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        break;

    case 4:
        // an elliptical orbit that clips the atmosphere each time round, losing energy
        position = vector3d(0.0, 0.0, MARS_RADIUS + 100000.0);
        velocity = vector3d(4000.0, 0.0, 0.0);
        orientation = vector3d(0.0, 90.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        break;

    case 5:
        // a descent from rest at the edge of the exosphere
        position = vector3d(0.0, -(MARS_RADIUS + EXOSPHERE), 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = false;
        break;

    case 6:
        // a geostationary orbit
        position = vector3d(20417942.77, 0.0, 0.0);
        velocity = vector3d(0.0, 1447.977192, 0.0);
        orientation = vector3d(0.0, 90.0, 0.0);
        delta_t = 0.1;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = false;
        autopilot_enabled = false;
        break;

    case 7:
        // hover at 500m
        position = vector3d(0.0, -(MARS_RADIUS + 710.0), 0.0);
        velocity = vector3d(0.0, 0.0, 0.0);
        orientation = vector3d(0.0, 0.0, 90.0);
        delta_t = 0.01;
        parachute_status = NOT_DEPLOYED;
        stabilized_attitude = true;
        autopilot_enabled = true;
        break;

    case 8:
        break;

    case 9:
        break;

    }
}