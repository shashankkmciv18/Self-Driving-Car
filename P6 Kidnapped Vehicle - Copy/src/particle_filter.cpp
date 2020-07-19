/**
 * particle_filter.cpp
 *
 * Created on: Dec 12, 2016
 * Author: Tiffany Huang
 */

#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>
#include <limits> //std::numeric_limits

#include "helper_functions.h"

using std::string;
using std::vector;

void ParticleFilter::init(double x, double y, double theta, double std[])
{
  /**
   * TODO: Set the number of particles. Initialize all particles to 
   *   first position (based on estimates of x, y, theta and their uncertainties
   *   from GPS) and all weights to 1. 
   * TODO: Add random Gaussian noise to each particle.
   * NOTE: Consult particle_filter.h for more information about this method 
   *   (and others in this file).
   */

  // TODO: Set the number of particles
  num_particles = 100;

  // Random Engine
  std::default_random_engine gen;

  // Normal distributions of x, y, theta
  std::normal_distribution<double> dist_x(0, std[0]);
  std::normal_distribution<double> dist_y(0, std[1]);
  std::normal_distribution<double> dist_theta(0, std[2]);

  // Particle initialization
  Particle particle;

  for (int i = 0; i < num_particles; ++i)
  {
    particle.id = i;
    particle.x = x;
    particle.y = y;
    particle.theta = theta;
    particle.weight = 1.0;

    // Add random Gaussian noise
    particle.x += dist_x(gen);
    particle.y += dist_y(gen);
    particle.theta += dist_theta(gen);

    // Add to list of particles
    particles.push_back(particle);
  }

  // Initialize
  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{

  /**
     * TODO: Add measurements to each particle and add random Gaussian noise.
     * NOTE: When adding noise you may find std::normal_distribution 
     *   and std::default_random_engine useful.
     *  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
     *  http://www.cplusplus.com/reference/random/default_random_engine/
     */

  // Random Engine
  std::default_random_engine gen;

  // Normal distributions of x, y, theta
  std::normal_distribution<double> norm_dist_x(0, std_pos[0]);
  std::normal_distribution<double> norm_dist_y(0, std_pos[1]);
  std::normal_distribution<double> norm_dist_theta(0, std_pos[2]);

  double theta_0; // Initial angle used for each particle

  // New state calculation
  // Update x, y and the yaw angle when the yaw rate is not equal to zero
  for (int i = 0; i < num_particles; ++i)
  {
    theta_0 = particles[i].theta;

    if (fabs(yaw_rate) < 0.00001)
    {
      particles[i].x += velocity * delta_t * cos(theta_0);
      particles[i].y += velocity * delta_t * sin(theta_0);
    }
    else
    {
      particles[i].x += (velocity / yaw_rate) * (sin(theta_0 + (yaw_rate * delta_t)) - sin(theta_0));
      particles[i].y += (velocity / yaw_rate) * (cos(theta_0) - cos(theta_0 + (yaw_rate * delta_t)));
      particles[i].theta += yaw_rate * delta_t;
    }

    // Add random Gaussian noise
    particles[i].x += norm_dist_x(gen);
    particles[i].y += norm_dist_y(gen);
    particles[i].theta += norm_dist_theta(gen);
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted,
                                     vector<LandmarkObs> &observations)
{
  /**
   * TODO: Find the predicted measurement that is closest to each 
   *   observed measurement and assign the observed measurement to this 
   *   particular landmark.
   * NOTE: this method will NOT be called by the grading code. But you will 
   *   probably find it useful to implement this method and use it as a helper 
   *   during the updateWeights phase.
   */

  double min_distance;     // Minimum Eucledian distance between observations and predictions
  double current_distance; // Eucledian distance between current observations and predictions
  int map_id;              // Map ID

  // Observations
  for (unsigned int i = 0; i < observations.size(); ++i)
  {
    // Reset minimum distance as impossibly large.
    min_distance = std::numeric_limits<double>::max();

    // Reset map ID as impossible value
    map_id = -1;

    // Predictions
    for (unsigned int j = 0; j < predicted.size(); ++j)
    {
      // See dist() function in helper_functions.h
      current_distance = dist(observations[i].x, observations[i].y, predicted[j].x, predicted[j].y);

      if (current_distance < min_distance)
      {
        min_distance = current_distance;
        map_id = predicted[j].id;
      }
    }

    // Update observation ID
    observations[i].id = map_id;
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[],
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  /**
   * TODO: Update the weights of each particle using a mult-variate Gaussian 
   *   distribution. You can read more about this distribution here: 
   *   https://en.wikipedia.org/wiki/Multivariate_normal_distribution
   * NOTE: The observations are given in the VEHICLE'S coordinate system. 
   *   Your particles are located according to the MAP'S coordinate system. 
   *   You will need to transform between the two systems. Keep in mind that
   *   this transformation requires both rotation AND translation (but no scaling).
   *   The following is a good resource for the theory:
   *   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
   *   and the following is a good resource for the actual equation to implement
   *   (look at equation 3.33) http://planning.cs.uiuc.edu/node99.html
   */

  double dx;           // Change in x-position
  double dy;           // Change in y-position
  double dx2_plus_dy2; // Square of 2D vector length

  // Square of sensor range
  double sensor_range_squared = sensor_range * sensor_range;

  double x_m; // Map x-coordinate
  double y_m; // Map y-coordinate

  double obs_x;  // Observed landmark x-coordinate
  double obs_y;  // Observed landmark y-coordinate
  double pred_x; // Predicted landmark x-coordinate
  double pred_y; // Predicted landmark y-coordinate

  // Multivariate-Gaussian normalization term
  double gauss_norm = 1 / (2 * M_PI * std_landmark[0] * std_landmark[1]);

  double exponent;     // Multivariate-Gaussian exponent term
  double weight_final; // Multivariate-Gaussian weight

  // Update for each particle
  for (int i = 0; i < num_particles; ++i)
  {
    // Vector of predicted landmark coordinates
    vector<LandmarkObs> landmarks_predicted;

    // Run through each landmark on map
    for (unsigned int j = 0; j < map_landmarks.landmark_list.size(); ++j)
    {
      // Get changes of x and y. Calculate square of vector length
      dx = particles[i].x - map_landmarks.landmark_list[j].x_f;
      dy = particles[i].y - map_landmarks.landmark_list[j].y_f;
      dx2_plus_dy2 = (dx * dx) + (dy + dy);

      // Add to predicted landmarks if in range
      if (dx2_plus_dy2 <= sensor_range_squared)
      {
        landmarks_predicted.push_back(LandmarkObs{map_landmarks.landmark_list[j].id_i,
                                                  map_landmarks.landmark_list[j].x_f,
                                                  map_landmarks.landmark_list[j].y_f});
      }
    }

    // Vector of observed landmark coordinates - homogenous transform to map coordinates
    vector<LandmarkObs> landmarks_observed;

    // Transform
    for (unsigned int j = 0; j < observations.size(); ++j)
    {
      x_m = particles[i].x + (cos(particles[i].theta) * observations[j].x) - (sin(particles[i].theta) * observations[j].y);
      y_m = particles[i].y + (sin(particles[i].theta) * observations[j].x) + (cos(particles[i].theta) * observations[j].y);
      landmarks_observed.push_back(LandmarkObs{observations[j].id, x_m, y_m});
    }

    // Find which observations correspond to which landmarks
    dataAssociation(landmarks_predicted, landmarks_observed);

    // Re-initialize particle weight
    particles[i].weight = 1.0;

    // Look for matching predicted and observed landmarks
    for (unsigned int j = 0; j < landmarks_observed.size(); ++j)
    {
      obs_x = landmarks_observed[j].x;
      obs_y = landmarks_observed[j].y;

      for (unsigned int k = 0; k < landmarks_predicted.size(); ++k)
      {
        if (landmarks_predicted[k].id == landmarks_observed[j].id)
        {
          pred_x = landmarks_predicted[k].x;
          pred_y = landmarks_predicted[k].y;
        }
      }

      // Calculate weight
      exponent = -1.0 * (pow(obs_x - pred_x, 2) / (2.0 * pow(std_landmark[0], 2)) +
                         pow(obs_y - pred_y, 2) / (2.0 * pow(std_landmark[1], 2)));
      weight_final = gauss_norm * exp(exponent);

      particles[i].weight *= weight_final;
    }
  }
}

void ParticleFilter::resample()
{
  /**
   * TODO: Resample particles with replacement with probability proportional 
   *   to their weight. 
   * NOTE: You may find std::discrete_distribution helpful here.
   *   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
   */

  vector<Particle> resampled_particles; // Vector of resampled particles
  vector<double> current_weights;       // Vector of current particle weights

  std::uniform_int_distribution<int> uint_dist(0, num_particles - 1); // Random integer number distribution of current particle weights
  std::default_random_engine gen;                                     // Random engine
  int particle_index = uint_dist(gen);                                // Random particle index

  // Retrieve current particle weights. Place into vector
  for (int i = 0; i < num_particles; ++i)
  {
    current_weights.push_back(particles[i].weight);
  }

  // Get maximum particle weight
  double weight_max = *max_element(current_weights.begin(), current_weights.end());

  // Random real number distribution: zero to maximum particle weight
  std::uniform_real_distribution<double> ureal_dist(0.0, weight_max);

  double beta = 0.0; // Beta in resampling wheel calculations

  // Resampling wheel
  for (int i = 0; i < num_particles; ++i)
  {
    // Omitting multiplication by weight_max, as shown in lesson
    // beta += ureal_dist(gen) * 2.0 * weight_max;
    beta += ureal_dist(gen) * 2.0;

    while (beta > current_weights[particle_index])
    {
      beta -= current_weights[particle_index];
      particle_index = (particle_index + 1) % num_particles;
    }

    resampled_particles.push_back(particles[particle_index]);
  }

  // Update particles vector
  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle &particle,
                                     const vector<int> &associations,
                                     const vector<double> &sense_x,
                                     const vector<double> &sense_y)
{
  // particle: the particle to which assign each listed association,
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations = associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X")
  {
    v = best.sense_x;
  }
  else
  {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length() - 1); // get rid of the trailing space
  return s;
}