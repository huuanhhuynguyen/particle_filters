#include "particle_filter.h"

#include <math.h>
#include <algorithm>
#include <iostream>
#include <iterator>
#include <numeric>
#include <random>
#include <string>
#include <vector>

#include "helper_functions.h"

using std::string;
using std::vector;


void ParticleFilter::init(double x, double y, double theta, double std[])
{
  num_particles = 100;

  auto std_x = std[0];
  auto std_y = std[1];
  auto std_theta = std[2];

  // Create gaussian distributions. Draw a sample by calling dist_x(gen).
  std::normal_distribution<double> distr_x{x, std_x};
  std::normal_distribution<double> distr_y{y, std_y};
  std::normal_distribution<double> distr_theta{theta, std_theta};
  std::default_random_engine gen;

  particles.clear();
  for (int i = 0; i < num_particles; ++i) {
    Particle p;
    p.x = distr_x(gen);
    p.y = distr_y(gen);
    p.theta = distr_theta(gen);
    p.weight = 1.0;

    particles.push_back(p);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[],
                                double velocity, double yaw_rate)
{
  auto std_x = std_pos[0];
  auto std_y = std_pos[1];
  auto std_theta = std_pos[2];

  // Create gaussian distributions. Draw a sample by calling noise_x(gen).
  std::normal_distribution<double> noise_x{0, std_x};
  std::normal_distribution<double> noise_y{0, std_y};
  std::normal_distribution<double> noise_theta{0, std_theta};
  std::default_random_engine gen;

  // Make prediction and add noise to each particle
  for (int i = 0; i < num_particles; ++i)
  {
    Particle& p = particles[i];

    double theta_new = p.theta + yaw_rate * delta_t;
    if (yaw_rate < 1e-3) {
      p.x += velocity * cos(p.theta) * delta_t + noise_x(gen);
      p.y += velocity * sin(p.theta) * delta_t + noise_y(gen);
    } else {
      p.x += velocity / yaw_rate * (sin(theta_new) - sin(p.theta));
      p.y += velocity / yaw_rate * (cos(p.theta) - cos(theta_new));
    }
    p.theta += yaw_rate * delta_t;

    // add noise
    p.x += noise_x(gen);
    p.y += noise_y(gen);
    p.theta += noise_theta(gen);

    // init particle weight
    p.weight = 1.0;
  }
}

void ParticleFilter::dataAssociation(vector<LandmarkObs> predicted, 
                                     vector<LandmarkObs>& observations)
{
   for (auto& obs : observations) {
     auto cmp_distance = [obs](const LandmarkObs& a, const LandmarkObs& b) {
       double dist_a = dist(obs.x, obs.y, a.x, a.y);
       double dist_b = dist(obs.x, obs.y, b.x, b.y);
       return dist_a < dist_b;
     };
     auto pClosest = std::min_element(predicted.begin(), predicted.end(), cmp_distance);
     obs.id = pClosest->id;
   }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
                                   const vector<LandmarkObs> &observations,
                                   const Map &map_landmarks)
{
  auto std_x = std_landmark[0];
  auto std_y = std_landmark[1];

  for (auto& particle : particles) {
    double p_x = particle.x;
    double p_y = particle.y;
    double p_theta = particle.theta;

    // Get all landmarks within the sensor range
    std::vector<LandmarkObs> landmarks_in_range;
    for (const auto& map_lm : map_landmarks.landmark_list) {
      double d = dist(map_lm.x_f, map_lm.y_f, p_x, p_y);
      if (d < sensor_range) {
        LandmarkObs lm;
        lm.id = map_lm.id_i;
        lm.x = map_lm.x_f;
        lm.y = map_lm.y_f;
        landmarks_in_range.push_back(lm);
      }
    }

    // Transform the observations into the global (map) coordinate frame
    std::vector<LandmarkObs> observations_global;
    for (const auto& obs: observations) {
      LandmarkObs obs_global;
      obs_global.x = obs.x * cos(p_theta) - obs.y * sin(p_theta) + p_x;
      obs_global.y = obs.x * sin(p_theta) + obs.y * cos(p_theta) + p_y;
      observations_global.push_back(obs_global);
    }

    // Associate the observations to the landmarks
    dataAssociation(landmarks_in_range, observations_global);

    // Calculate particle weight
    double weight = 1.0;
    for (const auto& obs : observations_global) {
      // find the associated landmark
      for (const auto& lm : landmarks_in_range) {
        if (lm.id == obs.id) {
          // Take the landmark as the mean of the Gaussian distribution
          double u_x = lm.x;
          double u_y = lm.y;
          // Calculate probability p(z | x)
          // Formula https://www.statisticshowto.com/bivariate-normal-distribution/
          // where x and y are assumed independent, i.e. their correlation = 0
          double z = (obs.x - u_x) * (obs.x - u_x) / std_x / std_x
                     + (obs.y - u_y) * (obs.y - u_y) / std_y / std_y;
          double prob = 1 / (2 * M_PI * std_x * std_y) * exp(- z / 2 );

          // weight = p(z1, z2, ... | x) = p(z1 | x) * p(z2 | x) * . . .
          weight *= prob;
          break;
        }
      }
    }
    particle.weight = weight;
  }

  // Normalize so that weights of all particles sum up to 1
  double norm = std::accumulate(particles.begin(), particles.end(), 0.0,
      [](double sum, const Particle& particle){ return sum + particle.weight; });
  for (auto& particle : particles) {
    particle.weight /= (norm + 1e-9);
  }
}

void ParticleFilter::resample()
{
  // Get a vector of particle weights
  std::vector<double> particle_weights;
  std::transform(particles.begin(), particles.end(), std::back_inserter(particle_weights),
      [](const Particle& particle){ return particle.weight; });

  // Generate a distribution of weights
  std::discrete_distribution<int> weight_distr(particle_weights.begin(), particle_weights.end());
  std::default_random_engine gen;

  // Resample particles based on the distribution
  std::vector<Particle> resampled_particles;
  for (int i = 0; i < num_particles; ++i) {
    int k = weight_distr(gen);
    resampled_particles.push_back(particles[k]);
  }

  particles = resampled_particles;
}

void ParticleFilter::SetAssociations(Particle& particle, 
                                     const vector<int>& associations, 
                                     const vector<double>& sense_x, 
                                     const vector<double>& sense_y)
{
  // particle: the particle to which assign each listed association, 
  //   and association's (x,y) world coordinates mapping
  // associations: The landmark id that goes along with each listed association
  // sense_x: the associations x mapping already converted to world coordinates
  // sense_y: the associations y mapping already converted to world coordinates
  particle.associations= associations;
  particle.sense_x = sense_x;
  particle.sense_y = sense_y;
}

string ParticleFilter::getAssociations(Particle best)
{
  vector<int> v = best.associations;
  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<int>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}

string ParticleFilter::getSenseCoord(Particle best, string coord)
{
  vector<double> v;

  if (coord == "X") {
    v = best.sense_x;
  } else {
    v = best.sense_y;
  }

  std::stringstream ss;
  copy(v.begin(), v.end(), std::ostream_iterator<float>(ss, " "));
  string s = ss.str();
  s = s.substr(0, s.length()-1);  // get rid of the trailing space
  return s;
}