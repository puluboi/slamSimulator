#include "ufastslam.hpp"
#include <iostream>
#include <iomanip>
#include <cmath>
#include <algorithm>
#include <numeric>

UFastSLAM::UFastSLAM(int numParticles) 
    : numParticles(numParticles), 
      randomGenerator(std::chrono::steady_clock::now().time_since_epoch().count()),
      normalDist(0.0, 1.0),        // gaussian noise generator: mean=0, std=1.0 for general random sampling
      motionNoiseStd(4),         // reduced motion model noise level: controls particle spread during prediction
                                   // lower values = tighter tracking but less robustness to model errors
                                   // higher values = more particle diversity but potentially more drift
      measurementNoiseStd(0.001),   // extremely small observation model noise for zero lidar noise scenario
                                   // lower values = observations trusted more, particles converge faster
                                   // higher values = observations trusted less, more robust to sensor noise
      resamplingThreshold(0.3),   // fraction of particles triggering resampling (0.4 = 40% of total particles)
                                   // lower values = more frequent resampling, faster convergence, risk of particle depletion
                                   // higher values = less frequent resampling, maintains diversity, slower convergence
      jitterScale(0.8) {          // particle regularization noise for resampling diversity maintenance
                                   // prevents particle degeneracy by adding small noise to copied particles
                                   // lower values = less noise, faster convergence, risk of sample impoverishment
                                   // higher values = more noise, maintains diversity, slower convergence
    
    particles.resize(numParticles);
    
    // initialize each particle with default values
    for (auto& particle : particles) {
        particle.weight = 1.0 / numParticles;
    }
    
    std::cout << "ufastslam initialized with " << numParticles << " particles" << std::endl;
}

void UFastSLAM::initializeParticles(sf::Vector2f initialPosition, float initialDirection) {
    initializeParticles(static_cast<double>(initialPosition.x),
                       static_cast<double>(initialPosition.y),
                       static_cast<double>(initialDirection * M_PI / 180.0));
}

void UFastSLAM::initializeParticles(double x, double y, double theta) {
    // initialize all particles around the starting pose with smaller random variations for better convergence
    double poseNoise = 1.0; // reduced initial particle spread in position (units): controls initial uncertainty
                            // smaller values = particles start closer together, assumes accurate initial position
                            // larger values = wider initial spread, more robust to initial position errors
    
    for (auto& particle : particles) {
        // add small gaussian noise to initial pose for particle diversity
        particle.pose(0) = x + normalDist(randomGenerator) * poseNoise;
        particle.pose(1) = y + normalDist(randomGenerator) * poseNoise;
        particle.pose(2) = theta + normalDist(randomGenerator) * (poseNoise * 0.01); // smaller angular spread
        
        // normalize angle
        while (particle.pose(2) > M_PI) particle.pose(2) -= 2.0 * M_PI;
        while (particle.pose(2) < -M_PI) particle.pose(2) += 2.0 * M_PI;
        
        // initialize velocities to zero
        particle.velocity = Eigen::Vector3d::Zero();
        
        // equal initial weights
        particle.weight = 1.0 / numParticles;
        
        // clear any existing landmarks
        particle.landmarks.clear();
    }
    
    std::cout << "ufastslam particles initialized at pose: [" 
              << x << ", " << y << ", " << theta << "] with spread: " << getParticleSpread() << std::endl;
}

Eigen::Vector3d UFastSLAM::motionModel(const Eigen::Vector3d& pose,
                                      const Eigen::Vector3d& velocity, 
                                      const Eigen::Vector3d& controlInput,
                                      float deltaTime) const {
    // velocity-based motion model matching the ekf-slam implementation exactly
    Eigen::Vector3d newPose;
    
    double x = pose(0);
    double y = pose(1);
    double theta = pose(2);
    double vx = velocity(0);
    double vy = velocity(1);
    double omega = velocity(2);
    
    double dt = deltaTime;
    
    // update position using current velocity (same as EKF-SLAM)
    newPose(0) = x + vx * dt;
    newPose(1) = y + vy * dt;
    newPose(2) = theta + omega * dt;
    
    // normalize angle to [-pi, pi]
    while (newPose(2) > M_PI) newPose(2) -= 2.0 * M_PI;
    while (newPose(2) < -M_PI) newPose(2) += 2.0 * M_PI;
    
    return newPose;
}

Eigen::Vector2d UFastSLAM::observationModel(const Eigen::Vector3d& robotPose, 
                                           const Eigen::Vector2d& landmarkPos) const {
    // calculate range-bearing observation from robot pose to landmark
    double dx = landmarkPos(0) - robotPose(0);
    double dy = landmarkPos(1) - robotPose(1);
    
    double range = std::sqrt(dx * dx + dy * dy);
    double bearing = std::atan2(dy, dx) - robotPose(2);
    
    // normalize bearing to [-pi, pi]
    while (bearing > M_PI) bearing -= 2.0 * M_PI;
    while (bearing < -M_PI) bearing += 2.0 * M_PI;
    
    return Eigen::Vector2d(range, bearing);
}

std::vector<Eigen::VectorXd> UFastSLAM::generateSigmaPoints(const Eigen::VectorXd& mean, 
                                                           const Eigen::MatrixXd& covariance) const {
    int n = mean.size();
    std::vector<Eigen::VectorXd> sigmaPoints;
    
    // calculate unscented transform parameters
    double lambda = utParams.alpha * utParams.alpha * (n + utParams.kappa) - n;
    
    // calculate matrix square root using cholesky decomposition
    Eigen::MatrixXd sqrtCov = ((n + lambda) * covariance).llt().matrixL();
    
    // first sigma point is the mean
    sigmaPoints.push_back(mean);
    
    // generate 2n additional sigma points
    for (int i = 0; i < n; i++) {
        sigmaPoints.push_back(mean + sqrtCov.col(i));
        sigmaPoints.push_back(mean - sqrtCov.col(i));
    }
    
    return sigmaPoints;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> UFastSLAM::unscentedTransform(
    const std::vector<Eigen::VectorXd>& sigmaPoints,
    const std::vector<double>& weights) const {
    
    int n = sigmaPoints[0].size();
    
    // calculate weighted mean
    Eigen::VectorXd mean = Eigen::VectorXd::Zero(n);
    for (size_t i = 0; i < sigmaPoints.size(); i++) {
        mean += weights[i] * sigmaPoints[i];
    }
    
    // calculate weighted covariance
    Eigen::MatrixXd covariance = Eigen::MatrixXd::Zero(n, n);
    for (size_t i = 0; i < sigmaPoints.size(); i++) {
        Eigen::VectorXd diff = sigmaPoints[i] - mean;
        covariance += weights[i] * diff * diff.transpose();
    }
    
    return std::make_pair(mean, covariance);
}

void UFastSLAM::predictParticles(const Eigen::Vector3d& controlInput, float deltaTime) {
    // debug output for prediction step
    static int predictDebugCounter = 0;
    bool debugOutput = (++predictDebugCounter % 1 == 0);
    
    if (debugOutput) {
        std::cout << "ufastslam predict debug - control input: [" 
                  << controlInput.transpose() << "], dt=" << deltaTime << std::endl;
        std::cout << "  motion noise std: " << motionNoiseStd << std::endl;
    }
    
    Eigen::Vector3d meanPoseBefore = getRobotPose();
    
    // predict each particle's pose using motion model with noise
    for (auto& particle : particles) {
        // store previous pose for debugging
        Eigen::Vector3d previousPose = particle.pose;
        
        // update velocity from control input exactly like EKF-SLAM
        // control input contains [ax, ay, omega] where ax,ay are accelerations and omega is angular velocity
        particle.velocity(0) += controlInput(0) * deltaTime; // vx += ax * dt
        particle.velocity(1) += controlInput(1) * deltaTime; // vy += ay * dt  
        particle.velocity(2) = controlInput(2);              // omega = control omega (not integrated)
        
        // predict new pose using motion model
        Eigen::Vector3d predictedPose = motionModel(particle.pose, particle.velocity, 
                                                  controlInput, deltaTime);
        
        // add motion noise scaled appropriately for the timestep
        double posNoiseScale = motionNoiseStd * std::sqrt(deltaTime); // scale noise with sqrt(dt) for brownian motion
        double angNoiseScale = motionNoiseStd * std::sqrt(deltaTime) * 0.01; // much smaller angular noise
        
        double noiseX = normalDist(randomGenerator) * posNoiseScale;   // position noise in x
        double noiseY = normalDist(randomGenerator) * posNoiseScale;   // position noise in y  
        double noiseTheta = normalDist(randomGenerator) * angNoiseScale; // orientation noise (much smaller)
        
        predictedPose(0) += noiseX;
        predictedPose(1) += noiseY;
        predictedPose(2) += noiseTheta;
        
        // normalize angle
        while (predictedPose(2) > M_PI) predictedPose(2) -= 2.0 * M_PI;
        while (predictedPose(2) < -M_PI) predictedPose(2) += 2.0 * M_PI;
        
        particle.pose = predictedPose;
        
        // debug first few particles prediction in detail
        if (debugOutput && (&particle - &particles[0]) < 3) {
            int idx = &particle - &particles[0];
            std::cout << "  particle " << idx << " prediction details:" << std::endl;
            std::cout << "    previous pose: [" << previousPose.transpose() << "]" << std::endl;
            std::cout << "    velocity: [" << particle.velocity.transpose() << "]" << std::endl;
            std::cout << "    predicted pose: [" << predictedPose.transpose() << "]" << std::endl;
            std::cout << "    added noise: [" << noiseX << ", " << noiseY << ", " << noiseTheta << "]" << std::endl;
        }
    }
    
    if (debugOutput) {
        Eigen::Vector3d meanPoseAfter = getRobotPose();
        std::cout << "  mean pose change: [" << (meanPoseAfter - meanPoseBefore).transpose() << "]" << std::endl;
        std::cout << "  particle spread after prediction: " << getParticleSpread() << std::endl;
    }
}

void UFastSLAM::updateParticleWeights(const std::vector<std::pair<Eigen::VectorXd, int>>& observations) {
    // implement proper ufastslam algorithm with integrated ukf landmark updates
    static int weightDebugCounter = 0;
    bool debugOutput = (++weightDebugCounter % 50 == 0);
    
    if (debugOutput) {
        std::cout << "ufastslam weight update debug - processing " << observations.size() 
                  << " observations using proper ukf-integrated approach" << std::endl;
    }
    
    double totalLikelihood = 0.0;
    std::vector<double> particleLikelihoods;
    particleLikelihoods.reserve(particles.size());
    
    // update importance weights based on observation likelihood
    for (size_t p = 0; p < particles.size(); p++) {
        auto& particle = particles[p];
        double likelihood = 1.0;
        
        for (const auto& obs : observations) {
            const Eigen::VectorXd& observation = obs.first;
            int landmarkIndex = obs.second;
            
            if (debugOutput && p == 0) {
                std::cout << "  processing landmark " << landmarkIndex 
                          << " with observation: [" << observation.transpose() << "]" << std::endl;
            }
            
            // ensure landmark exists in particle's map 
            if (landmarkIndex >= static_cast<int>(particle.landmarks.size())) {
                // initialize new landmark with observed position
                Eigen::Vector2d landmarkPos;
                double range = observation(0);
                double bearing = observation(1);
                
                // convert range-bearing to cartesian coordinates
                landmarkPos(0) = particle.pose(0) + range * std::cos(bearing + particle.pose(2));
                landmarkPos(1) = particle.pose(1) + range * std::sin(bearing + particle.pose(2));
                
                // add new landmark to particle
                while (static_cast<int>(particle.landmarks.size()) <= landmarkIndex) {
                    particle.landmarks.emplace_back(landmarkPos);
                }
                
                if (debugOutput && p == 0) {
                    std::cout << "    added new landmark " << landmarkIndex << " at: [" << landmarkPos.transpose() 
                              << "] from obs: [" << range << ", " << bearing << "] robot: [" 
                              << particle.pose(0) << ", " << particle.pose(1) << ", " << particle.pose(2) << "]" << std::endl;
                }
            }
            
            // proper ufastslam approach: use ukf for landmark update and weight calculation
            // this follows the theoretical framework from the unscented particle filter paper
            LandmarkEstimate& landmark = particle.landmarks[landmarkIndex];
            
            // perform ukf landmark update to get innovation covariance
            updateLandmarkEstimate(particle, landmarkIndex, observation.head<2>());
            
            // calculate expected observation using ukf prediction
            std::vector<Eigen::VectorXd> sigmaPoints = generateSigmaPoints(landmark.mean, landmark.covariance);
            std::vector<Eigen::VectorXd> transformedPoints;
            
            for (const auto& point : sigmaPoints) {
                Eigen::Vector2d obsPoint = observationModel(particle.pose, point);
                transformedPoints.push_back(obsPoint);
            }
            
            // calculate ukf weights for unscented transform
            int n = 2; // landmark dimension
            double lambda = utParams.alpha * utParams.alpha * (n + utParams.kappa) - n;
            std::vector<double> weights(2 * n + 1);
            weights[0] = lambda / (n + lambda);
            for (int i = 1; i < 2 * n + 1; i++) {
                weights[i] = 0.5 / (n + lambda);
            }
            
            // calculate predicted observation and innovation covariance using ukf
            auto [predObs, innovCov] = unscentedTransform(transformedPoints, weights);
            
            // add measurement noise according to theoretical framework
            Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
            R(0,0) = measurementNoiseStd * measurementNoiseStd;  // proper measurement noise variance
            R(1,1) = measurementNoiseStd * measurementNoiseStd;  // consistent with theoretical model
            innovCov += R;
            
            // calculate innovation for weight update
            Eigen::Vector2d innovation = observation.head<2>() - predObs;
            
            // normalize bearing innovation to [-pi, pi]
            while (innovation(1) > M_PI) innovation(1) -= 2.0 * M_PI;
            while (innovation(1) < -M_PI) innovation(1) += 2.0 * M_PI;
            
            // calculate particle importance weight using proper ukf innovation covariance
            double det = innovCov.determinant();
            if (det > 1e-12) {
                double mahalanobisDistSq = innovation.transpose() * innovCov.inverse() * innovation;
                double exponent = -0.5 * mahalanobisDistSq;
                
                // theoretical multivariate gaussian likelihood without artificial clamping
                double normalizationFactor = 1.0 / (2.0 * M_PI * std::sqrt(det));
                double obsLikelihood = normalizationFactor * std::exp(exponent);
                likelihood *= obsLikelihood;
                
                if (debugOutput && p == 0) {
                    std::cout << "    simplified update completed" << std::endl;
                    std::cout << "    innovation: [" << innovation.transpose() << "]" << std::endl;
                    std::cout << "    observation likelihood: " << obsLikelihood << std::endl;
                    std::cout << "    mahalanobis distance: " << std::sqrt(mahalanobisDistSq) << std::endl;
                }
            } else {
                if (debugOutput && p == 0) {
                    std::cout << "    warning: singular observation covariance matrix" << std::endl;
                }
            }
        }
        
        // store likelihood for normalization (don't update weight yet)
        particleLikelihoods.push_back(likelihood);
        totalLikelihood += likelihood;
        
        if (debugOutput && p < 5) {
            std::cout << "  particle " << p << " likelihood: " << likelihood 
                      << ", current weight: " << particle.weight << std::endl;
        }
    }
    
    // normalize weights using likelihoods directly to avoid precision loss
    if (totalLikelihood > 1e-15) {
        for (size_t i = 0; i < particles.size(); i++) {
            // set weight directly from normalized likelihood to avoid compounding precision errors
            particles[i].weight = particleLikelihoods[i] / totalLikelihood;
        }
        
        if (debugOutput) {
            std::cout << "  total likelihood: " << totalLikelihood << std::endl;
            std::cout << "  weight normalization completed" << std::endl;
        }
    } else {
        // if all weights are zero, reset to uniform distribution
        for (auto& particle : particles) {
            particle.weight = 1.0 / numParticles;
        }
        
        if (debugOutput) {
            std::cout << "  warning: all weights zero, reset to uniform distribution" << std::endl;
        }
    }
    
    if (debugOutput) {
        // calculate weight statistics with better precision handling
        double maxWeight = 0.0, minWeight = 1.0, meanWeight = 0.0;
        int numNonZeroWeights = 0;
        
        for (const auto& particle : particles) {
            if (particle.weight > 1e-10) {  // consider weights above this threshold as non-zero
                maxWeight = std::max(maxWeight, particle.weight);
                minWeight = std::min(minWeight, particle.weight);
                numNonZeroWeights++;
            }
            meanWeight += particle.weight;
        }
        meanWeight /= particles.size();
        
        std::cout << "  weight statistics - max: " << std::scientific << maxWeight 
                  << ", min: " << minWeight << ", mean: " << meanWeight << std::fixed << std::endl;
        std::cout << "  non-zero weights: " << numNonZeroWeights << "/" << particles.size() << std::endl;
        std::cout << "  effective sample size: " << calculateEffectiveSampleSize() << std::endl;
    }
}

double UFastSLAM::calculateEffectiveSampleSize() const {
    // calculate effective sample size for resampling decision
    double sumOfSquares = 0.0;
    for (const auto& particle : particles) {
        sumOfSquares += particle.weight * particle.weight;
    }
    
    // avoid division by zero when all weights are equal/uniform
    if (sumOfSquares < 1e-15) {
        return static_cast<double>(numParticles); // uniform distribution has ESS = N
    }
    
    return 1.0 / sumOfSquares;
}

void UFastSLAM::resampleParticles() {
    // debug output for resampling
    static int resampleDebugCounter = 0;
    bool debugOutput = (++resampleDebugCounter % 20 == 0);
    
    if (debugOutput) {
        double ess = calculateEffectiveSampleSize();
        std::cout << "ufastslam resampling debug - ess: " << ess 
                  << ", threshold: " << (resamplingThreshold * numParticles) << std::endl;
        
        // show weight distribution before resampling
        std::vector<double> weights;
        for (const auto& particle : particles) {
            weights.push_back(particle.weight);
        }
        std::sort(weights.begin(), weights.end(), std::greater<double>());
        
        std::cout << "  top 5 weights before resampling: ";
        for (int i = 0; i < std::min(5, static_cast<int>(weights.size())); i++) {
            std::cout << weights[i] << " ";
        }
        std::cout << std::endl;
    }
    
    // systematic resampling to avoid particle degeneracy
    std::vector<Particle> newParticles;
    newParticles.reserve(numParticles);
    
    // calculate cumulative distribution function
    std::vector<double> cdf(numParticles);
    cdf[0] = particles[0].weight;
    for (int i = 1; i < numParticles; i++) {
        cdf[i] = cdf[i-1] + particles[i].weight;
    }
    
    if (debugOutput) {
        std::cout << "  cdf range: [0, " << cdf.back() << "]" << std::endl;
    }
    
    // systematic sampling
    double step = 1.0 / numParticles;
    double start = (static_cast<double>(rand()) / RAND_MAX) * step;
    
    std::vector<int> resampleCounts(numParticles, 0);
    
    int j = 0;
    for (int i = 0; i < numParticles; i++) {
        double target = start + i * step;
        
        while (j < numParticles - 1 && cdf[j] < target) {
            j++;
        }
        
        // copy selected particle
        newParticles.push_back(particles[j]);
        newParticles.back().weight = 1.0 / numParticles; // reset weight
        
        // add small jitter to copied particles to maintain diversity (particle regularization)
        // this prevents particle degeneracy and maintains filter effectiveness
        newParticles.back().pose(0) += normalDist(randomGenerator) * jitterScale;
        newParticles.back().pose(1) += normalDist(randomGenerator) * jitterScale;
        newParticles.back().pose(2) += normalDist(randomGenerator) * jitterScale * 0.01; // smaller angular jitter
        
        resampleCounts[j]++;
    }
    
    if (debugOutput) {
        // count how many times each particle was resampled
        int maxResamples = *std::max_element(resampleCounts.begin(), resampleCounts.end());
        int numUniqueSurvivors = std::count_if(resampleCounts.begin(), resampleCounts.end(), 
                                              [](int count) { return count > 0; });
        
        std::cout << "  resampling statistics:" << std::endl;
        std::cout << "    unique survivors: " << numUniqueSurvivors << "/" << numParticles << std::endl;
        std::cout << "    max resamples of single particle: " << maxResamples << std::endl;
        std::cout << "    jitter scale applied: " << jitterScale << std::endl;
        
        // show which particles survived most
        std::cout << "    particles with >1 copy: ";
        for (int i = 0; i < numParticles; i++) {
            if (resampleCounts[i] > 1) {
                std::cout << i << "(" << resampleCounts[i] << ") ";
            }
        }
        std::cout << std::endl;
    }
    
    particles = std::move(newParticles);
    
    // add noise after resampling to prevent particle collapse
    // adaptive noise based on current particle spread - less noise when converged
    double currentSpread = getParticleSpread();
    double baseResampleNoise = 0.1; // reduced base noise for better convergence
    double adaptiveNoise = std::min(0.3, baseResampleNoise * (1.0 + currentSpread)); // scale with spread
    
    for (auto& particle : particles) {
        particle.pose(0) += normalDist(randomGenerator) * adaptiveNoise;        // x-position jitter
        particle.pose(1) += normalDist(randomGenerator) * adaptiveNoise;        // y-position jitter  
        particle.pose(2) += normalDist(randomGenerator) * (adaptiveNoise * 0.05); // very small orientation jitter
        
        // normalize angle
        while (particle.pose(2) > M_PI) particle.pose(2) -= 2.0 * M_PI;
        while (particle.pose(2) < -M_PI) particle.pose(2) += 2.0 * M_PI;
    }
    
    if (debugOutput) {
        std::cout << "  particle spread after resampling: " << getParticleSpread() << std::endl;
        std::cout << "  noise injection completed" << std::endl;
    }
}

int UFastSLAM::associateLandmark(const Particle& particle, const Eigen::Vector2d& observation) const {
    // simple nearest neighbor data association
    // in practice, more sophisticated association methods would be used
    
    if (particle.landmarks.empty()) {
        return -1; // no landmarks to associate with
    }
    
    double minDistance = std::numeric_limits<double>::max();
    int bestMatch = -1;
    
    for (size_t i = 0; i < particle.landmarks.size(); i++) {
        // calculate expected observation for this landmark
        Eigen::Vector2d expectedObs = observationModel(particle.pose, 
                                                     particle.landmarks[i].mean);
        
        // calculate mahalanobis distance
        Eigen::Vector2d innovation = observation - expectedObs;
        while (innovation(1) > M_PI) innovation(1) -= 2.0 * M_PI;
        while (innovation(1) < -M_PI) innovation(1) += 2.0 * M_PI;
        
        double distance = mahalanobisDistance(innovation, particle.landmarks[i].covariance);
        
        if (distance < minDistance) {
            minDistance = distance;
            bestMatch = static_cast<int>(i);
        }
    }
    
    // use validation gate for association
    double gateThreshold = 9.21; // chi-square 95% confidence threshold for 2 degrees of freedom
                                  // statistical test for data association: 9.21 corresponds to 95% confidence
                                  // lower values = stricter association, fewer false matches, risk of missing correct associations
                                  // higher values = looser association, more false matches, better recall
    if (minDistance < gateThreshold) {
        return bestMatch;
    }
    
    return -1; // no valid association found
}

double UFastSLAM::mahalanobisDistance(const Eigen::Vector2d& innovation, 
                                     const Eigen::Matrix2d& covariance) const {
    return innovation.transpose() * covariance.inverse() * innovation;
}

void UFastSLAM::updateLandmarkEstimate(Particle& particle, int landmarkIndex, 
                                      const Eigen::Vector2d& observation) {
    // use unscented kalman filter to update landmark estimate
    LandmarkEstimate& landmark = particle.landmarks[landmarkIndex];
    
    // generate sigma points for landmark position
    std::vector<Eigen::VectorXd> sigmaPoints = generateSigmaPoints(landmark.mean, landmark.covariance);
    
    // transform sigma points through observation model
    std::vector<Eigen::VectorXd> transformedPoints;
    for (const auto& point : sigmaPoints) {
        Eigen::Vector2d obsPoint = observationModel(particle.pose, point);
        transformedPoints.push_back(obsPoint);
    }
    
    // calculate weights for unscented transform
    int n = 2; // landmark is 2d
    double lambda = utParams.alpha * utParams.alpha * (n + utParams.kappa) - n;
    std::vector<double> weights(2 * n + 1);
    weights[0] = lambda / (n + lambda);
    for (int i = 1; i < 2 * n + 1; i++) {
        weights[i] = 1.0 / (2.0 * (n + lambda));
    }
    
    // calculate predicted observation and innovation covariance
    auto [predObs, innovCov] = unscentedTransform(transformedPoints, weights);
    
    // add measurement noise consistent with theoretical framework
    Eigen::Matrix2d R = Eigen::Matrix2d::Identity();
    R(0,0) = measurementNoiseStd * measurementNoiseStd;  // consistent measurement noise variance
    R(1,1) = measurementNoiseStd * measurementNoiseStd;  // following theoretical model
    innovCov += R;
    
    // calculate cross-covariance
    Eigen::MatrixXd crossCov = Eigen::MatrixXd::Zero(2, 2);
    for (size_t i = 0; i < sigmaPoints.size(); i++) {
        Eigen::VectorXd stateDiff = sigmaPoints[i] - landmark.mean;
        Eigen::VectorXd obsDiff = transformedPoints[i] - predObs;
        crossCov += weights[i] * stateDiff * obsDiff.transpose();
    }
    
    // calculate kalman gain and update
    Eigen::MatrixXd K = crossCov * innovCov.inverse();
    Eigen::Vector2d innovation = observation - predObs;
    
    // normalize bearing innovation
    while (innovation(1) > M_PI) innovation(1) -= 2.0 * M_PI;
    while (innovation(1) < -M_PI) innovation(1) += 2.0 * M_PI;
    
    // update landmark estimate
    landmark.mean += K * innovation;
    landmark.covariance -= K * innovCov * K.transpose();
    landmark.observationCount++;
}

void UFastSLAM::addNewLandmark(Particle& particle, const Eigen::Vector2d& observation) {
    // initialize new landmark from range-bearing observation
    double range = observation(0);
    double bearing = observation(1);
    
    // convert to cartesian coordinates
    Eigen::Vector2d landmarkPos;
    landmarkPos(0) = particle.pose(0) + range * std::cos(bearing + particle.pose(2));
    landmarkPos(1) = particle.pose(1) + range * std::sin(bearing + particle.pose(2));
    
    // add to particle's landmark map
    particle.landmarks.emplace_back(landmarkPos);
}

void UFastSLAM::predict(const Eigen::Vector3d& controlInput, float deltaTime) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // debug output for prediction step
    static int predictCounter = 0;
    bool debugOutput = (++predictCounter % 50 == 0); // print every 50th prediction
    
    if (debugOutput) {
        std::cout << "ufastslam predict debug #" << predictCounter << ":" << std::endl;
        std::cout << "  control input: [" << controlInput(0) << ", " << controlInput(1) 
                  << ", " << controlInput(2) << "]" << std::endl;
        std::cout << "  delta time: " << deltaTime << " seconds" << std::endl;
        std::cout << "  particle spread before: " << getParticleSpread() << std::endl;
        
        // show current best particle pose
        auto bestParticle = std::max_element(particles.begin(), particles.end(),
            [](const Particle& a, const Particle& b) {
                return a.weight < b.weight;
            });
        
        if (bestParticle != particles.end()) {
            std::cout << "  best particle pose before: [" << bestParticle->pose.transpose() << "]" << std::endl;
        }
    }
    
    predictParticles(controlInput, deltaTime);
    
    if (debugOutput) {
        std::cout << "  particle spread after: " << getParticleSpread() << std::endl;
        
        auto bestParticle = std::max_element(particles.begin(), particles.end(),
            [](const Particle& a, const Particle& b) {
                return a.weight < b.weight;
            });
        
        if (bestParticle != particles.end()) {
            std::cout << "  best particle pose after: [" << bestParticle->pose.transpose() << "]" << std::endl;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    totalPredictTime += std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
}

void UFastSLAM::update(const std::vector<std::pair<Eigen::VectorXd, int>>& observations) {
    auto startTime = std::chrono::high_resolution_clock::now();
    
    // debug output for update step
    static int updateDebugCounter = 0;
    bool debugOutput = (++updateDebugCounter % 25 == 0);
    
    if (debugOutput) {
        std::cout << "ufastslam update debug - processing " << observations.size() 
                  << " observations" << std::endl;
        std::cout << "  current particle spread: " << getParticleSpread() << std::endl;
        std::cout << "  effective sample size before update: " << calculateEffectiveSampleSize() << std::endl;
        
        // show current best particle info
        auto bestParticle = std::max_element(particles.begin(), particles.end(),
            [](const Particle& a, const Particle& b) {
                return a.weight < b.weight;
            });
        
        if (bestParticle != particles.end()) {
            std::cout << "  best particle pose: [" << bestParticle->pose.transpose() << "]" << std::endl;
            std::cout << "  best particle landmarks: " << bestParticle->landmarks.size() << std::endl;
            std::cout << "  best particle weight: " << bestParticle->weight << std::endl;
        }
    }
    
    // update particle weights based on observations using proper ufastslam with ukf landmark updates
    updateParticleWeights(observations);
    
    // check if resampling is needed
    double ess = calculateEffectiveSampleSize();
    bool needsResampling = ess < resamplingThreshold * numParticles;
    
    if (debugOutput) {
        std::cout << "  effective sample size after weight update: " << ess << std::endl;
        std::cout << "  resampling threshold: " << (resamplingThreshold * numParticles) << std::endl;
        std::cout << "  needs resampling: " << (needsResampling ? "yes" : "no") << std::endl;
    }
    
    if (needsResampling) {
        resampleParticles();
        
        if (debugOutput) {
            std::cout << "  resampling completed" << std::endl;
            std::cout << "  particle spread after resampling: " << getParticleSpread() << std::endl;
        }
    }
    
    auto endTime = std::chrono::high_resolution_clock::now();
    totalUpdateTime += std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
    updateCount++;
    
    if (debugOutput) {
        auto updateTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
        std::cout << "  update time: " << updateTime.count() << " μs" << std::endl;
        std::cout << "  average update time: " << (totalUpdateTime.count() / updateCount) << " μs" << std::endl;
    }
}

Eigen::Vector3d UFastSLAM::getRobotPose() const {
    // option 1: use best particle pose (consistent with landmark selection)
    auto bestParticle = std::max_element(particles.begin(), particles.end(),
        [](const Particle& a, const Particle& b) {
            return a.weight < b.weight;
        });
    
    if (bestParticle != particles.end()) {
        // debug: occasionally show best particle vs weighted average comparison
        static int poseDebugCounter = 0;
        bool showDebug = (++poseDebugCounter % 100 == 0);
        
        if (showDebug) {
            std::cout << "pose estimation debug:" << std::endl;
            std::cout << "  best particle pose: [" << bestParticle->pose.transpose() << "]" << std::endl;
            std::cout << "  best particle weight: " << bestParticle->weight << std::endl;
            
            // calculate weighted average for comparison
            Eigen::Vector3d weightedPose = Eigen::Vector3d::Zero();
            double totalWeight = 0.0;
            double weightedSinTheta = 0.0, weightedCosTheta = 0.0;
            
            for (const auto& particle : particles) {
                weightedPose(0) += particle.weight * particle.pose(0);
                weightedPose(1) += particle.weight * particle.pose(1);
                weightedSinTheta += particle.weight * std::sin(particle.pose(2));
                weightedCosTheta += particle.weight * std::cos(particle.pose(2));
                totalWeight += particle.weight;
            }
            
            if (totalWeight > 1e-15) {
                weightedPose(0) /= totalWeight;
                weightedPose(1) /= totalWeight;
                weightedPose(2) = std::atan2(weightedSinTheta / totalWeight, weightedCosTheta / totalWeight);
            }
            
            std::cout << "  weighted average pose: [" << weightedPose.transpose() << "]" << std::endl;
            std::cout << "  pose difference: [" << (bestParticle->pose - weightedPose).transpose() << "]" << std::endl;
        }
        
        return bestParticle->pose;
    }
    
    // fallback: calculate weighted average of particle poses using proper circular statistics
    // for angular components to handle wraparound at ±π boundary correctly
    Eigen::Vector3d weightedPose = Eigen::Vector3d::Zero();
    double totalWeight = 0.0;
    
    // accumulate weighted cartesian components and circular angle representation
    double weightedSinTheta = 0.0;
    double weightedCosTheta = 0.0;
    
    for (const auto& particle : particles) {
        // standard arithmetic averaging for position components
        weightedPose(0) += particle.weight * particle.pose(0);  // x position
        weightedPose(1) += particle.weight * particle.pose(1);  // y position
        
        // circular averaging for angular component using complex representation
        // this prevents artifacts when angles span the ±π discontinuity
        weightedSinTheta += particle.weight * std::sin(particle.pose(2));
        weightedCosTheta += particle.weight * std::cos(particle.pose(2));
        
        totalWeight += particle.weight;
    }
    
    if (totalWeight > 1e-15) {
        // normalize position components by total weight
        weightedPose(0) /= totalWeight;
        weightedPose(1) /= totalWeight;
        
        // recover mean angle from weighted sine and cosine components
        // using atan2 to properly handle all quadrants and discontinuities
        weightedPose(2) = std::atan2(weightedSinTheta / totalWeight, 
                                    weightedCosTheta / totalWeight);
        
        return weightedPose;
    } else {
        // fallback when all weights are zero: use unweighted circular mean
        Eigen::Vector3d meanPose = Eigen::Vector3d::Zero();
        double sinSum = 0.0;
        double cosSum = 0.0;
        
        for (const auto& particle : particles) {
            meanPose(0) += particle.pose(0);
            meanPose(1) += particle.pose(1);
            sinSum += std::sin(particle.pose(2));
            cosSum += std::cos(particle.pose(2));
        }
        
        if (!particles.empty()) {
            meanPose(0) /= particles.size();
            meanPose(1) /= particles.size();
            meanPose(2) = std::atan2(sinSum / particles.size(), 
                                    cosSum / particles.size());
        }
        return meanPose;
    }
}

sf::Vector2f UFastSLAM::getRobotPosition() const {
    Eigen::Vector3d pose = getRobotPose();
    return sf::Vector2f(static_cast<float>(pose(0)), static_cast<float>(pose(1)));
}

float UFastSLAM::getRobotDirection() const {
    Eigen::Vector3d pose = getRobotPose();
    return static_cast<float>(pose(2) * 180.0 / M_PI); // convert to degrees
}

std::vector<Eigen::Vector2d> UFastSLAM::getLandmarkPositions() const {
    // get landmark estimates from best particle (highest weight)
    auto bestParticle = std::max_element(particles.begin(), particles.end(),
        [](const Particle& a, const Particle& b) {
            return a.weight < b.weight;
        });
    
    if (bestParticle != particles.end()) {
        std::vector<Eigen::Vector2d> landmarks;
        for (const auto& landmark : bestParticle->landmarks) {
            landmarks.push_back(landmark.mean);
        }
        return landmarks;
    }
    
    return {};
}

int UFastSLAM::getNumLandmarks() const {
    // get number of landmarks from best particle
    auto bestParticle = std::max_element(particles.begin(), particles.end(),
        [](const Particle& a, const Particle& b) {
            return a.weight < b.weight;
        });
    
    if (bestParticle != particles.end()) {
        return static_cast<int>(bestParticle->landmarks.size());
    }
    
    return 0;
}

double UFastSLAM::getParticleSpread() const {
    // calculate standard deviation of particle positions (unweighted for debugging)
    Eigen::Vector3d meanPose = Eigen::Vector3d::Zero();
    
    // calculate unweighted mean
    for (const auto& particle : particles) {
        meanPose += particle.pose;
    }
    meanPose /= particles.size();
    
    double variance = 0.0;
    for (const auto& particle : particles) {
        Eigen::Vector3d diff = particle.pose - meanPose;
        variance += diff.squaredNorm();
    }
    variance /= particles.size();
    
    return std::sqrt(variance);
}

void UFastSLAM::startAlgorithmRound() const {
    algorithmStartTime = std::chrono::high_resolution_clock::now();
}

void UFastSLAM::endAlgorithmRound() const {
    // performance tracking implementation can be added here
}

void UFastSLAM::printComputationalLoad(const std::chrono::microseconds& totalRoundTime) const {
    if (updateCount > 0) {
        std::cout << "ufastslam performance:" << std::endl;
        std::cout << "  average predict time: " << (totalPredictTime.count() / updateCount) << " μs" << std::endl;
        std::cout << "  average update time: " << (totalUpdateTime.count() / updateCount) << " μs" << std::endl;
        std::cout << "  total algorithm time: " << totalRoundTime.count() << " μs" << std::endl;
        std::cout << "  particle spread: " << getParticleSpread() << std::endl;
    }
}

void UFastSLAM::printParticleStats() const {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "ufastslam particle statistics:" << std::endl;
    
    Eigen::Vector3d meanPose = getRobotPose();
    std::cout << "  estimated robot pose: [" 
              << meanPose(0) << ", " << meanPose(1) << ", " << meanPose(2) << "]" << std::endl;
    
    std::cout << "  particle spread: " << getParticleSpread() << std::endl;
    std::cout << "  number of landmarks: " << getNumLandmarks() << std::endl;
    std::cout << "  effective sample size: " << calculateEffectiveSampleSize() << std::endl;
    
    // detailed particle analysis
    std::vector<double> weights;
    std::vector<double> xPositions, yPositions, orientations;
    std::vector<int> landmarkCounts;
    
    for (const auto& particle : particles) {
        weights.push_back(particle.weight);
        xPositions.push_back(particle.pose(0));
        yPositions.push_back(particle.pose(1));
        orientations.push_back(particle.pose(2));
        landmarkCounts.push_back(static_cast<int>(particle.landmarks.size()));
    }
    
    // calculate statistics
    auto minMaxWeight = std::minmax_element(weights.begin(), weights.end());
    auto minMaxX = std::minmax_element(xPositions.begin(), xPositions.end());
    auto minMaxY = std::minmax_element(yPositions.begin(), yPositions.end());
    auto minMaxLandmarks = std::minmax_element(landmarkCounts.begin(), landmarkCounts.end());
    
    double meanWeight = std::accumulate(weights.begin(), weights.end(), 0.0) / weights.size();
    double meanX = std::accumulate(xPositions.begin(), xPositions.end(), 0.0) / xPositions.size();
    double meanY = std::accumulate(yPositions.begin(), yPositions.end(), 0.0) / yPositions.size();
    
    std::cout << "  weight distribution:" << std::endl;
    std::cout << "    min: " << *minMaxWeight.first << ", max: " << *minMaxWeight.second 
              << ", mean: " << meanWeight << std::endl;
    
    std::cout << "  position distribution:" << std::endl;
    std::cout << "    x range: [" << *minMaxX.first << ", " << *minMaxX.second 
              << "], mean: " << meanX << std::endl;
    std::cout << "    y range: [" << *minMaxY.first << ", " << *minMaxY.second 
              << "], mean: " << meanY << std::endl;
    
    std::cout << "  landmark count distribution:" << std::endl;
    std::cout << "    range: [" << *minMaxLandmarks.first << ", " << *minMaxLandmarks.second << "]" << std::endl;
    
    // show top 5 particles by weight
    std::vector<std::pair<double, int>> weightedIndices;
    for (int i = 0; i < numParticles; i++) {
        weightedIndices.push_back({particles[i].weight, i});
    }
    std::sort(weightedIndices.begin(), weightedIndices.end(), std::greater<std::pair<double, int>>());
    
    std::cout << "  top 5 particles by weight:" << std::endl;
    for (int i = 0; i < std::min(5, numParticles); i++) {
        int idx = weightedIndices[i].second;
        const auto& p = particles[idx];
        std::cout << "    particle " << idx << ": weight=" << p.weight 
                  << ", pose=[" << p.pose.transpose() << "], landmarks=" << p.landmarks.size() << std::endl;
    }
}

void UFastSLAM::reset() {
    // reset all particles to initial state
    for (auto& particle : particles) {
        particle.pose = Eigen::Vector3d::Zero();
        particle.velocity = Eigen::Vector3d::Zero();
        particle.landmarks.clear();
        particle.weight = 1.0 / numParticles;
    }
    
    // reset performance counters
    totalPredictTime = std::chrono::microseconds{0};
    totalUpdateTime = std::chrono::microseconds{0};
    updateCount = 0;
    
    std::cout << "ufastslam state reset to initial conditions." << std::endl;
}

void UFastSLAM::printDetailedParticleInfo(int particleIndex) const {
    if (particleIndex < 0 || particleIndex >= numParticles) {
        std::cout << "invalid particle index: " << particleIndex << std::endl;
        return;
    }
    
    const Particle& particle = particles[particleIndex];
    
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "particle " << particleIndex << " detailed info:" << std::endl;
    std::cout << "  pose: [" << particle.pose(0) << ", " << particle.pose(1) 
              << ", " << particle.pose(2) << "] rad" << std::endl;
    std::cout << "  velocity: [" << particle.velocity(0) << ", " << particle.velocity(1) 
              << ", " << particle.velocity(2) << "] units/s" << std::endl;
    std::cout << "  weight: " << particle.weight << std::endl;
    std::cout << "  landmarks: " << particle.landmarks.size() << std::endl;
    
    // show first few landmarks with uncertainty
    for (size_t i = 0; i < std::min(size_t(3), particle.landmarks.size()); i++) {
        const LandmarkEstimate& lm = particle.landmarks[i];
        std::cout << "    landmark " << i << ": pos=[" << lm.mean(0) << ", " << lm.mean(1) 
                  << "] uncertainty=[" << lm.covariance(0,0) << ", " << lm.covariance(1,1) 
                  << "] obs_count=" << lm.observationCount << std::endl;
    }
}

void UFastSLAM::printLandmarkInfo() const {
    // get best particle for landmark analysis
    auto bestParticle = std::max_element(particles.begin(), particles.end(),
        [](const Particle& a, const Particle& b) {
            return a.weight < b.weight;
        });
    
    if (bestParticle == particles.end()) {
        std::cout << "no particles available for landmark analysis" << std::endl;
        return;
    }
    
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "landmark analysis from best particle:" << std::endl;
    std::cout << "total landmarks: " << bestParticle->landmarks.size() << std::endl;
    
    for (size_t i = 0; i < bestParticle->landmarks.size(); i++) {
        const LandmarkEstimate& lm = bestParticle->landmarks[i];
        double uncertainty = std::sqrt(lm.covariance.trace() / 2.0); // average position uncertainty
        
        std::cout << "  landmark " << i << ":" << std::endl;
        std::cout << "    position: [" << lm.mean(0) << ", " << lm.mean(1) << "]" << std::endl;
        std::cout << "    uncertainty: " << uncertainty << " units" << std::endl;
        std::cout << "    observations: " << lm.observationCount << std::endl;
        std::cout << "    covariance det: " << lm.covariance.determinant() << std::endl;
    }
}

void UFastSLAM::printConvergenceMetrics() const {
    // calculate particle diversity metrics
    Eigen::Vector3d meanPose = getRobotPose();
    double positionVariance = 0.0;
    double orientationVariance = 0.0;
    double weightEntropy = 0.0;
    
    // calculate pose variance
    for (const auto& particle : particles) {
        Eigen::Vector3d diff = particle.pose - meanPose;
        positionVariance += particle.weight * (diff.head<2>().squaredNorm());
        orientationVariance += particle.weight * (diff(2) * diff(2));
        
        // calculate weight entropy
        if (particle.weight > 0) {
            weightEntropy -= particle.weight * std::log(particle.weight);
        }
    }
    
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "convergence metrics:" << std::endl;
    std::cout << "  position variance: " << positionVariance << " units²" << std::endl;
    std::cout << "  orientation variance: " << orientationVariance << " rad²" << std::endl;
    std::cout << "  weight entropy: " << weightEntropy << std::endl;
    std::cout << "  effective sample size: " << calculateEffectiveSampleSize() << std::endl;
    std::cout << "  particle spread: " << getParticleSpread() << " units" << std::endl;
    
    // analyze weight distribution
    double maxWeight = 0.0;
    double minWeight = 1.0;
    for (const auto& particle : particles) {
        maxWeight = std::max(maxWeight, particle.weight);
        minWeight = std::min(minWeight, particle.weight);
    }
    std::cout << "  weight range: [" << minWeight << ", " << maxWeight << "]" << std::endl;
    std::cout << "  weight ratio: " << (maxWeight / minWeight) << std::endl;
}

void UFastSLAM::debugMotionModel(const Eigen::Vector3d& controlInput, float deltaTime) const {
    std::cout << "motion model debug:" << std::endl;
    std::cout << "  control input: [" << controlInput(0) << ", " << controlInput(1) 
              << ", " << controlInput(2) << "]" << std::endl;
    std::cout << "  delta time: " << deltaTime << " seconds" << std::endl;
    
    // show motion prediction for first particle
    if (!particles.empty()) {
        const Particle& p = particles[0];
        Eigen::Vector3d predicted = motionModel(p.pose, p.velocity, controlInput, deltaTime);
        
        std::cout << "  example particle motion:" << std::endl;
        std::cout << "    before: pose=[" << p.pose(0) << ", " << p.pose(1) << ", " << p.pose(2) 
                  << "] vel=[" << p.velocity(0) << ", " << p.velocity(1) << ", " << p.velocity(2) << "]" << std::endl;
        std::cout << "    after:  pose=[" << predicted(0) << ", " << predicted(1) << ", " << predicted(2) << "]" << std::endl;
        
        Eigen::Vector3d change = predicted - p.pose;
        std::cout << "    change: [" << change(0) << ", " << change(1) << ", " << change(2) << "]" << std::endl;
    }
}
