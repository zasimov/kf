#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include <memory>

#include <Eigen/Dense>


struct KalmanFilterState {
  Eigen::VectorXd x_;  // mean vector
  Eigen::MatrixXd P_;  // state covariance matrix

  bool is_initialized_;

  KalmanFilterState(unsigned x_size)
  : x_(x_size), P_(x_size, x_size), is_initialized_(false){
  }
};


class AbstractKalmanFilter {
 public:
  AbstractKalmanFilter(std::shared_ptr<KalmanFilterState> state,
		       const Eigen::MatrixXd &R);

  virtual ~AbstractKalmanFilter() {
  }

  /**
   * Should be called once (with first measurement).
   */
  virtual void Init(const Eigen::VectorXd &z) = 0;


  /**
   * Predict implements "Predict" step.
   *
   *  F - transition matrix for dt
   *  Q - process covariance matrix for dt
   *  u - control vector
   *
   * Predict changes `x_` and `P_`. `x_` fill contain prediction for (t + 1)
   *
   */
  const Eigen::VectorXd& Predict(const Eigen::MatrixXd &F, const Eigen::MatrixXd &Q, const Eigen::VectorXd &u);

  /**
   * Update implements "Update" step"
   *
   *  z is a measurement vector.
   */
  void Update(const Eigen::VectorXd &z);

  /**
   * GetEstimate returns current estimate of state.
   */
  const Eigen::VectorXd& GetEstimate() const;

 protected:
  /**
   * GetH should return H matrix (X to Z projection).
   */
  virtual const Eigen::MatrixXd GetH() const = 0;

  /**
   * GetY calculates `y` vector.
   */
  virtual Eigen::VectorXd GetY(const Eigen::VectorXd &z) const = 0;

 protected:
  std::shared_ptr<KalmanFilterState> state_;

 private:
  Eigen::MatrixXd R_;  // measurement covariance matrix

  Eigen::MatrixXd I_;  // identity matrix
};


class LinearKalmanFilter : public AbstractKalmanFilter {
 public:
  LinearKalmanFilter(std::shared_ptr<KalmanFilterState> state,
		     const Eigen::MatrixXd &R,
		     const Eigen::MatrixXd &H);

  void Init(const Eigen::VectorXd &z);

 protected:
  const Eigen::MatrixXd GetH() const;
  Eigen::VectorXd GetY(const Eigen::VectorXd &z) const;

 private:
  Eigen::MatrixXd H_;  // measurement function

};


class ExtendedKalmanFilter : public AbstractKalmanFilter {
 public:
  ExtendedKalmanFilter(std::shared_ptr<KalmanFilterState> state,
		       const Eigen::MatrixXd &R);

  void Init(const Eigen::VectorXd &z);
 protected:
  const Eigen::MatrixXd GetH() const;
  Eigen::VectorXd GetY(const Eigen::VectorXd &z) const;
};


#endif /* __KALMAN_FILTER_H */
