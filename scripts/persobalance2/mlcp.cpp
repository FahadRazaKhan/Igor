#include "mlcp.hpp"

#include <Eigen/Core>

void mlcp_solve(unsigned n,
				double* x,
				const double* M,
				const double* q,
				const double* bilateral_mask,
				mlcp_config* params) {

  const auto is_bilateral = [&](unsigned i) -> bool{
	return bilateral_mask && bilateral_mask[i];
  };
  
  static const mlcp_config default_params = {100, 1e-8, false};
  
  const mlcp_config* par = params;
  if(!par) par = &default_params;
  
  using matrix_type = Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>;
  using vector_type = Eigen::Matrix<double, Eigen::Dynamic, 1>;

  using matrix_map_type = Eigen::Map<const matrix_type>;
  using vector_map_type = Eigen::Map<vector_type>;  
  
  matrix_map_type Mm(M, n, n);
  vector_map_type xm(x, n);

  vector_type old = vector_type::Zero(n);
  
  vector_type grad, p;
  double norm2 = 0;
  double error = 0;
  
  const bool nlnscg = par->nlnscg; 
  const double eps = par->precision;
  
  if(nlnscg) {
	grad.setZero(n);
	p.setZero(n);
  }

  // std::clog << "nlnscg: " << nlnscg << std::endl;
  
  unsigned k, kmax;
  for(k = 0, kmax = par->iterations; k < kmax; ++k) {

	old = xm;
	
	for(unsigned i = 0; i < n; ++i) {

	  // gauss-seidel
	  x[i] -= (q[i] + Mm.col(i).dot(xm)) / Mm(i, i);

	  // bileteral constraints
	  if(is_bilateral(i)) continue;

	  // projection
	  if( x[i] < 0 ) x[i] = 0;
	}
	
	// error
	grad = old - xm;
	const double norm2_next = grad.squaredNorm();

	error = std::sqrt(norm2_next);
	// std::clog << "mlcp error: " << error << std::endl;
	
	if(error < eps ) break;
	  
	if(!nlnscg) continue;

	const double beta = norm2 ? norm2_next / norm2 : 0;

	if( beta > 1 ) {
	  p.setZero(n);
	} else {
	  xm += beta * p;
	  p = beta * p - grad;
	}

	norm2 = norm2_next;
  }

  if(params) {
	params->iterations = k + 1;
	params->precision = error;
  }
  
}

