#ifndef MLCP_HPP
#define MLCP_HPP

#include "export.hpp"

extern "C" {

  struct mlcp_config {
	unsigned iterations;
	double precision;
	int nlnscg;
  };
  
  EXPORT void mlcp_solve(unsigned size,
						 double* x,
						 const double* M,
						 const double* q,
						 const double* bilateral_mask,
						 mlcp_config* params);
  
}


#endif
