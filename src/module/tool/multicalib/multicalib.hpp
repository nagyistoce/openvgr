/*
 multicalib.hpp

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#ifndef MULTICALIB_HPP
#define MULTICALIB_HPP

#include <string>
#include "calib_proc.hpp"

struct MulticalibOpt
{
  std::string ifile;
  std::string ofile;

  double interval;
  int output_projection_matrix;

  calib_opt_t calib_opt;

  MulticalibOpt() : interval(0.0), output_projection_matrix(0), calib_opt(CALIB_OPT_INIT) {}
};

#endif /* MULTICALIB_HPP */
