/*
 multicalib.h

 Copyright (c) 2011 AIST  All Rights Reserved.
 Eclipse Public License v1.0 (http://www.eclipse.org/legal/epl-v10.html)

 Written by Satoshi KAWABATA <kawabata.aist@gmail.com>

 $Date::                            $
*/

#ifndef MULTICALIB_H
#define MULTICALIB_H

#include <stdio.h>
#include "calib_proc.h"

typedef struct tag_multicalib_opt
{
  char *ifile;
  char *ofile;

  double interval;

  calib_opt_t calib_opt;
} multicalib_opt_t;

#define MULTICALIB_OPT_INIT {NULL, NULL, 0, CALIB_OPT_INIT}

#endif /* MULTICALIB_H */
