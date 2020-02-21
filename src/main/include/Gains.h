/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

class Gains {
 public:
  double kP;
  double kI;
  double kD;
  double kF;
  int kIzone;
  double kPeakOutput;

  Gains(double _kP, double _kI, double _kD, double _kF, int _kIzone, double _kPeakOutput) 
    :
    kP{_kP},
    kI{_kI},
    kD{_kD},
    kF{_kF},
    kIzone{_kIzone},
    kPeakOutput{_kPeakOutput}
    {

    }
};
