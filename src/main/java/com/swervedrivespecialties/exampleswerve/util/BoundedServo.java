/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import edu.wpi.first.wpilibj.Servo;

public class BoundedServo extends Servo{

    public BoundedServo(int port, double lowerBound, double upperBound){
        super(port);
        double lower = lowerBound + 1;
        double upper = upperBound + 1;
        this.setBounds(upper, upper, 1.5, lower, lower);
    }
}
