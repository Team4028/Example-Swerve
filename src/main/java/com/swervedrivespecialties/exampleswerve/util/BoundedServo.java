/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import edu.wpi.first.wpilibj.Servo;

public class BoundedServo {
    Servo servo;
    double lower;
    double upper;

    public BoundedServo(int port, double lowerBound, double upperBound){
        servo = new Servo(port);
        lower = lowerBound;
        upper = upperBound;
    }

    public void set(double val){
        servo.set(Math.min(upper, Math.max(val, lower)));
    }

    public double get(){
        return servo.get();
    }
}
