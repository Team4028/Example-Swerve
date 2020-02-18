/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import edu.wpi.first.wpiutil.CircularBuffer;

public class BeakCircularBuffer extends CircularBuffer{
    
    int size = 0;

    public BeakCircularBuffer(int size){
        super(size);
        this.size = size;
    }

    public double getMean(){
        double tot = 0;
        for (int i = 0; i < this.size; i++){
            tot += this.get(i);
        }
        double avg = tot / this.size;
        return avg;
    }


}
