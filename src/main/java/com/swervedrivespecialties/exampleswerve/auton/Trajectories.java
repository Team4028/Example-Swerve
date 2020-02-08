/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.auton;

import java.util.function.Supplier;

import com.swervedrivespecialties.exampleswerve.util.util;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathArcSegment;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.PathSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Trajectories {

    ////////// UNIVERSAL TRAJECTORY CONSTANTS /////////////
    private static final int kSubdivideIterations = 8;
    private static final double kDefaultMaxSpeed = 10 * 12;
    private static final double kMaxAccel = 13. * 12;
    private static final double kMaxCentripedalAccel = 25. * 12;


    public static class steallBallAuton {
        private static final Vector2 startingPos = new Vector2(11, 23); //The origin is a theoretical point in the back right corner of the field. 
        private static final Vector2 shootPos = new Vector2(100, 200);
        private static final double stealBallX = 120;
        private static final double ly = 47.5;
        private static final double stealBallSpeed = 10.0;
        private static final double goToShootSpeed = 10.0;

        private static final double lx = stealBallX - startingPos.x;
        private static final double lambda = (lx * lx + ly * ly)  / (2.0 * ly);

        private static final ITrajectoryConstraint[] toStealBallConstraints = getConstraint(stealBallSpeed);
        
        




    }


    public static void generateAllTrajectories(){
    }


    /////////////////// Helper Methods /////////////////////////

    public static Trajectory generateLineTrajectory(Vector2 line, double speed,  Rotation2 startRotation, Rotation2 endRotation){
        ITrajectoryConstraint[] lineTrajectoryConstraints = {new MaxVelocityConstraint(speed), 
                                                             new MaxAccelerationConstraint(kMaxAccel), 
                                                             new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
        Path linePath = new Path(startRotation);
        linePath.addSegment(new PathLineSegment(Vector2.ZERO, line), endRotation);
        linePath.subdivide(kSubdivideIterations);
        Trajectory resultTrajectory = new Trajectory(0.0, 0.0, linePath, lineTrajectoryConstraints);
        return resultTrajectory;
    }

    public static Trajectory generateLineTrajectory(Vector2 line, Rotation2 startRotation, Rotation2 endRotation){
        return generateLineTrajectory(line, kDefaultMaxSpeed, startRotation, endRotation);
    }

    private static Trajectory generateLineTrajectory(Vector2 line, Rotation2 endRotation){
        return generateLineTrajectory(line, Rotation2.ZERO, endRotation);
    }

    private static ITrajectoryConstraint[] getConstraint(double spd){
        ITrajectoryConstraint[] res = {new MaxVelocityConstraint(spd), new MaxAccelerationConstraint(kMaxAccel), new CentripetalAccelerationConstraint(kMaxCentripedalAccel)};
        return res;
    }
}
