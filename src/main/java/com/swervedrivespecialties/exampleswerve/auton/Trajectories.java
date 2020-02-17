/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.auton;

import java.util.function.Supplier;

import org.frcteam2910.common.control.CentripetalAccelerationConstraint;
import org.frcteam2910.common.control.ITrajectoryConstraint;
import org.frcteam2910.common.control.MaxAccelerationConstraint;
import org.frcteam2910.common.control.MaxVelocityConstraint;
import org.frcteam2910.common.control.Path;
import org.frcteam2910.common.control.PathArcSegment;
import org.frcteam2910.common.control.PathLineSegment;
import org.frcteam2910.common.control.Trajectory;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;

public class Trajectories {

    ////////// UNIVERSAL TRAJECTORY CONSTANTS /////////////
    private static final int kSubdivideIterations = 8;
    private static final double kDefaultMaxSpeed = 10 * 12;
    private static final double kMaxAccel = 13. * 12;
    private static final double kMaxCentripedalAccel = 25. * 12;
    //////////////////////////////////////////////////////


    public static class steallBallAuton { 
        private static final Vector2 goalPoint = Vector2.ZERO;
        private static final Rotation2 startRot = Rotation2.ZERO;
        private static final Vector2 shootPoint = new Vector2(45, 120); //must have that |y| > |x - 88|
        private static final double stealBallSpeed = 11;
        private static final double goShootSpeed = 11;

        private static final double rad = 88 - shootPoint.x;
        private static final Rotation2 firstShotRotation = getAngleToPointAt(shootPoint, goalPoint);
        
        private static Trajectory toStealBalls;
        private static Trajectory toShootFirstBatch;

        private static void generateToStealBall(){
            ITrajectoryConstraint[] stealBallConstraints = getConstraint(stealBallSpeed);
            Path stealBallPath = new Path(startRot);
            stealBallPath.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(88, 0)), startRot);
            stealBallPath.subdivide(kSubdivideIterations);
            toStealBalls = new Trajectory(0.0, 0.0, stealBallPath, stealBallConstraints);
        }

        private static void generateToShootFirstBatch(){
            ITrajectoryConstraint[] toShootFirstBatchConstraints = getConstraint(goShootSpeed);
            Path toShootFirstBatchPath = new Path(startRot);
            toShootFirstBatchPath.addSegment(new PathArcSegment(new Vector2(88, 0), new Vector2(88 - rad, rad), new Vector2(88, rad)), firstShotRotation);
            toShootFirstBatchPath.addSegment(new PathLineSegment(new Vector2(88 - rad, rad), shootPoint), firstShotRotation);
            toShootFirstBatchPath.subdivide(8);
            toShootFirstBatch = new Trajectory(0.0, 0.0, toShootFirstBatchPath, toShootFirstBatchConstraints);
        }     

        private static void generate(){
            generateToStealBall();
            generateToShootFirstBatch();
        }

        public static Supplier<Trajectory> toStealBallsTrajectorySupplier = () -> toStealBalls;
        public static Supplier<Trajectory> toShootFirstBatchTrajectorySupplier = () -> toShootFirstBatch;
    }


    public static void generateAllTrajectories(){
        steallBallAuton.generate();
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

    private static Rotation2 getAngleToPointAt(Vector2 self, Vector2 target){
        return target.subtract(self).getAngle();
    }
}

