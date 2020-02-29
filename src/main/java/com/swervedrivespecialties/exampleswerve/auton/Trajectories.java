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
    private static final int kSubdivideIterations = 10;
    private static final double kDefaultMaxSpeed = 10 * 12;
    private static final double kMaxAccel = 11 * 12; //13*12
    private static final double kMaxCentripedalAccel = 25. * 12;
    //////////////////////////////////////////////////////


    public static class steallBallAuton { 
        private static final Vector2 goalPoint = new Vector2(-176, 198);
        private static final Rotation2 startRot = Rotation2.ZERO;
        private static final double travelDist = 93.75;
        private static final Vector2 shootPoint = new Vector2(80, 124); //must have that |y| > |x - travelDist|
        private static final double stealBallSpeed = 11 * 12;
        private static final double goShootSpeed = 11.5 * 12;

        private static final double rad = travelDist - shootPoint.x;
        private static final Rotation2 firstShotRotation = getAngleToPointAt(shootPoint, goalPoint).rotateBy(Rotation2.fromDegrees(-87));
        public static final Rotation2 towardsBallsRotation = Rotation2.fromDegrees(25);
        private static final double driveForwardDistance = 30;
        
        private static Trajectory toStealBalls;
        private static Trajectory toShootFirstBatch;
        private static Trajectory pickupNext;

        private static void generateToStealBall(){
            ITrajectoryConstraint[] stealBallConstraints = getConstraint(stealBallSpeed);
            Path stealBallPath = new Path(startRot);
            stealBallPath.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(travelDist, 0)), Rotation2.fromDegrees(0));
            stealBallPath.subdivide(kSubdivideIterations);
            toStealBalls = new Trajectory(0.0, 0.0, stealBallPath, stealBallConstraints);
        }

        private static void generateToShootFirstBatch(){
            ITrajectoryConstraint[] toShootFirstBatchConstraints = getConstraint(goShootSpeed);
            Path toShootFirstBatchPath = new Path(startRot);
            //toShootFirstBatchPath.addSegment(new PathArcSegment(new Vector2(travelDist, 0), new Vector2(travelDist - rad, rad), new Vector2(travelDist, rad)), firstShotRotation);
            toShootFirstBatchPath.addSegment(new PathLineSegment(new Vector2(travelDist, 0), shootPoint), firstShotRotation);
            toShootFirstBatchPath.subdivide(kSubdivideIterations);
            toShootFirstBatch = new Trajectory(0.0, 0.0, toShootFirstBatchPath, toShootFirstBatchConstraints);
        }     

        private static void generatePickupNext(){
            ITrajectoryConstraint[] pickupNextConstraints = getConstraint(8);
            Path pickupNextBallz = new Path(towardsBallsRotation);
            pickupNextBallz.addSegment(new PathLineSegment(shootPoint, shootPoint.add(Vector2.fromAngle(towardsBallsRotation).scale(driveForwardDistance))));
            pickupNextBallz.subdivide(kSubdivideIterations);
            pickupNext = new Trajectory(0.0, 0.0, pickupNextBallz, pickupNextConstraints);
        }

        private static void generate(){
            generateToStealBall();
            generateToShootFirstBatch();
            generatePickupNext();
        }

        public static Supplier<Trajectory> toStealBallsTrajectorySupplier = () -> toStealBalls;
        public static Supplier<Trajectory> toShootFirstBatchTrajectorySupplier = () -> toShootFirstBatch;
        public static Supplier<Trajectory> toPickupNextTrajectorySupplier = () -> pickupNext;
    }

    public static class steallBallBestAuton { 
        private static final Vector2 goalPoint = new Vector2(-224, 200);
        private static final Rotation2 startRot = Rotation2.ZERO;
        private static final double travelDist = 92;
        private static final Vector2 shootPoint = new Vector2(36, 200); //must have that |y| > |x - travelDist|
        private static final double stealBallSpeed = 11 * 12;
        private static final double goShootSpeed = 11.5 * 12;

        private static final double rad = travelDist - shootPoint.x;
        private static final Rotation2 firstShotRotation = getAngleToPointAt(shootPoint, goalPoint).rotateBy(Rotation2.fromDegrees(-105));
        
        private static Trajectory toStealBalls;
        private static Trajectory toShootFirstBatch;

        private static void generateToStealBall(){
            ITrajectoryConstraint[] stealBallConstraints = getConstraint(stealBallSpeed);
            Path stealBallPath = new Path(startRot);
            stealBallPath.addSegment(new PathLineSegment(Vector2.ZERO, new Vector2(travelDist, 0)), Rotation2.fromDegrees(0));
            stealBallPath.subdivide(kSubdivideIterations);
            toStealBalls = new Trajectory(0.0, 0.0, stealBallPath, stealBallConstraints);
        }

        private static void generateToShootFirstBatch(){
            ITrajectoryConstraint[] toShootFirstBatchConstraints = getConstraint(goShootSpeed);
            Path toShootFirstBatchPath = new Path(startRot);
            toShootFirstBatchPath.addSegment(new PathArcSegment(new Vector2(travelDist, 0), new Vector2(travelDist - rad, rad), new Vector2(travelDist, rad)), firstShotRotation);
            toShootFirstBatchPath.addSegment(new PathLineSegment(new Vector2(travelDist - rad, rad), shootPoint), firstShotRotation);
            toShootFirstBatchPath.subdivide(kSubdivideIterations);
            toShootFirstBatch = new Trajectory(0.0, 0.0, toShootFirstBatchPath, toShootFirstBatchConstraints);
        }     

        private static void generate(){
            generateToStealBall();
            generateToShootFirstBatch();
        }

        public static Supplier<Trajectory> toStealBallsTrajectorySupplier = () -> toStealBalls;
        public static Supplier<Trajectory> toShootFirstBatchTrajectorySupplier = () -> toShootFirstBatch;
    }

    public static class sevenBallAuton {
        public static Trajectory toGetNextBatch;

        private static void generateToGetNextBatch(){
            ITrajectoryConstraint[] toGetNextBatchConstraints = getConstraint(12 * 12);
            Path toGetNextBatchPath = new Path(steallBallBestAuton.firstShotRotation);
            toGetNextBatchPath.addSegment(new PathArcSegment(steallBallBestAuton.shootPoint, new Vector2(160, 170), new Vector2(120, 180)), Rotation2.fromDegrees(30));
            toGetNextBatchPath.subdivide(kSubdivideIterations);
            toGetNextBatch = new Trajectory(0.0, 36.0, toGetNextBatchPath, toGetNextBatchConstraints);
        }

        public static Supplier<Trajectory> toGetNextBatchTrajectorySupplier = () -> toGetNextBatch;

        public static Trajectory toShootNextShot;

        private static void generateToShootNextShot(){
            ITrajectoryConstraint[] toShootNextShotConstraints = getConstraint(12 * 12);
            Path toShootNextShotPath = new Path(Rotation2.fromDegrees(30));
            toShootNextShotPath.addSegment(new PathLineSegment(new Vector2(90, 140), new Vector2(65, 110)), Rotation2.fromDegrees(140));
            toShootNextShotPath.subdivide(kSubdivideIterations);
            toShootNextShot = new Trajectory(0.0, 36.0, toShootNextShotPath, toShootNextShotConstraints);
        }

        public static Supplier<Trajectory> toShootNextShotTrajectorySupplier = () -> toGetNextBatch;

        public static void generate(){
            generateToGetNextBatch();
            generateToShootNextShot();
        }
    }

    public static void generateAllTrajectories(){
        steallBallAuton.generate();
        steallBallBestAuton.generate();
        sevenBallAuton.generate();
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

