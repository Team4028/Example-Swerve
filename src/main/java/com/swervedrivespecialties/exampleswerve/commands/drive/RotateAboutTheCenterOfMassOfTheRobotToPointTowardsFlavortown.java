/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.drive;

import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.control.PidController;
import org.frcteam2910.common.robot.Utilities;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateAboutTheCenterOfMassOfTheRobotToPointTowardsFlavortown extends CommandBase {

  final static double kDefaultTimeout = 2;
  final static double kMaxRotation = .75;
  double timeout;
  double starttime;

  boolean shouldStopFirstCycle;
  boolean isFirstCycle;

  private static DrivetrainSubsystem _drive;
  private PidController _pidController = new PidController(new PidConstants(0.013, 0, 0.0008));
  private double _currentTime;
  private double kAcceptableError = 0.5;

  public RotateAboutTheCenterOfMassOfTheRobotToPointTowardsFlavortown(DrivetrainSubsystem drive) {
    // Use requires() here to declare subsystem dependencies
    // eg. requires(chassis);
    _drive = drive;
    addRequirements(_drive);
    _pidController.setContinuous(true);
    _pidController.setInputRange(-180, 180);
    _pidController.setOutputRange(-kMaxRotation, kMaxRotation);
    _pidController.setSetpoint(0);
  }
  
  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    starttime = Timer.getFPGATimestamp();
    _currentTime = Timer.getFPGATimestamp();
    isFirstCycle = true;
    shouldStopFirstCycle = !Limelight.getInstance().getHasTarget();
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    if (isFirstCycle){
      isFirstCycle = false;
    }
    double minSS = DrivetrainSubsystem.getInstance().getMinControllerSpeed();
    double additionalSS =  Robot.getRobotContainer().getPrimaryRightTrigger();
    double speedScale = minSS + (1 - minSS) * additionalSS * additionalSS;

    double forward = -Robot.getRobotContainer().getPrimaryLeftYAxis();
    forward = Utilities.deadband(forward);
    // Square the forward stick
    forward = speedScale * Math.copySign(Math.pow(forward, 2.0), forward);

    double strafe = -Robot.getRobotContainer().getPrimaryLeftXAxis();
    strafe = Utilities.deadband(strafe);
    // Square the strafe stick
    strafe = speedScale * Math.copySign(Math.pow(strafe, 2.0), strafe);

    double localTime = Timer.getFPGATimestamp();
    double deltaTime = localTime - _currentTime;
    _currentTime = localTime;

    double err = -getMinAngleDiff(_drive.getGyroAngle().toDegrees(), Limelight.getInstance().locateFlavortownUSA().toDegrees());
    double rot =  -_pidController.calculate(err, deltaTime);
    _drive.drive(new Translation2d(forward, strafe), rot, true); //because the angle is continually changing, it's assumed if you drive while calling this you intend to drive field oriented
    
    SmartDashboard.putNumber("AngleError", err);
    SmartDashboard.putNumber("Rotation Value", rot);
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    if (isFirstCycle && shouldStopFirstCycle){
      System.out.println("TRIED TO AIM WITHOUT VISION!!! STOPPING!!!");
      return shouldStopFirstCycle;
    }
    return (Math.abs(getMinAngleDiff(_drive.getGyroAngle().toDegrees(), Limelight.getInstance().locateFlavortownUSA().toDegrees())) < kAcceptableError);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    _drive.stop();
  }

  private double getMinAngleDiff(double a1, double a2){
    double rawDiff = Math.toRadians(a1 - a2);
    double c = Math.cos(rawDiff);
    double s = Math.sin(rawDiff);
    double minDiff = Math.atan2(s, c);
    return Math.toDegrees(minDiff);
  }
}
