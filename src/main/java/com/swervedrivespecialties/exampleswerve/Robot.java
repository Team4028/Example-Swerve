package com.swervedrivespecialties.exampleswerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot{

    static RobotContainer robotContainer;
    Compressor _compressor = new Compressor(0);

    @Override
    public void robotInit() {
        Trajectories.generateAllTrajectories();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        if (!isDisabled()){
            robotContainer.logAllData();
            robotContainer.outputToSDB();
        }
    }

    @Override
    public void autonomousInit() {
        robotContainer.setupLogging(true);
        robotContainer.configureDrive();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        robotContainer.setupLogging(false);
        CommandScheduler.getInstance().cancelAll();
    }
    @Override
    public void teleopPeriodic(){
    }

    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
