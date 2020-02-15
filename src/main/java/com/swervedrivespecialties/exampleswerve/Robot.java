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
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot{

    static RobotContainer robotContainer;
    static Infeed _infeed = Infeed.get_instance();
    Shooter _shooter = Shooter.getInstance();

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
        RobotContainer.configureInfeed();
    }

    @Override
    public void autonomousPeriodic() {
       // DrivetrainSubsystem.getInstance().drive(new Translation2d(1., 0), 0, true);
    }

    @Override
    public void teleopInit() {
        robotContainer.setupLogging(false);
        _shooter.runShooter(Shooter.Shot.getStopShot());
        CommandScheduler.getInstance().cancelAll();
        RobotContainer.configureInfeed();
    }
    @Override
    public void teleopPeriodic(){
    }

    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
