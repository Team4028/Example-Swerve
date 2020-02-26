package com.swervedrivespecialties.exampleswerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.auton.Trajectories;
import com.swervedrivespecialties.exampleswerve.commands.drive.DriveSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.subsystems.DrivetrainSubsystem;
import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;

import org.frcteam2910.common.math.Vector2;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot{

    static RobotContainer robotContainer;
    static Infeed _infeed = Infeed.get_instance();
    Shooter _shooter = Shooter.getInstance();

    Compressor _compressor = new Compressor(0);

    //CANSparkMax spark = new CANSparkMax(0, MotorType.kBrushless);

    @Override
    public void robotInit() {
        Trajectories.generateAllTrajectories();
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        robotContainer.outputToSDB();
        if (!isDisabled()){
            robotContainer.logAllData();
        }
        SmartDashboard.putNumber("NavX", DrivetrainSubsystem.getInstance().getGyroAngle().toDegrees());
        SmartDashboard.putNumber("Distance LL", Limelight.getInstance().getDistanceToTarget(Target.HIGH));
    }


    @Override
    public void autonomousInit() {
        robotContainer.setupLogging(true);
        robotContainer.configureDrive();
       RobotContainer.configureInfeed();
        //robotContainer.getAuton().schedule();
        robotContainer.configureClimber();
    }

    @Override
    public void autonomousPeriodic() {
        DriveSubsystemCommands.getLineDriveCommand(new Vector2(120, 0), true).schedule();
    }

    @Override
    public void teleopInit() {
        robotContainer.setupLogging(false);
        robotContainer.configureClimber();
        _shooter.runShooter(Shooter.Shot.getStopShot());
        CommandScheduler.getInstance().cancelAll();
        //RobotContainer.configureInfeed();
        robotContainer.configureDrive();
    }
    @Override
    public void teleopPeriodic(){
    }

    public static RobotContainer getRobotContainer(){
        return robotContainer;
    }
}
