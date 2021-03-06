package com.swervedrivespecialties.exampleswerve.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMax.IdleMode;
import com.swervedrivespecialties.exampleswerve.Robot;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.util.LogDataBE;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.frcteam2910.common.control.PidConstants;
import org.frcteam2910.common.drivers.Gyroscope;
import org.frcteam2910.common.drivers.SwerveModule;
import org.frcteam2910.common.math.Rotation2;
import org.frcteam2910.common.math.Vector2;
import org.frcteam2910.common.robot.Utilities;
import org.frcteam2910.common.robot.drivers.Mk2SwerveModuleBuilder;
import org.frcteam2910.common.robot.drivers.NavX;

public class DrivetrainSubsystem implements Subsystem {
    private static final double TRACKWIDTH = 22.5;
    private static final double WHEELBASE = 24.41;

    private static final double FRONT_LEFT_ANGLE_OFFSET = -Math.toRadians(248.4);//224.4); //211.770524);//278.06 Practice
    private static final double FRONT_RIGHT_ANGLE_OFFSET = -Math.toRadians(248.2);//229.6);//250.524496);//45.48
    private static final double BACK_LEFT_ANGLE_OFFSET = -Math.toRadians(115.0);//93.51
    private static final double BACK_RIGHT_ANGLE_OFFSET = -Math.toRadians(55.683811);//300.77

    private static final PidConstants ANGLE_CONSTANTS = new PidConstants(1.5, 0.0, .5);
    private static final double ANGLE_REDUCTION = 18.0 / 1.0;
    private static final double WHEEL_DIAMETER = 4;
    private static final double DRIVE_REDUCTION = 6.92 / 1.0;

    boolean isFieldOriented = true;
    double curMinControllerSpeed = .25;

    private static DrivetrainSubsystem instance;

    CANSparkMax frontLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backLeftDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backRightDrive = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontLeftSteer = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax frontRightSteer = new CANSparkMax(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backLeftSteer = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax backRightSteer = new CANSparkMax(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    private final SwerveModule frontLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER), FRONT_LEFT_ANGLE_OFFSET)
            .angleMotor(frontLeftSteer, ANGLE_CONSTANTS, ANGLE_REDUCTION)
            .driveMotor(frontLeftDrive , DRIVE_REDUCTION, WHEEL_DIAMETER)
            .build();
    private final SwerveModule frontRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER), FRONT_RIGHT_ANGLE_OFFSET)
            .angleMotor(frontRightSteer, ANGLE_CONSTANTS, ANGLE_REDUCTION)
            .driveMotor(frontRightDrive, DRIVE_REDUCTION, WHEEL_DIAMETER)
            .build();
    private final SwerveModule backLeftModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER), BACK_LEFT_ANGLE_OFFSET)
            .angleMotor(backLeftSteer, ANGLE_CONSTANTS, ANGLE_REDUCTION)
            .driveMotor(backLeftDrive, DRIVE_REDUCTION, WHEEL_DIAMETER)
            .build();
    private final SwerveModule backRightModule = new Mk2SwerveModuleBuilder(
            new Vector2(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0))
            .angleEncoder(new AnalogInput(RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER), BACK_RIGHT_ANGLE_OFFSET)
            .angleMotor(backRightSteer, ANGLE_CONSTANTS, ANGLE_REDUCTION)
            .driveMotor(backRightDrive, DRIVE_REDUCTION, WHEEL_DIAMETER)
            .build();

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(TRACKWIDTH / 2.0, -WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, WHEELBASE / 2.0),
            new Translation2d(-TRACKWIDTH / 2.0, -WHEELBASE / 2.0)
    );

    private final Gyroscope gyroscope = new NavX(SPI.Port.kMXP);

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroRotation(), new Pose2d());

    public DrivetrainSubsystem() {
        gyroscope.calibrate();
        gyroscope.setInverted(true); // You might not need to invert the gyro

        frontLeftModule.setName("Front Left");
        frontRightModule.setName("Front Right");
        backLeftModule.setName("Back Left");
        backRightModule.setName("Back Right");

        frontLeftDrive.setIdleMode(IdleMode.kBrake);
        frontRightDrive.setIdleMode(IdleMode.kBrake);
        backLeftDrive.setIdleMode(IdleMode.kBrake);
        backRightDrive.setIdleMode(IdleMode.kBrake);
        frontLeftSteer.setIdleMode(IdleMode.kBrake);
        frontRightSteer.setIdleMode(IdleMode.kBrake);
        backLeftSteer.setIdleMode(IdleMode.kBrake);
        backRightSteer.setIdleMode(IdleMode.kBrake);
    }

    public static DrivetrainSubsystem getInstance() {
        if (instance == null) {
            instance = new DrivetrainSubsystem();
        }

        return instance;
    }

    public void updateOdometry(){
        odometry.update(getGyroRotation(), getModuleStates());
    }

    @Override
    public void periodic() {
        frontLeftModule.updateSensors();
        frontRightModule.updateSensors();
        backLeftModule.updateSensors();
        backRightModule.updateSensors();

        SmartDashboard.putNumber("Front Left Module Angle", Math.toDegrees(frontLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Front Right Module Angle", Math.toDegrees(frontRightModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Left Module Angle", Math.toDegrees(backLeftModule.getCurrentAngle()));
        SmartDashboard.putNumber("Back Right Module Angle", Math.toDegrees(backRightModule.getCurrentAngle()));

        updateOdometry();

        SmartDashboard.putNumber("Gyroscope Angle", gyroscope.getAngle().toDegrees());

        frontLeftModule.updateState(TimedRobot.kDefaultPeriod);
        frontRightModule.updateState(TimedRobot.kDefaultPeriod);
        backLeftModule.updateState(TimedRobot.kDefaultPeriod);
        backRightModule.updateState(TimedRobot.kDefaultPeriod);
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented) {
        rotation *= 2.0 / Math.hypot(WHEELBASE, TRACKWIDTH);
        ChassisSpeeds speeds;
        if (fieldOriented) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation,
                    Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees()));
        } else {
            speeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        }

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);

        frontLeftModule.setTargetVelocity(states[0].speedMetersPerSecond, states[0].angle.getRadians());
        //System.out.println(states[0].angle.getRadians() + " : " + frontLeftModule.getCurrentAngle());
        frontRightModule.setTargetVelocity(states[1].speedMetersPerSecond, states[1].angle.getRadians());
        backLeftModule.setTargetVelocity(states[2].speedMetersPerSecond, states[2].angle.getRadians());
        backRightModule.setTargetVelocity(states[3].speedMetersPerSecond, states[3].angle.getRadians());
    }

    public void resetGyroscope() {
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle().rotateBy(Rotation2.fromDegrees(180)));
    }

    public void customZeroGyro(){
        gyroscope.setAdjustmentAngle(gyroscope.getUnadjustedAngle());
    }

    public void toggleMinControllerSpeed(){
        if (curMinControllerSpeed < 1){
            curMinControllerSpeed += .25;
        } else {
            curMinControllerSpeed = .25;
        }
    }

    public double getMinControllerSpeed(){
        return curMinControllerSpeed;
    }

    public void resetMinControllerSpeed(){
        curMinControllerSpeed = .25;
    }

    public void setFieldOriented(boolean isFO){
        isFieldOriented = isFO;
    }

    public boolean getFieldOriented(){
        return isFieldOriented;
    }

    public void toggleFieldOriented(){
        setFieldOriented(!getFieldOriented());
    }

    public void outPutToSDB(){
        SmartDashboard.putNumber("FL", frontLeftModule.getCurrentAngle());
        SmartDashboard.putNumber("FR", frontRightModule.getCurrentAngle());
        SmartDashboard.putNumber("BL", backLeftModule.getCurrentAngle());
        SmartDashboard.putNumber("BR", backRightModule.getCurrentAngle());
        SmartDashboard.putNumber("Kinematic Position X", getKinematicPosition().x);
        SmartDashboard.putNumber("Kinematic Position Y", getKinematicPosition().y);
        SmartDashboard.putNumber("Kinematics Theta", getGyroAngle().toDegrees());
    }

    public Rotation2d getGyroRotation(){
        return Rotation2d.fromDegrees(gyroscope.getAngle().toDegrees());
    }

    private SwerveModuleState getCurrentState(SwerveModule mod){
        double velo = util.inchesToMeters(mod.getCurrentVelocity());
        Rotation2d rot = Rotation2d.fromDegrees(Math.toDegrees(mod.getCurrentAngle()));
        return new SwerveModuleState(velo, rot);
    }

    private SwerveModuleState[] getModuleStates(){
        return new SwerveModuleState[] {getCurrentState(frontLeftModule),
                                        getCurrentState(frontRightModule), 
                                        getCurrentState(backLeftModule), 
                                        getCurrentState(backRightModule)};
    }

    public Vector2 getKinematicPosition(){
        Pose2d odPos = odometry.getPoseMeters();
        double x = util.metersToInches(odPos.getTranslation().getX());
        double y = util.metersToInches(odPos.getTranslation().getY());
        return new Vector2(x, y);
    }

    public Vector2 getKinematicVelocity(){
        ChassisSpeeds chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
        double x_dot = util.metersToInches(chassisSpeeds.vxMetersPerSecond);
        double y_dot = util.metersToInches(chassisSpeeds.vyMetersPerSecond);
        return new Vector2(x_dot, y_dot);
    }

    public double getGyroRate(){
        return gyroscope.getRate();
    }

    public Rotation2 getGyroAngle(){
        return gyroscope.getAngle();
    }

    public void reset(){
        odometry.resetPosition(new Pose2d(), getGyroRotation());
    }

    public void stop(){
        drive(new Translation2d(), 0.0, true);
    }

    public void updateLogData(LogDataBE logData){  
        //   logData.AddData("Forward Velocity", Double.toString(getKinematicVelocity().x));
        //   logData.AddData("Strafe Velocity", Double.toString(getKinematicVelocity().y));
        //   logData.AddData("Angular Velocity", Double.toString(gyroscope.getRate()));  
        logData.AddData("Velocity", Double.toString(getKinematicVelocity().length));
    }

    public void setCurrentLimit(int curLim){
        frontLeftDrive.setSmartCurrentLimit(curLim);
        frontRightDrive.setSmartCurrentLimit(curLim);
        backLeftDrive.setSmartCurrentLimit(curLim);
        backRightDrive.setSmartCurrentLimit(curLim);
    }

    public void setRapRate(double rate){
        frontLeftDrive.setOpenLoopRampRate(rate);
    }

    public SwerveDriveOdometry getShooterOdometry(Translation2d current){
        return new SwerveDriveOdometry(kinematics, getGyroRotation());
    }

    public void updateShooterOdometry(SwerveDriveOdometry odom){
        odom.update(getGyroRotation(), getModuleStates());
    }

    public Translation2d getDriveVec(){
        double minSS = getMinControllerSpeed();
        double additionalSS =  Robot.getRobotContainer().getPrimaryRightTrigger();
        double speedScale = minSS + (1 - minSS) * additionalSS * additionalSS;
    
        double forward = -Robot.getRobotContainer().getPrimaryLeftYAxis();
        forward = Utilities.deadband(forward);
        // Square the forward stick
        //the negative on the forward and strafe are because the zero convention our drivers use is 180 degrees from that of our auto. 
        //this and the Rotation by 180 in the Zero Gyro Command are enough to make the auto convention the thing we run our swerve off of, 
        //but let the drivers still drive in their convention just on top of this. 
        forward = -speedScale * Math.copySign(Math.pow(forward, 2.0), forward); 
    
        double strafe = -Robot.getRobotContainer().getPrimaryLeftXAxis();
        strafe = Utilities.deadband(strafe);
        // Square the strafe stick
        strafe = -speedScale * Math.copySign(Math.pow(strafe, 2.0), strafe);

        return new Translation2d(forward, strafe);
    }

    public void xDrive(){
        frontLeftModule.setTargetVelocity(0.0, Math.toRadians(45));
        frontRightModule.setTargetVelocity(0.0, Math.toRadians(-45));
        backLeftModule.setTargetVelocity(0.0, Math.toRadians(-45));
        backRightModule.setTargetVelocity(0.0, Math.toRadians(45));
    }
}
