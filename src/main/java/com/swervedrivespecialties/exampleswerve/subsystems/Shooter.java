/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.subsystems.Limelight.Target;
import com.swervedrivespecialties.exampleswerve.util.BoundedServo;
import com.swervedrivespecialties.exampleswerve.util.BeakSubsystem;
import com.swervedrivespecialties.exampleswerve.util.LogDataBE;
import com.swervedrivespecialties.exampleswerve.util.ShooterTable;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase{

    private double kMaxSpeed = 5440.0; //Native Units
    private double kShooterTolerance = 20;

    private static final double kServoHome = .3;
    private static final double kServoHomeEpsilon = 0.02;
    private static final double kShooterDistanceDelta = 1 * 12;
    private static final double kShooterDefaultDistance = 27 * 12; 
    private static final double kMinKickerVBus = .7;
    private static final int kNumShooterTableDecimalPlaces = 1;

    private static final double kBackKickerVBus = .95;
    private static final double kBackShooterVBus = -.95;
    
    private static final double kAutoShotDistance = 14 * 12;

    private boolean hasHadOdometry;

    private boolean canShoot = false;


    private static final double kServoLowerLimit = .3;
    private static final double kServoUpperLimit = .55;

    private boolean isAlternateShot = false;
    public boolean isShooting = false;
    public boolean isSensorDistanceLocked = false;

    //4880 12.5
    //5040 fresh 

    private static Shooter _instance = new Shooter();
    private static Limelight _ll = Limelight.getInstance();
    private static DrivetrainSubsystem _dt = DrivetrainSubsystem.getInstance();
    private SwerveDriveOdometry curOdom;

    private static final ShooterTable primaryTable = ShooterTable.getPrimaryTable();
    private static final ShooterTable secondaryTable = ShooterTable.getSecondaryTable();

    private double _shooterShootDistance;
    private double _shooterSensorDistance = 0;
    private double _shooterDistanceOffset = 0;

    public static Shooter getInstance(){
        return _instance;
    }

    private TalonSRX _kickerTalon = new TalonSRX(RobotMap.KICKER_TALON);
    private CANSparkMax _shooterNEO = new CANSparkMax(RobotMap.SHOOTER_MASTER_NEO, MotorType.kBrushless);
    private CANSparkMax _shooterSlave = new CANSparkMax(RobotMap.SHOOTER_SLAVE_NEO, MotorType.kBrushless);
    //private BoundedServo _linearActuator = new BoundedServo(0, kServoLowerLimit, kServoUpperLimit);

    private Servo _linearActuator = new Servo(0);
    private CANPIDController _pidController;
    private CANEncoder _encoder;
    private double _P = 0.000505; //.00045 //0.0005
    private double _I = 0;
    private double _D = 0.000053; //.000045 //0.000053
    private double _F = 0.00019656543; // 00019656543
    
    private double minOutput = -1;
    private double maxOutput = 1;
    private int _MtrTargetRPM;
   
    private Shooter(){

        _shooterNEO.restoreFactoryDefaults();
        _shooterSlave.restoreFactoryDefaults();
        _shooterNEO.setIdleMode(IdleMode.kCoast);
        _shooterNEO.setIdleMode(IdleMode.kCoast);


        _shooterNEO.setInverted(true);
        _shooterSlave.setInverted(false);
        _shooterSlave.follow(_shooterNEO, true);
        _encoder = _shooterNEO.getEncoder();
        _pidController = new CANPIDController(_shooterNEO);

        //_shooterSlave.follow(_shooterNEO, true);

        _pidController.setP(_P);
        _pidController.setI(_I);
        _pidController.setD(_D);
        _pidController.setFF(_F);
        _pidController.setOutputRange(minOutput, maxOutput);

        hasHadOdometry = false;
        
    } 

    public void runShooter(Shot s){
        double spd = s.speed;
        double actuatorVal = s.actuatorPosition;
        SmartDashboard.putNumber("spd", spd);
        SmartDashboard.putNumber("Target RPM", spd);
        SmartDashboard.putNumber("velo", _encoder.getVelocity());
        SmartDashboard.putNumber("Vello", _encoder.getVelocity());
        SmartDashboard.putNumber("ActuatorVal", actuatorVal); 
        double talonSpeed = spd > 0 ? 1 * (kMinKickerVBus + 0 * (1 - kMaxSpeed) *  (spd / kMaxSpeed)): 0.0;
        _kickerTalon.set(ControlMode.PercentOutput, -talonSpeed);
        if (spd > kShooterTolerance){
            _pidController.setReference(spd, ControlType.kVelocity);
            System.out.println(_shooterNEO.getOutputCurrent());
        } else {
            _shooterNEO.set(0.0);
        }
        System.out.println("actuator Val"+actuatorVal);
        _linearActuator.set(actuatorVal);
    }

    public void bangShooter(boolean go){
        _shooterNEO.set(go ? 1.0 : 0.0);
    }

    public void backKicker(){
        _kickerTalon.set(ControlMode.PercentOutput, kBackKickerVBus);
    }

    public void backShooter(){
        _shooterNEO.set(kBackShooterVBus);
    }

    public void stopKicker(){
        _kickerTalon.set(ControlMode.PercentOutput, 0.0);
    }

    public void stopShooter(){
        _shooterNEO.set(0.0);
    }

    private String getFormattedDistanceStr(double val){
        return util.getRoundedString(val/12, kNumShooterTableDecimalPlaces);
    }

    public void outputToSDB(){
        SmartDashboard.putBoolean("Is At Speed", canShoot);
        SmartDashboard.putString("Shooter Sensor Distance", getFormattedDistanceStr(_shooterSensorDistance));
        SmartDashboard.putString("Shooter Offset", getFormattedDistanceStr(_shooterDistanceOffset));
        SmartDashboard.putString("Shot Distance", getFormattedDistanceStr(_shooterShootDistance));
        SmartDashboard.putString("Target RPM", getFormattedDistanceStr(getShot().speed));
        SmartDashboard.putString("Actuator Value", Double.toString(getShot().actuatorPosition));
        SmartDashboard.putString("RPM", getFormattedDistanceStr(_encoder.getVelocity()));
        SmartDashboard.putBoolean("Is Normal Shot", !isAlternateShot);
    }

    public Shot getShot(){
        ShooterTable curTable = isAlternateShot ? secondaryTable : primaryTable;
        return curTable.CalcShooterValues(_shooterShootDistance/12).getShot();
    }

    public void resetServo(){
        _linearActuator.set(kServoHome);
    }

    public boolean isServoReset(){
        return Math.abs(_linearActuator.get() - kServoHome) <= kServoHomeEpsilon;
    }

    public void analogRunKicker(double v){
        _kickerTalon.set(ControlMode.PercentOutput, v);
    }

    public void updateLogData(LogDataBE logData){  
        logData.AddData("Is Shooting", Boolean.toString(isShooting));
        logData.AddData("Shooter Velo", Double.toString(_encoder.getVelocity()));
        logData.AddData("Shooter Target", Double.toString(getShot().speed));
        logData.AddData("Shooter Applied Output", Double.toString(_shooterNEO.getAppliedOutput()));
        logData.AddData("Shooter Current", Double.toString(_shooterNEO.getOutputCurrent()));
        logData.AddData("Shooter Tenperature", Double.toString(_shooterNEO.getMotorTemperature()));
        logData.AddData("Shooter Distance", Double.toString(_shooterShootDistance));
        logData.AddData("Shooter Sensor Distance", Double.toString(_shooterSensorDistance));
        logData.AddData("Shooter Distance Offset", Double.toString(_shooterDistanceOffset));
    }

    public void teleopInit(){
    }

    public void updateShooterDistance(){
        updateSensorDistance();
        _shooterShootDistance = _shooterSensorDistance + _shooterDistanceOffset;
    }

    public void setAutoShooterDistance(){
        _shooterDistanceOffset = kAutoShotDistance;
    }

    public void incrementShooterDistance(){
        _shooterDistanceOffset += kShooterDistanceDelta;
    }

    public void decrementShooterDistance(){
        _shooterDistanceOffset -= kShooterDistanceDelta;
    }

    public void resetShooterDistance(){
        _shooterDistanceOffset = kShooterDefaultDistance - _shooterSensorDistance;
    }

    public void clearShooterDistanceOffset(){
        _shooterDistanceOffset = 0;
    }

    private void updateSensorDistance(){
        if (!isSensorDistanceLocked){
            if (_ll.hasOdom()){
                hasHadOdometry = true;
                curOdom = _dt.getShooterOdometry(_ll.getTargetToBot());
                _shooterSensorDistance = _ll.offsetLLDist(_ll.getDistanceToTarget(Target.HIGH));
            } else if (_ll.hasRange()){
                if (hasHadOdometry){
                    _dt.updateShooterOdometry(curOdom);
                }
                _shooterSensorDistance = _ll.offsetLLDist(_ll.getDistanceToTarget(Target.HIGH));
            } else if (hasHadOdometry){
                _dt.updateShooterOdometry(curOdom);
                _shooterSensorDistance = util.metersToInches(curOdom.getPoseMeters().getTranslation().getNorm());
            }
        }
        SmartDashboard.putBoolean("Limelight HasR", _ll.hasRange());
    }

    public double getShooterSensorDistance(){
        return _shooterSensorDistance;
    }
    
    public void toggleIsAlternateShot(){
        isAlternateShot = !isAlternateShot;
    }

    public static class Shot{
        public double speed;
        public double actuatorPosition;

        public Shot(double spd, double actPos){
            speed = spd;
            actuatorPosition = actPos;
        }

        public static Shot zeroShot = new Shot(0, kServoHome);

        public static Shot getStopShot(){
            return new Shot(0, getInstance()._linearActuator.get());
        }
    }

    public double getSpeed(){
        return _encoder.getVelocity();
    }

    public void updateCanShoot(boolean can){
        canShoot = can;
    }

    public boolean getSpedUp(){
        return canShoot;
    }

    @Override
    public void periodic()
    {
        updateShooterDistance();
    }
}