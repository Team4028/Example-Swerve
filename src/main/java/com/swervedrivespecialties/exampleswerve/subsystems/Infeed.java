/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.swervedrivespecialties.exampleswerve.RobotMap;
import com.swervedrivespecialties.exampleswerve.commands.infeed.InfeedSubsystemCommands;
import com.swervedrivespecialties.exampleswerve.commands.infeed.YeetIntake;
import com.swervedrivespecialties.exampleswerve.util.LogDataBE;
import com.swervedrivespecialties.exampleswerve.util.util;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {

  public static final double kEncoderCountsPerBall = 7000;
  private static final double kConveyorTalonConstantVBus = -0.50;
  private static final double kConveyToShootConstantVBUS = -.5;
  private static final double kInfeedVBus = -.7;
  private static final double kSingulatorVBus = .45;
  private static final double kSingulateToShootVBus = .5;

  private static final boolean kPreConveyorNormal = true;
  private static final boolean kPreShooterNormal = true;
  private static final boolean kPostSingulatorNormal = true;
  private static final boolean kMidConveyorNormal = true;

  private static final boolean usesFourthPhotoEye = false;

  private static final double kBackInfeedVBus = .9;
  private static final double kBackSingulatorVBus = -.95;
  private static final double kBackConveyorVBus = .95;

  private static Infeed _instance = new Infeed();

  public static Infeed get_instance() {
    return _instance;
  }

  private TalonSRX _conveyorTalon;
  private DigitalInput _preConveyorSensor;
  private DigitalInput _preShooterSensor;
  private boolean _preConveyorSensorLastCycle;
  private boolean _preConveyorSensorThisCycle;
  private boolean _isFirstCycle;

  private final Value SOLENOID_OUT_POSITION = Value.kReverse;
  private final Value SOLENOID_UP_POSITION = SOLENOID_OUT_POSITION == Value.kReverse ? Value.kForward : Value.kReverse;

  private TalonSRX _singulatorTalon;
  private VictorSPX _infeedVictor;
  private DigitalInput _postSingulatorSensor;
  private DigitalInput _midConveyorSensor;
  private DoubleSolenoid _infeedSolenoid;
  private boolean hasStoppedSingulating = false;
  int numBallsConveyed = 0;

  /**
   * Creates a new Infeed.
   */
  private Infeed() {
    _conveyorTalon = new TalonSRX(RobotMap.CONVEYOR_MOTOR);
    _conveyorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    //_preConveyorSensor = new DigitalInput(RobotMap.PRE_CONVEYOR_SENSOR);
    _preShooterSensor = new DigitalInput(RobotMap.PRE_SHOOTER_SENSOR);
    _conveyorTalon.setNeutralMode(NeutralMode.Brake);

    _singulatorTalon = new TalonSRX(RobotMap.SINGULATOR_MOTOR);
    _infeedVictor = new VictorSPX(RobotMap.INFEED_MOTOR);
    _postSingulatorSensor = new DigitalInput(RobotMap.POST_SINGULATOR_SENSOR);
    _infeedSolenoid = new DoubleSolenoid(0, 1);
    _infeedSolenoid.set(SOLENOID_UP_POSITION);
    _midConveyorSensor = new DigitalInput(RobotMap.MID_CONVEYOR_SENSOR);
  }

  public void zeroEcnoder(){
    _conveyorTalon.setSelectedSensorPosition(0);
  }

  public double getConveyorPosiiton(){
    return _conveyorTalon.getSelectedSensorPosition();
  }

  public void conveyConveyor(){
    _conveyorTalon.set(ControlMode.PercentOutput, kConveyorTalonConstantVBus);
  }

  public void conveyConveyorToShoot(){
    _conveyorTalon.set(ControlMode.PercentOutput, kConveyToShootConstantVBUS);
  }

  public void stopConveyor(){
    _conveyorTalon.set(ControlMode.PercentOutput, 0.0);
  }

  public void outputToSDB() {
    SmartDashboard.putBoolean("PRE-SHOOTER SENSOR", _preShooterSensor.get());
    SmartDashboard.putNumber("CONVEYOR TALON ENCODER", _conveyorTalon.getSelectedSensorPosition());
    SmartDashboard.putBoolean("PRE-CONVEYOR SENSOR", _postSingulatorSensor.get());
    SmartDashboard.putBoolean("POST-SINGULATOR", _postSingulatorSensor.get());
    SmartDashboard.putBoolean("MID-CONVEYOR", _midConveyorSensor.get());
    SmartDashboard.putBoolean("INFEED SOLENOID OUT JIMBO", getIsSolenoidOut());
    SmartDashboard.putNumber("Cell Count", numBallsConveyed + util.iversonBrackets(hasStoppedSingulating || getPostSingulatorSensor()));
  }

  public boolean getHasBallConveyedBallLength(){
    return _conveyorTalon.getSelectedSensorPosition() > kEncoderCountsPerBall;
  }

  public boolean preConveyorSensorPressed() {
    if (_isFirstCycle) {
      _preConveyorSensorThisCycle = getPreConveyorSensor();
      _isFirstCycle = false;
      return false;
    } else {
      _preConveyorSensorLastCycle = _preConveyorSensorThisCycle;
      _preConveyorSensorThisCycle = getPreConveyorSensor();
      return _preConveyorSensorThisCycle && !_preConveyorSensorLastCycle;
    }
  }

  public boolean getPreConveyorSensor(){
    return _postSingulatorSensor.get() != kPreConveyorNormal;
  }

  public boolean getPreShooterSensor() {
    return _preShooterSensor.get() != kPreShooterNormal;
  }

  public boolean getPostSingulatorSensor(){
    return _postSingulatorSensor.get() != kPostSingulatorNormal;
  }

  public void runInfeed(){
    _infeedVictor.set(ControlMode.PercentOutput, kInfeedVBus);
  }

  public void stopInfeed(){
    _infeedVictor.set(ControlMode.PercentOutput, 0.0);
  }

  public void runSingulator(){
    _singulatorTalon.set(ControlMode.PercentOutput, kSingulatorVBus);
  }

  public void runSingulatorToShoot(){
    _singulatorTalon.set(ControlMode.PercentOutput, kSingulateToShootVBus);
  }

  public void stopSingulator(){
    _singulatorTalon.set(ControlMode.PercentOutput, 0.0);
  }

  public void backConveyor(){
    _conveyorTalon.set(ControlMode.PercentOutput, kBackConveyorVBus);
  }

  public void backSingulator(){
    _singulatorTalon.set(ControlMode.PercentOutput, kBackSingulatorVBus);
  }

  public void backInfeed(){
    _infeedVictor.set(ControlMode.PercentOutput, kBackInfeedVBus);
  }


  public void setSolenoidOut(boolean out) {
    Value setVal = out ? SOLENOID_OUT_POSITION : SOLENOID_UP_POSITION;
    _infeedSolenoid.set(setVal);
    System.out.println("Solenoid set value: " + setVal);
  }

  public boolean getIsSolenoidOut(){
    return _infeedSolenoid.get() == SOLENOID_OUT_POSITION;
  }

  public void configInfeed(){
    _infeedSolenoid.set(SOLENOID_UP_POSITION);
    resetBallsConveyed();
  }

  public void resetBallsConveyed(){
    numBallsConveyed = 0;
  }

  private void updateHasStopSingulating(){
    if (!hasStoppedSingulating){
      hasStoppedSingulating = (numBallsConveyed >= 3 || getPreShooterSensor()) && getPostSingulatorSensor();
    }
  }

  public boolean getHasStoppedSingulating(){
    return hasStoppedSingulating;
  }

  public void resetHasStoppedSingulating(){
    hasStoppedSingulating = false;
  }

  public boolean getCanSingulate(){
    return !(getPostSingulatorSensor() && (getPreShooterSensor() || numBallsConveyed > 2)) && !hasStoppedSingulating; 
  }

  public boolean getMidConveyorSensor(){
    return _midConveyorSensor.get() != kMidConveyorNormal;
  }

  public boolean getHasFourthEye(){
    return usesFourthPhotoEye;
  }

  public void updatLogData(LogDataBE logData){
    logData.AddData("IS INFEED COMMAND RUNNING", Boolean.toString(CommandScheduler.getInstance().isScheduled(YeetIntake.ifCommand)));
    logData.AddData("IS SINGULATOR COMMAND RUNNING", Boolean.toString(CommandScheduler.getInstance().isScheduled(YeetIntake.sCommand)));
    logData.AddData("IS CONVEYOR COMMAND RUNNING", Boolean.toString(CommandScheduler.getInstance().isScheduled(conveyorCommand)));
    logData.AddData("HAS STOPPED SINGULATING", Boolean.toString(hasStoppedSingulating));
    logData.AddData("NUMBER BALLS CONVEYED", Integer.toString(numBallsConveyed));
    logData.AddData("SINGULATOR MOTOR COMMAND", Double.toString(_singulatorTalon.getMotorOutputPercent()));
    logData.AddData("CONVEYOR TALON COMMAND", Double.toString(_conveyorTalon.getMotorOutputPercent()));
  }

  private static final CommandBase conveyorCommand = InfeedSubsystemCommands.getConveyCommand().withTimeout(4);

  @Override
  public void periodic() {
    updateHasStopSingulating();
    if (preConveyorSensorPressed() && numBallsConveyed < 3) { 
      numBallsConveyed++;
      conveyorCommand.schedule();
    }
  }
}