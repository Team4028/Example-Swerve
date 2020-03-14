/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
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
  private static final double kConveyorTalonConstantVBus = -0.4;//-0.55; //-.5
  private static final double kConveyToShootConstantVBUS = -.9; //.8
  private static final double kInfeedVBus = -.7;
  private static final double kSingulatorVBus = .4; //.35 //.45
  private static final double kSingulateToShootVBus = .65; //.50 //.6

  private static final boolean kPreConveyorNormal = true;
  private static final boolean kPreShooterNormal = true;
  private static final boolean kPostSingulatorNormal = true;
  private static final boolean kMidConveyorNormal = true;

  private static final boolean usesFourthPhotoEye = true;

  private static final double kBackInfeedVBus = .9;
  private static final double kBackSingulatorVBus = -.95;
  private static final double kBackConveyorVBus = .95;

  private static final int kNumHandleConveyorLagCycles = 1;
  private static final double kHandleConveyorLagVbus = .4;

  public static final double kReverseBallOffKickerTime = 1.0;
  private static final double kReverseBallOffPreKickVBus = .15;

  public static final double kPostEyeEncoderTicks = -50.0;

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

  private final Value SOLENOID_OUT_POSITION = Value.kForward;
  private final Value SOLENOID_UP_POSITION = SOLENOID_OUT_POSITION == Value.kReverse ? Value.kForward : Value.kReverse;

  private TalonSRX _singulatorTalon;
  private VictorSPX _infeedVictor;
  private DigitalInput _postSingulatorSensor;
  private DigitalInput _midConveyorSensor;
  private DoubleSolenoid _infeedSolenoid;
  private boolean hasStoppedSingulating = false;
  private boolean isSingulatorHandlingConveyorLag = false;
  private int numHandingConveyorLagCyclesCompleted = 0;
  int numBallsConveyed = 0;

  /**
   * Creates a new Infeed.
   */
  private Infeed() {
    _conveyorTalon = new TalonSRX(RobotMap.CONVEYOR_MOTOR);
    _conveyorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _conveyorTalon.setInverted(true);
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
    SmartDashboard.putBoolean("Is Singulator Running", Math.abs(_singulatorTalon.getMotorOutputPercent()) > .03);
    SmartDashboard.putBoolean("Is Infeed Running", Math.abs(_infeedVictor.getMotorOutputPercent()) > .03);
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

  public void resetConveyorSensorPressed(){
    _preConveyorSensorLastCycle = false;
    _preConveyorSensorThisCycle = false;
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
    _infeedVictor.set(ControlMode.PercentOutput,  getIsSolenoidOut() ? kInfeedVBus : 0);
  }

  public void stopInfeed(){
    _infeedVictor.set(ControlMode.PercentOutput, 0.0);
  }

  public void runSingulator(){
    _singulatorTalon.set(ControlMode.PercentOutput, isSingulatorHandlingConveyorLag ? kHandleConveyorLagVbus : kSingulatorVBus);
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

  public void runConveyorReverse(){
    _conveyorTalon.set(ControlMode.PercentOutput, kReverseBallOffPreKickVBus);
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
    _infeedSolenoid.set(SOLENOID_OUT_POSITION);
    resetBallsConveyed();
  }

  public void resetBallsConveyed(){
    numBallsConveyed = 0;
    resetHasStoppedSingulating();
  }

  private void updateHasStopSingulating(){
    if (!hasStoppedSingulating){
      hasStoppedSingulating = ((numBallsConveyed > 2 && !cs.isScheduled(conveyorCommand)) || getPreShooterSensor()) && getPostSingulatorSensor();
    }
  }

  public boolean getHasStoppedSingulating(){
    return hasStoppedSingulating;
  }

  public void resetHasStoppedSingulating(){
    hasStoppedSingulating = false;
  }

  public boolean getCanSingulate(){
    return !(getPostSingulatorSensor() && (getPreShooterSensor() || (numBallsConveyed > 3 && !cs.isScheduled(conveyorCommand)))) && !hasStoppedSingulating; 
  }

  public boolean getMidConveyorSensor(){
    return _midConveyorSensor.get() != kMidConveyorNormal;
  }

  public boolean getHasFourthEye(){
    return usesFourthPhotoEye;
  }

  public void putHasStoppedSingulating(boolean putval){
    hasStoppedSingulating = putval;
  }

  public void startHandleConveyorLag(){
    isSingulatorHandlingConveyorLag = true;
    numHandingConveyorLagCyclesCompleted = 0;
  }

  public int getNumBallsConveyed(){
    return numBallsConveyed;
  }

  private void updateSingulatorHandlingConveyorLag(){
    if (isSingulatorHandlingConveyorLag){
      if (numHandingConveyorLagCyclesCompleted < kNumHandleConveyorLagCycles){
        numHandingConveyorLagCyclesCompleted++;
      } else {
        isSingulatorHandlingConveyorLag = false;
      }
    }
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
  CommandScheduler cs = CommandScheduler.getInstance();
  

  @Override
  public void periodic() {
    updateHasStopSingulating();
    updateSingulatorHandlingConveyorLag();
    if (preConveyorSensorPressed() && numBallsConveyed < 3 && !cs.isScheduled(conveyorCommand) && !cs.isScheduled(InfeedSubsystemCommands.CONVEY_TO_SHOOT)) { 
      numBallsConveyed++;
      conveyorCommand.schedule();
    }
  }
}