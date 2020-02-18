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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Infeed extends SubsystemBase {

  public static final double kEncoderCountsPerBall = 7500;
  private static final double kConveyorTalonConstantVBus = -0.35;
  private static final double kConveyToShootConstantVBUS = -.7;
  private static final double kInfeedVBus = .5;
  private static final double kSingulatorVBus = -.45;
  private static final double kSingulateToShootVBus = -.5;

  private static final boolean kPreConveyorNormal = false;
  private static final boolean kPreShooterNormal = false;
  private static final boolean kPostSingulatorNormal = false;

  private static Infeed _instance = new Infeed();

  public static Infeed get_instance() {
    return _instance;
  }

  private TalonSRX _conveyorTalon;
  private DigitalInput _preConveyorSensor;
  private DigitalInput _preShooterSensor;
  private DigitalInput _midConveyorSensor;
  private boolean _preConveyorSensorLastCycle;
  private boolean _preConveyorSensorThisCycle;
  private boolean _midConveyorSensorLastCycle;
  private boolean _midConveyorSensorThisCycle;
  private boolean _isFirstCycle;

  private final Value SOLENOID_OUT_POSITION = Value.kReverse;
  private final Value SOLENOID_UP_POSITION = SOLENOID_OUT_POSITION == Value.kReverse ? Value.kForward : Value.kReverse;

  private TalonSRX _singulatorTalon;
  private VictorSPX _infeedVictor;
  private DigitalInput _postSingulatorSensor;
  private DoubleSolenoid _infeedSolenoid;

  /**
   * Creates a new Infeed.
   */
  private Infeed() {
    _conveyorTalon = new TalonSRX(RobotMap.CONVEYOR_MOTOR);
    _conveyorTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    _conveyorTalon.setNeutralMode(NeutralMode.Brake);

    _singulatorTalon = new TalonSRX(RobotMap.SINGULATOR_MOTOR);
    _infeedVictor = new VictorSPX(RobotMap.INFEED_MOTOR);
    _preConveyorSensor = new DigitalInput(RobotMap.CONVEYOR_SENSOR);
    _midConveyorSensor = new DigitalInput(RobotMap.PRE_CONVEYOR_SENSOR);
    _preShooterSensor = new DigitalInput(RobotMap.PRE_SHOOTER_SENSOR);
    _postSingulatorSensor = new DigitalInput(RobotMap.POST_SINGULATOR_SENSOR);
    _infeedSolenoid = new DoubleSolenoid(0, 1);
    _infeedSolenoid.set(SOLENOID_OUT_POSITION);
  }

  public void zeroEcnoder(){
    _conveyorTalon.setSelectedSensorPosition(0);
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
    SmartDashboard.putBoolean("PRE-CONVEYOR SENSOR", _preConveyorSensor.get());
    SmartDashboard.putBoolean("INFEED SOLENOID OUT JIMBO", getIsSolenoidOut());
    System.out.println(_infeedSolenoid.get());
  }

  public boolean getHasBallConveyedBallLength(){
    return midConveyorSensorPressed();
  }

  public boolean preConveyorSensorPressed() {
    if (_isFirstCycle) {
      _preConveyorSensorThisCycle = _preConveyorSensor.get();
      _isFirstCycle = false;
      return false;
    } else {
      _preConveyorSensorLastCycle = _preConveyorSensorThisCycle;
      _preConveyorSensorThisCycle = _preConveyorSensor.get();
      return !_preConveyorSensorThisCycle && _preConveyorSensorLastCycle;
    }
  }

  public boolean midConveyorSensorPressed() {
      _midConveyorSensorLastCycle = _midConveyorSensorThisCycle;
      _midConveyorSensorThisCycle = _midConveyorSensor.get();
      return !_midConveyorSensorThisCycle && _midConveyorSensorLastCycle;
  }
 
  public boolean getMidConveyorSensor() {
    return _midConveyorSensor.get() == kPreConveyorNormal;
  }

  public boolean getPreConveyorSensor(){
    return _preConveyorSensor.get() == kPreConveyorNormal;
  }

  public boolean getPreShooterSensor() {
    return _preShooterSensor.get() == kPreShooterNormal;
  }

  public boolean getPostSingulatorSensor() {
    return _postSingulatorSensor.get() == kPostSingulatorNormal;
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


  public void setSolenoidOut(boolean out) {
    Value setVal = out ? SOLENOID_OUT_POSITION : SOLENOID_UP_POSITION;
    _infeedSolenoid.set(setVal);
    System.out.println("Solenoid set value: " + setVal);
  }

  public boolean getIsSolenoidOut(){
    return _infeedSolenoid.get() == SOLENOID_OUT_POSITION;
  }

  @Override
  public void periodic() {
    if (preConveyorSensorPressed()) {
      CommandBase conveyorCommand = InfeedSubsystemCommands.getConveyCommand();
      conveyorCommand.schedule();
    }
  }
}
