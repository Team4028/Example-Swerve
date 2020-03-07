/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.swervedrivespecialties.exampleswerve.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    VictorSPX climbVictor;
    DoubleSolenoid climbSolenoid = new DoubleSolenoid(2, 3);

    double kClimberYeetVBus = -.65; 
    double kClimberStandardVBus = -.15;
    double kClimberDeadband = .05;
    double kClimberYeetDeadband = .85;
    Value kClimberSolenoidDefault = Value.kForward;

    VictorSPX _infeedVictor;
    //private TalonSRX _TestTalon;


    private Climber(){
        climbVictor = new VictorSPX(RobotMap.CLIMBER_MOTOR);
        _infeedVictor = new VictorSPX(RobotMap.INFEED_MOTOR);
        //_TestTalon = new TalonSRX(2);
    }

    private static Climber instance = new Climber();

    public static Climber getInstance(){
        return instance;
    }

    public void run(double spd){
        climbVictor.set(ControlMode.PercentOutput, climbSolenoid.get() != kClimberSolenoidDefault ? getClimbVBus(spd) : 0);
    }

    private double getClimbVBus(double spd){
        spd *= -1;
        if (spd < kClimberDeadband){
            return 0;
        } else if (spd < kClimberYeetDeadband) {
            return kClimberStandardVBus;
        } else {
            return kClimberYeetDeadband;
        }
    }

    public void resetClimbSolenoid(){
        climbSolenoid.set(kClimberSolenoidDefault);
    }

    public void toggleClimbSolenoid(){
        if (climbSolenoid.get() == Value.kForward){
            climbSolenoid.set(Value.kReverse);
        } else if (climbSolenoid.get() == Value.kReverse){
            climbSolenoid.set(Value.kForward);
        }
    }

    public void GondolaVbus(){
        _infeedVictor.set(ControlMode.PercentOutput, 0.6);
        //_TestTalon.set(ControlMode.PercentOutput, 0.6);
    }
    public void GondolaNegVbus(){
        _infeedVictor.set(ControlMode.PercentOutput, -0.6);
        //_TestTalon.set(ControlMode.PercentOutput, -0.6);
    }
    public void GondolaStop(){
        _infeedVictor.set(ControlMode.PercentOutput, 0);
        //_TestTalon.set(ControlMode.PercentOutput, 0);
    }
}