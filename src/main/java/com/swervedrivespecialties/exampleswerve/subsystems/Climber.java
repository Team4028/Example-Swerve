/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.swervedrivespecialties.exampleswerve.RobotMap;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase{
    VictorSPX climbVictor;
    DoubleSolenoid climbSolenoid = new DoubleSolenoid(2, 3);

    double kClimberNominalVBus = .10;
    double kClimberDeadband = .05;
    Value kClimberSolenoidDefault = Value.kForward;


    private Climber(){
        climbVictor = new VictorSPX(RobotMap.CLIMBER_MOTOR);
    }

    private static Climber instance = new Climber();

    public static Climber getInstance(){
        return instance;
    }

    public void run(double spd){
        climbVictor.set(ControlMode.PercentOutput, Math.abs(spd) > kClimberDeadband ? (spd - kClimberDeadband) * kClimberNominalVBus : 0.0);
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
}