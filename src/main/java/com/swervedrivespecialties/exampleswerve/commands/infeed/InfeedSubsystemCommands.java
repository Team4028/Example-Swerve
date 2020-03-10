/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.commands.infeed;

import com.swervedrivespecialties.exampleswerve.subsystems.Infeed;
import com.swervedrivespecialties.exampleswerve.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

/**
 * Add your docs here.
 */
public class InfeedSubsystemCommands {
    public static Infeed infeed = Infeed.get_instance();

    
    public static CommandBase getConveyToShootCommand(){
        return new conveyToShoot(infeed);
    }

    public static CommandBase getConveyCommand(){
        return new convey(infeed);
    }  

    public static CommandBase getRunInfeedCommand(){
        return new runInfeed(infeed);
    }

    public static CommandBase getRunSingulatorCommand(){
        return new runSingulator(infeed);
    }

    public static CommandBase getToggleInfeedSolenoidCommand(){
        return new ToggleInfeedSolenoid(infeed);
    }
    public static CommandBase getYeetIntake(){
        return new YeetIntake(infeed);
    }
    public static CommandBase getAutonInfeedCommand(double ifTime, double singTime){
        return new ParallelCommandGroup(getRunInfeedCommand().withTimeout(ifTime), getRunSingulatorCommand().withTimeout(singTime));
    }
    public static CommandBase getSwitchCameraCommand(){
        return new SwitchCamera();
    }

    public static CommandBase getYeetSingulatorCommand(){
        return new YeetSingulator(infeed);
    }

    public static CommandBase getResetInfeedCommand(){
        return new ResetInfeed(infeed);
    }

    public static CommandBase getAnalogBackKickerCommand(){
        return new AnalogBackKicker(Shooter.getInstance());
    }

    public static CommandBase getBackConveyorFixedComand(){
        return new BackConveyorFixed(infeed);
    }

    public static CommandBase getBumpConveyorForwardCommand(){
        return new BumpConveyorForward(infeed);
    }

    public static CommandBase BACK_INFEED = new BackInfeed(infeed);
    public static CommandBase BACK_SINGULATOR = new BackSingulator(infeed);
    public static CommandBase BACK_CONVEYOR = new BackConveyor(infeed);
    public static CommandBase CONVEY_TO_SHOOT = new conveyToShoot(infeed);
}
