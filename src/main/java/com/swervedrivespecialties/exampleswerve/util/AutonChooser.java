/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import com.swervedrivespecialties.exampleswerve.commands.auton.autons.opponentball.DoNothing;
import com.swervedrivespecialties.exampleswerve.commands.auton.autons.opponentball.OpponentBallAuton;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class AutonChooser {

    private SendableChooser<AUTONS> autonChooser = new SendableChooser<>();

    private static AutonChooser instance = new AutonChooser();
    
    public static AutonChooser getInstance(){
        return instance;
    }

    private enum AUTONS{
        DO_NOTHING,
        STEAL_BALLS, 
        OWN_TRENCH
    }

    private AutonChooser(){
        autonChooser.setDefaultOption("Do Nothing", AUTONS.DO_NOTHING);
        autonChooser.addOption("Steal Balls", AUTONS.STEAL_BALLS);
        autonChooser.addOption("Run And Gun", AUTONS.OWN_TRENCH);
    }

    public CommandBase getAuton(){
        if (autonChooser.getSelected() == AUTONS.STEAL_BALLS){
            return new OpponentBallAuton();
        } else if (autonChooser.getSelected() == AUTONS.OWN_TRENCH) {
            return new DoNothing();
        } else {
            return new DoNothing();
        }
    }

    public void outputToSDB(){
        SmartDashboard.putData("Auton", autonChooser);
        SmartDashboard.putString("AutonChoosers:AutonAction", autonChooser.getSelected().toString());
    }
}