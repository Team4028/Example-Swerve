/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class Chameleon implements Subsystem {
  
  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("chameleon-vision").getSubTable("CellCam");
  private NetworkTableEntry _theta = nt.getEntry("targetYaw");
  private static Chameleon _instance = new Chameleon();
  public static Chameleon getInstance(){
    return _instance;
  }

  public Chameleon() {}

  public double getAngle1(){
    return _theta.getDouble(0.0);
  }
  public double getDistanceToPowerCell(){
    double distance = 0;
    return distance;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
