/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import com.swervedrivespecialties.exampleswerve.util.BeakXBoxController.DPADButton.Direction;

import org.frcteam2910.common.robot.input.DPadButton;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class BeakXBoxController {

    private static final double kDeadband = .05;

    private XboxController controller;

    public JoystickButton a;
    public JoystickButton b;
    public JoystickButton x;
    public JoystickButton y;
    public JoystickButton start;
    public JoystickButton back;
    public JoystickButton left_bumper;
    public JoystickButton right_bumper;
    public TriggerButton left_trigger_button;
    public TriggerButton right_trigger_button;
    public DPADButton dpad_up;
    public DPADButton dpad_down;
    public DPADButton dpad_left;
    public DPADButton dpad_right;

    public BeakXBoxController(int port){
        controller = new XboxController(port);
        a = new JoystickButton(controller, 1);
        b = new JoystickButton(controller, 2);
        x = new JoystickButton(controller, 3);
        y = new JoystickButton(controller, 4);
        left_bumper = new JoystickButton(controller, 5);
        right_bumper = new JoystickButton(controller, 6);
        back = new JoystickButton(controller, 7);
        start = new JoystickButton(controller, 8);
        left_trigger_button = new TriggerButton(controller, true);
        right_trigger_button = new TriggerButton(controller, false);
        dpad_down = new DPADButton(controller, Direction.DOWN);
        dpad_up = new DPADButton(controller, Direction.UP);
        dpad_left = new DPADButton(controller, Direction.LEFT);
        dpad_right = new DPADButton(controller, Direction.RIGHT);
    }



    public double getLeftXAxis(){
        return controller.getRawAxis(0);
    }

    public double getLeftYAxis(){
        return controller.getRawAxis(1);
    }

    public double getRighXAxis(){
        return controller.getRawAxis(4);

    }

    public double getRightYAxis(){
        return controller.getRawAxis(5);
    }

    public double getLeftTrigger(){
        return controller.getRawAxis(2);
    }

    public double getRightTrigger(){
        return controller.getRawAxis(3);
    }

    public static class TriggerButton extends Button{
        private XboxController tbController;
        private Hand hand;

        public TriggerButton(XboxController controller, boolean left){
            tbController = controller;
            hand = left ? Hand.kLeft : Hand.kRight;
        }

        @Override
        public boolean get() {
            return tbController.getTriggerAxis(hand) > kDeadband;
        }
    }

    public static class DPADButton extends Button{

        public enum Direction {
            UP (0), 
            RIGHT (90), 
            DOWN (180),
            LEFT (270);

            private final int angle;

            Direction(int angleVal){
                angle = angleVal;
            }

            public int getAngle(){
                return angle;
            }
        }

        XboxController dpbController;
        Direction direction;

        public DPADButton(XboxController controller, Direction dir){
            dpbController = controller;
            direction = dir;
        }        

        @Override 
        public boolean get(){
            return dpbController.getPOV() == direction.getAngle();
        }
    }
}
