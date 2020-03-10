/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.swervedrivespecialties.exampleswerve.util;

import java.util.Iterator;
import java.util.LinkedList;


/**
 * Add your docs here.
 */
public class ShooterTable {

    private static ShooterTable _primary = new ShooterTable(true);
    private static ShooterTable _secondary = new ShooterTable(false);
	
	public static ShooterTable getPrimaryTable() {
		return _primary;
    }

    public static ShooterTable getSecondaryTable(){
        return _secondary;
    }

    private int _indexCounter;
    private int _currentIndex = 0;
    private ShooterTableEntry ste;

    private LinkedList<ShooterTableEntry> _Table = null;

    private ShooterTable(boolean isPrimary)
    {
        _Table = isPrimary ? LoadPrimaryTable() : LoadSecondaryTable();
        _currentIndex = 1;

		Iterator<ShooterTableEntry> itr = _Table.iterator();
		while(itr.hasNext()) {
            ste = itr.next();
        }	
    }
    
    public ShooterTableEntry CalcShooterValues (double distanceInFeet)
	{
		ShooterTableEntry steBelow = null;
		ShooterTableEntry steAbove = null;
		ShooterTableEntry steCurrent = null;
		
		
		Iterator<ShooterTableEntry> itr = _Table.iterator();
		while(itr.hasNext()) {
			steCurrent = itr.next();
			
			
			if (steCurrent.DistanceInFeet < distanceInFeet) {
                steBelow = steCurrent;
                continue;
            }

            else if (steCurrent.DistanceInFeet == distanceInFeet) {
                steBelow = steCurrent;
                steAbove = steCurrent;
                break;
            }
            // if longer, snapshot Above, stop looping
            else if (steCurrent.DistanceInFeet > distanceInFeet) {
                steAbove = steCurrent;
                break;
            }
        }

        if (steBelow != null && steAbove != null) {
            // find the scale factor which is how far we are between the below & above ste
            double scaleFactor = (distanceInFeet - steBelow.DistanceInFeet)
                    / (steAbove.DistanceInFeet - steBelow.DistanceInFeet);

            // round to int
            double stg1Adj = scaleFactor * (steAbove.MotorTargetRPM - steBelow.MotorTargetRPM);
            int stg1CalculatedRPM = steBelow.MotorTargetRPM + (int) (Math.round(stg1Adj));
        
            double actuatorValue = steBelow.ActuatorVal + (scaleFactor * (steAbove.ActuatorVal - steBelow.ActuatorVal));

            // build the return object
            ste = new ShooterTableEntry(_indexCounter++, distanceInFeet, stg1CalculatedRPM, actuatorValue);
        } else if (steAbove != null) {
            ste = steAbove;
        } else {
            ste = steBelow;
        }

        return ste;
    }

    public ShooterTableEntry getNextEntry() {
        if (!get_IsAtUpperEntry()) {
            _currentIndex++;
        }

        return _Table.get(_currentIndex);
    }

    public ShooterTableEntry getCurrentEntry() {
        return _Table.get(_currentIndex);
    }

    public ShooterTableEntry getPreviousEntry() {
        if (!get_IsAtLowerEntry()) {
            _currentIndex--;
        }

        return _Table.get(_currentIndex);
    }

   
    
	//============================================================================================
	// properties follow
	//============================================================================================
	
	public Boolean get_IsAtUpperEntry() {
		if (_currentIndex == _Table.size() - 1){
			return true; 
		} else {
			return false;
		}
	}
	
	public Boolean get_IsAtLowerEntry() {
		if (_currentIndex == 0){
			return true;
		} else {
			return false;
		}
	}
	
	//============================================================================================
	// helpers follow
	//============================================================================================
	// create a linked list
	private LinkedList<ShooterTableEntry> LoadPrimaryTable() {

		LinkedList<ShooterTableEntry> primarytable = new LinkedList<ShooterTableEntry>();
		
        _indexCounter = 0;
        
		//======================================================================================
		//									Position	feet Stg1  
		//======================================================================================
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 11.5, 2600, .28));
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 13.38, 3110, .28)); //30 //2910|33 //34
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 18.5, 3400, .33)); //3400|33  //36 //3200|39 //41
        primarytable.add(new ShooterTableEntry(_indexCounter++, 21.0, 3324 , .35)); //3907|37 //40 //3707|43 //44
        // primarytable.add(new ShooterTableEntry(_indexCounter++, 27.5, 4228, .38)); //41 //4028|44
        // primarytable.add(new ShooterTableEntry(_indexCounter++,  38, 4300, .48)); 
        // primarytable.add(new ShooterTableEntry(_indexCounter++,  42, 4400, .50));
        
        
        
		return primarytable;
    }
    
    private LinkedList<ShooterTableEntry> LoadSecondaryTable() {

		LinkedList<ShooterTableEntry> secondarytable = new LinkedList<ShooterTableEntry>();
		
		_indexCounter = 0;
		//======================================================================================
		//									Position	feet Stg1  
		//======================================================================================
		secondarytable.add(new ShooterTableEntry(_indexCounter++,  25, 2910, .33));
        secondarytable.add(new ShooterTableEntry(_indexCounter++,  27, 2910, .33));
        secondarytable.add(new ShooterTableEntry(_indexCounter++,  29, 2910, .33));
		return secondarytable;
	}
}



