// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib;

/** Add your docs here. */
public class AutoIDUtility {

    /** Return the tag ID of the station with the given reef tag ID for the Auto first coral
     * 
     * @param _tagID Tag ID of the station based on the first coral tag ID
     * @return
     */
    public static int getStationTagID(int _tagID) {
        if (_tagID == 9) {
            return 2;
        } else if (_tagID == 11) {
            return 1;
        } else if (_tagID == 20) {
            return 13;
        } else if (_tagID == 22) {
            return 12;
        }
        return 0;

    }
    /** Return the tag ID of the next reef tag it with the given reef tag ID for the Auto first coral
     * 
     * @param _tagID Tag ID of the front reef tag to place the second coral on.
     * @return
     */
    public static int getNextReefTagID(int _tagID) {
        if (_tagID == 9) {
            return 8;
        } else if (_tagID == 11) {
            return 6;
        } else if (_tagID == 20) {
            return 19;
        } else if (_tagID == 22) {
            return 17;
        }
        return 0;
    }
}
