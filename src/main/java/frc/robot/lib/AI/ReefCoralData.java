// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.AI;

import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralLiftState;

/** Add your docs here. */
public class ReefCoralData {
    public ReefCoralData() {
    }
    boolean L2R = false;
    int L2R_FailedCounter = 0;
    boolean L3R = false;
    int L3R_FailedCounter = 0;
    boolean L2L = false;
    int L2L_FailedCounter = 0;
    boolean L3L = false;
    int L3L_FailedCounter = 0;

    public void setValue(CoralLiftState _coralState, AprilTagAlignState _aprilState) {
        switch (_coralState) {
            case L2:
                switch (_aprilState) {
                    case LEFT:
                        L2L = true;
                        break;
                    case RIGHT:
                        L2R = true;
                        break;
                    default:
                        break;
                }
                break;
            case L3:
                switch (_aprilState) {
                    case LEFT:
                        L3L = true;
                        break;
                    case RIGHT:
                        L3R = true;
                        break;
                    default:
                        break;
                }
                break;
            case START:
                break;
            default:
                break;

        }
    }

}
