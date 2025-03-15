// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.AI;

import java.util.HashMap;

import frc.robot.lib.AprilTagAlignState;
import frc.robot.lib.CoralLiftState;
import frc.robot.lib.RobotAlignStates;
import frc.robot.lib.g;

public class StateManager {
    public RobotAlignStates currentState = RobotAlignStates.UNKNOWN;
    public RobotAlignStates lastState = RobotAlignStates.UNKNOWN;
    HashMap <String, ReefCoralData> reefMap = new HashMap<>();
    public StateManager() {
        reefMap.put("BACK", new ReefCoralData());
        reefMap.put("BACK_RIGHT", new ReefCoralData());
        reefMap.put("BACK_LEFT", new ReefCoralData());
        reefMap.put("FRONT", new ReefCoralData());
        reefMap.put("FRONT_RIGHT", new ReefCoralData());        
        reefMap.put("FRONT_LEFT", new ReefCoralData());

    }
    public void setState(RobotAlignStates _state) {
        currentState = _state;
        setRobotData(_state);
        lastState = _state;
    }
    public void setRobotData(CoralLiftState _coralState, AprilTagAlignState _aprilState, RobotAlignStates _robotState){
        g.CORALLIFT.state = _coralState;
        g.VISION.aprilTagAlignState = _aprilState;
        g.ROBOT.drive.setTargetRobotAngle(_robotState);
        reefMap.get(_robotState.toString()).setValue(_coralState, _aprilState);
    }
    public void setRobotData(RobotAlignStates _state) {
        if(!reefMap.get(_state.toString()).L3R) {
            setRobotData(CoralLiftState.L3, AprilTagAlignState.RIGHT, _state);
        }else if(!reefMap.get(_state.toString()).L3L) {
            setRobotData(CoralLiftState.L3, AprilTagAlignState.LEFT, _state);
        }else if(!reefMap.get(_state.toString()).L2R) {
            setRobotData(CoralLiftState.L2, AprilTagAlignState.RIGHT, _state);
        }else if(!reefMap.get(_state.toString()).L2L) {
            setRobotData(CoralLiftState.L2, AprilTagAlignState.LEFT, _state);
        }

        // switch (_state) {
        //     case BACK:
        //         if(!reefMap.get(_state.toString()).L3R) {
        //             setRobotData(CoralLiftState.L3, AprilTagAlignState.RIGHT, _state);
        //         }else if(!reefMap.get(_state.toString()).L3L) {
        //             setRobotData(CoralLiftState.L3, AprilTagAlignState.LEFT, _state);
        //         }else if(!reefMap.get(_state.toString()).L2R) {
        //             setRobotData(CoralLiftState.L2, AprilTagAlignState.RIGHT, _state);
        //         }else if(!reefMap.get(_state.toString()).L2L) {
        //             setRobotData(CoralLiftState.L2, AprilTagAlignState.LEFT, _state);
        //         }
        //         break;
        //     case BACK_LEFT:
        //         reefMap.get("BACK_LEFT").L2R = true;
        //         reefMap.get("BACK_LEFT").L3R = true;
        //         reefMap.get("BACK_LEFT").L2L = true;
        //         reefMap.get("BACK_LEFT").L3L = true;
        //         break;
        //     case BACK_RIGHT:
        //         reefMap.get("BACK_RIGHT").L2R = true;
        //         reefMap.get("BACK_RIGHT").L3R = true;
        //         reefMap.get("BACK_RIGHT").L2L = true;
        //         reefMap.get("BACK_RIGHT").L3L = true;
        //         break;
        //     case FRONT:
        //         reefMap.get("FRONT").L2R = true;
        //         reefMap.get("FRONT").L3R = true;
        //         reefMap.get("FRONT").L2L = true;
        //         reefMap.get("FRONT").L3L = true;
        //         break;
        //     case FRONT_LEFT:
        //         reefMap.get("FRONT_LEFT").L2R = true;
        //         reefMap.get("FRONT_LEFT").L3R = true;
        //         reefMap.get("FRONT_LEFT").L2L = true;
        //         reefMap.get("FRONT_LEFT").L3L = true;
        //         break;
        //     case FRONT_RIGHT:
        //         reefMap.get("FRONT_RIGHT").L2R = true;
        //         reefMap.get("FRONT_RIGHT").L3R = true;
        //         reefMap.get("FRONT_RIGHT").L2L = true;
        //         reefMap.get("FRONT_RIGHT").L3L = true;
        //         break;
        //     case STATION_LEFT:
                
        //         break;
        //     case STATION_RIGHT:
        //         break;
        //     case UNKNOWN:
        //         break;
        //     default:
        //         break;

        // }
    }
}
