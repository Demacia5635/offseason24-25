// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Intake;

import java.util.Collections;
import java.util.HashMap;

/** Add your docs here. */
public class IntakeConstants {
public static final int INTAKE_MOTOR_DOWN_ID = -1;
public static final int INTAKE_MOTOR_UP_ID = -1;
  public static final String CANBUS = "canivore";
  public static final double NOTE_VOLTEGE = 4.0;
  public static final double POWER =1;
  public static final double NOTE_CURRENT = 0.0;
  public enum NotePosition{
    NO_NOTE,
    FIRST_TOUCH,
    IR_SENSOR,
    SECOND_TOUCH,
    FULLY_IN
  }

//   public static HashMap<NotePosition, Double> NotePositionToVoltage;
//    static {
//     //NPTV is NotePositionToVoltage
//       HashMap<NotePosition, Double> NPTV = new HashMap<>();
//       NPTV.put(NotePosition.NO_NOTE, 5.0);
//       NPTV.put(NotePosition.FIRST_TOUCH, 4.0);
//       NPTV.put(NotePosition.IR_SENSOR, 4.0);
//       NPTV.put(NotePosition.SECOND_TOUCH, 4.0);
//       NPTV.put(NotePosition.FULLY_IN, 0.0);
//       NotePositionToVoltage = (HashMap<NotePosition, Double>) NPTV.clone();
//   }
// }
public static final HashMap<NotePosition, Double> NotePositionToVoltage = new HashMap<>() {{
    put(NotePosition.NO_NOTE, 5.0);
    put(NotePosition.FIRST_TOUCH, 4.0);
    put(NotePosition.IR_SENSOR, 4.0);
    put(NotePosition.SECOND_TOUCH, 4.0);
    put(NotePosition.FULLY_IN, 0.0);
}};
}