package frc.robot.dayC;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.chassis.subsystems.Chassis;
import frc.robot.dayC.Data;
import frc.robot.dayC.Commands.MoveX;
import frc.robot.dayC.Commands.MoveY;
import frc.robot.dayC.Commands.Turn;

public class DayC {

  public static Command getCommand(Chassis chassis) {
    SequentialCommandGroup cmd = new SequentialCommandGroup();
    cmd.addRequirements(chassis);
    
    for(Data.COMMAND command : Data.COMMANDS_ARR) {
      switch (command) {
        case MOVE_FWD:
          cmd.addCommands(new MoveX(chassis, 1));
          break;
        
        case MOVE_BWD:
          cmd.addCommands(new MoveX(chassis, -1));
          break;
        
        case MOVE_RIGHT:
          cmd.addCommands(new MoveY(chassis, 1));
          break;
        
        case MOVE_LEFT:
          cmd.addCommands(new MoveY(chassis, -1));
          break;
        
        case STEER_RIGHT:
          cmd.addCommands(new Turn(chassis, 0.25 * 2 * Math.PI));
          break;
        
        case STEER_LEFT:
          cmd.addCommands(new Turn(chassis, -0.25 * 2 * Math.PI));
          break;
      
        default:
          break;
      }
    }
    
    return cmd;
  }
}
