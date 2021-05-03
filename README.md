# 2021 Infinite Recharge Official

[![Build Status](https://github.com/FRC6854/2021InfiniteRechargeOfficial/workflows/Java%20CI/badge.svg)](https://github.com/FRC6854/2021InfiniteRechargeOfficial/actions)

Build project with internet connection before deploying to robot to fetch all `vendordeps`. Also use latest `VIKING` package.

The best way to get started with understanding the code here is going to [WPILib Docs](https://docs.wpilib.org/en/stable/) to looks at how you should get started using WPILib and an overview of an FRC robot program.

## Layout
The current project uses our layout from last season.

`frc/robot/subsystems` contains all of our moving parts on our bot. For example our drivetrain and conveyor code is found here. We are using the new command-based WPILib package this year for our robot structure. This means that the way we create Commands and Subsystems will change a bit from what it was last year but the logic is almost exactly the same. Our subsystems also use the Singleton pattern where we can. This prevents multiple instances of a subsystem being created and it is more performant when you want to reference the subsystem somewhere else.

`frc/robot/commands` contains all of code for controlling the subsystems during runtime. They run in the CommandScheduler and some commands are in an infinite loop so that they will run all the time when another command isn't scheduled. For example our `ArcadeDrive` command will run forever since there should be not exit case.

`frc/robot/paths` contains all of code for the autonomous paths that we compile along with the code.

`frc/robot/utils` contains helper classes that we use for calculations, wrappers, and etc.

`frc/robot/auto` pre-planned autonomous code that is used when we enable the robot in Autonomous mode. The `AutoManager` contains the logic to select the auto command, and `frc/robot/auto/auto_commands` contains the CommandGroups with the logic for the individual auto plans.