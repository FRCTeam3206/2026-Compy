# Team 3206 2026 Drivebase

MAXSwerve drivebase code for Team 3206. The code is based on the [REVrobotics MAXSwerve Java Template v 2026.0](https://github.com/REVrobotics/MAXSwerve-Java-Template) modified with features that our team uses.

## Features
### Added
* Code from RobotContainer is moved to Robot to create a simpler code structure.
* Logging is enabled via DataLog and Epilogue.
* Spotless has been added for consitent code formatting.
    * Run `./gradlew spotlessApply` at the top level of the code repository to format code.
* GitHub CI Workflows have been added for verifying that the code builds and that it is properly formatted.

### In process
* Use NavX 2 as the gyro.
* Include basic simulation support for the drivetrain.
* Include PhotonVision.
