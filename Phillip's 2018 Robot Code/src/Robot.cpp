

//pls no touch code unless you know what you're doing
//if you ruin it Elizabeth will bash in your knees
//or she'll just be really disappointed in you
//either way, you don't want that
//so don't mess with it pls <3


#include <iostream>
#include <string>
#include <IterativeRobot.h>
#include "ctre/Phoenix.h"
#include "Drive/DifferentialDrive.h"
#include "Joystick.h"
#include "ctre/phoenix/MotorControl/CAN/TalonSRX.h"
#include "ctre/phoenix/MotorControl/CAN/WPI_TalonSRX.h"
#include "WPILib.h"
#include "Drive/RobotDriveBase.h"
#include <Timer.h>
#include <LiveWindow/LiveWindow.h>
#include <SmartDashboard/SendableChooser.h>
#include <SmartDashboard/SmartDashboard.h>
#include "Constants.h"


class Robot: public frc::IterativeRobot {
public:

	/* Sets each talon to what motor they control, names compressor and solenoids*/


	WPI_TalonSRX * rghtFront = new WPI_TalonSRX(17);
	WPI_TalonSRX * rghtFollower = new WPI_TalonSRX(15);
	WPI_TalonSRX * leftFront = new WPI_TalonSRX(13);
	WPI_TalonSRX * leftFollower = new WPI_TalonSRX(12);

	WPI_TalonSRX * elvtr = new WPI_TalonSRX(11);
	WPI_TalonSRX * clmbr = new WPI_TalonSRX(10);
	WPI_TalonSRX * harvy1 = new WPI_TalonSRX(14);
	WPI_TalonSRX * harvy2 = new WPI_TalonSRX(16);

	Compressor	 * c = new Compressor(0);


	DoubleSolenoid * solen = new DoubleSolenoid(0,1);
	DoubleSolenoid * solen2 = new DoubleSolenoid(2,3);

	Encoder 	 * enc = new Encoder(0, 1, false, Encoder::k4X);

//	DifferentialDrive * _diffDrive = new DifferentialDrive(*leftFront, //Sets up drive
//			*rghtFront);

	SpeedControllerGroup * m_left = new SpeedControllerGroup(*leftFront, *leftFollower);
	SpeedControllerGroup * m_right = new SpeedControllerGroup(*rghtFront, *rghtFollower);

	DifferentialDrive * _diffDrive = new DifferentialDrive(*m_left, //Sets up drive
			*m_right);



	DigitalInput * lowSensor = new DigitalInput(7);
	DigitalInput * highSensor = new DigitalInput(8);

	Joystick * _joystick = new Joystick(0);

	Faults _faults_L;
	Faults _faults_R;

	bool pivot = false;
	bool prevState = false;
	bool currState = false;
	int AutoStep ;
	frc::Timer m_timer;


	cs::UsbCamera cam1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);

	float rotLeft = (leftFront->GetSelectedSensorPosition(kPIDLoopIdx)/4096)/2;
	float rotRght = (rghtFront->GetSelectedSensorPosition(kPIDLoopIdx)/4096/2);
	float distLeft;
	float distRght;

	float rots1;
	float rots2;
	bool step;

	bool scaleHght = true;
	bool switchHght = false;

	bool inPosition;
	bool rghtPosition;
	bool leftPosition;


	void TeleopInit() { //This is run when teleop starts

		leftFront->SetNeutralMode(NeutralMode::Coast);
		rghtFront->SetNeutralMode(NeutralMode::Coast);
		rghtFront->ConfigPeakOutputForward(1.0, kTimeoutMs);
		rghtFront->ConfigPeakOutputReverse(-1.0, kTimeoutMs);
		leftFront->ConfigPeakOutputForward(1.0, kTimeoutMs);
		leftFront->ConfigPeakOutputReverse(-1.0, kTimeoutMs);
		//^Sets the talons to coast mode and sets their max output back to 1
	}

	void TeleopPeriodic() {
		std::stringstream work;

		c->SetClosedLoopControl(true); //Setting Compressor to a closed loop


/*Button numbers:
 * #	Corresponding Button
 * 1	A
 * 2	B
 * 3	X
 * 4	Y
 * 5	left bumper
 * 6	right bumper
 * 7	back
 * 8	start
 * 9	left joystick
 * 10	right joystick
 *
 * Axis
 * 0	left x
 * 1	left y
 * 2	left trigger
 * 3	right trigger
 * 4	right x
 * 5	right y
 */




		currState = (_joystick->GetRawButton(8));
		if (currState != prevState)
		{
			if (currState)
			{
				pivot = !pivot;
			}
			else
			{

			}
			prevState = currState;
		}
		if(pivot)
		{
			solen2->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else
		{
			solen2->Set(frc::DoubleSolenoid::Value::kReverse);
		}




		if (lowSensor->Get() == 0    ) //if the low sensor gets a value the elevator can't move up
		{
			if (_joystick->GetRawAxis(2) > 0 && _joystick->GetRawAxis(3) == 0) //Sets it so the left trigger and right trigger can't both be pressed
			{
				elvtr->Set(0);
			}
			else if (_joystick->GetRawAxis(3) > 0 && _joystick->GetRawAxis(2) == 0) //left trigger makes elevator go up
			{									//^right trigger can't be pressed while left trigger is
				elvtr->Set(-_joystick->GetRawAxis(3));
			}

			else
			{
				elvtr->Set(0);
			}
		}
		else if (highSensor->Get() == 0) //if the high sensor gets a value you cannot move the elevator up
			{
			if (_joystick->GetRawAxis(3) > 0 && _joystick->GetRawAxis(2) == 0) //left trigger makes elevator go up
			{									//^right trigger can't be pressed while left trigger is
				elvtr->Set(0);
			}
			else if (_joystick->GetRawAxis(2) > 0 && _joystick->GetRawAxis(3) == 0) //left trigger makes elevator go up
			{									//^right trigger can't be pressed while left trigger is
				elvtr->Set(_joystick->GetRawAxis(2));
			}
			else
			{
				elvtr->Set(0);
			}
		}
		else
		{
			if (_joystick->GetRawAxis(3) > 0 && _joystick->GetRawAxis(2) == 0) //left trigger makes elevator go up
			{									//^right trigger can't be pressed while left trigger is
				elvtr->Set(-_joystick->GetRawAxis(3));
			}
			else if (_joystick->GetRawAxis(2) > 0 && _joystick->GetRawAxis(3) == 0) //Sets it so the left trigger and right trigger can't both be pressed
			{
				elvtr->Set(_joystick->GetRawAxis(2));
			}

			else
			{
				elvtr->Set(0);
			}
		}


		if (_joystick->GetRawButton(1) == true) //When A is held down the harvester opens and the wheel run inward
		{
			cubeIn();
			solen->Set(frc::DoubleSolenoid::Value::kReverse);
		}
		else if (_joystick->GetRawButton(2)== true) //When B is held down the wheels spit out the cube
		{
			cubeOut();
			solen->Set(frc::DoubleSolenoid::Value::kForward);
		}
		else if (_joystick->GetRawButton(4)==true)
		{
			cubeIn();
		}
		else if (_joystick->GetRawButton(1) == false) //when A isn't pressed the harvester defaults to the open and off position
		{

			solen->Set(frc::DoubleSolenoid::Value::kForward);
			harvy1->Set(0);
			harvy2->Set(0);
		}


		/* get gamepad stick values */
		double forw = 1 * _joystick->GetRawAxis(1); /* positive is forward */
		double turn = -1 * _joystick->GetRawAxis(0); /* positive is right */


		/* deadband gamepad 10%*/
		if (fabs(forw) < 0.10)
			forw = 0;
		if (fabs(turn) < 0.10)
			turn = 0;

		/* drive robot */
		_diffDrive->ArcadeDrive(forw, turn, false);

		/* -------- [2] Make sure Gamepad Forward is positive for FORWARD, and GZ is positive for RIGHT */
		work << " GF:" << forw << " GT:" << turn;

		/* get sensor values */
		//double leftPos = _leftFront->GetSelectedSensorPosition(0);
		//double rghtPos = _rghtFront->GetSelectedSensorPosition(0);
		double leftVelUnitsPer100ms = leftFront->GetSelectedSensorVelocity(0);
		double rghtVelUnitsPer100ms = rghtFront->GetSelectedSensorVelocity(0);

		work << " L:" << leftVelUnitsPer100ms << " R:" << rghtVelUnitsPer100ms;

		/* drive motor at least 25%, Talons will auto-detect if sensor is out of phase */
		leftFront->GetFaults(_faults_L);
		rghtFront->GetFaults(_faults_R);

		if (_faults_L.SensorOutOfPhase) {
			work << " L sensor is out of phase";
		}
		if (_faults_R.SensorOutOfPhase) {
			work << " R sensor is out of phase";
		}

	 	std::cout << rotLeft;

		/* print to console */
		std::cout << work.str() << std::endl;
	}

	void RobotInit() {
		//sets motors to follower each other and mirror eachother's actions//
		rghtFollower->Follow(*rghtFront);
		leftFollower->Follow(*leftFront);


		/*  Adjust inverts so all motor drive in the correction direction */
		rghtFront->SetInverted(false);
		rghtFollower->SetInverted(false);
		leftFront->SetInverted(false);
		leftFollower->SetInverted(false);
		elvtr->SetInverted(false);
		harvy1->SetInverted(false);
		harvy2->SetInverted(false);

		/* adjust sensor phase so sensor moves
		 * positive when Talon LEDs are green */
		rghtFront->SetSensorPhase(false);
		leftFront->SetSensorPhase(false);

		cam1.SetBrightness(60);
		//cam1.SetResolution(160,160);
		cam1.SetExposureManual(60);
		cam1.SetExposureHoldCurrent();
		cam1.SetWhiteBalanceManual(50);

		leftFront->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		rghtFront->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);

		leftFront->ConfigNominalOutputForward(0, kTimeoutMs);
		leftFront->ConfigNominalOutputReverse(0, kTimeoutMs);
		leftFront->ConfigPeakOutputForward(1.0, kTimeoutMs);
		leftFront->ConfigPeakOutputReverse(-1.0, kTimeoutMs);

		/* set closed loop gains in slot0 */
		leftFront->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		leftFront->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		leftFront->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		leftFront->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);


		rghtFront->ConfigNominalOutputForward(0, kTimeoutMs);
		rghtFront->ConfigNominalOutputReverse(0, kTimeoutMs);
		rghtFront->ConfigPeakOutputForward(1.0, kTimeoutMs);
		rghtFront->ConfigPeakOutputReverse(-1.0, kTimeoutMs);

		/* set closed loop gains in slot0 */
		rghtFront->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		rghtFront->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		rghtFront->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		rghtFront->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

		_diffDrive->SetExpiration(0.1);
		m_timer.Start();



	}
	void cubeIn() {
		harvy1->Set(.7);
		harvy2->Set(-.7);
	}
	void cubeOut() {
		harvy1->Set(-1);
		harvy2->Set(1);
	}
	void resetEncoders() {
		leftFront->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		rghtFront->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
	}
	void Drive(float dist, float power) {
			float RError;
			float LError;
//			dist = dist*2*4096*18.849;
			float ticks = dist / (3.14159 * 6.0) * (2.0 * 4096.0);  //calculate encoder counts from inches

			leftFront->ConfigPeakOutputForward(power, kTimeoutMs);
			leftFront->ConfigPeakOutputReverse(-power, kTimeoutMs);
			rghtFront->ConfigPeakOutputForward(power, kTimeoutMs);
			rghtFront->ConfigPeakOutputReverse(-power, kTimeoutMs);

			rghtFollower->Set(ControlMode::Follower, 17);
			leftFollower->Set(ControlMode::Follower, 13);
			leftFront->Set(ControlMode::Position, -ticks);
			rghtFront->Set(ControlMode::Position, ticks);

			if (rghtFront->GetSelectedSensorPosition(0) >= ticks + 200 )
			{
				rghtPosition = true;
			}
			else {
				rghtPosition = false;
			}
			if (leftFront->GetSelectedSensorPosition(0) <= -ticks - 200)
			{
				leftPosition = true;
			}
			else
			{
				leftPosition = false;
			}
			if (rghtPosition == true && leftPosition == true)
			{
				inPosition = true;
			}
			else {
				inPosition = false;
			}

	}
	void turnRight(float dist, float power)
	{
		leftFront->ConfigPeakOutputForward(power, kTimeoutMs);
		leftFront->ConfigPeakOutputReverse(-power, kTimeoutMs);
		rghtFront->ConfigPeakOutputForward(power, kTimeoutMs);
		rghtFront->ConfigPeakOutputReverse(-power, kTimeoutMs);
		//^sets the max output for each motor to be the value you put in when you call the function

		float ticks = dist * (2.0 * 4096.0);  //calculate encoder counts from rotations

		leftFront->Set(ControlMode::Position, -ticks);
		rghtFront->Set(ControlMode::Position, -ticks);

		if (rghtFront->GetSelectedSensorPosition(0) <= -ticks )
		{
			rghtPosition = true;
		}
		else {
			rghtPosition = false;
		}
		if (leftFront->GetSelectedSensorPosition(0) <= -ticks )
		{
			leftPosition = true;
		}
		else
		{
			leftPosition = false;
		}

		if (rghtPosition == true && leftPosition == true)
		{
			inPosition = true;
		}
		else
		{
			inPosition = false;
		}


	}
	void turnLeft(float dist, float power)
	{

		leftFront->ConfigPeakOutputForward(power, kTimeoutMs);
		leftFront->ConfigPeakOutputReverse(-power, kTimeoutMs);
		rghtFront->ConfigPeakOutputForward(power, kTimeoutMs);
		rghtFront->ConfigPeakOutputReverse(-power, kTimeoutMs);
		//^sets the max output for each motor to be the value you put in when you call the function

		float ticks = dist* (2.0 * 4096.0);  //calculate encoder counts from rotations

		leftFront->Set(ControlMode::Position, ticks);
		rghtFront->Set(ControlMode::Position, ticks);

		if (abs (rghtFront->GetSelectedSensorPosition(0) - ticks) < 500  )
		{
			rghtPosition = true;
		}
		else {
			rghtPosition = false;
		}
		if (abs (leftFront->GetSelectedSensorPosition(0) - ticks) < 500 )
		{
			leftPosition = true;
		}
		else
		{
			leftPosition = false;
		}
		if (leftPosition == true && rghtPosition == true )
		{
			inPosition = true;
		}
		else {
			inPosition = false;
		}
		//^this allows you to check that the function is complete before moving on
	}

	void autoDrive(float inches, bool forward) {
		//float rotations = inches * 217.51175555937088446065331354019;
		int ticksperinch = 4100/18.849; //Steps per rot/circumference
		float ticks = inches * ticksperinch;
		//sets up the sensor/encoder
		resetEncoders();
		float leftAmount = ticks;
		float rightAmount = -ticks;
		std::cout << "leftAmount: " << leftAmount << std::endl;
		if(forward)
		{
			leftAmount = leftAmount*-1;
			rightAmount = rightAmount*-1;
		}
		//flipRight -1 for backwards, 1 for forwards
		//flipLeft -1 for forwards, 1 for backwards
		//int currentPositionRight= RMiddle.GetEncPosition();
		//int currentPositionLeft= LMiddle.GetEncPosition();
		//power = 1;
		//Set the middle motors to what you need distance-wise and the other motors to the main talon's port #.
		//sets the motors to run using encoders
		//set to follower because the middle motors are the ones with the encoders
		rghtFront->Set(ControlMode::Position, rightAmount);
		std::cout<<"Right Amount:"<<rightAmount<<"\n";
		std::cout<<"Left Amount:" << leftAmount<<"\n";
		leftFront->Set(ControlMode::Position, leftAmount);
		std::cout<<"after sets\n";
		int rightEncoder = rghtFront->GetSelectedSensorPosition(0);
		int leftEncoder = leftFront->GetSelectedSensorPosition(0);
		int LPrevious = 10000000;
		int RPrevious = 10000000;
		while (rightEncoder != RPrevious || leftEncoder != LPrevious) //Wait until old and current values are the same, to make sure the motors are no longer spinning before continuing.
		{
			LPrevious = leftEncoder;
			RPrevious = rightEncoder;
			Wait(0.1);
			rightEncoder = rghtFront->GetSelectedSensorPosition(0);
			leftEncoder = leftFront->GetSelectedSensorPosition(0);
			std::cout<<"In the loop\n";
			std::cout<< rightEncoder << std::endl;

		}
		std::cout<<"Out the loop\n";
//		//decides the behavior of the motors when power is set to 0 (other option is coast)
		//next step: PLEASE PUT WHILE LOOP HERE, THAT MAKES IT SO THAT WHILE CURRENTPOSITION < SETPOSITION
		//THE MOTOR KEEPS RUNNING! :)
		rghtFront->SetNeutralMode(NeutralMode::Brake);
		leftFront->SetNeutralMode(NeutralMode::Brake);
	}

	/*void elvtrHeight(bool height) {
		if (height == true)
	}*/

	void AutonomousInit() override {

		m_timer.Reset();
		m_timer.Start();

		m_autoSelected = m_chooser.GetSelected();
		m_autoSelected = SmartDashboard::GetString("Auto Selector", middle);
		std::cout << "Auto selected: " << m_autoSelected << std::endl;

		leftFront->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
		rghtFront->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);

		leftFront->SetNeutralMode(NeutralMode::Brake);
		rghtFront->SetNeutralMode(NeutralMode::Brake);

		AutoStep = 1;

		resetEncoders();



	}


	void AutonomousPeriodic() {

		std::string gameData;
		gameData = frc::DriverStation::GetInstance().GetGameSpecificMessage();

		std::cout << "Right:" << rotRght <<std::endl;
		std::cout << "Left:" << rotLeft <<std::endl;
		std::cout << "Step:" << AutoStep <<std::endl;
		rotLeft = -(leftFront->GetSelectedSensorPosition(0)/4096/2);
		rotRght =rghtFront->GetSelectedSensorPosition(0)/4096/2;



//		if (m_autoSelected == middle)
		if (m_autoSelected == middle)
		{

			if (gameData[0] == 'L')//if we own the left side of the switch
			{
				if(AutoStep==1 && inPosition == false)
					{
					solen2->Set(frc::DoubleSolenoid::Value::kForward);
					Drive(30.000, .5);
					}
				if (AutoStep==1 && inPosition == true)
					{
					resetEncoders();
					inPosition = false;
					AutoStep = 2;
					}
				if (AutoStep==2 && inPosition == false)
					{
					turnLeft(.5000, .5);
					}
				if (AutoStep==2 && inPosition == true )
				{
					resetEncoders();
					inPosition = false;
					AutoStep = 3;
				}
				if (AutoStep == 3 && inPosition == false)
				{
					Drive(30.000, .5);
				}
				if (AutoStep == 3 && inPosition == true)
				{
					resetEncoders();
					AutoStep = 4;
					inPosition = false;
				}
				if (AutoStep == 4 && inPosition == false)
				{
					turnRight(0.50000, .5);
				}


				//leftFront->Set(ControlMode::Position, -200000);






				//autoDrive(16.0, true);


				/*if (rotRght <= 1.00 )
				{
					_diffDrive->ArcadeDrive(-0.5, 0.0);
					leftFront->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
					step2 = true;
				}
				else if (rotLeft< 1 && rotLeft > 0 && step2 == true)
				{//turn
					_diffDrive->ArcadeDrive(0.0, .5);
					step3 = true;
					std::cout << "step 2";
					rghtFront->SetSelectedSensorPosition(0, kPIDLoopIdx, kTimeoutMs);
				}
				else if ( step3 == true && rotRght < 2 )
				{
					_diffDrive->ArcadeDrive(-0.5, 0.0);
				}
				else
				{
					_diffDrive->ArcadeDrive(0.0, 0.0);
				}







				/*if (rotLeft <= 1.0 && step1 == true)
				{
					leftFront->Set(-.4);
					step2 = true;
					std::cout << "step 1";
				}
				else if (rotLeft > 1.000 && step2 == true)
				{
					leftFront->Set(-.25);
					step3 = true;
					std::cout << "step 2";
				}
				else if (rotLeft >  1.000 &&  rotLeft < 2.000 && step3 == true )
				{
					leftFront->Set(-.4);
					std::cout << "step 3";

				}
				else
				{
					leftFront->Set(0);
				}

				if (rotRght < 1.000 && rotRght < 2.000)
				{
					rghtFront->Set(.4);

				}
				else if (rotRght > 1.00 && rotRght < 2.000)
				{
					rghtFront->Set(.25);
				}
				else if (rotRght > 2.00 && rotRght  < 3.000)
				{
					rghtFront->Set(.4);
				}
				else
				{
					rghtFront->Set(0);
				}

				/*if (m_timer.Get() < 2.0)
				{
					elvtr->Set(-1.0);
				}
				else
				{
					elvtr->Set(0);
				}
				if (m_timer.Get() >5 && m_timer.Get() < 7 )
				{
					harvy1->Set(-1);
					harvy2->Set(1);
				}
				else
				{
					harvy1->Set(0);
					harvy2->Set(0);
				}*/
				}
			else if (gameData[0]== 'R')//if we own the right side of the switch
			{

			}
			else
			{
				std::cout << "NO GAME DATAAAAA";
			}
		}




		else if (m_autoSelected == left)
		{
			if (gameData[0] == 'L')//we own the left side of the switch
			{ //it's gonna move forward, turn, and put a cube in the switch
				leftFront->Set(ControlMode::PercentOutput,0);
				rghtFront->Set(ControlMode::PercentOutput,0);
				leftFollower->Set(ControlMode::PercentOutput,0);
				rghtFollower->Set(ControlMode::PercentOutput,0);

				solen2->Set(frc::DoubleSolenoid::Value::kForward);

				if (rotLeft >= -20000 )
				{
					_diffDrive->ArcadeDrive(-0.5, 0.0);
					rots1 = rotLeft;
					std::cout << rotLeft;
				}
				else if (rotLeft < -20000 && rotLeft >= -40000)
				{//90 degree turn
					_diffDrive->ArcadeDrive(0.0,-0.3);

				}
				else
				{
					_diffDrive->ArcadeDrive(0.0,0.0);
				}
//				if (rotLeft <= 60.0 )
//				{
//					_diffDrive->ArcadeDrive(-0.3, 0.0);
//					rots2 = rotLeft;
//					std::cout << rotLeft;
//				}

				if (m_timer.Get() < 2.0)
				{
					elvtr->Set(-1.0);
				}
				else
				{
					elvtr->Set(0);
				}
				if (m_timer.Get() >5 && m_timer.Get() < 7 )
				{
					harvy1->Set(-1);
					harvy2->Set(1);
				}
				else
				{
					harvy1->Set(0);
					harvy2->Set(0);
				}

			}
			else if (gameData[1] == 'L' && gameData[0] == 'R')//we own the left side of the scale but not the switch
			{
				/*if (distance < 300)
				{ //move forward 300 inches
					_diffDrive->ArcadeDrive(-0.5, 0.0);
					rot = rotLeft;
				}
				else if (rotLeft - rot < 1)
				{//90 degree turn
					leftFront->Set(-.5);
					rghtFront->Set(.5);
					dist = distance;
				}
				else if (distance - dist < 30 )
				{//move forward 30 inches
					_diffDrive->ArcadeDrive(-0.5, 0.0);
				}
				else
				{
					_diffDrive->ArcadeDrive(0.0,0.0);
				}

				if (m_timer.Get() > 2.0)
				{
					solen2->Set(frc::DoubleSolenoid::Value::kReverse);
				}
				else
				{
					solen2->Set(frc::DoubleSolenoid::Value::kForward);
				}
				if (m_timer.Get() > 2.0 && highSensor->Get() != 0 )
				{
					elvtr->Set(-1.0);
				}
				else
				{
					elvtr->Set(0);
				}*/



				// move forward and either put a cube in the scale of just get to the null zone
			}
			else if (gameData[0] == 'R' && gameData[1] == 'R')//we own the right side of both
			{

//					if (rotLeft < 1.00)
//					{
//						leftFront->Set(-.4);
//					}
//					else if (rotLeft > 1.00 && rotLeft <2.00)
//					{
//						leftFront->Set(.25);
//					}
//					else
//					{
//						leftFront->Set(0);
//					}
//					if (rotRght > -1.00)
//					{
//						rghtFront->Set(.4);
//
//					}
//					else if (rotRght < -1.00)
//					{
//						rghtFront->Set(-.25);
//					}
//					else
//					{
//						rghtFront->Set(0);
//					}
//
//					if (m_timer.Get() < 2.0)
//					{
//						elvtr->Set(-1.0);
//					}
//					else
//					{
//						elvtr->Set(0);
//					}
//					if (m_timer.Get() >5 && m_timer.Get() < 7 )
//					{
//						harvy1->Set(-1);
//						harvy2->Set(1);
//					}
//					else
//					{
//						harvy1->Set(0);
//						harvy2->Set(0);
//					}
//


				//move forward and turn to get close to our side of the switch
			}
		}
		else if (m_autoSelected == right)
		{
			if (gameData[0] == 'R')
			{
				/*if (distance <129)
				{ //move forward 129 inches
					_diffDrive->ArcadeDrive(-0.5, 0.0);
					rot = rotLeft;
				}
				else if (rot - rotLeft < 1)
				{//90 degree turn
					leftFront->Set(.5);
					rghtFront->Set(-.5);
					dist = distance;
				}
				else if (distance - dist < 30 )
				{//move forward 30 inches
					_diffDrive->ArcadeDrive(-0.5, 0.0);
					dist = distance;
				}
				else if (distance - dist == 0 )
				{
					harvy1->Set(-1);
					harvy2->Set(1);
				}
				else
				{
					_diffDrive->ArcadeDrive(0.0,0.0);
				}

				if (m_timer.Get() > 5.0)
				{//puts the harvester out
					solen2->Set(frc::DoubleSolenoid::Value::kReverse);
				}
				else
				{
					solen2->Set(frc::DoubleSolenoid::Value::kForward);
				}
				if (m_timer.Get() > 5.0 && m_timer.Get() < 7.0)
				{//moves the elevator up
					elvtr->Set(-1.0);
				}
				else
				{
					elvtr->Set(0);
				}*/

			}
			else if (gameData[1] == 'R' && gameData[0] == 'L')
			{ //move forward and put a cube in the scale or just get to the null zone
				/*if (distance < 300)
				{ //move forward 300 inches
					_diffDrive->ArcadeDrive(-0.5, 0.0);
					rot = rotLeft;
				}
				else if (rotLeft - rot < 1)
				{//90 degree turn
					leftFront->Set(.5);
					rghtFront->Set(-.5);
					dist = distance;
				}
				else if (distance - dist < 30 )
				{//move forward 30 inches
					_diffDrive->ArcadeDrive(-0.5, 0.0);
				}
				else
				{
					_diffDrive->ArcadeDrive(0.0,0.0);
				}*/

				if (m_timer.Get() > 2.0)
				{
					solen2->Set(frc::DoubleSolenoid::Value::kReverse);
				}
				else
				{
					solen2->Set(frc::DoubleSolenoid::Value::kForward);
				}
				if (m_timer.Get() > 2.0 && highSensor->Get() != 0 )
				{
					elvtr->Set(-1.0);
				}
				else
				{
					elvtr->Set(0);
				}

			}

		}
		else
		{

		}

	}

private:
	frc::LiveWindow& m_lw = *LiveWindow::GetInstance();
	frc::SendableChooser<std::string> m_chooser;
	std::unique_ptr<frc::Command> autochooser;
	const std::string kAutoNameDefault = "Default";
	const std::string kAutoNameCustom = "My Auto";
	const std::string middle = "middle";
	const std::string left = "left";
	const std::string right = "right";
	std::string m_autoSelected;

};

START_ROBOT_CLASS(Robot)
