//====================================================================
//Project Lynxmotion Phoenix
//Description: 
//    This is the hardware configuration file for the Hex Robot.
//  
//    This version of the Configuration file is set up to run on the
//    Lynxmotion BotboardDuino board, which is similiar to the Arduino Duemilanove
//
//    This version of configuration file assumes that the servos will be controlled
//    by a Lynxmotion Servo controller SSC-32 and the user is using a Lynxmotion 
//    PS2 to control the robot.
//
//Date: March 18, 2012
//Programmer: Kurt (aka KurtE)
//
//NEW IN V1.1 (2013-05-17)
//   - Support for Arduino Pro Mini on Bot Board (originally for Basic Atom Pro)
//NEW IN V1.0
//   - First Release
//
//====================================================================

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
//[CONDITIONAL COMPILING] - COMMENT IF NOT WANTED
// Define other optional compnents to be included or not...

//comment if terminal monitor is not required
#define OPT_TERMINAL_MONITOR  

//uncomment the board you want to use
#define __BOTBOARDUINO__    //botboarduino board
//#define __BOTBOARD_ARDUINOPROMINI__  //arduino pro mini on botboard (originally for BasicAtomPro)

//====================================================================
#ifdef OPT_TERMINAL_MONITOR   // turning off terminal monitor will turn these off as well...
#define OPT_SSC_FORWARDER  // only useful if terminal monitor is enabled
#define OPT_FIND_SERVO_OFFSETS    // Only useful if terminal monitor is enabled
#endif

#define OPT_GPPLAYER

// Which type of control(s) do you want to compile in
#define DBGSerial         Serial

#if defined(UBRR1H)
#define SSCSerial         Serial1
#else
#endif

#define USEPS2

//==================================================================================================================================
//==================================================================================================================================
//==================================================================================================================================
// CHR-3
//==================================================================================================================================
#define USE_SSC32
//#define	cSSC_BINARYMODE	1			// Define if your SSC-32 card supports binary mode.

//[SERIAL CONNECTIONS]

// Warning I will undefine some components as the non-megas don't have enough memory...
//#undef OPT_FIND_SERVO_OFFSETS 

#define cSSC_BAUD   38400   //SSC32 BAUD rate

//--------------------------------------------------------------------
//[Botboarduino Pin Numbers]
#ifdef __BOTBOARDUINO__
  #define SOUND_PIN       5   // Botboarduino JR pin number
  #define PS2_DAT         6        
  #define PS2_CMD         7
  #define PS2_SEL         8
  #define PS2_CLK         9
// If we are using a SSC-32 then:
// If were are running on an Arduino Mega we will use one of the hardware serial port, default to Serial1 above.
// If on Non mega, if the IO pins are set to 0, we will overload the hardware Serial port 
// Else we will user SoftwareSerial to talk to the SSC-32
  #define cSSC_OUT       12   //Output pin for Botboard - Input of SSC32 (Yellow)
  #define cSSC_IN        13   //Input pin for Botboard - Output of SSC32 (Blue)
#endif

#ifdef __BOTBOARD_ARDUINOPROMINI__
  #define SOUND_PIN      11   // Bot Board JR pin number (with Arduino Pro Mini plugged)
  #define PS2_DAT        14       
  #define PS2_CMD        15
  #define PS2_SEL        16
  #define PS2_CLK        17
// If we are using a SSC-32 then:
// If were are running on an Arduino Mega we will use one of the hardware serial port, default to Serial1 above.
// If on Non mega, if the IO pins are set to 0, we will overload the hardware Serial port 
// Else we will user SoftwareSerial to talk to the SSC-32
  #define cSSC_OUT       10   //Output pin for Botboard - Input of SSC32 (Yellow)
  #define cSSC_IN         9   //Input pin for Botboard - Output of SSC32 (Blue)
#endif

//====================================================================
//[SSC PIN NUMBERS]
#define cRRCoxaPin      0   //Rear Right leg Hip Horizontal
#define cRRFemurPin     1   //Rear Right leg Hip Vertical
#define cRRTibiaPin     2   //Rear Right leg Knee
#define cRRTarsPin      3   // Tar

#define cRMCoxaPin      4   //Middle Right leg Hip Horizontal
#define cRMFemurPin     5   //Middle Right leg Hip Vertical
#define cRMTibiaPin     6   //Middle Right leg Knee
#define cRMTarsPin      7   // Tar

#define cRFCoxaPin      8   //Front Right leg Hip Horizontal
#define cRFFemurPin     9   //Front Right leg Hip Vertical
#define cRFTibiaPin     10   //Front Right leg Knee
#define cRFTarsPin      11   // Tar

#define cLRCoxaPin      16   //Rear Left leg Hip Horizontal
#define cLRFemurPin     17   //Rear Left leg Hip Vertical
#define cLRTibiaPin     18   //Rear Left leg Knee
#define cLRTarsPin      19   // Tar

#define cLMCoxaPin      20   //Middle Left leg Hip Horizontal
#define cLMFemurPin     21   //Middle Left leg Hip Vertical
#define cLMTibiaPin     22   //Middle Left leg Knee
#define cLMTarsPin      23   // Tar

#define cLFCoxaPin      24   //Front Left leg Hip Horizontal
#define cLFFemurPin     25   //Front Left leg Hip Vertical
#define cLFTibiaPin     26   //Front Left leg Knee
#define cLFTarsPin      27   // Tar


//--------------------------------------------------------------------
//[MIN/MAX ANGLES]
#define cRRCoxaMin1     -300      //Mechanical limits of the Right Rear Leg
#define cRRCoxaMax1     300
#define cRRFemurMin1    -750
#define cRRFemurMax1    500
#define cRRTibiaMin1    -550
#define cRRTibiaMax1    750
#define cRRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cRMCoxaMin1     -300      //Mechanical limits of the Right Middle Leg
#define cRMCoxaMax1     300
#define cRMFemurMin1    -750
#define cRMFemurMax1    500
#define cRMTibiaMin1    -550
#define cRMTibiaMax1    750
#define cRMTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRMTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cRFCoxaMin1     -300      //Mechanical limits of the Right Front Leg
#define cRFCoxaMax1     300
#define cRFFemurMin1    -750
#define cRFFemurMax1    500
#define cRFTibiaMin1    -550
#define cRFTibiaMax1    750
#define cRFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cRFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLRCoxaMin1     -300      //Mechanical limits of the Left Rear Leg
#define cLRCoxaMax1     300
#define cLRFemurMin1    -750
#define cLRFemurMax1    500
#define cLRTibiaMin1    -550
#define cLRTibiaMax1    750
#define cLRTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLRTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLMCoxaMin1     -300      //Mechanical limits of the Left Middle Leg
#define cLMCoxaMax1     300
#define cLMFemurMin1    -750
#define cLMFemurMax1    500
#define cLMTibiaMin1    -550
#define cLMTibiaMax1    750
#define cLMTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLMTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

#define cLFCoxaMin1     -300     //Mechanical limits of the Left Front Leg
#define cLFCoxaMax1     300
#define cLFFemurMin1    -750
#define cLFFemurMax1    500
#define cLFTibiaMin1    -550
#define cLFTibiaMax1    750
#define cLFTarsMin1     -1300	//4DOF ONLY - In theory the kinematics can reach about -160 deg
#define cLFTarsMax1	500	//4DOF ONLY - The kinematics will never exceed 23 deg though..

//--------------------------------------------------------------------
//[LEG DIMENSIONS]
//Universal dimensions for each leg in mm
#define cXXCoxaLength     35    // HeZapod 34.59mm
#define cXXFemurLength    100	// HeZapod 100mm
#define cXXTibiaLength    100	// HeZapod 100mm
#define cXXTarsLength     85    // 4DOF only... (HeZapod is 3DOF)

#define cRRCoxaLength     cXXCoxaLength	    //Right Rear leg
#define cRRFemurLength    cXXFemurLength
#define cRRTibiaLength    cXXTibiaLength
#define cRRTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRMCoxaLength     cXXCoxaLength	    //Right middle leg
#define cRMFemurLength    cXXFemurLength
#define cRMTibiaLength    cXXTibiaLength
#define cRMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cRFCoxaLength     cXXCoxaLength	    //Rigth front leg
#define cRFFemurLength    cXXFemurLength
#define cRFTibiaLength    cXXTibiaLength
#define cRFTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLRCoxaLength     cXXCoxaLength	    //Left Rear leg
#define cLRFemurLength    cXXFemurLength
#define cLRTibiaLength    cXXTibiaLength
#define cLRTarsLength	  cXXTarsLength    //4DOF ONLY

#define cLMCoxaLength     cXXCoxaLength	    //Left middle leg
#define cLMFemurLength    cXXFemurLength
#define cLMTibiaLength    cXXTibiaLength
#define cLMTarsLength	  cXXTarsLength	    //4DOF ONLY

#define cLFCoxaLength     cXXCoxaLength	    //Left front leg
#define cLFFemurLength    cXXFemurLength
#define cLFTibiaLength    cXXTibiaLength
#define cLFTarsLength	  cXXTarsLength	    //4DOF ONLY


//--------------------------------------------------------------------
//[BODY DIMENSIONS]
#define cRRCoxaAngle1   -600   //Default Coxa setup angle, decimals = 1
#define cRMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cRFCoxaAngle1    600      //Default Coxa setup angle, decimals = 1
#define cLRCoxaAngle1    -600   //Default Coxa setup angle, decimals = 1
#define cLMCoxaAngle1    0      //Default Coxa setup angle, decimals = 1
#define cLFCoxaAngle1    600      //Default Coxa setup angle, decimals = 1

#define cRROffsetX      -39     //Distance X from center of the body to the Right Rear coxa
#define cRROffsetZ      78     //Distance Z from center of the body to the Right Rear coxa
#define cRMOffsetX      -80    //Distance X from center of the body to the Right Middle coxa
#define cRMOffsetZ      0       //Distance Z from center of the body to the Right Middle coxa
#define cRFOffsetX      -39     //Distance X from center of the body to the Right Front coxa
#define cRFOffsetZ      -78    //Distance Z from center of the body to the Right Front coxa

#define cLROffsetX      39      //Distance X from center of the body to the Left Rear coxa
#define cLROffsetZ      78     //Distance Z from center of the body to the Left Rear coxa
#define cLMOffsetX      80     //Distance X from center of the body to the Left Middle coxa
#define cLMOffsetZ      0       //Distance Z from center of the body to the Left Middle coxa
#define cLFOffsetX      39      //Distance X from center of the body to the Left Front coxa
#define cLFOffsetZ      -78    //Distance Z from center of the body to the Left Front coxa

//--------------------------------------------------------------------
//[START POSITIONS FEET]
#define cHexInitXZ	 120
#define CHexInitXZCos60  70         // COS(60) = .5
#define CHexInitXZSin60  112   // sin(60) = .866
#define CHexInitY	50 //30


#define cRRInitPosX     CHexInitXZCos60      //Start positions of the Right Rear leg
#define cRRInitPosY     CHexInitY
#define cRRInitPosZ     CHexInitXZSin60

#define cRMInitPosX     cHexInitXZ      //Start positions of the Right Middle leg
#define cRMInitPosY     CHexInitY
#define cRMInitPosZ     0

#define cRFInitPosX     CHexInitXZCos60      //Start positions of the Right Front leg
#define cRFInitPosY     CHexInitY
#define cRFInitPosZ     -CHexInitXZSin60

#define cLRInitPosX     CHexInitXZCos60      //Start positions of the Left Rear leg
#define cLRInitPosY     CHexInitY
#define cLRInitPosZ     CHexInitXZSin60

#define cLMInitPosX     cHexInitXZ      //Start positions of the Left Middle leg
#define cLMInitPosY     CHexInitY
#define cLMInitPosZ     0

#define cLFInitPosX     CHexInitXZCos60      //Start positions of the Left Front leg
#define cLFInitPosY     CHexInitY
#define cLFInitPosZ     -CHexInitXZSin60
//--------------------------------------------------------------------
//[Tars factors used in formula to calc Tarsus angle relative to the ground]
#define cTarsConst	720	//4DOF ONLY
#define cTarsMulti	2	//4DOF ONLY
#define cTarsFactorA	70	//4DOF ONLY
#define cTarsFactorB	60	//4DOF ONLY
#define cTarsFactorC	50	//4DOF ONLY

