#ifndef ACVARSHEADER_H
#define ACVARSHEADER_H
	#include "MarlinConfig.h" 

	#define MAX_MODEL_COOLING_PRECENT_VALUE 1
  	#define MSG_MY_VERSION "V1.1_Marlin-1.1.9"
	#define TFT_MAX_CMD_SIZE 96
    #define TFTBUFSIZE 4
	
	extern uint16_t			tftFileNumber;
	extern unsigned long 	tftStartTime;
	extern bool 			tftUsbConnected;
	extern uint8_t 			tftSdCardStatus;
	extern unsigned char 	tftSelectedLevelingMode;
	extern float 			tftSelectedOffsetZ;;

	//Power loss state
	extern int 				tftCurrentState;
	extern float 			tftPowerLossHeadE;
	extern float 			tftPowerLossHeadX;
	extern float 			tftPowerLossHeadY;
	extern float 			tftPowerLossHeadZ;
	extern long  			tftPowerLossCard;
	extern long 			tftPowerLossCardCurrent;

	extern uint8_t			commands_in_queue;

	static char 	TFTcmdbuffer[TFTBUFSIZE][TFT_MAX_CMD_SIZE];
	static int 		TFTbuflen = 0;
	static int 		TFTbufindr = 0;
	static int 		TFTbufindw = 0;
	static bool 	TFTfromsd[TFTBUFSIZE];
	static bool 	TFTcomment_mode = false;
	static char 	*TFTstrchr_pointer;
	static char 	serial3_char;
	static int 		serial3_count = 0;
	
	
	static unsigned int  TFT_MAX_MODEL_COOLING = MAX_MODEL_COOLING_PRECENT_VALUE * 255;
	static unsigned char AC_MODE_LEVEL_AUTO   = 0x55;
	static unsigned char AC_MODE_LEVEL_MANUAL = 0xaa;

	#define TFT_OK 			 0
	#define TFT_PAUSING 	10
	#define TFT_PAUSED 		11
	#define TFT_RECOVERY 	20
	#define TFT_RECOVERING 	21

	

	void setupSdCard();
	void setupFilament();
	void setupOffsetZ();
	void setupFan2();
	void setupPowerLoss();

	void eventPowerLoss();
	void dumpRecoveryData();

	void updateSdcard();
	void updateUsbline();

	void scanFilament();
	void scanFan2();

	float tftCodeValue();
	bool  tftCodeSeen(char code);
	void  tftCommandScan();
	void  tftGetSdStatus();
	void  tftGetCommand();
	void  tftSetup();
	void  tftIdle();
	void  tftLoop();
	void  tftManageInactivity();
	void  tftGetSdcardCommands();

	void  tftPrintPause();
	void  tftPrintStart();
	void  tftPrintStop();
	void  tftPrintResume();
	void  tftPrintRecover();

	void clearTftState();

	void setAutoLevelingMode();
	void setManualLevelingMode();
	bool isAutoLeveling();

	void gcode_M1000();
	void gcode_M1001();
	void gcode_M1005();
	
	void NEWFlushSerialRequestResend();
#endif // ACVARSHEADER_H