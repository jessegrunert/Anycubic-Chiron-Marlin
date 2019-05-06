
#include "AcTft.h"
#include "Marlin.h"

#include "ultralcd.h"
#include "planner.h"
#include "stepper.h"
#include "endstops.h"
#include "temperature.h"
#include "cardreader.h"
#include "configuration_store.h"
#include "language.h"
#include "pins_arduino.h"
#include "math.h"
#include "nozzle.h"
#include "printcounter.h"
#include "duration_t.h"
#include "types.h"
#include "parser.h"

// **************************************************
// Anycubic Chiron TFT Suppot
// **************************************************
long 			gcode_N, gcode_LastN, Stopped_gcode_LastN = 0;
uint16_t		tftFileNumber;
unsigned long	tftStartTime = 0;
bool			tftUsbConnected = 0;
uint8_t			tftSdCardStatus = 0xff;
unsigned char 	tftSelectedLevelingMode;
float 			tftSelectedOffsetZ;
int 			tftCurrentState = TFT_OK;
float 			tftPowerLossHeadE = 0.0;
float 			tftPowerLossHeadX = 0.0;
float 			tftPowerLossHeadY = 0.0;
float 			tftPowerLossHeadZ = 0.0;
long  			tftPowerLossCard = 0;
long 			tftPowerLossCardCurrent = 0;

static void debug_print_bilinear_leveling_grid() {
  SERIAL_ECHOLNPGM("Bilinear Leveling Grid:");
  print_2d_array(GRID_MAX_POINTS_X, GRID_MAX_POINTS_Y, 3,
    [](const uint8_t ix, const uint8_t iy) { return z_values[ix][iy]; }
    );
}


void NEWFlushSerialRequestResend()
{
	NewSerial.flush();
}

float tftCodeValue()
{
  return (strtod(&TFTcmdbuffer[TFTbufindr][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindr] + 1], NULL));
}

bool tftCodeSeen(char code)
{
  TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindr], code);
  return (TFTstrchr_pointer != NULL);  //Return True if a character was found
}

void updateSdcard()
{
  static unsigned int sdcardUpdateCounter = 0;
  sdcardUpdateCounter++;
  if (sdcardUpdateCounter %1000 == 0)
  {
  	  sdcardUpdateCounter = 0;
	  bool tftSdCardStatusCurrent = IS_SD_INSERTED;
	  if (tftSdCardStatusCurrent != tftSdCardStatus)
	  {
	    if (tftSdCardStatusCurrent)
	    {
	      card.initsd();
	      NEW_SERIAL_PROTOCOLPGM("J00");
	      TFT_SERIAL_ENTER();            
	    }
	    else
	    {
	      card.release();
	      NEW_SERIAL_PROTOCOLPGM("J01");
	      TFT_SERIAL_ENTER();
	    }
	    tftSdCardStatus = tftSdCardStatusCurrent;
	  }
  }  
}

void updateUsbline()
{
	static long int updateUsblineCounter = 0;
	static bool tftUsbOnline = false;
	if(tftUsbConnected==false)
	{
		if(tftUsbOnline == true)
		{
			updateUsblineCounter++;
			tftUsbOnline = false;
			if(updateUsblineCounter > 1)
			{              
				tftUsbConnected=true;
			  	NEW_SERIAL_PROTOCOLPGM("J03");//usb connect
			  	TFT_SERIAL_ENTER(); 
			  	updateUsblineCounter = 0;              
			}    
		}
	}
	else if(tftUsbConnected==true)
	{
		if(tftUsbOnline == false)
	  	{
	    	updateUsblineCounter++;
	    	if(updateUsblineCounter > 50000) 
	    	{	          
	      		tftUsbOnline = false;
	      		tftUsbConnected = false;
				NEW_SERIAL_PROTOCOLPGM("J12");//ready
				TFT_SERIAL_ENTER();
				updateUsblineCounter = 0;
			}
		}
		else
		{
			updateUsblineCounter = 0;
			tftUsbOnline = false;
		}
	}      
}

void setupFilament()
{
	pinMode(FIL_RUNOUT_PIN,INPUT);
	WRITE(FIL_RUNOUT_PIN,HIGH);
}

void scanFilament()
{
	static char scanFilamentStatus = READ(FIL_RUNOUT_PIN) & 0xff;
	static unsigned char scanFilamentCurrentStatus;
	static unsigned int scanFilamentStatusCounter = 0;
 	scanFilamentCurrentStatus = READ(FIL_RUNOUT_PIN) & 0xff;
 	if (scanFilamentCurrentStatus > scanFilamentStatus)
    {
	    scanFilamentStatusCounter++;
	    if(scanFilamentStatusCounter >= 50000)
	    {
			scanFilamentStatusCounter=0;  
			FilamentLack();
			if((card.sdprinting==true))
			{    
				NEW_SERIAL_PROTOCOLPGM("J23");
				TFT_SERIAL_ENTER();
				card.pauseSDPrint();                    
				tftCurrentState = TFT_PAUSING;   
			}
			else if((card.sdprinting==false))
			{                         
				NEW_SERIAL_PROTOCOLPGM("J15");
				TFT_SERIAL_ENTER();     
			}  
	      	scanFilamentStatus = scanFilamentCurrentStatus;                      
	    }    
	}
	else if (scanFilamentCurrentStatus != scanFilamentStatus)
	{
		scanFilamentStatusCounter=0;
		scanFilamentStatus = scanFilamentCurrentStatus;
	}
}

void scanFan2()
{
  if (thermalManager.degHotend(0) > 65)
  {
    WRITE(V5_COOLING_PIN, HIGH);
  }
  else
  {
    WRITE(V5_COOLING_PIN, LOW);
  }
}

void eventPowerLoss()
{
	if (card.sdprinting)
	{
		SERIAL_ECHOLNPGM("Power failure, saving settings ...");
		tftPowerLossHeadE = current_position[E_AXIS];
	    tftPowerLossHeadX = current_position[X_AXIS];
	    tftPowerLossHeadY = current_position[Y_AXIS];
	    tftPowerLossHeadZ = current_position[Z_AXIS];
	    tftPowerLossCard  = tftPowerLossCardCurrent;
		settings.save();
		SERIAL_ECHOLNPGM("Settings saved");
	}
}

void setupPowerLoss()
{
	pinMode(POWER_LOSS_MONITOR_PIN,INPUT);
	pinMode(POWER_LOSS_CONTROL_PIN,OUTPUT);
	WRITE(POWER_LOSS_CONTROL_PIN,HIGH);
	attachInterrupt(POWER_LOSS_PIN, eventPowerLoss, CHANGE);
}

void gcode_M1005()
{
	if (tftCurrentState == TFT_RECOVERY)
	{
		tftCurrentState = TFT_RECOVERING;
	}
}

void tftGetSdcardCommands()
{
	if(tftCurrentState == TFT_RECOVERING)
  	{
  		clear_command_queue();
  		dumpRecoveryData();
  		fanSpeeds[0] = TFT_MAX_MODEL_COOLING;               
		card.setIndex(tftPowerLossCard); 
		destination[E_AXIS] = tftPowerLossHeadE;
		current_position[E_AXIS] = tftPowerLossHeadE;
		planner.set_e_position_mm(tftPowerLossHeadE);
		planner.buffer_line (X_MIN_POS,Y_MIN_POS,tftPowerLossHeadZ,tftPowerLossHeadE,feedrate_percentage,active_extruder);
		destination[X_AXIS] = tftPowerLossHeadX; //SET A NEW ORIGNAL COORDINATE    
		destination[Y_AXIS] = tftPowerLossHeadY;
		if(current_position[Z_AXIS] > 0.3)
		{
			current_position[Z_AXIS] = tftPowerLossHeadZ - 0.1;
		}
		else
		{
			current_position[Z_AXIS] = tftPowerLossHeadZ;
		}
		feedrate_mm_s = MMM_TO_MMS(2000.0);     
		clearTftState();
	}
}

void dumpRecoveryData()
{
	SERIAL_ECHO("RECOVERY_DATA ");
	SERIAL_ECHOPAIR("E: ", tftPowerLossHeadE);
	SERIAL_ECHOPAIR(" X: ", tftPowerLossHeadX);
	SERIAL_ECHOPAIR(" Y: ", tftPowerLossHeadY);
	SERIAL_ECHOPAIR(" Z: ", tftPowerLossHeadZ);
	SERIAL_ECHOLNPAIR(" SDPOS: ", tftPowerLossCard);
}

void setupFan2()
{
	SET_OUTPUT(V5_COOLING_PIN);
	WRITE(V5_COOLING_PIN, LOW);  
}


void setupSdCard()
{
	SET_INPUT(SD_DETECT_PIN);
	WRITE(SD_DETECT_PIN, HIGH);
	card.initsd();
}

void setAutoLevelingMode()
{
	tftSelectedLevelingMode = AC_MODE_LEVEL_AUTO;;
}

void setManualLevelingMode()
{
	tftSelectedLevelingMode = AC_MODE_LEVEL_MANUAL;;
}

bool isAutoLeveling()
{
	return tftSelectedLevelingMode == AC_MODE_LEVEL_AUTO;
}

void gcode_M1000()
{
	for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
	{
		for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
		{
      z_values[x][y] = -3.5f; 
    }
  }
  bilinear_grid_spacing[0] = int((RIGHT_PROBE_BED_POSITION-LEFT_PROBE_BED_POSITION)/(GRID_MAX_POINTS_X-1));
  bilinear_grid_spacing[1] = int((BACK_PROBE_BED_POSITION-FRONT_PROBE_BED_POSITION)/(GRID_MAX_POINTS_Y-1));
  bilinear_start[0] = LEFT_PROBE_BED_POSITION;
  bilinear_start[1] = FRONT_PROBE_BED_POSITION;
  zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
  tftSelectedOffsetZ = zprobe_zoffset;
  setManualLevelingMode();
  set_bed_leveling_enabled(true);
  enqueue_and_echo_commands_P(PSTR("M500"));
  SERIAL_ECHOLNPGM("Done, Manual Leveling was actived!");
}

void gcode_M1001()
{
	for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
	{
		for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
		{
			z_values[x][y] = -0.1f; 
		}
	}
	bilinear_grid_spacing[0] = int((RIGHT_PROBE_BED_POSITION-LEFT_PROBE_BED_POSITION)/(GRID_MAX_POINTS_X-1));
	bilinear_grid_spacing[1] = int((BACK_PROBE_BED_POSITION-FRONT_PROBE_BED_POSITION)/(GRID_MAX_POINTS_Y-1));
	bilinear_start[0] = LEFT_PROBE_BED_POSITION;
	bilinear_start[1] = FRONT_PROBE_BED_POSITION;
	zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
	tftSelectedOffsetZ = zprobe_zoffset;
	setAutoLevelingMode();
	set_bed_leveling_enabled(true);
	enqueue_and_echo_commands_P(PSTR("M500"));
	debug_print_bilinear_leveling_grid(); 
	SERIAL_ECHOLNPGM("Done, Auto Leveling was actived!");      
}



void setupOffsetZ()
{
	if (isAutoLeveling())
	{
		set_bed_leveling_enabled(true);
		SERIAL_ECHOLNPGM("echo:Auto leveling is active");
	}
	else
	{
	    for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
	    {
	      for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
	      {
	          z_values[x][y] = -3.5f; 
	      }
	    }
	    set_bed_leveling_enabled(true);
	    SERIAL_ECHOLNPGM("echo:Manual leveling is active");
	}
	zprobe_zoffset = Z_PROBE_OFFSET_FROM_EXTRUDER;
	debug_print_bilinear_leveling_grid();
}

void clearTftState()
{
	tftCurrentState = TFT_OK;
}

void tftManageInactivity()
{
	scanFilament();
	scanFan2();
}

void tftLoop()
{
	if (commands_in_queue == 0)
	{
	 	if(tftCurrentState == TFT_PAUSING) //when pause sd printing,send "ok"to tft as read buffer carry out
	 	{
	 		tftCurrentState = TFT_PAUSED;
	 		enqueue_and_echo_commands_P(PSTR("G91"));
	 		enqueue_and_echo_commands_P(PSTR("G1 Z+20"));
			planner.synchronize();
			NEW_SERIAL_PROTOCOLPGM("J18");// pausing done
			TFT_SERIAL_ENTER();
		}
	} 
}

void tftSetup()
{
  	NewSerial.begin(115200);
	TFT_SERIAL_ENTER();
	NEW_SERIAL_PROTOCOLPGM("J17"); //j17 main board reset
	TFT_SERIAL_ENTER();
	delay(10);
	NEW_SERIAL_PROTOCOLPGM("J12"); //  READY
	TFT_SERIAL_ENTER();
	setupOffsetZ();
	delay(10);
	setupPowerLoss();
	PowerOnMusic();
	setupFan2();
	setupSdCard();
	clearTftState();
	_delay_ms(1000);  // wait 1sec to display the splash screen
}

void tftIdle()
{
	tftCommandScan();
	updateSdcard();
	updateUsbline();
}

char itostrBuffer[4];

#define DIGIT(n) ('0' + (n))
#define DIGIMOD(n, f) DIGIT((n)/(f) % 10)
#define RJDIGIT(n, f) ((n) >= (f) ? DIGIMOD(n, f) : ' ')
#define MINUSOR(n, alt) (n >= 0 ? (alt) : (n = -n, '-'))

char* itostr2(const uint8_t& x)
{
    int xx = x;
    itostrBuffer[0] = DIGIMOD(xx, 10);
    itostrBuffer[1] = DIGIMOD(xx, 1);
    itostrBuffer[2] = '\0';
    return itostrBuffer;
}

char* itostr3(const int& x)
{
	int xx = x;
	itostrBuffer[0] = MINUSOR(xx, RJDIGIT(xx, 100));
	itostrBuffer[1] = RJDIGIT(xx, 10);
	itostrBuffer[2] = DIGIMOD(xx, 1);
	itostrBuffer[3] = '\0';
	return itostrBuffer;
}

void tftGetSdStatus()
{
  if(card.cardOK)
  {
    NEW_SERIAL_PROTOCOL(itostr2(card.percentDone()));
  }
  else
  {
    NEW_SERIAL_PROTOCOLPGM("J02");
  }
}

void  tftPrintPause()
{
	if (card.sdprinting)
	{
		card.pauseSDPrint();
		tftCurrentState = TFT_PAUSING;
		NEW_SERIAL_PROTOCOLPGM("J05");//j05 pausing
        TFT_SERIAL_ENTER();
	}
	else
	{
		NEW_SERIAL_PROTOCOLPGM("J16");//j16,if status error, send stop print flag in case TFT no response
        TFT_SERIAL_ENTER();
	}
}
void  tftPrintStart()
{
	if((!planner.movesplanned()) && (tftCurrentState == TFT_OK))
	{
		clearTftState();
		card.startFileprint();
		tftStartTime=millis();
		NEW_SERIAL_PROTOCOLPGM("J06");//hotend heating 
		TFT_SERIAL_ENTER();
	}
}
void  tftPrintStop()
{
	if( (card.sdprinting) || tftCurrentState == TFT_PAUSED)
	{
		clearTftState(); 
		card.stopSDPrint();
		enqueue_and_echo_commands_P(PSTR("M84"));
		NEW_SERIAL_PROTOCOLPGM("J16");//STOP
    	TFT_SERIAL_ENTER();
    	disable_X();
  		disable_Y();
  		disable_Z();
  		disable_E0();                          
	}
}
void  tftPrintResume()
{	
	if (tftCurrentState == TFT_PAUSED)
	{
		enqueue_and_echo_command_now("G1 Z-20");
      	enqueue_and_echo_command_now("G90");   
		clearTftState();
		card.startFileprint();
		NEW_SERIAL_PROTOCOLPGM("J04");//j4ok printing form sd card
		TFT_SERIAL_ENTER();
	}
}
void  tftPrintRecover()
{
	if ((!planner.movesplanned()) && (tftCurrentState == TFT_OK))
    {
    	if(card.cardOK)
        {
        	tftCurrentState = TFT_RECOVERY;
        }
        card.startFileprint();
        tftStartTime=millis();   
        NEW_SERIAL_SUCC_START; 
    }
    TFT_SERIAL_ENTER();
}

void tftCommandScan()
{
	if(TFTbuflen<(TFTBUFSIZE-1))
	{
		tftGetCommand();
	}
	if(TFTbuflen)
	{
		TFTbuflen = (TFTbuflen-1);
		TFTbufindr = (TFTbufindr + 1)%TFTBUFSIZE; 
	}
	static unsigned int Scancount = 0;
	if( (thermalManager.degHotend(0)<5) || ((thermalManager.degHotend(0)>280)) )
	{
		Scancount++;
	}
	if(Scancount>61000)
	{
		//T0 unnormal
		Scancount=0;
		NEW_SERIAL_PROTOCOLPGM("J10");
		TFT_SERIAL_ENTER();
	}
}

void tftGetCommand()
{
	char *starpos = NULL;
	while( NewSerial.available() > 0  && TFTbuflen < TFTBUFSIZE)
	{        
		serial3_char = NewSerial.read(); 
		if (serial3_char == '\n' ||
      		serial3_char == '\r' ||
      		(serial3_char == ':' && TFTcomment_mode == false) ||
      		serial3_count >= (TFT_MAX_CMD_SIZE - 1) )
		{
     		if(!serial3_count)
     		{ //if empty line
       			TFTcomment_mode = false; //for new command
       			return;
           	}
     		TFTcmdbuffer[TFTbufindw][serial3_count] = 0; //terminate string
     		if(!TFTcomment_mode)
     		{
       			TFTcomment_mode = false; //for new command
       			TFTfromsd[TFTbufindw] = false;
       			if(strchr(TFTcmdbuffer[TFTbufindw], 'N') != NULL)
       			{
       				TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], 'N');
       				gcode_N = (strtol(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL, 10));
       				if(gcode_N != gcode_LastN+1 && (strstr_P(TFTcmdbuffer[TFTbufindw], PSTR("M110")) == NULL) )
       				{
                 		NEW_SERIAL_ERROR_START;
                 		NEWFlushSerialRequestResend();
                 		serial3_count = 0;
                 		return;
               		}
               
               		if(strchr(TFTcmdbuffer[TFTbufindw], '*') != NULL)
               		{
                		byte checksum = 0;
                		byte count = 0;
                		while(TFTcmdbuffer[TFTbufindw][count] != '*')
                		{
                 			checksum = checksum^TFTcmdbuffer[TFTbufindw][count++];
               			}
               			TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], '*');
               
	            		if( (int)(strtod(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL)) != checksum)
	            		{
			                NEW_SERIAL_ERROR_START;
			                NEWFlushSerialRequestResend();
			                NEW_SERIAL_ERROR_START;
			                NEWFlushSerialRequestResend();
			                serial3_count = 0;
			                return;
	              		}
            		}
            		else
					{
	             		NEW_SERIAL_ERROR_START;
	             		NEWFlushSerialRequestResend();
	             		serial3_count = 0;
	             		return;
           			}  
           			gcode_LastN = gcode_N;
       				//if no errors, continue parsing
         		}
       			else  // if we don't receive 'N' but still see '*'
       			{
       				if((strchr(TFTcmdbuffer[TFTbufindw], '*') != NULL))
       				{
                 		NEW_SERIAL_ERROR_START;
                 		serial3_count = 0;
                 		return;
               		}
             	}
				if((strchr(TFTcmdbuffer[TFTbufindw], 'A') != NULL))
				{
					TFTstrchr_pointer = strchr(TFTcmdbuffer[TFTbufindw], 'A');
					int tftCommandCode = (int)((strtod(&TFTcmdbuffer[TFTbufindw][TFTstrchr_pointer - TFTcmdbuffer[TFTbufindw] + 1], NULL)));
					switch(tftCommandCode)
					{
	   					case 0://A0 GET HOTEND TEMP 
	             			NEW_SERIAL_PROTOCOLPGM("A0V ");
							NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degHotend(0) + 0.5)));
							TFT_SERIAL_ENTER();
							break;
	                    case 1: //A1  GET HOTEND TARGET TEMP
							NEW_SERIAL_PROTOCOLPGM("A1V ");
							NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degTargetHotend(0) + 0.5)));
							TFT_SERIAL_ENTER();
							break;    
	                    case 2://A2 GET HOTBED TEMP
							NEW_SERIAL_PROTOCOLPGM("A2V ");
							NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degBed() + 0.5)));    
							TFT_SERIAL_ENTER();
							break;                     
	                    case 3://A3 GET HOTBED TARGET TEMP
							NEW_SERIAL_PROTOCOLPGM("A3V ");
							NEW_SERIAL_PROTOCOL(itostr3(int(thermalManager.degTargetBed() + 0.5)));                                  
							TFT_SERIAL_ENTER();
							break;
	                    case 4://A4 GET FAN SPEED 
						{ 
							unsigned int temp;
							temp=((fanSpeeds[0]*100)/TFT_MAX_MODEL_COOLING+1);
							temp=constrain(temp,0,100);
							NEW_SERIAL_PROTOCOLPGM("A4V ");    
							NEW_SERIAL_PROTOCOL(temp);  
							TFT_SERIAL_ENTER();
							break;
						}                        
	                    case 5:// A5 GET CURRENT COORDINATE 
							NEW_SERIAL_PROTOCOLPGM("A5V");
							TFT_SERIAL_SPACE();
							NEW_SERIAL_PROTOCOLPGM("X: ");
							NEW_SERIAL_PROTOCOL(current_position[X_AXIS]);
							TFT_SERIAL_SPACE();
							NEW_SERIAL_PROTOCOLPGM("Y: ");
							NEW_SERIAL_PROTOCOL(current_position[Y_AXIS]);
							TFT_SERIAL_SPACE();
							NEW_SERIAL_PROTOCOLPGM("Z: ");
							NEW_SERIAL_PROTOCOL(current_position[Z_AXIS]); 
							TFT_SERIAL_SPACE(); 
							TFT_SERIAL_ENTER();                       
							break;
	                    case 6: //A6 GET SD CARD PRINTING STATUS
							if(card.sdprinting)
							{
								NEW_SERIAL_PROTOCOLPGM("A6V ");
								tftGetSdStatus();
							}
							else
							{
								NEW_SERIAL_PROTOCOLPGM("A6V ---");
							}
							TFT_SERIAL_ENTER();
							break;                          
	                    case 7://A7 GET PRINTING TIME
	                    {                           
							NEW_SERIAL_PROTOCOLPGM("A7V ");
							if(tftStartTime != 0) //print time
							{
								uint16_t time = millis()/60000 - tftStartTime/60000;                     
								NEW_SERIAL_PROTOCOL(itostr3(time/60));
								TFT_SERIAL_SPACE();
								NEW_SERIAL_PROTOCOLPGM("H");
								TFT_SERIAL_SPACE();
								NEW_SERIAL_PROTOCOL(itostr3(time%60));
								TFT_SERIAL_SPACE();
								NEW_SERIAL_PROTOCOLPGM("M");
							}
							else
							{
								TFT_SERIAL_SPACE();
								NEW_SERIAL_PROTOCOLPGM("999:999");
							}
							TFT_SERIAL_ENTER();                       
							break;                      
						}
	                    case 8: //A8 GET  SD LIST
							if(!IS_SD_INSERTED)
							{
								NEW_SERIAL_PROTOCOLPGM("J02");
								TFT_SERIAL_ENTER();
							}
							else
							{
								if(tftCodeSeen('S'))
								{
									tftFileNumber = tftCodeValue();
								}
								NEW_SERIAL_PROTOCOLPGM("FN ");
								TFT_SERIAL_ENTER();
								card.ls();
								NEW_SERIAL_PROTOCOLPGM("END");
								TFT_SERIAL_ENTER();
							}
							break;
	                    case 9: // a9 pasue sd
	                    	tftPrintPause();
	                    	break;
	                    case 10:// A10 resume sd print
	                    	tftPrintResume();
	                      	break;
	                    case 11://A11 STOP SD PRINT
	                    	tftPrintStop();                      
	                        break;                       
	                    case 12: //a12 kill
	                    	break;                          
	                    case 13: //A13 SELECTION FILE
							if((!planner.movesplanned()))
							{
								starpos = (strchr(TFTstrchr_pointer + 4,'*'));
								if(starpos!=NULL)
								{
									*(starpos-1)='\0';
								}
								card.openFile(TFTstrchr_pointer + 4,true);
								TFT_SERIAL_ENTER();
							}
							break;
	                    case 14: //A14 START PRINTING
	                    	tftPrintStart();
	                       	break;
	                    case 15://A15 RESUMING FROM OUTAGE
	                      	tftPrintRecover();
	                      	break;
	                    case 16://a16 set hotend temp
						{
							unsigned int tempvalue;
							if(tftCodeSeen('S')) 
							{
								tempvalue=constrain(tftCodeValue(),0,275);       
								thermalManager.setTargetHotend(tempvalue,0);  
							} 
							else if((tftCodeSeen('C'))&&(!planner.movesplanned())) 
							{
								tempvalue=constrain(tftCodeValue(),0,275);       
								thermalManager.setTargetHotend(tempvalue,0);                  
							}
							break;
						}
	                    case 17:// a17 set hotbed temp
						{
							unsigned int tempbed;
							if(tftCodeSeen('S'))
							{
								tempbed=constrain(tftCodeValue(),0,150);
								thermalManager.setTargetBed(tempbed);
							}
							break;                
						}
	                    case 18://a18 set fan speed
							unsigned int temp;
							if (tftCodeSeen('S'))
							{
								unsigned int test=179;
								temp=(tftCodeValue()*TFT_MAX_MODEL_COOLING/100);
								temp=constrain(temp,0,TFT_MAX_MODEL_COOLING);
								fanSpeeds[0] = temp;
							}               
							else
							{
								fanSpeeds[0]=TFT_MAX_MODEL_COOLING; //fanSpeeds[0]=179; 
							}
							TFT_SERIAL_ENTER();                          
							break;
						case 19: // A19 CLOSED STEPER DIRV
							if((!tftUsbConnected)&&(!card.sdprinting))
							{                             
								quickstop_stepper(); 
								disable_X();
								disable_Y();
								disable_Z();
								disable_E0();                                                       
							}                          
							TFT_SERIAL_ENTER();
							break;                          
	                    case 20:// a20 read printing speed
	                    {
	                    	if(tftCodeSeen('S'))
	                     	{
	                      		feedrate_percentage=constrain(tftCodeValue(),40,999);
	                    	}
	                    	else
	                    	{
	                     		NEW_SERIAL_PROTOCOLPGM("A20V ");
	                     		NEW_SERIAL_PROTOCOL(feedrate_percentage);                            
	                     		TFT_SERIAL_ENTER();
	                   		}
	                   		break;
	                 	}
	                    case 21: //a21 all home
							if((!planner.movesplanned()))
							{
								if(tftCodeSeen('X')||tftCodeSeen('Y')||tftCodeSeen('Z'))
								{
									if(tftCodeSeen('X'))
									{
										enqueue_and_echo_commands_P(PSTR("G28 X"));
									}
									if(tftCodeSeen('Y'))
									{
										enqueue_and_echo_commands_P(PSTR("G28 Y"));
									}
									if(tftCodeSeen('Z'))
									{
										enqueue_and_echo_commands_P(PSTR("G28 Z"));
									}
								}
								else if (tftCodeSeen('C'))
								{
									enqueue_and_echo_commands_P(PSTR("G28"));
								}                 
							}                          
	                		break;
	                    case 22: // A22 move X /Y/Z
	                    {
	                    	if((!planner.movesplanned()))
	                    	{
		                         float coorvalue;
		                         unsigned int movespeed=0;
		                         char value[30];
	                     		if(tftCodeSeen('F'))
	                     		{
	                            	movespeed =tftCodeValue();//movespeed=constrain(tftCodeValue(), 1,5000);                     
	                         	}
	                         	enqueue_and_echo_commands_P(PSTR("G91"));                       
	                         	if (tftCodeSeen('X'))
	                         	{
	                          		coorvalue=tftCodeValue(); 
	                          		if((coorvalue<=0.2)&&coorvalue>0)
	                          		{
	                           			sprintf_P(value,PSTR("G1 X0.1F%i"),movespeed);
	                           			enqueue_and_echo_command_now(value);
	                         		}
	                         		else if ((coorvalue<=-0.1)&&coorvalue>-1)
	                         		{
	                           			sprintf_P(value,PSTR("G1 X-0.1F%i"),movespeed);
	                           			enqueue_and_echo_command_now(value);
	                         		}
	                         		else
	                         		{
	                           			sprintf_P(value,PSTR("G1 X%iF%i"),int(coorvalue),movespeed);
	                           			enqueue_and_echo_command_now(value);
	                         		}                      
	                       		}
	                       		else if (tftCodeSeen('Y'))
	                       		{
	                        		coorvalue=tftCodeValue();
	                        		if ((coorvalue<=0.2)&&coorvalue>0)
	                        		{
	                         			sprintf_P(value,PSTR("G1 Y0.1F%i"),movespeed);
	                         			enqueue_and_echo_command_now(value);
	                       			}
	                       			else if ((coorvalue<=-0.1)&&coorvalue>-1)
	                       			{
	                        	 		sprintf_P(value,PSTR("G1 Y-0.1F%i"),movespeed);
	                         			enqueue_and_echo_command_now(value);
	                       			}
	                       			else
	                       			{
	                         			sprintf_P(value,PSTR("G1 Y%iF%i"),int(coorvalue),movespeed);
	                         			enqueue_and_echo_command_now(value);
	                       			}                                  
	                     		}  
	                     		else if(tftCodeSeen('Z'))
	                     		{
	                      			coorvalue=tftCodeValue();
	                      			if((coorvalue<=0.2)&&coorvalue>0)
	                      			{
	                       				sprintf_P(value,PSTR("G1 Z0.1F%i"),movespeed);
	                       				enqueue_and_echo_command_now(value);
	                     			}
	                     			else if((coorvalue<=-0.1)&&coorvalue>-1)
	                     			{
	                       				sprintf_P(value,PSTR("G1 Z-0.1F%i"),movespeed);
	                       				enqueue_and_echo_command_now(value);
	                     			}
	                     			else
	                     			{
	                       				sprintf_P(value,PSTR("G1 Z%iF%i"),int(coorvalue),movespeed);
	                       				enqueue_and_echo_command_now(value);
	                     			}                                     
	                   			}
		                       else if(tftCodeSeen('E'))
		                       {
		                        	coorvalue=tftCodeValue();
		                        	if( (coorvalue<=0.2)&&coorvalue>0)
		                        	{
		                         		sprintf_P(value,PSTR("G1 E0.1F%i"),movespeed);
		                         		enqueue_and_echo_command_now(value);
		                       		}
		                       		else if ((coorvalue<=-0.1)&&coorvalue>-1)
		                       		{
		                         		sprintf_P(value,PSTR("G1 E-0.1F%i"),movespeed);
		                         		enqueue_and_echo_command_now(value);
		                       		}
		                       		else
		                       		{
		                         		sprintf_P(value,PSTR("G1 E%iF500"),int(coorvalue));
		                         		enqueue_and_echo_command_now(value);
		                       		}                    
	                     		}
	                 			enqueue_and_echo_commands_P(PSTR("G90"));                         
	               			}
	               			TFT_SERIAL_ENTER();                          
	               			break; 
	           			}                         
	                    case 23: //a23 prheat pla
	                    	if((!planner.movesplanned()))
	                    	{                  
		                         thermalManager.setTargetBed(50);
		                         thermalManager.setTargetHotend(190, 0);
		                         NEW_SERIAL_SUCC_START;    
		                         TFT_SERIAL_ENTER();                       
	                   		}
	                   		break;
	                    case 24://a24 prheat abs
	                        if((!planner.movesplanned()))
	                        {
								thermalManager.setTargetBed(80);
								thermalManager.setTargetHotend(240, 0);
								NEW_SERIAL_SUCC_START;
								TFT_SERIAL_ENTER();
	                    	}
	                   		break;                                             
	                    case 25: //a25 cool down
	                        if((!planner.movesplanned()))
	                        {
	                         	thermalManager.setTargetHotend(0,0);
	                         	thermalManager.setTargetBed(0);
	                        	NEW_SERIAL_PROTOCOLPGM("J12");//
	                        	TFT_SERIAL_ENTER();
	                        }
	                        break;
	                    case 26://a26 refresh
	                        card.initsd();
	                        if(!IS_SD_INSERTED)
	                        {
	                        	NEW_SERIAL_PROTOCOLPGM("J02");TFT_SERIAL_ENTER();
	                       	}
	                   		break;              
	                    case 29://A29 bed grid read  
	                    {
	                        unsigned char temp_x=0,temp_y=0;                       
	                        if(tftCodeSeen('X'))
	                        {
	                        temp_x=tftCodeValue();
	                        }
	                        if(tftCodeSeen('Y'))
	                        {
	                          temp_y=tftCodeValue();
	                        }
	                        float Zvalue=z_values[temp_x][temp_y];
	                        Zvalue=Zvalue*100;
	                        NEW_SERIAL_PROTOCOLPGM("A29V ");
	                        NEW_SERIAL_PROTOCOL(Zvalue);                            
	                        TFT_SERIAL_ENTER();
	                        break;
	                    }
	                    case 30://a30 auto leveling
	                    {
	                    	if(!isAutoLeveling())
	                        {                            
	                        	NEW_SERIAL_PROTOCOLPGM("J24");// forbid auto leveling
	                        	TFT_SERIAL_ENTER();    
	                        	break;
	                        }                      
	                        if((planner.movesplanned())||(card.sdprinting)) 
	                        {
	                       		NEW_SERIAL_PROTOCOLPGM("J24");// forbid auto leveling
	                        	TFT_SERIAL_ENTER();                          
	                        }
	                        else 
	                        {
	                        	NEW_SERIAL_PROTOCOLPGM("J26");//start auto leveling
	                        	TFT_SERIAL_ENTER();                                  
	                        } 
	                        if(tftCodeSeen('S') )
	                        {
	                          setAutoLevelingMode();
	                          enqueue_and_echo_commands_P(PSTR("G28\nG29"));
	                        }                      
	                        break;
	                    }
	                    case 31: //a31 zoffset set get or save
	                    {
	                    	if(!isAutoLeveling())
	                    	{
	                     		break;
	                    	}
	                    	if(tftCodeSeen('S'))
	                    	{           
	                    		float value=constrain(tftCodeValue(),-1.0,1.0);
	                      		tftSelectedOffsetZ += value;         
	                       		for (uint8_t x = 0; x < GRID_MAX_POINTS_X; x++)
	                       		{
	                         		for (uint8_t y = 0; y < GRID_MAX_POINTS_Y; y++)
	                         		{
	                           			z_values[x][y] += value;
	                         		}
	                       		}
	                       		NEW_SERIAL_PROTOCOLPGM("A31V ");
	                       		NEW_SERIAL_PROTOCOL(tftSelectedOffsetZ);
	                       		TFT_SERIAL_ENTER();             
	                     	} 
	                     	if(tftCodeSeen('G'))
	                     	{
	                       		NEW_SERIAL_PROTOCOLPGM("A31V ");
	                       		NEW_SERIAL_PROTOCOL(tftSelectedOffsetZ);
	                       		TFT_SERIAL_ENTER();
	                     	}
	                     	if(tftCodeSeen('D'))
	                     	{
	                     		enqueue_and_echo_commands_P(PSTR("M500"));
	                     	}
	                     	TFT_SERIAL_ENTER();                      
	                     	break;
	                   	}
	                    case 33:// a33 get version info    
	                    	NEW_SERIAL_PROTOCOLPGM("J33 ");
	                        NEW_SERIAL_PROTOCOLPGM(MSG_MY_VERSION);                         
	                        TFT_SERIAL_ENTER();                          
	                        break;
	                    case 34: //a34 bed grid write
	                    {
	                    	uint8_t x_array=0,y_array=0,result=0;     
	                      	if(!isAutoLeveling())
	                      	{
	                       		break;
	                     	}                       
							if(tftCodeSeen('X'))
							{
								x_array=constrain(tftCodeValue(),0,GRID_MAX_POINTS_X);
							}
							if(tftCodeSeen('Y'))
							{
								y_array=constrain(tftCodeValue(),0,GRID_MAX_POINTS_Y);
							}	                           
							if(tftCodeSeen('V'))
							{
								float i=constrain(tftCodeValue()/100,-10,10);
								z_values[x_array][y_array] = i;
							} 
							if(tftCodeSeen('S'))
							{
								settings.save();
							}
	                        if(tftCodeSeen('C'))
	                        {
	                         	//settings.load();
	                        }
	                        break;
	                    }
	                   	default:
	                   		break;
	                }   
	        	}       
				TFTbufindw = (TFTbufindw + 1)%TFTBUFSIZE;
				TFTbuflen += 1;
			}
     		serial3_count = 0; //clear buffer
		}
		else
		{
			if(serial3_char == ';')
			{
				TFTcomment_mode = true;
			}
			if(!TFTcomment_mode)
			{
				TFTcmdbuffer[TFTbufindw][serial3_count++] = serial3_char;
			}
		}     
	}
}
