#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <fstream>

#include "DT5742.h"


unsigned int channelOffset(  int fHandle, uint8_t group, uint8_t channel) {

  if (channel > 7) {
    std::cerr << "Channel OFfset, Bad Channel!" << channel << std::endl;
    return 0;
  }
  if (group > 1) {
    std::cerr << "Only have groups 0 and 1" << std::endl;
    return 0; 
  }

  // Chs 0-7 @ group 0, Chs 8-15 group 1
  int groupReg = 0x0;
  int offsetReg = 0x0; 
  if (group == 0) {
    groupReg = 0x10A4;
    offsetReg = 0x1098;
  }
  else {
    groupReg = 0x11A4;
    offsetReg = 0x1198; 
  }
  // First write the channel to the group register it belongs to
  auto err = CAEN_DGTZ_WriteRegister(fHandle, groupReg, channel);
  //
  if (err != CAEN_DGTZ_Success) {
    std::cerr << "Error reading DC offset: " << err << std::endl;
    return 0;
  }

  
  unsigned int dcOffset; 
  err = CAEN_DGTZ_ReadRegister(fHandle, offsetReg, &dcOffset);

  return (dcOffset&0xFFFF); 
  
}

bool writeDCOffset( int fHandle, uint8_t group, uint8_t channel, unsigned int offset) {

  if (channel > 7) {
    std::cerr << "Channel OFfset, Bad Channel!" << channel << std::endl;
    return false;
  }
  if (group > 1) {
    std::cerr << "Only have groups 0 and 1" << std::endl;
    return false; 
  }

  unsigned int offsetReg = 0x0;
  unsigned int groupStatusReg = 0x0; 
  if (group == 0) {
    offsetReg = 0x1098;
    groupStatusReg = 0x1088; 
  }
  else {
    offsetReg = 0x1198;
    groupStatusReg = 0x1188; 
  }

  if (offset > 65536) {
    std::cerr << "DC Offset too large, only have 16-bits!" << std::endl;
    return false;
  }

  // First verify that the second bit of the group status registers remains 0;
  unsigned int status = 0x0; 
  auto err = CAEN_DGTZ_ReadRegister(fHandle, groupStatusReg, &status);

  if (err != CAEN_DGTZ_Success) {
    std::cerr << "Error reading Group Status: " << err << std::endl;
    return false;
  }

  if (0x2 == (status & 0x2)) {
    std::cerr << "Group Status bit currently set, wait a while." << std::endl;
    return false;
  }

  // Good to go
  unsigned int toWrite = offset;
  toWrite |= (channel << 16);
  
  err = CAEN_DGTZ_WriteRegister(fHandle, offsetReg, toWrite);

  if (err != CAEN_DGTZ_Success) {
    std::cerr << "Error writing DC Offset: " << err << std::endl;
    return false;
  }


  return true; 
}


void DT5742::printTriggerMode(CAEN_DGTZ_TriggerMode_t tmode) {
  switch (tmode) {
  case CAEN_DGTZ_TRGMODE_DISABLED: {
    std::cout << "Disabled Trigger" << std::endl;
    break;
  }
  case CAEN_DGTZ_TRGMODE_EXTOUT_ONLY: {
    std::cout << "External Out Trigger Mode Only" << std::endl;
    break;
  }
  case CAEN_DGTZ_TRGMODE_ACQ_ONLY: {
    std::cout << "Acquisition Trigger Mode Only" << std::endl;
    break;
  }
  case CAEN_DGTZ_TRGMODE_ACQ_AND_EXTOUT: {
    std::cout << "Acquisition Trigger Mode and External Out" << std::endl;
    break;
  }
  }
}
//=======================
void DT5742::printFrequency(CAEN_DGTZ_DRS4Frequency_t freq) {
  switch (freq) {
  case CAEN_DGTZ_DRS4_5GHz:
    std::cout << "DRS4 Freq: 5 GHz" << std::endl;
    break;
  case CAEN_DGTZ_DRS4_2_5GHz:
    std::cout << "DRS4 Freq: 2.5 GHz" << std::endl;
    break;
  case CAEN_DGTZ_DRS4_1GHz:
    std::cout << "DRS4 Freq: 1.0 GHz" << std::endl;
    break;
  case CAEN_DGTZ_DRS4_750MHz:
    std::cout << "DRS4 Freq: 0.75 GHz" << std::endl;
    break;
  case _CAEN_DGTZ_DRS4_COUNT_:
    std::cout << "DRS4 COUNT?" << std::endl;
    break;
  default:
    std::cout << "Unknown Freq" << std::endl;
    break;
  }
}
//=======================
void DT5742::PrintLastErrorCode() {
  std::cout << ">> ";
  switch (fLastErrorCode) {
  case (CAEN_DGTZ_Success):
    std::cout << "Operation completed successfully";
    break;
  case (CAEN_DGTZ_CommError):
    std::cout << "Communication error";
    break;
  defaut:
    std::cout << "Check manual CAENDigitizer page 12 for code:" << fLastErrorCode;
  }
  std::cout << " >>" << std::endl;
}
//=======================
int DT5742::Close() {
  fLastErrorCode = CAEN_DGTZ_CloseDigitizer(fHandle); 
  return fLastErrorCode;
}
//=======================
int DT5742::Init() {
  std::cout << "Init() ... ";
  fLastErrorCode = CAEN_DGTZ_OpenDigitizer(CAEN_DGTZ_USB, // link type
					   0, // link num
					   0, // conetNode
					   0, // VMEBaseAddress
					   &fHandle);
  if (fLastErrorCode != CAEN_DGTZ_Success) {
    PrintLastErrorCode();
    exit(1);
  }

  std::cout << "[SUCCESULL]" << std::endl;

  fLastErrorCode = CAEN_DGTZ_Reset(fHandle); // Reset everything 

  std::this_thread::sleep_for(std::chrono::milliseconds(500)); // need to think of a better approach

  CAEN_DGTZ_SetRecordLength(fHandle,1024); // Set the record length to 1024 samples (5 GS / s)

  uint32_t gemask = 0x3; 
  CAEN_DGTZ_SetGroupEnableMask(fHandle, gemask); // groups "11"
  gemask = 0xffff;
  CAEN_DGTZ_SetChannelEnableMask(fHandle, gemask); // channels "1111 1111 1111 1111"
  //CAEN_DGTZ_SetGroupFastTriggerThreshold(fHandle, 0, 26214); //Set Trigger Threshold to ~500 mV
  //CAEN_DGTZ_SetGroupFastTriggerThreshold(fHandle, 1, 26214); //Set Trigger Threshold to ~500 mV
  CAEN_DGTZ_SetMaxNumAggregatesBLT(fHandle, 1); // 1000? // 1 is fast but is there a risk of lose?

  //CAEN_DGTZ_DRS4Frequency_t freq = CAEN_DGTZ_DRS4_1GHz;
  //CAEN_DGTZ_DRS4Frequency_t freq = CAEN_DGTZ_DRS4_2_5GHz;
  CAEN_DGTZ_DRS4Frequency_t freq = CAEN_DGTZ_DRS4_5GHz;
  CAEN_DGTZ_SetDRS4SamplingFrequency(fHandle, freq); // 0: 5Gs   2: 1Gs  3: 0.75 GHz

  //CAEN_DGTZ_SetAcquisitionMode(fHandle,CAEN_DGTZ_SW_CONTROLLED); // START/STOP SOFTWARE CONTROLLED
  //CAEN_DGTZ_SetSWTriggerMode(fHandle, CAEN_DGTZ_TRGMODE_ACQ_ONLY);  // SOFTWARE TRIGGER?
  //CAEN_DGTZ_SetIOLevel(fHandle, CAEN_DGTZ_IOLevel_NIM); // TTL like trigger
  CAEN_DGTZ_SetFastTriggerMode(fHandle, CAEN_DGTZ_TRGMODE_ACQ_ONLY);// Trigger with TR0
  //CAEN_DGTZ_SetFastTriggerDigitizing(fHandle, CAEN_DGTZ_ENABLE);    // Enables digitization of TR0
  CAEN_DGTZ_SetFastTriggerDigitizing(fHandle, CAEN_DGTZ_DISABLE);    // Enables digitization of TR0

  //fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x811C, &reg);
  uint32_t reg = (3<<18)|(1<<16);
  fLastErrorCode = CAEN_DGTZ_WriteRegister(fHandle, 0x811C, reg);
  
  //CAEN_DGTZ_SetGroupDCOffset(fHandle, 0, 0x0000); // from 0x0000 to 0xffff (65535) => default 0x7fff
  //CAEN_DGTZ_SetGroupDCOffset(fHandle, 1, 0x0000); // from 0x0000 to 0xffff (65535) => default 0x7fff  

  reg = (0xf<<17)|(0x6ABC);
  fLastErrorCode = CAEN_DGTZ_WriteRegister(fHandle, 0x1098, reg);
  reg = (0xf<<17)|(0x6ABC);
  fLastErrorCode = CAEN_DGTZ_WriteRegister(fHandle, 0x1198, reg);

  
  PrintInfo();
  
  //DIGITIZER IS NOW OPEN
  return fLastErrorCode;
}
//=======================
int DT5742::Read(int run) { // run = number of triggers
  CAEN_DGTZ_EventInfo_t eventInfo;
  char *EventPtr;
  CAEN_DGTZ_X742_EVENT_t *Event742;
  char *buffer = NULL;
  uint32_t size;
  fLastErrorCode = CAEN_DGTZ_Success;
  fLastErrorCode = CAEN_DGTZ_AllocateEvent(fHandle, (void**)&Event742);
  fLastErrorCode = CAEN_DGTZ_MallocReadoutBuffer(fHandle,&buffer,&size);

  // Open File
  std::ofstream fout;
  fout.open("output.dat", std::fstream::out | std::fstream::binary | std::fstream::trunc); 
  
  // Start acquisition
  fLastErrorCode = CAEN_DGTZ_SWStartAcquisition(fHandle);
  if (fLastErrorCode != CAEN_DGTZ_Success) {
    std::cerr << "Could not start acquisition: " << fLastErrorCode << std::endl;
  }

  // Monitor status
  int nRecEvents = 0;
  int nTimesNoReading = 0;
  int nTimesAtFull = 0;
  int nIterations = 0;
  bool InSpill = false;
  auto eventStart = std::chrono::steady_clock::now();
  for(;(nRecEvents<run) || InSpill;++nIterations) {

    //if((nRecEvents%300)==0 && nRecEvents>0)
    if((nIterations%1000)==0)
      std::cout << "Number of events in disk: " << nRecEvents << std::endl;
    //std::this_thread::sleep_for(std::chrono::microseconds(100)); //think of a better way to do this!
    // Read Acq. Status Register
    uint32_t reg; 
    fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x8104, &reg); //AcquisitionStatus
    //if (fLastErrorCode != CAEN_DGTZ_Success)
    //std::cout << "Could not read out register: " << fLastErrorCode << std::endl;
    //std::cout << "Register value:" << std::hex << "0x" << reg << " EVENTS:" << std::dec << nRecEvents << std::endl;
    int running = (reg>>2) & 0b1; // 3rd bit => 1=running | 0=stopped
    int ready   = (reg>>3) & 0b1; // 4th bit => 1=ev ready | 0=no event
    int buffull = (reg>>4) & 0x1; // 5th bit => 1=full | 0=not full
    if((buffull==1)) {
      nTimesAtFull++;
      //std::cout << "[ WARNING: running " << running << " | ready " << ready << " | buffull " << buffull << std::endl;
    }
    if(ready==1) {
      //atempt a write operation
      fLastErrorCode = CAEN_DGTZ_ReadData(fHandle,
					  CAEN_DGTZ_SLAVE_TERMINATED_READOUT_MBLT,
					  buffer,
					  &size);
      if (fLastErrorCode)
	std::cout << "Error " << fLastErrorCode << std::endl;
      fout.write(buffer, size);
      nRecEvents++;
      eventStart = std::chrono::steady_clock::now();
      //fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x8104, &reg); //AcquisitionStatus
      //std::cout << "Register value:" << std::hex << "0x" << reg << std::endl;
      //running = (reg>>2) & 0b1; // 3rd bit => 1=running | 0=stopped
      //ready   = (reg>>3) & 0b1; // 4th bit => 1=ev ready | 0=no event
      //buffull = (reg>>4) & 0x1; // 5th bit => 1=full | 0=not full
      //std::cout << "] WARNING: running " << running << " | ready " << ready << " | buffull " << buffull << std::endl;
    } else {
      nTimesNoReading++;
    }

    auto eventStop = std::chrono::steady_clock::now();

    std::chrono::duration<double> elapsed = eventStop-eventStart;
    if (elapsed.count() > 1) {
      InSpill = false;
    }
    else {
      InSpill = true;
    }
  }
  std::cout << "=================" << std::endl;
  std::cout << "===== STATS =====" << std::endl;
  std::cout << "Number of events acquired: " << nRecEvents << std::endl;
  std::cout << "Number of cycles with no reading: " << nTimesNoReading << std::endl;
  std::cout << "Number of cycles with buffer full: " << nTimesAtFull << std::endl;
  std::cout << "Number of total read attempts: " <<  nIterations << std::endl;
  if(nIterations != (nRecEvents + nTimesNoReading)) {
    std::cout << "  ** WARNING: check stats  ** " << std::endl;
  } else {
    std::cout << "  ** not a single missed event at this cpp program speed ** " << std::endl;
  }
  
  fout.close();
  fLastErrorCode = CAEN_DGTZ_SWStopAcquisition(fHandle); 
  if (fLastErrorCode != CAEN_DGTZ_Success)
    std::cout << "Could not stop acquisition: " << fLastErrorCode << std::endl;

  return 0; 
}
//=======================
void DT5742::PrintInfo() {
  std::cout << "=== COMMUNICATION ====" << std::endl;

  fLastErrorCode = CAEN_DGTZ_GetInfo(fHandle, &fBoardInfo);
  if (fLastErrorCode != CAEN_DGTZ_Success) {
    PrintLastErrorCode();
    Close();
    return;
  }
  std::cout <<   "Model: " << fBoardInfo.ModelName << std::endl;
  std::cout <<   "Channels: " << fBoardInfo.Channels << std::endl;
  std::cout <<   "ADC_NBits: " << fBoardInfo.ADC_NBits << std::endl;
  std::cout <<   "ROC FPGA: " << fBoardInfo.ROC_FirmwareRel << std::endl;
  std::cout <<   "AMC FPGA: " << fBoardInfo.AMC_FirmwareRel << std::endl;
  std::cout <<   "Board Family: " << fBoardInfo.FamilyCode << std::endl; 

  std::cout << "=== TRIGGER ====" << std::endl;

  CAEN_DGTZ_TriggerMode_t tmode; 
  fLastErrorCode = CAEN_DGTZ_GetSWTriggerMode(fHandle, &tmode);
  std::cout << "Software Trigger Mode: "; 
  printTriggerMode(tmode);
  
  fLastErrorCode = CAEN_DGTZ_GetExtTriggerInputMode(fHandle, &tmode);
  std::cout << "External Trigger Mode: "; 
  printTriggerMode(tmode); 

  CAEN_DGTZ_RunSyncMode_t runsyncmode;
  fLastErrorCode = CAEN_DGTZ_GetRunSynchronizationMode(fHandle, &runsyncmode);
  std::cout << "Run synchronization mode: ";
  std::cout << runsyncmode << std::endl;
  
  CAEN_DGTZ_IOLevel_t iolevel;
  fLastErrorCode = CAEN_DGTZ_GetIOLevel(fHandle, &iolevel);
  std::cout << "IO Level: ";
  if(iolevel==CAEN_DGTZ_IOLevel_NIM)
    std::cout << "[NIM] ";
  else
    std::cout << "[TTL] ";
  std::cout << std::endl;
  
  CAEN_DGTZ_TriggerPolarity_t polarity;
  fLastErrorCode = CAEN_DGTZ_GetTriggerPolarity(fHandle, 0, &polarity);
  std::cout << "Trigger Polarity: ";
  if(polarity==CAEN_DGTZ_TriggerOnRisingEdge)
    std::cout << "[On rising edge] ";
  else
    std::cout << "[On falling edge] ";
  std::cout << std::endl;

  uint32_t TriggerThreshold = 0;
  fLastErrorCode = CAEN_DGTZ_GetGroupFastTriggerThreshold(fHandle, 0, &TriggerThreshold);
  std::cout << "Trig Thresh Group 0:" << TriggerThreshold << std::endl; 
  fLastErrorCode = CAEN_DGTZ_GetGroupFastTriggerThreshold(fHandle, 1, &TriggerThreshold);
  std::cout << "Trig Thresh Group 1:" << TriggerThreshold << std::endl;

  uint32_t FastTrigDCOffset = 0;
  fLastErrorCode = CAEN_DGTZ_GetGroupFastTriggerDCOffset(fHandle, 0, &FastTrigDCOffset); 
  std::cout << "Fast Trig DC Offset:" << FastTrigDCOffset << std::endl; 
  fLastErrorCode = CAEN_DGTZ_GetGroupFastTriggerDCOffset(fHandle, 1, &FastTrigDCOffset); 
  std::cout << "Fast Trig DC Offset Group 2:" << FastTrigDCOffset << std::endl; 

  CAEN_DGTZ_EnaDis_t ft_digtz_enabled;
  fLastErrorCode = CAEN_DGTZ_GetFastTriggerDigitizing(fHandle, &ft_digtz_enabled);
  std::cout << "Fast trigger digitizing: ";
  if (ft_digtz_enabled==CAEN_DGTZ_ENABLE)
    std::cout << "[Enabled]" << std::endl;
  else
    std::cout << "[Disabled]" << std::endl;

  CAEN_DGTZ_TriggerMode_t fast_trig_mode;
  fLastErrorCode = CAEN_DGTZ_GetFastTriggerMode(fHandle, &fast_trig_mode);
  std::cout << "Fast trigger: ";
  if (fast_trig_mode==CAEN_DGTZ_TRGMODE_ACQ_ONLY) {
    std::cout << "[Acq only]" << std::endl; 
  }
  else {
    std::cout << "[Disabled]" << std::endl;
  }

  CAEN_DGTZ_DRS4Frequency_t freq;
  fLastErrorCode = CAEN_DGTZ_GetDRS4SamplingFrequency(fHandle, &freq);
  std::cerr << "DRS4 Sampling Frequency: ";
  printFrequency(freq);

  std::cout << "=== ACQUISITION ====" << std::endl;
  uint32_t mask; 
  fLastErrorCode = CAEN_DGTZ_GetGroupEnableMask(fHandle, &mask); 
  std::cout << "Group Enable Mask: " << std::hex << "0x" <<  mask << std::dec << std::endl; 
  fLastErrorCode = CAEN_DGTZ_GetChannelEnableMask(fHandle, &mask); 
  std::cout << "Channel Enable Mask: " << std::hex << "0x" <<  mask << std::dec << std::endl; 
  uint32_t sz; 
  fLastErrorCode = CAEN_DGTZ_GetRecordLength(fHandle, &sz); 
  std::cout << "Record Length: " << sz << std::endl; 
  fLastErrorCode = CAEN_DGTZ_GetPostTriggerSize(fHandle, &sz); 
  std::cout << "Post Trigger Size: " << sz << std::endl; 
  CAEN_DGTZ_AcqMode_t mode;
  fLastErrorCode = CAEN_DGTZ_GetAcquisitionMode(fHandle,&mode);
  std::cout << "Acquisition Mode: ";
  if(mode==CAEN_DGTZ_SW_CONTROLLED)
    std::cout << "[Software controlled] ";
  else if(mode==CAEN_DGTZ_S_IN_CONTROLLED)
    std::cout << "[S_IN CONTROLLED] ";
  else
    std::cout << "[First Trigger Controlled] ";
  std::cout << std::endl;
  
  std::cout << "====================" << std::endl;

  uint32_t reg;
  fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x810C, &reg); // Global Trigger Mask [31:0]
  std::cout << "Global Trigger Mask => Reg 0x810C:" << std::endl;
  std::cout << " ex_trig: " << ((reg >> 30) & 0b1) << std::endl; // bit 30
  std::cout << " sw_trig: " << ((reg >> 31) & 0b1) << std::endl; // bit 31

  fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x811C, &reg); // Front Pannel IO Control [31:0]
  std::cout << "Front Pannel IO Control => Reg 0x811C:" << std::hex << reg << std::dec << std::endl;
  std::cout << " bit16: " << ((reg >> 16) & 0b1) << std::endl; // bit 18
  std::cout << " bit17: " << ((reg >> 17) & 0b1) << std::endl; // bit 18
  std::cout << " bit18: " << ((reg >> 18) & 0b1) << std::endl; // bit 18
  std::cout << " bit19: " << ((reg >> 19) & 0b1) << std::endl; // bit 19
  std::cout << " bit20 (busy out): " << ((reg >> 20) & 0b1) << std::endl; // bit 20

  std::cout << "CH-wise configuration: " << std::endl;
  for(int ich=0; ich!=8; ++ich) {
    reg = ich;
    fLastErrorCode = CAEN_DGTZ_WriteRegister(fHandle, 0x10A4, reg);
    fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x1080, &reg);
    std::cout << " channel " << ich << " => thr " << reg;
    fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x1098, &reg);
    std::cout << "  adc offset " << reg << " || ";
    //
    fLastErrorCode = CAEN_DGTZ_WriteRegister(fHandle, 0x11A4, reg);
    fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x1180, &reg);
    std::cout << " channel " << 8+ich << " => thr " << reg;
    fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x1198, &reg);
    std::cout << "  adc offset " << reg << std::endl;

  }
  std::cout << "TR-wise configuration: " << std::endl;
  fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x10D4, &reg);
  std::cout << " tr0  => thr " << reg;
  fLastErrorCode = CAEN_DGTZ_ReadRegister(fHandle, 0x10DC, &reg);
  std::cout << "  adc offset " << reg << std::endl;

  uint32_t numEvents;
  
  fLastErrorCode = CAEN_DGTZ_GetMaxNumAggregatesBLT(fHandle, &numEvents);
  if (fLastErrorCode != CAEN_DGTZ_Success) {
    std::cerr << "Could not read Max Aggregate Events  " << fLastErrorCode << std::endl;
  }
  std::cout << "MaxAggregatesBLT:" << numEvents << std::endl; 

}

