#include "CAENDigitizer.h"

class DT5742 {

public:
  DT5742() {}
  int Init();
  int Read(int run);
  int Close();
  void PrintLastErrorCode();
  
protected:
  void PrintInfo();
  void printTriggerMode(CAEN_DGTZ_TriggerMode_t tmode);
  void printFrequency(CAEN_DGTZ_DRS4Frequency_t freq);

  CAEN_DGTZ_ErrorCode fLastErrorCode;
  int fHandle;
  CAEN_DGTZ_BoardInfo_t fBoardInfo;
};
