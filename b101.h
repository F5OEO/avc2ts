
#include "tc358743_regs.h"

class B101
{

  public:
    B101();
    bool IsPresent();
    bool Start();
    bool Init();
    ~B101();
    /*
    void GetDisplaySize(int &Width, int &Height, int &Rotate);
    void SetOmxBuffer(unsigned char *Buffer);
    int GetPicture(int fps);
    */
  private:
     int i2c_fd=NULL;
     bool b101detected=false;
   public:  
      unsigned int width, height, fps, frame_interval;
      unsigned int frame_width, frame_height;

};