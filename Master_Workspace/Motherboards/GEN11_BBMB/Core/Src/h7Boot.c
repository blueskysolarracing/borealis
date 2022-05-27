#include "h7Boot.h"
#include "main.h"
void arm_boot(){
  	*(uint32_t*)0x52002008 = 0x08192A3B;
  	*(uint32_t*)0x52002008 = 0x4C5D6E7F;
  	// Write new boot address
  	*(uint32_t*)0x52002044 = 0x08000800; // default is 0x1ff00800
  	// Initiate OPTSTART Request
  	*(uint32_t*)0x52002018 |= 0x00000002;
  	while(*(uint32_t*)0x52002018 & 0x00000002);
  	// Lock OPTCR
  	*(uint32_t*)0x52002018 |= 0x00000001;
}
void disarm_boot(){
  	*(uint32_t*)0x52002008 = 0x08192A3B;
  	*(uint32_t*)0x52002008 = 0x4C5D6E7F;
  	// Write new boot address
  	*(uint32_t*)0x52002044 = 0x1ff00800; // default is
  	// Initiate OPTSTART Request
  	*(uint32_t*)0x52002018 |= 0x00000002;
  	while(*(uint32_t*)0x52002018 & 0x00000002);
  	// Lock OPTCR
  	*(uint32_t*)0x52002018 |= 0x00000001;
}
