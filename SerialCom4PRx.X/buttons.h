struct Button{
   unsigned int btnBit : 4;// the bit to check to retrieve the button state
   unsigned int btnDown : 1;// signals the falling edge when the button is pressed
   unsigned int btnUp : 1;// signals the rasing edge when the button is released
};

// indexes of the buttons to be accessed in aa butto array
#define CYCLE 0x00

#define INC1 0x00
#define DEC1 0x01
#define INC2 0x02
#define DEC2 0x03
#define INC3 0x04
#define DEC3 0x05
#define INC4 0x06
#define DEC4 0x07

// falling edge: when port is 0 and oldport is 1
//#define fTrig(port, btn, old) (((~port & btn) & (old & btn))!=0)
//#define fTrig(port, btn, old) (((~port & (1<<btn)) & old) >> btn)
#define fTrig(port, btn, old) ((~port & (1<<btn) & old) >> btn)
// raising edge: when port is 1 and oldport is 0
//#define rTrig(port, btn, old) (((port & btn) & (~old & btn))!=0)
//#define rTrig(port, btn, old) (((port & (1<<btn)) & ~old) >> btn)
#define rTrig(port, btn, old) ((port & (1<<btn) & ~old) >> btn)

// proptotype of method used to check the buttons state
void checkButtons();