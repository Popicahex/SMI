#ifndef _KEYMAP_H_
#define _KEYMAP_H_
#include <Arduino.h>
#define KEY_MAX 21
// Definirea structurii ST_KEY_MAP
typedef struct {
    String keyname;
    byte keycode;
} ST_KEY_MAP;
#define IR_TYPE_NORMAL 1
#define IR_TYPE_EM 2
typedef enum {
    IR_KEYCODE_1,
    IR_KEYCODE_2,
    IR_KEYCODE_3,
    IR_KEYCODE_4,
    IR_KEYCODE_5,
    IR_KEYCODE_6,
    IR_KEYCODE_7,
    IR_KEYCODE_8,
    IR_KEYCODE_9,
    IR_KEYCODE_0,
    IR_KEYCODE_STAR,    // *
    IR_KEYCODE_POUND,   // #
    IR_KEYCODE_UP,
    IR_KEYCODE_DOWN,
    IR_KEYCODE_OK,
    IR_KEYCODE_LEFT,
    IR_KEYCODE_RIGHT,
} E_NORMAL_IR_KEYCODE;
typedef enum {
    EM_IR_KEYCODE_A = 0,
    EM_IR_KEYCODE_B,
    EM_IR_KEYCODE_C,
    EM_IR_KEYCODE_D,
    EM_IR_KEYCODE_UP,
    EM_IR_KEYCODE_PLUS,
    EM_IR_KEYCODE_LEFT,
    EM_IR_KEYCODE_OK,
    EM_IR_KEYCODE_RIGHT,
    EM_IR_KEYCODE_0,
    EM_IR_KEYCODE_DOWN,    
    EM_IR_KEYCODE_REDUCE,  
    EM_IR_KEYCODE_1,
    EM_IR_KEYCODE_2,
    EM_IR_KEYCODE_3,
    EM_IR_KEYCODE_4,
    EM_IR_KEYCODE_5,
    EM_IR_KEYCODE_6,
    EM_IR_KEYCODE_7,
    EM_IR_KEYCODE_8,
    EM_IR_KEYCODE_9,
} E_EM_IR_KEYCODE;
// Declarațiile array-urilor de mapare a tastelor IR
extern const ST_KEY_MAP normal_ir_keymap[];
extern const ST_KEY_MAP em_ir_keymap[];
#endif  /* _KEYMAP_H_ */