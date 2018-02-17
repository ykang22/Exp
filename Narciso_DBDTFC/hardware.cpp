#define INIT_ONCE 
#include"hardware.h" 
#include"init.h" 
 
const AN_T AN_ConverterType[32] = { 
//ANALG_L(topside):8x AD1851 (16bit, ±3V) (DA output) 
AD1851, AD1851, AD1851, AD1851, 
AD1851, AD1851, AD1851, AD1851, 
//ANALG_M(topside):8x AD7894 (14bit, ±10V)(AD input) 
AD7894, AD7894, AD7894, AD7894, 
AD7894, AD7894, AD7894, AD7894, 
//ANALG_N(bottomside):8x AD7894 14bit,±10V)(AD input) 
AD7894, AD7894, AD7894, AD7894, 
AD7894, AD7894, AD7894, AD7894, 
// ANALG_O (bottomside): N/A 
NOADDA, NOADDA, NOADDA, NOADDA, 
NOADDA, NOADDA, NOADDA, NOADDA 
}; 
