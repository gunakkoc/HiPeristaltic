#ifndef TMC2209_driver
#define TMC2209_driver
#endif

typedef union CONF_GCONF { //n = 10, RW
    struct {
        uint32_t I_scale_analog : 1; //def 1
        uint32_t internal_Rsense : 1;
        uint32_t en_SpreadCycle : 1; //0 for StealthChop PWM mode voltage control, 1 for SpreadCycle current control. def 0
        uint32_t shaft : 1;
        uint32_t index_otpw : 1;
        uint32_t index_step : 1;
        uint32_t pdn_disable : 1;
        uint32_t mstep_reg_select : 1; //set to 1 for UART control of mres, otherwise pin controlled.
        uint32_t multistep_filt : 1; //def 1
        uint32_t test_mode : 1; //set 0
        uint32_t empty : 22;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_GCONF_t;

typedef union CONF_GSTAT { //n = 3, RW(C)
    struct {
        uint32_t reset : 1; //resetted
        uint32_t drv_err : 1; //critical error, write 1 to clear
        uint32_t uv_cp : 1; //overcurrent, no rewriting to clear
        // uint32_t openload_a : 1;
        // uint32_t openload_b : 1;
        // uint32_t short_a : 1;
        // uint32_t short_b : 1;
        // uint32_t s2ga : 1;
        uint32_t empty : 29;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_GSTAT_t;

typedef union CONF_CHOPCONF { //n = 32, RW
    struct {
        uint32_t toff : 4; //toff0, toff1, toff2, toff3
        uint32_t hstrt : 3; //hs0, hs1, hs2
        uint32_t hend : 4; //hend0, hend1, hend2, hend3
        uint32_t reserved_1 : 4; //0
        uint32_t tbl : 2; //tbl0, tbl1
        uint32_t vsense : 1; 
        uint32_t reserved_2 : 6; //0
        uint32_t mres : 4; //mres0, mres1, mres2, mres3
        uint32_t intpol : 1; //def 1
        uint32_t dedge : 1;
        uint32_t diss2g : 1;
        uint32_t diss2vs : 1;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_CHOPCONF_t;

typedef union CONF_VACTUAL { // n = 24, W
//VACTUAL = v[Hz] / ( f CLK [Hz] / 2^24 ) = v[Hz] / 0.715Hz
    struct {
        int32_t vactual : 24; //signed +/- 2^23 also with int32 the sign extension is done and uint32 will not do sign extension
        uint32_t empty : 8;
    } fields;
    uint32_t val; //signed +/- 2^23
    uint8_t byte_arr[4];
} CONF_VACTUAL_t;

typedef union CONF_MSCNT { // n = 10, R
    struct {
        uint32_t mscnt : 10;
        uint32_t empty : 22;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_MSCNT_t;

typedef union CONF_IHOLD_IRUN { //n = 32, W
    struct {
        uint32_t ihold : 5; //def 16
        uint32_t reserved_1 : 3;
        uint32_t irun : 5; //def 31
        uint32_t reserved_2 : 3;
        uint32_t iholddelay : 4; // def 1 but can be 2 as well, not 0
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_IHOLD_IRUN_t;

typedef union CONF_TPOWERDOWN { // n = 8, W
    struct {
        uint32_t tpowerdown : 8; //def 20
        uint32_t empty : 24;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_TPOWERDOWN_t;

typedef union CONF_PWMCONF { // n = 32, RW
    struct {
        uint32_t pwm_ofs : 8; //maybe read initial val from PWM_OFS_AUTO
        uint32_t pwm_grad : 8; //maybe read initial val from PWM_GRAD_AUTO
        uint32_t pwm_freq : 2;
        uint32_t pwm_autoscale : 1;
        uint32_t pwm_autograd : 1;
        uint32_t freewheel : 2; //0: normal, 1: freewheel, 2: coil short to LS drivers, 3: coil short to HS drivers
        uint32_t reserved : 2;
        uint32_t pwm_reg : 4;
        uint32_t pwm_lim : 4;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_PWMCONF_t;

typedef union CONF_PWMAUTO { // n = 16, R
    struct {
        uint32_t pwm_ofs_auto : 8; //might be useful to store
        uint32_t pwm_grad_auto : 8; //might be useful to store
        uint32_t empty : 16;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_PWMAUTO_t;

typedef union CONF_TPWMTHRS { // n = 20, W
    struct {
        uint32_t tpwmthrs : 20; //if TPWMTHRS < TSTEP then stealthChop is disabled, spreadCycle is enabled. set 999999 on official firmware for klipper, 0 def disables this
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_TPWMTHRS_t;

typedef union CONF_TSTEP { // n = 20, R
    struct {
        uint32_t tstep : 20; //time between steps
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_TSTEP_t;

typedef union CONF_TCOOLTHRS { // n = 20, W
    struct {
        uint32_t tcoolthrs : 20;
        uint32_t empty : 12;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
} CONF_TCOOLTHRS_t;

typedef union CONF_DRV_STATUS { // n = 32, R
    struct {
        uint32_t otpw : 1;
        uint32_t ot : 1;
        uint32_t s2ga : 1;
        uint32_t s2gb : 1;
        uint32_t s2vsa : 1;
        uint32_t s2vsb : 1;
        uint32_t ola : 1;
        uint32_t t120 : 1;
        uint32_t t143 : 1;
        uint32_t t150 : 1;
        uint32_t t157 : 1;
        uint32_t reserved_1 : 4;
        uint32_t cs_actual : 5;
        uint32_t reserved_2 : 9;
        uint32_t stealth : 1;
        uint32_t stst : 1;
    } fields;
    uint32_t val;
    uint8_t byte_arr[4];
}  CONF_DRV_STATUS_t;

typedef struct TMC2209_CONF {
	CONF_GCONF_t GCONF; //rw
	CONF_GSTAT_t GSTAT; //rw(c)
	CONF_CHOPCONF_t CHOPCONF; //rw
	CONF_VACTUAL_t VACTUAL; //w only
	CONF_MSCNT_t MSCNT; //r only
	CONF_IHOLD_IRUN_t IHOLD_IRUN; //w only
	CONF_TPOWERDOWN_t TPOWERDOWN; //w only
	CONF_PWMCONF_t PWMCONF; //rw
	CONF_PWMAUTO_t PWMAUTO; //r only
	CONF_DRV_STATUS_t DRV_STATUS;
	CONF_TPWMTHRS_t TPWMTHRS; //w only
	CONF_TSTEP_t TSTEP; //r only
	CONF_TCOOLTHRS_t TCOOLTHRS; //w only
	uint8_t addr_motor;
} TMC2209_CONF_t;


//in CHOPCONF
const static uint8_t TMC2209_MSTEP256 = 0b0000;
const static uint8_t TMC2209_MSTEP128 = 0b0001;
const static uint8_t TMC2209_MSTEP64 = 0b0010;
const static uint8_t TMC2209_MSTEP32 = 0b0011;
const static uint8_t TMC2209_MSTEP16 = 0b0100;
const static uint8_t TMC2209_MSTEP8 = 0b0101;
const static uint8_t TMC2209_MSTEP4 = 0b0110;
const static uint8_t TMC2209_MSTEP2 = 0b0111;
const static uint8_t TMC2209_MSTEP1 = 0b1000;

static const uint8_t reg_GCONF = 0x00;
static const uint8_t reg_GSTAT = 0x01;
static const uint8_t reg_CHOPCONF = 0x6C;
static const uint8_t reg_VACTUAL = 0x22;
static const uint8_t reg_MSCNT = 0x6A;
static const uint8_t reg_IHOLD_IRUN = 0x10;
static const uint8_t reg_TPOWERDOWN = 0x11;
static const uint8_t reg_PWMCONF = 0x70;
static const uint8_t reg_PWMAUTO = 0x72;
static const uint8_t reg_TPWMTHRS = 0x13;
static const uint8_t reg_TSTEP = 0x12;
static const uint8_t reg_TCOOLTHRS = 0x14;
static const uint8_t reg_DRV_STATUS = 0x6F;

static const uint8_t TMC2209_SYNC_BYTE = 0x05;

HAL_StatusTypeDef TMC2209_ReadRegister(uint8_t addr, uint8_t reg, uint32_t *value);
HAL_StatusTypeDef TMC2209_WriteRegister(uint8_t addr, uint8_t reg, uint32_t value);
HAL_StatusTypeDef TMC2209_Init(USART_HandleTypeDef huart);
