#include <stdio.h>
#include <string.h>
#include "xparameters.h"
#include <stdlib.h>
#include "xuartlite.h"
#include "sleep.h"
#include "xil_io.h"
#include "mb_interface.h"


#define DEBUG_MODE 0

#define DBG(...) do { if (DEBUG_MODE) xil_printf(__VA_ARGS__); } while(0)

#define X_MAX 640
#define Y_MAX 480
#define DELAY_1_SEC 9000000
#define OFFSET 10
#define RESULT_MAX_LENGTH 5
#define PLOT_TOP_X 50
#define PLOT_TOP_Y 180
#define PLOT_LENGTH 580
#define PLOT_HEIGHT_SMALL 130
#define PLOT_HEIGHT_BIG 270
#define PLOT_BOT_X 50
#define PLOT_BOT_Y 330
#define MAG_OFFSET_Y 150
#define POINTS_TO_PLOT 32
#define POINTS_TO_PLOT_PARAM 10
#define DB_FLOOR -45.0f
#define PHASE_MIN (-1.5708f) // -pi/2 
#define PHASE_MAX (1.5708f) //  pi/2 
#define CIRC_ORIGIN_X 10
#define CIRC_ORIGIN_Y 10
#define CIRC_CELL_SIZE 40


#define PHASE_MIN_Q ((int32_t)(-11796480)) 
#define PHASE_MAX_Q ((int32_t)( 11796480))  
#define RAD_TO_DEG_Q ((int32_t)(3754936L))

#define MAX_ELEMENTS 64
#define MAX_NODES 32
#define MAX_UNKNOWN 64
#define MAX_NAME 32
#define MAX_LINE 128
#define MAX_SPICE_LEN 4096
#define SCALE 1000
#define MAX_FREQ_POINTS 512


// LU Solver 1 
#define LU1_STATUS_ADDR 0x44A10000UL
#define LU1_BRAM_BASE 0xC0000000UL
#define LU1_BRAM_END 0xC0001FFFUL
#define BTN_GPIO_BASEADDR   XPAR_GPIO_BUTTON_BASEADDR
#define BTN_LEFT 4
#define BTN_RIGHT 8
#define BTN_CENTER 1
#define BTN_DEBOUNCE_DELAY 9000
#define COORD_DISPLAY_X 330
#define COORD_DISPLAY_Y 20
#define COORD_CLEAR_W 300
#define COORD_CLEAR_H 30

// Solver 2
#define LU2_STATUS_ADDR 0x44A20000UL
#define LU2_BRAM_BASE 0xC0002000UL
#define LU2_BRAM_END 0xC0003FFFUL


#define BRAM_B_OFFSET_WORDS 50
#define BRAM_X_OFFSET_WORDS 60


#define LU_CMD_START 0x1  
#define LU_CMD_SKIP_DECOMP 0x17  

/* Status bits */
#define LU_STATUS_DONE_BIT 0x02
#define LU_STATUS_ERR_BIT 0x08


#define ATAN2_PI 205887L  
#define ATAN2_PI_2 102944L   
#define ATAN2_K1 64356L  
#define ATAN2_K2 18451L  


typedef struct { char R; char G; char B; } color;
typedef struct { int x; int y; } cursor;


unsigned char font4x6[41][3] = {
    {0x69,0x9f,0x99}, /* A */
    {0xE9,0xE9,0x9E}, /* B */
    {0x69,0x88,0x96}, /* C */
    {0xE9,0x99,0x9E}, /* D */
    {0xF8,0xF8,0x8F}, /* E */
    {0xF8,0xF8,0x88}, /* F */
    {0x78,0xB9,0x96}, /* G */
    {0x99,0xF9,0x99}, /* H */
    {0x22,0x22,0x22}, /* I */
    {0xF1,0x11,0x96}, /* J */
    {0x9A,0xCA,0x99}, /* K */
    {0x88,0x88,0x8F}, /* L */
    {0x9F,0xB9,0x99}, /* M */
    {0x9D,0xB9,0x99}, /* N */
    {0x69,0x99,0x96}, /* O */
    {0xE9,0xE8,0x88}, /* P */
    {0x69,0x9B,0x96}, /* Q */
    {0xE9,0xEA,0x99}, /* R */
    {0x78,0x61,0x1E}, /* S */
    {0xF4,0x44,0x44}, /* T */
    {0x99,0x99,0x96}, /* U */
    {0x99,0x95,0x52}, /* V */
    {0x99,0x9B,0xF9}, /* W */
    {0x95,0x25,0x99}, /* X */
    {0x95,0x22,0x22}, /* Y */
    {0xF1,0x24,0x8F}, /* Z */
    {0x69,0x99,0x96}, /* 0 */
    {0x26,0x22,0x2F}, /* 1 */
    {0x69,0x12,0x4F}, /* 2 */
    {0x69,0x21,0x96}, /* 3 */
    {0x99,0xF1,0x11}, /* 4 */
    {0xF8,0xE1,0x96}, /* 5 */
    {0x68,0xE9,0x96}, /* 6 */
    {0xF1,0x24,0x44}, /* 7 */
    {0x69,0x69,0x96}, /* 8 */
    {0x69,0x61,0x16}, /* 9 */
    {0x02,0x00,0x20}, /* : */
    {0x00,0x00,0x20}, /* . */
    {0x00,0xF0,0x00}, /* - */
	{0x04,0xF4,0x40}, /* + */
    {0x00,0x00,0x00} /* space */
};

static const int32_t dB_table[501] = {
/*guard*/ -90000,
/*0.002*/ -53979, /*0.004*/ -47959, /*0.006*/ -44437, /*0.008*/ -41938,
/*0.010*/ -40000, /*0.012*/ -38416, /*0.014*/ -37077, /*0.016*/ -35918,
/*0.018*/ -34895, /*0.020*/ -33979, /*0.022*/ -33152, /*0.024*/ -32396,
/*0.026*/ -31701, /*0.028*/ -31057, /*0.030*/ -30458, /*0.032*/ -29897,
/*0.034*/ -29370, /*0.036*/ -28874, /*0.038*/ -28404, /*0.040*/ -27959,
/*0.042*/ -27535, /*0.044*/ -27131, /*0.046*/ -26745, /*0.048*/ -26375,
/*0.050*/ -26021, /*0.052*/ -25680, /*0.054*/ -25352, /*0.056*/ -25036,
/*0.058*/ -24731, /*0.060*/ -24437, /*0.062*/ -24152, /*0.064*/ -23876,
/*0.066*/ -23609, /*0.068*/ -23350, /*0.070*/ -23098, /*0.072*/ -22853,
/*0.074*/ -22615, /*0.076*/ -22384, /*0.078*/ -22158, /*0.080*/ -21938,
/*0.082*/ -21724, /*0.084*/ -21514, /*0.086*/ -21310, /*0.088*/ -21110,
/*0.090*/ -20915, /*0.092*/ -20724, /*0.094*/ -20537, /*0.096*/ -20355,
/*0.098*/ -20175, /*0.100*/ -20000, /*0.102*/ -19828, /*0.104*/ -19659,
/*0.106*/ -19494, /*0.108*/ -19332, /*0.110*/ -19172, /*0.112*/ -19016,
/*0.114*/ -18862, /*0.116*/ -18711, /*0.118*/ -18562, /*0.120*/ -18416,
/*0.122*/ -18273, /*0.124*/ -18132, /*0.126*/ -17993, /*0.128*/ -17856,
/*0.130*/ -17721, /*0.132*/ -17589, /*0.134*/ -17458, /*0.136*/ -17329,
/*0.138*/ -17202, /*0.140*/ -17077, /*0.142*/ -16954, /*0.144*/ -16833,
/*0.146*/ -16713, /*0.148*/ -16595, /*0.150*/ -16478, /*0.152*/ -16363,
/*0.154*/ -16250, /*0.156*/ -16138, /*0.158*/ -16027, /*0.160*/ -15918,
/*0.162*/ -15810, /*0.164*/ -15703, /*0.166*/ -15598, /*0.168*/ -15494,
/*0.170*/ -15391, /*0.172*/ -15289, /*0.174*/ -15189, /*0.176*/ -15090,
/*0.178*/ -14992, /*0.180*/ -14895, /*0.182*/ -14799, /*0.184*/ -14704,
/*0.186*/ -14610, /*0.188*/ -14517, /*0.190*/ -14425, /*0.192*/ -14334,
/*0.194*/ -14244, /*0.196*/ -14155, /*0.198*/ -14067, /*0.200*/ -13979,
/*0.202*/ -13893, /*0.204*/ -13807, /*0.206*/ -13723, /*0.208*/ -13639,
/*0.210*/ -13556, /*0.212*/ -13473, /*0.214*/ -13392, /*0.216*/ -13311,
/*0.218*/ -13231, /*0.220*/ -13152, /*0.222*/ -13073, /*0.224*/ -12995,
/*0.226*/ -12918, /*0.228*/ -12841, /*0.230*/ -12765, /*0.232*/ -12690,
/*0.234*/ -12616, /*0.236*/ -12542, /*0.238*/ -12468, /*0.240*/ -12396,
/*0.242*/ -12324, /*0.244*/ -12252, /*0.246*/ -12181, /*0.248*/ -12111,
/*0.250*/ -12041, /*0.252*/ -11972, /*0.254*/ -11903, /*0.256*/ -11835,
/*0.258*/ -11768, /*0.260*/ -11701, /*0.262*/ -11634, /*0.264*/ -11568,
/*0.266*/ -11502, /*0.268*/ -11437, /*0.270*/ -11373, /*0.272*/ -11309,
/*0.274*/ -11245, /*0.276*/ -11182, /*0.278*/ -11119, /*0.280*/ -11057,
/*0.282*/ -10995, /*0.284*/ -10934, /*0.286*/ -10873, /*0.288*/ -10812,
/*0.290*/ -10752, /*0.292*/ -10692, /*0.294*/ -10633, /*0.296*/ -10574,
/*0.298*/ -10516, /*0.300*/ -10458, /*0.302*/ -10400, /*0.304*/ -10343,
/*0.306*/ -10286, /*0.308*/ -10229, /*0.310*/ -10173, /*0.312*/ -10117,
/*0.314*/ -10061, /*0.316*/ -10006, /*0.318*/  -9951, /*0.320*/  -9897,
/*0.322*/  -9843, /*0.324*/  -9789, /*0.326*/  -9736, /*0.328*/  -9683,
/*0.330*/  -9630, /*0.332*/  -9577, /*0.334*/  -9525, /*0.336*/  -9473,
/*0.338*/  -9422, /*0.340*/  -9370, /*0.342*/  -9319, /*0.344*/  -9269,
/*0.346*/  -9218, /*0.348*/  -9168, /*0.350*/  -9119, /*0.352*/  -9069,
/*0.354*/  -9020, /*0.356*/  -8971, /*0.358*/  -8922, /*0.360*/  -8874,
/*0.362*/  -8826, /*0.364*/  -8778, /*0.366*/  -8730, /*0.368*/  -8683,
/*0.370*/  -8636, /*0.372*/  -8589, /*0.374*/  -8543, /*0.376*/  -8496,
/*0.378*/  -8450, /*0.380*/  -8404, /*0.382*/  -8359, /*0.384*/  -8313,
/*0.386*/  -8268, /*0.388*/  -8223, /*0.390*/  -8179, /*0.392*/  -8134,
/*0.394*/  -8090, /*0.396*/  -8046, /*0.398*/  -8002, /*0.400*/  -7959,
/*0.402*/  -7915, /*0.404*/  -7872, /*0.406*/  -7829, /*0.408*/  -7787,
/*0.410*/  -7744, /*0.412*/  -7702, /*0.414*/  -7660, /*0.416*/  -7618,
/*0.418*/  -7576, /*0.420*/  -7535, /*0.422*/  -7494, /*0.424*/  -7453,
/*0.426*/  -7412, /*0.428*/  -7371, /*0.430*/  -7331, /*0.432*/  -7290,
/*0.434*/  -7250, /*0.436*/  -7210, /*0.438*/  -7171, /*0.440*/  -7131,
/*0.442*/  -7092, /*0.444*/  -7052, /*0.446*/  -7013, /*0.448*/  -6974,
/*0.450*/  -6936, /*0.452*/  -6897, /*0.454*/  -6859, /*0.456*/  -6821,
/*0.458*/  -6783, /*0.460*/  -6745, /*0.462*/  -6707, /*0.464*/  -6670,
/*0.466*/  -6632, /*0.468*/  -6595, /*0.470*/  -6558, /*0.472*/  -6521,
/*0.474*/  -6484, /*0.476*/  -6448, /*0.478*/  -6411, /*0.480*/  -6375,
/*0.482*/  -6339, /*0.484*/  -6303, /*0.486*/  -6267, /*0.488*/  -6232,
/*0.490*/  -6196, /*0.492*/  -6161, /*0.494*/  -6125, /*0.496*/  -6090,
/*0.498*/  -6055, /*0.500*/  -6021, /*0.502*/  -5986, /*0.504*/  -5951,
/*0.506*/  -5917, /*0.508*/  -5883, /*0.510*/  -5849, /*0.512*/  -5815,
/*0.514*/  -5781, /*0.516*/  -5747, /*0.518*/  -5713, /*0.520*/  -5680,
/*0.522*/  -5647, /*0.524*/  -5613, /*0.526*/  -5580, /*0.528*/  -5547,
/*0.530*/  -5514, /*0.532*/  -5482, /*0.534*/  -5449, /*0.536*/  -5417,
/*0.538*/  -5384, /*0.540*/  -5352, /*0.542*/  -5320, /*0.544*/  -5288,
/*0.546*/  -5256, /*0.548*/  -5224, /*0.550*/  -5193, /*0.552*/  -5161,
/*0.554*/  -5130, /*0.556*/  -5099, /*0.558*/  -5067, /*0.560*/  -5036,
/*0.562*/  -5005, /*0.564*/  -4974, /*0.566*/  -4944, /*0.568*/  -4913,
/*0.570*/  -4883, /*0.572*/  -4852, /*0.574*/  -4822, /*0.576*/  -4792,
/*0.578*/  -4761, /*0.580*/  -4731, /*0.582*/  -4702, /*0.584*/  -4672,
/*0.586*/  -4642, /*0.588*/  -4612, /*0.590*/  -4583, /*0.592*/  -4554,
/*0.594*/  -4524, /*0.596*/  -4495, /*0.598*/  -4466, /*0.600*/  -4437,
/*0.602*/  -4408, /*0.604*/  -4379, /*0.606*/  -4351, /*0.608*/  -4322,
/*0.610*/  -4293, /*0.612*/  -4265, /*0.614*/  -4237, /*0.616*/  -4208,
/*0.618*/  -4180, /*0.620*/  -4152, /*0.622*/  -4124, /*0.624*/  -4096,
/*0.626*/  -4069, /*0.628*/  -4041, /*0.630*/  -4013, /*0.632*/  -3986,
/*0.634*/  -3958, /*0.636*/  -3931, /*0.638*/  -3904, /*0.640*/  -3876,
/*0.642*/  -3849, /*0.644*/  -3822, /*0.646*/  -3795, /*0.648*/  -3768,
/*0.650*/  -3742, /*0.652*/  -3715, /*0.654*/  -3688, /*0.656*/  -3662,
/*0.658*/  -3635, /*0.660*/  -3609, /*0.662*/  -3583, /*0.664*/  -3557,
/*0.666*/  -3531, /*0.668*/  -3504, /*0.670*/  -3479, /*0.672*/  -3453,
/*0.674*/  -3427, /*0.676*/  -3401, /*0.678*/  -3375, /*0.680*/  -3350,
/*0.682*/  -3324, /*0.684*/  -3299, /*0.686*/  -3274, /*0.688*/  -3248,
/*0.690*/  -3223, /*0.692*/  -3198, /*0.694*/  -3173, /*0.696*/  -3148,
/*0.698*/  -3123, /*0.700*/  -3098, /*0.702*/  -3073, /*0.704*/  -3049,
/*0.706*/  -3024, /*0.708*/  -2999, /*0.710*/  -2975, /*0.712*/  -2950,
/*0.714*/  -2926, /*0.716*/  -2902, /*0.718*/  -2878, /*0.720*/  -2853,
/*0.722*/  -2829, /*0.724*/  -2805, /*0.726*/  -2781, /*0.728*/  -2757,
/*0.730*/  -2734, /*0.732*/  -2710, /*0.734*/  -2686, /*0.736*/  -2662,
/*0.738*/  -2639, /*0.740*/  -2615, /*0.742*/  -2592, /*0.744*/  -2569,
/*0.746*/  -2545, /*0.748*/  -2522, /*0.750*/  -2499, /*0.752*/  -2476,
/*0.754*/  -2453, /*0.756*/  -2430, /*0.758*/  -2407, /*0.760*/  -2384,
/*0.762*/  -2361, /*0.764*/  -2338, /*0.766*/  -2315, /*0.768*/  -2293,
/*0.770*/  -2270, /*0.772*/  -2248, /*0.774*/  -2225, /*0.776*/  -2203,
/*0.778*/  -2180, /*0.780*/  -2158, /*0.782*/  -2136, /*0.784*/  -2114,
/*0.786*/  -2092, /*0.788*/  -2069, /*0.790*/  -2047, /*0.792*/  -2025,
/*0.794*/  -2004, /*0.796*/  -1982, /*0.798*/  -1960, /*0.800*/  -1938,
/*0.802*/  -1917, /*0.804*/  -1895, /*0.806*/  -1873, /*0.808*/  -1852,
/*0.810*/  -1830, /*0.812*/  -1809, /*0.814*/  -1788, /*0.816*/  -1766,
/*0.818*/  -1745, /*0.820*/  -1724, /*0.822*/  -1703, /*0.824*/  -1681,
/*0.826*/  -1660, /*0.828*/  -1639, /*0.830*/  -1618, /*0.832*/  -1598,
/*0.834*/  -1577, /*0.836*/  -1556, /*0.838*/  -1535, /*0.840*/  -1514,
/*0.842*/  -1494, /*0.844*/  -1473, /*0.846*/  -1453, /*0.848*/  -1432,
/*0.850*/  -1412, /*0.852*/  -1391, /*0.854*/  -1371, /*0.856*/  -1351,
/*0.858*/  -1330, /*0.860*/  -1310, /*0.862*/  -1290, /*0.864*/  -1270,
/*0.866*/  -1250, /*0.868*/  -1230, /*0.870*/  -1210, /*0.872*/  -1190,
/*0.874*/  -1170, /*0.876*/  -1150, /*0.878*/  -1130, /*0.880*/  -1110,
/*0.882*/  -1091, /*0.884*/  -1071, /*0.886*/  -1051, /*0.888*/  -1032,
/*0.890*/  -1012, /*0.892*/   -993, /*0.894*/   -973, /*0.896*/   -954,
/*0.898*/   -934, /*0.900*/   -915, /*0.902*/   -896, /*0.904*/   -877,
/*0.906*/   -857, /*0.908*/   -838, /*0.910*/   -819, /*0.912*/   -800,
/*0.914*/   -781, /*0.916*/   -762, /*0.918*/   -743, /*0.920*/   -724,
/*0.922*/   -705, /*0.924*/   -687, /*0.926*/   -668, /*0.928*/   -649,
/*0.930*/   -630, /*0.932*/   -612, /*0.934*/   -593, /*0.936*/   -574,
/*0.938*/   -556, /*0.940*/   -537, /*0.942*/   -519, /*0.944*/   -501,
/*0.946*/   -482, /*0.948*/   -464, /*0.950*/   -446, /*0.952*/   -427,
/*0.954*/   -409, /*0.956*/   -391, /*0.958*/   -373, /*0.960*/   -355,
/*0.962*/   -336, /*0.964*/   -318, /*0.966*/   -300, /*0.968*/   -282,
/*0.970*/   -265, /*0.972*/   -247, /*0.974*/   -229, /*0.976*/   -211,
/*0.978*/   -193, /*0.980*/   -175, /*0.982*/   -158, /*0.984*/   -140,
/*0.986*/   -122, /*0.988*/   -105, /*0.990*/    -87, /*0.992*/    -70,
/*0.994*/    -52, /*0.996*/    -35, /*0.998*/    -17, /*1.000*/      0,
};

color black = { .R=0x0, .G=0x0, .B=0x0 };
color white = { .R=0xF, .G=0xF, .B=0xF };
color grey = { .R=0x5, .G=0x5, .B=0x5 };
color orange = { .R = 0xF, .G = 0x8, .B = 0x0 };
color green = { .R = 0x0, .G = 0xF, .B = 0x0 };

typedef struct {
    char  name[MAX_NAME];
    char  n1[MAX_NAME];
    char  n2[MAX_NAME];
    char  type;
    float value;
} Element;

typedef struct { float re; float im; } Complex;
typedef struct { char suffix; float mult; } Suffix;


typedef struct {
    float freq_start;
    float freq_stop;
    int points_per_decade;
    int is_sweep;              
    char probe_node[MAX_NAME];   

   
    int has_param_sweep;        
    char sweep_element[MAX_NAME];
    float sweep_start;
    float sweep_stop;
    float sweep_step;
} SimParams;


const Suffix suffixes[] = {
    {'f',1e-15f},{'p',1e-12f},{'n',1e-9f},{'u',1e-6f},{'m',1e-3f},
    {'k',1e3f},  {'M',1e6f},  {'G',1e9f}, {'T',1e12f}, {'\0',1.0f}
};


static Complex A[MAX_UNKNOWN][MAX_UNKNOWN];
static Complex Z[MAX_UNKNOWN];
static Complex X[MAX_UNKNOWN];
static char unknowns[MAX_UNKNOWN][MAX_NAME];


static float sweep_freqs[MAX_FREQ_POINTS];
static float sweep_re[MAX_FREQ_POINTS];
static float sweep_im[MAX_FREQ_POINTS];

static float psweep_vals[MAX_FREQ_POINTS];
static float psweep_re[MAX_FREQ_POINTS];
static float psweep_im[MAX_FREQ_POINTS];


static int32_t plot_mag_q[POINTS_TO_PLOT]; 
static int32_t plot_phase_q[POINTS_TO_PLOT];  
static int32_t plot_freqs_q[POINTS_TO_PLOT]; 

static int plot_is_voltage_sweep = 0;

static int32_t plot_lin_fullscale_q  = 65536;   


volatile char *spice_ram = (volatile char *)XPAR_AXI_BRAM_CTRL_0_S_AXI_BASEADDR + 512;
volatile int32_t *matrix_mem = (volatile int32_t *)LU1_BRAM_BASE;
volatile int32_t *matrix_mem2 = (volatile int32_t *)LU2_BRAM_BASE;


char spice_receive_buffer[MAX_SPICE_LEN];


static float freq_list[MAX_FREQ_POINTS];


void drawFilledRect(int *DDR_addr, int top_left_x, int top_left_y, int length, int width, color c);
void drawFilledCircle(int *DDR_addr, int cent_x, int cent_y, int radius, color c);
void clearScreen(int *DDR_addr);
void writeRGB(int *DDR_addr, int x, int y, color c);
void setVideoMemAddr(volatile int *TFT_addr, int *DDR_addr);
void disableVGA(volatile int *TFT_addr);
void enableVGA(volatile int *TFT_addr);
int fontIndexFromAscii(unsigned char c);
void drawText(int *DDR_addr, cursor *curs, const char *text, color c);
void drawLine(int *DDR_addr, int x0, int y0, int x1, int y1, color c);
void drawPlot(int *DDR_addr, color c);
void plot_sweep_results(int *DDR_addr, float *re, float *im, float *xvals, int npts, int is_voltage_sweep);
void drawElement(int *DDR_addr, int cell_x, int cell_y, int element, color c);
void drawCircuitFromSpice(int *DDR_addr, Element *elements, int ne, char nodes[MAX_NODES][MAX_NAME], int nnodes, color c, const char *probe_node);
int32_t floatToQ1616(float x);
float q1616ToFloat(int32_t x);
void floatToAscii(float f, char *buf, int bufsize);
uint32_t read_buttons(void);
void debounce(void);
void receive_spice_netlist(XUartLite *UartInstance, int pre_received);
int32_t to_dB_q(int32_t mag_q);
int dB_to_pixel_y_mdb(int32_t mdb);
int lin_mag_to_pixel_y(int32_t mag_q);
static int32_t rad_to_deg_q1616(int32_t rad_q);

void redraw_plot_points(int *DDR_addr, int32_t *mag_q, int32_t *phase_q, int selected);
void draw_point_info (int *DDR_addr, int idx, int32_t *mag_q, int32_t *phase_q);
void int_to_str(int val, char *buf, int bufsize);

static XUartLite *g_uart = NULL;

void uart_send_str(const char *s){
    while (*s) {
        XUartLite_Send(g_uart, (u8 *)s, 1);
        while (XUartLite_IsSending(g_uart));
        s++;
    }
}


float parse_value(const char *str){
    
    
    char buf[32];
    strncpy(buf, str, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';



    size_t len = strlen(buf);
    char suffix = '\0';

    if (len > 0) {
        char last = buf[len-1];
        for (int i = 0; suffixes[i].suffix != '\0'; i++) {
            if (suffixes[i].suffix == last) {
                suffix = last;
                buf[len-1] = '\0';
                break;
            }
        }
    }

    float val = atof(buf);
    
    for (int i = 0; suffixes[i].suffix != '\0'; i++)
        if (suffixes[i].suffix == suffix){
            return val * suffixes[i].mult;
        }
    return val;


}

float parse_freq(const char *str){
    char buf[32];
    strncpy(buf, str, sizeof(buf)-1);
    buf[sizeof(buf)-1] = '\0';

    size_t len = strlen(buf);
    float mult = 1.0f;

    if (len > 0) {
        char last = buf[len-1];
        if (last == 'K' || last == 'k') { 
            mult = 1e3f; 
            buf[len-1] = '\0'; 
        }
        
        else if (last == 'M') { 
            mult = 1e6f; 
            buf[len-1] = '\0'; 
        }
        
        else if (last == 'G') { 
            mult = 1e9f; 
            buf[len-1] = '\0'; 
        }
    }
    return atof(buf) * mult;
}


void parse_sim_params(const char *text, SimParams *params){

    params->freq_start = 1000.0f;
    params->freq_stop = 1000.0f;
    params->points_per_decade = 1;
    params->is_sweep = 0;
    params->probe_node[0] = '\0';
    params->has_param_sweep = 0;
    params->sweep_element[0] = '\0';
    params->sweep_start = 0.0f;
    params->sweep_stop = 1.0f;
    params->sweep_step = 0.1f;

    char line[MAX_LINE];
    int in_header = 0;

    while (*text) {
        int len = 0;
        while (*text && *text != '\n' && len < MAX_LINE-1){
            line[len++] = *text++;
        }
        if (*text == '\n') {
            text++;
        }
        line[len] = '\0';

        char *r = strchr(line, '\r');
        
        if (r) {
            *r = '\0';
        }
        char *s = line;
        
        while (*s == ' ' || *s == '\t') {
            s++;
        }
        if (*s == '\0' || *s == '*') {
            continue;
        }

        if (!strncmp(s, "# header", 8)) { 
            in_header = 1; 
            continue; 
        }
        
        if (!strncmp(s, "# code", 6)) { 
            break; 
        }

        if (in_header) {
            
            if (!strncmp(s, ".ac", 3)) {

                char s_start[32], s_stop[32];
                int ppd = 10;
                int matched = sscanf(s + 3, " %31s %31s %d", s_start, s_stop, &ppd);

                if (matched >= 1) {
                    params->freq_start = parse_freq(s_start);
                    params->freq_stop = params->freq_start;
                    params->is_sweep = 0;
                }
                if (matched >= 2) {
                    params->freq_stop = parse_freq(s_stop);
                    params->points_per_decade = (matched >= 3) ? ppd : 10;
                    params->is_sweep = (params->freq_stop > params->freq_start) ? 1 : 0;
                }
            }
            
            else if (!strncmp(s, ".node", 5)) {
                
                char nname[MAX_NAME];
                
                if (sscanf(s + 5, " %31s", nname) == 1) {
                    
                    strncpy(params->probe_node, nname, MAX_NAME-1);

                    params->probe_node[MAX_NAME-1] = '\0';
                }
            }
            else if (!strncmp(s, ".sweep", 6)) {
                char ename[MAX_NAME], s_start[32], s_stop[32], s_step[32];
                int  matched = sscanf(s + 6, " %31s %31s %31s %31s", ename, s_start, s_stop, s_step);
                if (matched >= 3) {
                    
                    strncpy(params->sweep_element, ename, MAX_NAME-1);
                    params->sweep_element[MAX_NAME-1] = '\0';
                    params->sweep_start = parse_value(s_start);
                    params->sweep_stop  = parse_value(s_stop);
                    
                    if (matched >= 4) {
                        
                        params->sweep_step = parse_value(s_step);
                    } else {
                        
                        float span = parse_value(s_stop) - parse_value(s_start);
                        
                        params->sweep_step = (span >= 0 ? span : -span) / 10.0f;
                        if (params->sweep_step == 0.0f) {
                            params->sweep_step = 1.0f;
                        }
                    }
                    params->has_param_sweep = 1;
                }
            }
        }
    }
}


int build_freq_list(SimParams *params)
{
    if (!params->is_sweep) {
        freq_list[0] = params->freq_start;
        freq_list[1] = params->freq_start;
        return 1;
    }

    float log_start = 0.0f, log_stop = 0.0f;
    
    {
        
        float v = params->freq_start;

        
        while (v >= 10.0f) { 
            log_start += 1.0f; 
            v /= 10.0f; 
        }
        while (v <  1.0f)  { 
            log_start -= 1.0f; 
            v *= 10.0f; 
        
        }
        
        v = params->freq_stop;
        while (v >= 10.0f) { 
            log_stop += 1.0f; 
            v /= 10.0f; 
        }

        while (v <  1.0f)  { 
            log_stop -= 1.0f; 
            v *= 10.0f; 
        }
    }

    int total_decades = (int)(log_stop - log_start + 0.5f);
    if (total_decades < 1) total_decades = 1;

    int total_points = total_decades * params->points_per_decade + 1;
    if (total_points > MAX_FREQ_POINTS) total_points = MAX_FREQ_POINTS;

    int count = 0;
    for (int i = 0; i < total_points && count < MAX_FREQ_POINTS; i++) {
        float exp_val = log_start + (log_stop - log_start) * i / (float)(total_points - 1);
        int e = (int)exp_val;
        float frac_e = exp_val - (float)e;

        float f = 1.0f;
        if (e >= 0) { 
            int k = e;  while (k--) f *= 10.0f; 
        }
        else { int k = -e; while (k--) f /= 10.0f; 
        }
        
        
        
        {
            float x  = frac_e * 2.302585f;
            float ex = 1.0f + x + (x*x)/2.0f + (x*x*x)/6.0f + (x*x*x*x)/24.0f;
            f *= ex;
        }
        freq_list[count++] = f;
    }
    return count;
}


int find_node(char nodes[MAX_NODES][MAX_NAME], int n, const char *name)
{
    for (int i = 0; i < n; i++)
        if (!strcmp(nodes[i], name)){
            return i;
        }
    return -1;
}


int parse_netlist_from_buffer(const char *text, Element elements[MAX_ELEMENTS], int *ne, char nodes[MAX_NODES][MAX_NAME], int *nnodes)
{
    char line[MAX_LINE];
    int in_code = 0;

    *ne = 0;
    *nnodes = 0;

    while (*text) {
        int len = 0;
        while (*text && *text != '\n' && len < MAX_LINE-1){
            line[len++] = *text++;
        }
        if (*text == '\n') {
            text++;
        }


        line[len] = '\0';

        char *r = strchr(line, '\r');
        if (r) *r = '\0';

        char *s = line;
        while (*s == ' ' || *s == '\t') s++;

        if (*s == '\0' || *s == '*') {
            continue;
        }



        if (!strncmp(s, "# code",   6)) { 
            in_code = 1; 
            continue; 
        }


        if (!strncmp(s, "# header", 8)) { 
            in_code = 0; 
            continue; 
        }




        if (!in_code || *ne >= MAX_ELEMENTS) {
            continue;
        }

        Element *e = &elements[*ne];
        char valstr[32];

        if (sscanf(s, "%15s %15s %15s %31s", e->name, e->n1, e->n2, valstr) != 4){
            continue;

        }

        e->type  = e->name[0];
        e->value = parse_value(valstr);

        if (strcmp(e->n1, "0") && *nnodes < MAX_NODES && find_node(nodes, *nnodes, e->n1) < 0){
            strcpy(nodes[(*nnodes)++], e->n1);
        }

        if (strcmp(e->n2, "0") && *nnodes < MAX_NODES && find_node(nodes, *nnodes, e->n2) < 0){
            strcpy(nodes[(*nnodes)++], e->n2);
        }

        (*ne)++;
    }
    return 1;
}


void stamp_admittance(int i, int j, Complex y)
{
    if (i >= 0) { 
        A[i][i].re += y.re; A[i][i].im += y.im; 
    }

    if (j >= 0) { 

        A[j][j].re += y.re; A[j][j].im += y.im; 
    }

    if (i >= 0 && j >= 0) {


        A[i][j].re -= y.re; A[i][j].im -= y.im;
        A[j][i].re -= y.re; A[j][i].im -= y.im;
    }
}

void build_ac_mna(Element elements[MAX_ELEMENTS], int ne, char nodes[MAX_NODES][MAX_NAME], int nnodes, float omega, int *nunknown)
{
    int vs = 0;
    for (int i = 0; i < ne; i++){
        if (elements[i].type == 'V') {
            vs++;
        }
    }

    *nunknown = nnodes + vs;

    if (*nunknown > MAX_UNKNOWN) {
        *nunknown = MAX_UNKNOWN;
    }

    for (int i = 0; i < *nunknown; i++) {
        Z[i].re = Z[i].im = 0;
        for (int j = 0; j < *nunknown; j++){
            A[i][j].re = A[i][j].im = 0;
        }
    }

    vs = 0;
    for (int k = 0; k < ne; k++) {
        Element *e = &elements[k];
        int i = strcmp(e->n1, "0") ? find_node(nodes, nnodes, e->n1) : -1;
        int j = strcmp(e->n2, "0") ? find_node(nodes, nnodes, e->n2) : -1;
        Complex y = {0, 0};

        if (e->type == 'R') {
            y.re = 1.0f / e->value;
            stamp_admittance(i, j, y);
        } else if (e->type == 'C') {
            y.im = omega * e->value;
            stamp_admittance(i, j, y);
        } else if (e->type == 'L') {
            y.im = -1.0f / (omega * e->value);
            stamp_admittance(i, j, y);
        } else if (e->type == 'I') {
            if (i >= 0) Z[i].re -= e->value;
            if (j >= 0) Z[j].re += e->value;
        } else if (e->type == 'V') {
            if (nnodes + vs < MAX_UNKNOWN) {
                int row = nnodes + vs++;
                if (i >= 0) A[row][i].re = A[i][row].re =  1.0f;
                if (j >= 0) A[row][j].re = A[j][row].re = -1.0f;
                Z[row].re = e->value;
            }
        }
    }

    for (int i = 0; i < nnodes && i < MAX_UNKNOWN; i++){
        sprintf(unknowns[i], "V(%s)", nodes[i]);
    }

    vs = 0;
    for (int i = 0; i < ne && nnodes + vs < MAX_UNKNOWN; i++){
        if (elements[i].type == 'V'){
            sprintf(unknowns[nnodes + vs++], "I(%s)", elements[i].name);
        }
    }
}


void print_float(float f)
{
    if (f < 0) { xil_printf("-"); f = -f; }
    int int_part  = (int)f;
    int frac_part = (int)((f - int_part) * 1000.0f);
    xil_printf("%d.%03d", int_part, frac_part);
}

void print_matrix(int n)
{
    if (!DEBUG_MODE) {
        return;
    }
    if (n > MAX_UNKNOWN) {
        n = MAX_UNKNOWN;
    }

    xil_printf("\nUnknowns:\n");
    for (int i = 0; i < n; i++)
        xil_printf("%s\n", unknowns[i]);

    xil_printf("\nMatrix A (re,im):\n");
    for (int i = 0; i < n; i++) {
        xil_printf("[ ");
        for (int j = 0; j < n; j++) {
            print_float(A[i][j].re);
            xil_printf(",");
            print_float(A[i][j].im);
            xil_printf(" ");
        }
        xil_printf("]\n");
    }

    xil_printf("\nb (re,im):\n");
    for (int i = 0; i < n; i++) {
        print_float(Z[i].re);
        xil_printf(",");
        print_float(Z[i].im);
        xil_printf("\n");
    }
}


void save_matrices_to_memory_at(int n, volatile int32_t *mem, int is_voltage_sweep, float voltage_value)
{
    int idx = 0;
    if (!is_voltage_sweep) {
		for (int i = 0; i < n; i++) {
			for (int j = 0; j < n; j++) {
				mem[idx++] = floatToQ1616(A[i][j].re);
				mem[idx++] = floatToQ1616(A[i][j].im);
			}
		}

	    for (int i = 0; i < n; i++) {
	        mem[idx++] = floatToQ1616(Z[i].re);
	        mem[idx++] = floatToQ1616(Z[i].im);
	    }
    } else {
    	mem[58] = floatToQ1616(voltage_value);
    	mem[59] = floatToQ1616(0);
    }

}

void save_matrices_to_memory(int n)
{
    save_matrices_to_memory_at(n, matrix_mem, 0, 0);
}

void load_matrices_from_memory(int n)
{
    int idx = 0;
    for (int i = 0; i < n; i++)
        for (int j = 0; j < n; j++) {
            A[i][j].re = q1616ToFloat(matrix_mem[idx++]);
            A[i][j].im = q1616ToFloat(matrix_mem[idx++]);
        }
    for (int i = 0; i < n; i++) {
        Z[i].re = q1616ToFloat(matrix_mem[idx++]);
        Z[i].im = q1616ToFloat(matrix_mem[idx++]);
    }
    DBG("Matrices loaded from memory at %p\n\r", matrix_mem);
}


void receive_spice_netlist(XUartLite *UartInstance, int pre_received)
{
    int idx = 0;
    u8  c;

    // (void)pre_received;

    
    XUartLite_ResetFifos(UartInstance);
    xil_printf("FPGA_READY: Waiting for netlist data...\n\r");

    while (1) {
        if (XUartLite_Recv(UartInstance, &c, 1) > 0) {
            if (c == '~') break;
            if (idx < MAX_SPICE_LEN - 1)
                spice_receive_buffer[idx++] = (char)c;
        }
    }
    spice_receive_buffer[idx] = '\0';
    DBG("FPGA_STATUS: Received %d bytes. Starting MNA...\n\r", idx);
}


void transmit_sweep_results(int nfreqs){
    char line[128];
    char s_freq[32], s_re[32], s_im[32];

    uart_send_str("SWEEP_START\r\n");

    for (int i = 0; i < nfreqs; i++) {
        floatToAscii(sweep_freqs[i], s_freq, sizeof(s_freq));
        floatToAscii(sweep_re[i], s_re, sizeof(s_re));
        floatToAscii(sweep_im[i], s_im, sizeof(s_im));

        int pos = 0, part_len;

        part_len = strlen(s_freq);
        if (pos + part_len < (int)sizeof(line)-4) { 
            memcpy(line+pos, s_freq, part_len); 
            pos += part_len; 
        }
        if (pos < (int)sizeof(line)-4) {
            line[pos++] = ',';
        }

        part_len = strlen(s_re);
        if (pos + part_len < (int)sizeof(line)-4) { 
            memcpy(line+pos, s_re, part_len); 
            pos += part_len; 
        }
        if (pos < (int)sizeof(line)-4) {
            line[pos++] = ',';
        }

        part_len = strlen(s_im);
        if (pos + part_len < (int)sizeof(line)-4) { 
            memcpy(line+pos, s_im, part_len); 
            pos += part_len; 
        }

        line[pos++] = '\r';
        line[pos++] = '\n';
        line[pos] = '\0';

        uart_send_str(line);
    }

    uart_send_str("SWEEP_END\r\n");
}


void transmit_param_sweep_results(int npts)
{
    char line[128], s_val[32], s_re[32], s_im[32];
    uart_send_str("PARAM_SWEEP_START\r\n");
    for (int i = 0; i < npts; i++) {
        floatToAscii(psweep_vals[i], s_val, sizeof(s_val));
        floatToAscii(psweep_re[i], s_re, sizeof(s_re));
        floatToAscii(psweep_im[i], s_im, sizeof(s_im));
        int pos = 0, plen;
        plen = strlen(s_val); 
        if (pos+plen<(int)sizeof(line)-4){
            memcpy(line+pos,s_val,plen);
            pos+=plen;
        }
        if (pos<(int)sizeof(line)-4) {
            line[pos++]=',';
        }

        plen = strlen(s_re);  if (pos+plen<(int)sizeof(line)-4){
            memcpy(line+pos,s_re, plen);
            pos+=plen;
        }
        if (pos<(int)sizeof(line)-4) {
            line[pos++]=',';
        }
        plen = strlen(s_im);  
        if (pos+plen<(int)sizeof(line)-4){
            memcpy(line+pos,s_im, plen);pos+=plen;
        }
        line[pos++]='\r'; 
        line[pos++]='\n'; 
        line[pos]='\0';
        uart_send_str(line);
    }
    uart_send_str("PARAM_SWEEP_END\r\n");
}

static int32_t isqrt_q1616(int64_t val_q32)
{
   
    if (val_q32 <= 0) {
        return 0;
    }
    int64_t x = val_q32 >> 1;
    for (int i = 0; i < 20; i++) {
        if (x == 0) {
            break;
        }
        x = (x + val_q32 / x) >> 1;
    }
    
    return (int32_t)x;
}


static int32_t atan2_q1616(int32_t y, int32_t x)
{
    if (x == 0 && y == 0) return 0;

    int32_t ax = (x < 0) ? -x : x;
    int32_t ay = (y < 0) ? -y : y;
    int32_t a;

    if (ax >= ay) {
        
        int32_t r  = (int32_t)(((int64_t)ay << 16) / ax);
        int32_t r2 = (int32_t)(((int64_t)r  *  r)  >> 16);
        int32_t num   = (int32_t)(((int64_t)ATAN2_K1 * r)  >> 16);
        int32_t denom = (int32_t)(0x10000L + (((int64_t)ATAN2_K2 * r2) >> 16));
        a = (int32_t)(((int64_t)num << 16) / denom);
    } else {
       
        int32_t r  = (int32_t)(((int64_t)ax << 16) / ay);
        int32_t r2 = (int32_t)(((int64_t)r  *  r)  >> 16);
        int32_t num   = (int32_t)(((int64_t)ATAN2_K1 * r)  >> 16);
        int32_t denom = (int32_t)(0x10000L + (((int64_t)ATAN2_K2 * r2) >> 16));
        a = (int32_t)(ATAN2_PI_2 - (int32_t)(((int64_t)num << 16) / denom));
    }

    if (x < 0) a = (int32_t)(ATAN2_PI - a);
    if (y < 0) a = -a;
    return a;
}


static int phase_to_pixel_y(int32_t phase_q) {
    if (phase_q >  PHASE_MAX_Q) phase_q =  PHASE_MAX_Q;
    if (phase_q <  PHASE_MIN_Q) phase_q =  PHASE_MIN_Q;
    int32_t num   = phase_q - PHASE_MIN_Q;          
    int32_t denom = PHASE_MAX_Q - PHASE_MIN_Q;     
   
    return PLOT_HEIGHT_SMALL - (int)(((int64_t)num * PLOT_HEIGHT_SMALL) / denom);
}


static int downsample_index(int i, int nin, int nout) {
    if (nout <= 1) return 0;
    return (int)((float)i * (float)(nin - 1) / (float)(nout - 1) + 0.5f);
}


void plot_sweep_results(int *DDR_addr, float *re, float *im, float *xvals, int npts, int is_voltage_sweep) {

    if (npts <= 0) {
        return;
    }
    plot_is_voltage_sweep = is_voltage_sweep;

  
    if (is_voltage_sweep && npts > POINTS_TO_PLOT) return;

    int n = (npts < POINTS_TO_PLOT) ? npts : POINTS_TO_PLOT;


    for (int i = 0; i < n; i++) {
        int src = (npts <= POINTS_TO_PLOT) ? i : downsample_index(i, npts, POINTS_TO_PLOT);
        if (src >= npts) {
            src = npts - 1;
        }

        int32_t re_q = floatToQ1616(re[src]);
        int32_t im_q = floatToQ1616(im[src]);

       
        int64_t sum2  = (int64_t)re_q * re_q + (int64_t)im_q * im_q;
        plot_mag_q[i] = isqrt_q1616(sum2);

        
        plot_phase_q[i] = rad_to_deg_q1616(atan2_q1616(im_q, re_q));

       
        plot_freqs_q[i] = floatToQ1616(xvals[src]);
    }

   
    for (int i = n; i < POINTS_TO_PLOT; i++) {
        plot_mag_q[i] = 0;
        plot_phase_q[i] = 0;
        plot_freqs_q[i] = (n > 0) ? plot_freqs_q[n-1] : 0;
    }

 
    if (is_voltage_sweep) {
        int32_t peak = 1; 
        for (int i = 0; i < n; i++) {
            if (plot_mag_q[i] > peak) {
                peak = plot_mag_q[i];
            }
        }
        plot_lin_fullscale_q = peak;
    }

    
	drawFilledRect(DDR_addr, OFFSET, PLOT_TOP_Y - 5, X_MAX, Y_MAX, black);

    
    drawPlot(DDR_addr, white);

   
    redraw_plot_points(DDR_addr, plot_mag_q, plot_phase_q, 0);

    
    draw_point_info(DDR_addr, 0, plot_mag_q, plot_phase_q);
}


XUartLite UartLiteInstance;

int main()
{
    xil_printf("MicroBlaze SPICE Sweep Solver OK\n\r");

    XUartLite_Initialize(&UartLiteInstance, XPAR_UARTLITE_0_DEVICE_ID);
    XUartLite_ResetFifos(&UartLiteInstance);
    g_uart = &UartLiteInstance;

    int *DDR_addr = (int *)0x81000000;
    volatile int *TFT_addr = (volatile int *)0x44a00000;

    // Unit 1 
    volatile int32_t *BRAM1_base = (volatile int32_t *)LU1_BRAM_BASE;
    volatile int32_t *BRAM1_A_addr = BRAM1_base;
    volatile int32_t *BRAM1_b_addr = BRAM1_base + BRAM_B_OFFSET_WORDS;
    volatile int32_t *BRAM1_x_addr = BRAM1_base + BRAM_X_OFFSET_WORDS;
    volatile int32_t *LU1_status = (volatile int32_t *)LU1_STATUS_ADDR;

    // Unit 2
    volatile int32_t *BRAM2_base = (volatile int32_t *)LU2_BRAM_BASE;
    volatile int32_t *BRAM2_A_addr = BRAM2_base;
    volatile int32_t *BRAM2_b_addr = BRAM2_base + BRAM_B_OFFSET_WORDS;
    volatile int32_t *BRAM2_x_addr = BRAM2_base + BRAM_X_OFFSET_WORDS;
    volatile int32_t *LU2_status = (volatile int32_t *)LU2_STATUS_ADDR;

    // VGA 
    disableVGA(TFT_addr);
    setVideoMemAddr(TFT_addr, DDR_addr);
    clearScreen(DDR_addr);
    enableVGA(TFT_addr);

  
    while (1)
    {
        DBG("\nWaiting for new netlist...\n\r");

        // Receive netlist
        receive_spice_netlist(&UartLiteInstance, 0);

        // Parse simulation parameters 
        SimParams sim;
        parse_sim_params(spice_receive_buffer, &sim);

        if (sim.is_sweep){
            DBG("AC sweep: %d Hz to %d Hz, %d pts/decade\n\r", (int)sim.freq_start, (int)sim.freq_stop, sim.points_per_decade);
        }
        else {
            DBG("Single AC point: %d Hz\n\r", (int)sim.freq_start);
        }
        if (sim.probe_node[0] != '\0'){
            DBG("Probe node: %s\n\r", sim.probe_node);
        }
        else {
            DBG("No .node directive found - will record node index 0\n\r");
        }

        // Parse circuit elements
        Element elements[MAX_ELEMENTS];
        char nodes[MAX_NODES][MAX_NAME];
        int ne, nnodes;

        parse_netlist_from_buffer(spice_receive_buffer, elements, &ne, nodes, &nnodes);
        DBG("Parsed %d elements, %d nodes\n\r", ne, nnodes);
        drawCircuitFromSpice(DDR_addr, elements, ne, nodes, nnodes, white, sim.probe_node);
        // Build frequency list
        int nfreqs = build_freq_list(&sim);
        DBG("Running %d frequency point(s)...\n\r", nfreqs);

        int nunknown_tmp;
        float omega_tmp = 6.28318f * freq_list[0];
        build_ac_mna(elements, ne, nodes, nnodes, omega_tmp, &nunknown_tmp);

        int probe_idx = 0;
        if (sim.probe_node[0] != '\0') {
            char target[MAX_NAME + 4];
            target[0] = 'V'; target[1] = '(';
            int ni = 2;
            for (int c = 0; sim.probe_node[c] && ni < MAX_NAME+2; c++) {
                target[ni++] = sim.probe_node[c];
            }
            target[ni++] = ')';
            target[ni]   = '\0';

            for (int i = 0; i < nunknown_tmp; i++) {
                if (!strcmp(unknowns[i], target)) {
                    probe_idx = i;
                    break;
                }
            }
            DBG("Probe unknown index: %d  (%s)\n\r", probe_idx, unknowns[probe_idx]);
        }

        // Frequency sweep loop
        {
            int fi = 0;
            while (fi < nfreqs)
            {
                int have_second = (fi + 1 < nfreqs) || (elements[0].type == 'V');

               
                float freq_a  = freq_list[fi];
                float omega_a = 6.28318f * freq_a;
                int   nu_a;

                DBG("--- Point %d/%d : %d Hz (Unit1) ---\n\r", fi + 1, nfreqs, (int)freq_a);

                build_ac_mna(elements, ne, nodes, nnodes, omega_a, &nu_a);
                print_matrix(nu_a);
                save_matrices_to_memory_at(nu_a, BRAM1_base, 0, 0);

                Xil_Out32((UINTPTR)(LU1_status + 1), (u32)BRAM1_A_addr);
                Xil_Out32((UINTPTR)(LU1_status + 2), (u32)BRAM1_b_addr);
                Xil_Out32((UINTPTR)(LU1_status + 3), (u32)BRAM1_x_addr);

               
                int   nu_b   = 0;
                float freq_b = 0.0f;
                if (have_second) {
                    freq_b        = freq_list[fi + 1];
                    float omega_b = 6.28318f * freq_b;

                    DBG("--- Point %d/%d : %d Hz (Unit2) ---\n\r",
                        fi + 2, nfreqs, (int)freq_b);

                    build_ac_mna(elements, ne, nodes, nnodes, omega_b, &nu_b);
                    save_matrices_to_memory_at(nu_b, BRAM2_base, 0, 0);

                    Xil_Out32((UINTPTR)(LU2_status + 1), (u32)BRAM2_A_addr);
                    Xil_Out32((UINTPTR)(LU2_status + 2), (u32)BRAM2_b_addr);
                    Xil_Out32((UINTPTR)(LU2_status + 3), (u32)BRAM2_x_addr);
                }

                // Fire both simultanously
                mbar(0);
                Xil_Out32((UINTPTR)LU1_status, LU_CMD_START);
                if (have_second){
                    Xil_Out32((UINTPTR)LU2_status, LU_CMD_START);
                }
                mbar(0);

                // Wait for Unit 1
                uint32_t st1;
                do {
                    st1 = *LU1_status;
                    if (st1 & LU_STATUS_ERR_BIT) {
                        xil_printf("ERROR: Unit1 failed at %d Hz\n\r", (int)freq_a);
                        break;
                    }
                } while (!(st1 & LU_STATUS_DONE_BIT)){
                    Xil_Out32((UINTPTR)LU1_status, 0);
                }

                // Wait for Unit 2
                if (have_second) {
                    uint32_t st2;
                    do {
                        st2 = *LU2_status;
                        if (st2 & LU_STATUS_ERR_BIT) {
                            xil_printf("ERROR: Unit2 failed at %d Hz\n\r", (int)freq_b);
                            break;
                        }
                    } while (!(st2 & LU_STATUS_DONE_BIT));
                    Xil_Out32((UINTPTR)LU2_status, 0);
                }

                
                {
                    int xi = 0;
                    for (int j = 0; j < nu_a; j++) {
                        X[j].re = q1616ToFloat(BRAM1_x_addr[xi++]);
                        X[j].im = q1616ToFloat(BRAM1_x_addr[xi++]);
                    }
                    if (DEBUG_MODE) {
                        xil_printf("Solution at %d Hz:\n\r", (int)freq_a);
                        for (int i = 0; i < nu_a; i++) {
                            xil_printf("  %s = ", unknowns[i]);
                            print_float(X[i].re);
                            xil_printf(" + j");
                            print_float(X[i].im);
                            xil_printf("\n\r");
                        }
                    }
                    sweep_freqs[fi] = freq_a;
                    sweep_re[fi] = (probe_idx < nu_a) ? X[probe_idx].re : 0.0f;
                    sweep_im[fi] = (probe_idx < nu_a) ? X[probe_idx].im : 0.0f;
                }

                
                if (have_second) {
                    int xi = 0;
                    for (int j = 0; j < nu_b; j++) {
                        X[j].re = q1616ToFloat(BRAM2_x_addr[xi++]);
                        X[j].im = q1616ToFloat(BRAM2_x_addr[xi++]);
                    }
                    if (DEBUG_MODE) {
                        xil_printf("Solution at %d Hz:\n\r", (int)freq_b);
                        for (int i = 0; i < nu_b; i++) {
                            xil_printf("  %s = ", unknowns[i]);
                            print_float(X[i].re);
                            xil_printf(" + j");
                            print_float(X[i].im);
                            xil_printf("\n\r");
                        }
                    }
                    sweep_freqs[fi + 1] = freq_b;
                    sweep_re[fi + 1] = (probe_idx < nu_b) ? X[probe_idx].re : 0.0f;
                    sweep_im[fi + 1] = (probe_idx < nu_b) ? X[probe_idx].im : 0.0f;
                }

                fi += have_second ? 2 : 1;
            }
        }

        DBG("Sweep complete. Sending results...\n\r");

        
        int npts = 0;   

        if (sim.has_param_sweep) {
           
            float fixed_omega    = 6.28318f * sim.freq_start;
            int   sweep_elem_idx = -1;
            int   is_voltage_sweep = 0; 

            for (int ei = 0; ei < ne; ei++) {
                char ea[MAX_NAME], eb[MAX_NAME];
                int ci;
                strncpy(ea, elements[ei].name, MAX_NAME-1); 
                ea[MAX_NAME-1]='\0';
                strncpy(eb, sim.sweep_element,  MAX_NAME-1); 
                eb[MAX_NAME-1]='\0';
                for (ci=0; ea[ci]; ci++) {
                    if(ea[ci]>='a') {
                        ea[ci]-=32;
                    }
                }
                for (ci=0; eb[ci]; ci++) {
                    if(eb[ci]>='a') {
                        eb[ci]-=32;
                    }
                }
                if (!strcmp(ea, eb)) { 
                    sweep_elem_idx = ei; 
                    break; 
                }
            }

            if (sweep_elem_idx < 0) {
                xil_printf("ERROR: sweep element %s not found\n\r", sim.sweep_element);
            } else {
                is_voltage_sweep = (elements[sweep_elem_idx].type == 'V');
                DBG("Param sweep: element type '%c', skip_decomp=%d\n\r",
                    elements[sweep_elem_idx].type, is_voltage_sweep);

                float orig_val = elements[sweep_elem_idx].value;
                int   going_up = (sim.sweep_stop >= sim.sweep_start);

                float sweep_val_list[MAX_FREQ_POINTS];
                int   nsweep = 0;
                {
                    float v = sim.sweep_start;
                    while (nsweep < MAX_FREQ_POINTS) {
                        if ( going_up && v > sim.sweep_stop + 1e-9f) {
                            break;
                        }
                        if (!going_up && v < sim.sweep_stop - 1e-9f) {
                            break;
                        }
                        sweep_val_list[nsweep++] = v;
                        v += sim.sweep_step;
                    }
                }

                int si = 0;
                while (si < nsweep && npts < MAX_FREQ_POINTS) {

                    int have_second = (si + 1 < nsweep) &&
                                      (npts + 1 < MAX_FREQ_POINTS);

                    
                    int   nu_a = 5;
                    float val_a = sweep_val_list[si];
                    elements[sweep_elem_idx].value = val_a;

                    if (!is_voltage_sweep) {
                    	build_ac_mna(elements, ne, nodes, nnodes, fixed_omega, &nu_a);
                    }
                    save_matrices_to_memory_at(nu_a, BRAM1_base, is_voltage_sweep, val_a);

                    Xil_Out32((UINTPTR)(LU1_status + 1), (u32)BRAM1_A_addr);
                    Xil_Out32((UINTPTR)(LU1_status + 2), (u32)BRAM1_b_addr);
                    Xil_Out32((UINTPTR)(LU1_status + 3), (u32)BRAM1_x_addr);

                    
                    int   nu_b = 5;
                    float val_b = 0.0f;
                    if (have_second) {
                        val_b = sweep_val_list[si + 1];
                        elements[sweep_elem_idx].value = val_b;

                        if (!is_voltage_sweep)
                        	build_ac_mna(elements, ne, nodes, nnodes, fixed_omega, &nu_b);

                        save_matrices_to_memory_at(nu_b, BRAM2_base, is_voltage_sweep, val_b);

                        Xil_Out32((UINTPTR)(LU2_status + 1), (u32)BRAM2_A_addr);
                        Xil_Out32((UINTPTR)(LU2_status + 2), (u32)BRAM2_b_addr);
                        Xil_Out32((UINTPTR)(LU2_status + 3), (u32)BRAM2_x_addr);
                    }

                    uint32_t cmd_a = (is_voltage_sweep && si >= 0)
                                     ? LU_CMD_SKIP_DECOMP : LU_CMD_START;
                    uint32_t cmd_b = (is_voltage_sweep && si >= 0)
                                     ? LU_CMD_SKIP_DECOMP : LU_CMD_START;

                    mbar(0);
                    Xil_Out32((UINTPTR)LU1_status, cmd_a);
                    if (have_second)
                        Xil_Out32((UINTPTR)LU2_status, cmd_b);
                    mbar(0);

                    DBG("Fired Unit1(val="); if (DEBUG_MODE) print_float(val_a);
                    DBG(", cmd=0x%02X)", (unsigned)cmd_a);
                    if (have_second) {
                        DBG("  Unit2(val="); 
                        if (DEBUG_MODE) {
                            print_float(val_b);
                        }
                        DBG(", cmd=0x%02X)", (unsigned)cmd_b);
                    }
                    DBG("\n\r");

                    
                    uint32_t st1;
                    do {
                        st1 = *LU1_status;
                        if (st1 & LU_STATUS_ERR_BIT) {
                            xil_printf("ERROR: Unit1 failed at val=");
                            print_float(val_a); 
                            xil_printf("\n\r");
                            break;
                        }
                    } while (!(st1 & LU_STATUS_DONE_BIT));
                    Xil_Out32((UINTPTR)LU1_status, 0);

                    
                    if (have_second) {
                        uint32_t st2;
                        do {
                            st2 = *LU2_status;
                            if (st2 & LU_STATUS_ERR_BIT) {
                                xil_printf("ERROR: Unit2 failed at val=");
                                print_float(val_b); 
                                xil_printf("\n\r");
                                break;
                            }
                        } while (!(st2 & LU_STATUS_DONE_BIT));
                        Xil_Out32((UINTPTR)LU2_status, 0);
                    }

                   
                    {
                        int xi = 0;
                        for (int j = 0; j < nu_a; j++) {
                            X[j].re = q1616ToFloat(BRAM1_x_addr[xi++]);
                            X[j].im = q1616ToFloat(BRAM1_x_addr[xi++]);
                        }
                        psweep_vals[npts] = val_a;
                        psweep_re[npts] = (probe_idx < nu_a) ? X[probe_idx].re : 0.0f;
                        psweep_im[npts] = (probe_idx < nu_a) ? X[probe_idx].im : 0.0f;
                        npts++;
                    }

                   
                    if (have_second) {
                        int xi = 0;
                        for (int j = 0; j < nu_b; j++) {
                            X[j].re = q1616ToFloat(BRAM2_x_addr[xi++]);
                            X[j].im = q1616ToFloat(BRAM2_x_addr[xi++]);
                        }
                        psweep_vals[npts] = val_b;
                        psweep_re[npts]   = (probe_idx < nu_b) ? X[probe_idx].re : 0.0f;
                        psweep_im[npts]   = (probe_idx < nu_b) ? X[probe_idx].im : 0.0f;
                        npts++;
                    }

                    si += have_second ? 2 : 1;
                }

                elements[sweep_elem_idx].value = orig_val;
            }

            DBG("Param sweep done. %d points. Sending...\n\r", npts);
            transmit_param_sweep_results(npts);
            plot_sweep_results(DDR_addr, psweep_re, psweep_im, psweep_vals, npts, is_voltage_sweep);

        } else {
            
            transmit_sweep_results(nfreqs);
            plot_sweep_results(DDR_addr, sweep_re, sweep_im, sweep_freqs, nfreqs, 0);
        }

        xil_printf("SWEEP_DONE\n\r");

        
        {
            int nplot    = POINTS_TO_PLOT;
            int selected = 0;

            // Poll until centre button pressed
            int done = 0;
            while (!done) {
                uint32_t btns = read_buttons();

                if (btns & BTN_CENTER) {
                    debounce();
                    while (read_buttons() & BTN_CENTER);  
                    done = 1;
                } else if (btns & BTN_RIGHT) {
                    debounce();
                    while (read_buttons() & BTN_RIGHT);
                    selected = (selected + 1) % nplot;
                    redraw_plot_points(DDR_addr, plot_mag_q, plot_phase_q, selected);
                    draw_point_info   (DDR_addr, selected,   plot_mag_q, plot_phase_q);
                } else if (btns & BTN_LEFT) {
                    debounce();
                    while (read_buttons() & BTN_LEFT);
                    selected = (selected - 1 + nplot) % nplot;
                    redraw_plot_points(DDR_addr, plot_mag_q, plot_phase_q, selected);
                    draw_point_info   (DDR_addr, selected,   plot_mag_q, plot_phase_q);
                }
            }
            
        }
    }

    return 0;
}



void drawFilledRect(int *DDR_addr, int top_left_x, int top_left_y, int length, int width, color c) {
    int packed = ((c.R & 0x0F) * 0x100000) + ((c.G & 0x0F) * 0x1000) + ((c.B & 0x0F) * 0x10);

    for (int j = 0; j < width; j++) {
        int *row = DDR_addr + 1024 * (top_left_y + j) + top_left_x;
        if (packed == 0)
            memset(row, 0, length * sizeof(int));
        else
            for (int i = 0; i < length; i++)
                row[i] = packed;
    }
}

void drawFilledCircle(int *DDR_addr, int cent_x, int cent_y, int radius, color c) {
    for (int x = cent_x - radius; x < cent_x + radius; x++)
        for (int y = cent_y - radius; y < cent_y + radius; y++)
            if ((cent_x-x)*(cent_x-x) + (cent_y-y)*(cent_y-y) <= radius*radius)
                writeRGB(DDR_addr, x, y, c);
}

void drawText(int *DDR_addr, cursor *curs, const char *text, color c)
{
    while (*text) {
        int idx   = fontIndexFromAscii(*text);
        unsigned char *glyph = font4x6[idx];

        for (int row = 0; row < 6; row++) {
            unsigned char byte    = glyph[row / 2];
            unsigned char rowbits = (row % 2 == 0) ? (byte >> 4) & 0x0F: byte & 0x0F;
            for (int col = 0; col < 4; col++) {
                color px = (rowbits & (1 << (3 - col))) ? c : black;
                for (int i = 0; i < 2; i++)
                    for (int j = 0; j < 2; j++)
                        writeRGB(DDR_addr, curs->x + col*2 + i, curs->y + row*2 + j, px);
            }
        }
        curs->x += 10;
        text++;
    }
}

void drawLine(int *DDR_addr, int x0, int y0, int x1, int y1, color c)
{
    int dx = abs(x1-x0), sx = x0<x1 ? 1 : -1;
    int dy = -abs(y1-y0), sy = y0<y1 ? 1 : -1;
    int err = dx+dy;
    while (1) {
        writeRGB(DDR_addr, x0, y0, c);
        if (x0==x1 && y0==y1) break;
        int e2 = 2*err;
        if (e2 >= dy) { 
            err+=dy; 
            x0+=sx; 
        }
        if (e2 <= dx) { 
            err+=dx; 
            y0+=sy; 
        }
    }
}

void drawPlot(int *DDR_addr, color c)
{
    if (plot_is_voltage_sweep) {
       
        int volt_spacing = PLOT_LENGTH / 9;
        int prev_v_x = -1;

        int v_max = (int)(plot_lin_fullscale_q >> 16);
        int num_y_labels = v_max + 1;  
        if (num_y_labels > 11) {
            num_y_labels = 11; 
        }
        if (num_y_labels < 2) {
            num_y_labels = 2;   
        }
        int volt_y_spacing = PLOT_HEIGHT_BIG / (num_y_labels - 1);

        for (int j = 0; j < num_y_labels; j++) {
            int y = PLOT_TOP_Y + j * volt_y_spacing;
            int v_int = v_max - (v_max * j / (num_y_labels - 1));

            char label[6];
            int_to_str(v_int, label, sizeof(label));

            drawLine(DDR_addr, PLOT_TOP_X, y, PLOT_TOP_X + 2, y, white);
            drawLine(DDR_addr, PLOT_TOP_X + 3, y, PLOT_TOP_X + PLOT_LENGTH - 1, y, grey);
            cursor curs = {.x = OFFSET, .y = y - 3};
            drawText(DDR_addr, &curs, label, white);
        }
        for (int j = 0; j < 9; j++) {
			int x = PLOT_TOP_X + j * volt_spacing;

			
			int32_t v_q = plot_freqs_q[0] + (int32_t)(((int64_t)(plot_freqs_q[POINTS_TO_PLOT-1] - plot_freqs_q[0]) * j) / 9);

			
			int v_int = (int)(v_q >> 16);

			char label[6];
			
			int_to_str(v_int, label, sizeof(label));

			drawLine(DDR_addr, x, PLOT_TOP_Y + PLOT_HEIGHT_BIG - 2,
							   x, PLOT_TOP_Y + PLOT_HEIGHT_BIG, white);
			drawLine(DDR_addr, x, PLOT_TOP_Y + PLOT_HEIGHT_BIG - 3,
							   x, PLOT_TOP_Y + 1, grey);

			if (v_int != prev_v_x) {
				cursor curs = {.x = x - 8, .y = PLOT_TOP_Y + PLOT_HEIGHT_BIG + 2};
				drawText(DDR_addr, &curs, label, white);
				prev_v_x = v_int;
			}
		}
        
		for (int i = 0; i < 1; i++) {
			
			drawLine(DDR_addr, PLOT_TOP_X + i, PLOT_TOP_Y,
							   PLOT_TOP_X + i, PLOT_TOP_Y + PLOT_HEIGHT_BIG, c);
			drawLine(DDR_addr, PLOT_TOP_X + i + PLOT_LENGTH, PLOT_TOP_Y,
							   PLOT_TOP_X + i + PLOT_LENGTH, PLOT_TOP_Y + PLOT_HEIGHT_BIG, c);
			
			drawLine(DDR_addr, PLOT_TOP_X, PLOT_TOP_Y + i,
							   PLOT_TOP_X + PLOT_LENGTH, PLOT_TOP_Y + i, c);
			drawLine(DDR_addr, PLOT_TOP_X, PLOT_TOP_Y + i + PLOT_HEIGHT_BIG,
							   PLOT_TOP_X + PLOT_LENGTH, PLOT_TOP_Y + i + PLOT_HEIGHT_BIG, c);
		}
    } else {
    	const char* freq_labels[] = {"1E+3", "1E+4", "1E+5", "1E+6", "1E+7", "1E+8", "1E+9", "1E+10", ""};
    	int freq_spacing = PLOT_LENGTH / 8;
    	const char* mag_labels[] = {"0", "-9", "-18", "-27", "-36", "-45"};
    	int mag_spacing = PLOT_HEIGHT_SMALL / 5; 
    	const char* phase_labels[] = {"180", "90", "0", "-90", "-180"};
    	int phase_spacing = PLOT_HEIGHT_SMALL / 4;
        
        for (int j = 0; j < 9; j++) {
			cursor curs_mag = {.x = PLOT_TOP_X + j * freq_spacing, .y = PLOT_TOP_Y + PLOT_HEIGHT_SMALL + 2};
			drawLine(DDR_addr, PLOT_TOP_X + j * freq_spacing, PLOT_TOP_Y + PLOT_HEIGHT_SMALL - 2,
					PLOT_TOP_X + j * freq_spacing, PLOT_TOP_Y + PLOT_HEIGHT_SMALL, c);
			drawText(DDR_addr, &curs_mag, freq_labels[j], c);
			drawLine(DDR_addr, PLOT_TOP_X + j * freq_spacing, PLOT_TOP_Y + 1,
					PLOT_TOP_X + j * freq_spacing, PLOT_TOP_Y + PLOT_HEIGHT_SMALL - 3, grey);

			cursor curs_phase = {.x = PLOT_BOT_X + j * freq_spacing, .y = PLOT_BOT_Y + PLOT_HEIGHT_SMALL + 2};
			drawLine(DDR_addr, PLOT_BOT_X + j * freq_spacing, PLOT_BOT_Y + PLOT_HEIGHT_SMALL - 2,
					PLOT_BOT_X + j * freq_spacing, PLOT_BOT_Y + PLOT_HEIGHT_SMALL, c);
			drawText(DDR_addr, &curs_phase, freq_labels[j], c);
			drawLine(DDR_addr, PLOT_BOT_X + j * freq_spacing, PLOT_BOT_Y + 1,
					PLOT_BOT_X + j * freq_spacing, PLOT_BOT_Y + PLOT_HEIGHT_SMALL - 3, grey);
		}
		
		for (int j = 0; j < 6; j++) {
			cursor curs_mag = {.x = OFFSET, .y = PLOT_TOP_Y + j * mag_spacing};
			drawLine(DDR_addr, PLOT_TOP_X, PLOT_TOP_Y + j * mag_spacing,
							   PLOT_TOP_X + 2, PLOT_TOP_Y + j * mag_spacing, c);
			drawLine(DDR_addr, PLOT_TOP_X + 3, PLOT_TOP_Y + j * mag_spacing,
							   PLOT_TOP_X + PLOT_LENGTH - 1, PLOT_TOP_Y + j * mag_spacing, grey);
			drawText(DDR_addr, &curs_mag, mag_labels[j], c);
		}
		
		for (int j = 0; j < 5; j++) {
			cursor curs_phase = {.x = OFFSET, .y = PLOT_BOT_Y + j * phase_spacing};
			drawLine(DDR_addr, PLOT_BOT_X, PLOT_BOT_Y + j * phase_spacing,
							   PLOT_BOT_X + 2, PLOT_BOT_Y + j * phase_spacing, c);
			drawLine(DDR_addr, PLOT_BOT_X + 3, PLOT_BOT_Y + j * phase_spacing,
							   PLOT_BOT_X + PLOT_LENGTH - 1, PLOT_BOT_Y + j * phase_spacing, grey);
			drawText(DDR_addr, &curs_phase, phase_labels[j], c);
		}
		for (int i = 0; i < 1; i++) {
			
			drawLine(DDR_addr, PLOT_TOP_X + i, PLOT_TOP_Y,
							   PLOT_TOP_X + i, PLOT_TOP_Y + PLOT_HEIGHT_SMALL, c);
			drawLine(DDR_addr, PLOT_TOP_X + i + PLOT_LENGTH, PLOT_TOP_Y,
							   PLOT_TOP_X + i + PLOT_LENGTH, PLOT_TOP_Y + PLOT_HEIGHT_SMALL, c);
			
			drawLine(DDR_addr, PLOT_TOP_X, PLOT_TOP_Y + i,
							   PLOT_TOP_X + PLOT_LENGTH, PLOT_TOP_Y + i, c);
			drawLine(DDR_addr, PLOT_TOP_X, PLOT_TOP_Y + i + PLOT_HEIGHT_SMALL,
							   PLOT_TOP_X + PLOT_LENGTH, PLOT_TOP_Y + i + PLOT_HEIGHT_SMALL, c);
			
			drawLine(DDR_addr, PLOT_BOT_X + i, PLOT_BOT_Y,
							   PLOT_BOT_X + i, PLOT_BOT_Y + PLOT_HEIGHT_SMALL, c);
			drawLine(DDR_addr, PLOT_BOT_X + i + PLOT_LENGTH, PLOT_BOT_Y,
							   PLOT_BOT_X + i + PLOT_LENGTH, PLOT_BOT_Y + PLOT_HEIGHT_SMALL, c);
			
			drawLine(DDR_addr, PLOT_BOT_X, PLOT_BOT_Y + i,
							   PLOT_BOT_X + PLOT_LENGTH, PLOT_BOT_Y + i, c);
			drawLine(DDR_addr, PLOT_BOT_X, PLOT_BOT_Y + i + PLOT_HEIGHT_SMALL,
							   PLOT_BOT_X + PLOT_LENGTH, PLOT_BOT_Y + i + PLOT_HEIGHT_SMALL, c);


		}
    }
}

void drawElement(int *DDR_addr, int cell_x, int cell_y, int element, color c)
{
    if (element == 0) {
        drawLine(DDR_addr, cell_x + CIRC_CELL_SIZE/4,    cell_y, cell_x + 3*CIRC_CELL_SIZE/4, cell_y, c);
        drawLine(DDR_addr, cell_x + CIRC_CELL_SIZE/3,    cell_y + 5, cell_x + 2*CIRC_CELL_SIZE/3, cell_y + 5, c);
        drawLine(DDR_addr, cell_x + 5*CIRC_CELL_SIZE/12, cell_y + 10, cell_x + 7*CIRC_CELL_SIZE/12, cell_y + 10, c);
    } else if (element == 1) {
        drawLine(DDR_addr, cell_x + CIRC_CELL_SIZE/2, cell_y,
                           cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE, c);
    } else if (element == 2) {
        drawLine(DDR_addr, cell_x,                cell_y + CIRC_CELL_SIZE/2,
                           cell_x + CIRC_CELL_SIZE, cell_y + CIRC_CELL_SIZE/2, c);
    } else if (element == 3) {
        drawLine(DDR_addr, cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE/2,
                           cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE, c);
        drawLine(DDR_addr, cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE/2,
                           cell_x + CIRC_CELL_SIZE,   cell_y + CIRC_CELL_SIZE/2, c);
    } else if (element == 4) {
        drawLine(DDR_addr, cell_x,                cell_y + CIRC_CELL_SIZE/2,
                           cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE/2, c);
        drawLine(DDR_addr, cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE/2,
                           cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE, c);
    } else if (element == 5) {
        cursor curs = { .x = cell_x + CIRC_CELL_SIZE/2 - 4,
                        .y = cell_y + CIRC_CELL_SIZE/2 - 6 };
        drawText(DDR_addr, &curs, "R", c);
    } else if (element == 6) {
        cursor curs = { .x = cell_x + CIRC_CELL_SIZE/2 - 4,
                        .y = cell_y + CIRC_CELL_SIZE/2 - 6 };
        drawText(DDR_addr, &curs, "C", c);
    } else if (element == 7) {
        cursor curs = { .x = cell_x + CIRC_CELL_SIZE/2 - 4,
                        .y = cell_y + CIRC_CELL_SIZE/2 - 6 };
        drawText(DDR_addr, &curs, "L", c);
    } else if (element == 8) {
        cursor curs = { .x = cell_x + CIRC_CELL_SIZE/2 - 4,
                        .y = cell_y + CIRC_CELL_SIZE/2 - 6 };
        drawText(DDR_addr, &curs, "V", c);
    } else if (element == 9) {
        drawLine(DDR_addr, cell_x,                cell_y + CIRC_CELL_SIZE/2,
                           cell_x + CIRC_CELL_SIZE, cell_y + CIRC_CELL_SIZE/2, c);
        drawLine(DDR_addr, cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE/2,
                           cell_x + CIRC_CELL_SIZE/2, cell_y + CIRC_CELL_SIZE, c);
    }
}

void drawCircuitFromSpice(int* DDR_addr, Element *elements, int ne,
                          char nodes[MAX_NODES][MAX_NAME], int nnodes, color c,
                          const char *probe_node) {

    
    int col_of[MAX_NODES];
    for (int i = 0; i < nnodes; i++) {
        col_of[i] = -1;
    }
    
    for (int i = 0; i < nnodes; i++) {
        if (!strcmp(nodes[i], "vdd") || !strcmp(nodes[i], "VDD")) {
            col_of[i] = 0;
            break;
        }
    }
    
    for (int i = 0; i < nnodes; i++) {
        if (col_of[i] != -1) {
            break;
        }    
        col_of[i] = 0; 
        break;
    }

   
    int changed = 1;
    while (changed) {
        changed = 0;
        for (int k = 0; k < ne; k++) {
            if (strcmp(elements[k].n1, "0") == 0 || strcmp(elements[k].n2, "0") == 0) {
                continue;
            }
            if (elements[k].type == 'V' || elements[k].type == 'I') {
                continue;
            }
            int i1 = find_node(nodes, nnodes, elements[k].n1);
            int i2 = find_node(nodes, nnodes, elements[k].n2);
            if (i1 < 0 || i2 < 0) {
                continue;
            }
            if (col_of[i1] != -1 && col_of[i2] == -1) {
                col_of[i2] = col_of[i1] + 2;
                changed = 1;
            } else if (col_of[i2] != -1 && col_of[i1] == -1) {
                col_of[i1] = col_of[i2] - 2;
                changed = 1;
            }
        }
    }

    
    int max_col = 0;
    for (int i = 0; i < nnodes; i++)
        if (col_of[i] > max_col) {
            max_col = col_of[i];
        }

    int src_col = 0;
    for (int k = 0; k < ne; k++) {
        if (elements[k].type == 'V') {
            const char *named = strcmp(elements[k].n1, "0") ? elements[k].n1 : elements[k].n2;
            int idx = find_node(nodes, nnodes, named);
            if (idx >= 0) {
                src_col = col_of[idx];
            }    
            break;
        }
    }

    
    int top_row[MAX_NODES*2];
    for (int i = 0; i <= max_col; i++) {
        top_row[i] = 2; 
    }
    
    top_row[src_col] = 3;
    
    top_row[max_col] = 4;

    
    for (int k = 0; k < ne; k++) {
        if (strcmp(elements[k].n1, "0") == 0 ||
            strcmp(elements[k].n2, "0") == 0) continue;
        if (elements[k].type == 'V' || elements[k].type == 'I') continue;

        int i1 = find_node(nodes, nnodes, elements[k].n1);
        int i2 = find_node(nodes, nnodes, elements[k].n2);
        if (i1 < 0 || i2 < 0) continue;

        int c1 = col_of[i1];
        int c2 = col_of[i2];
        int col_mid = (c1 < c2 ? c1 : c2) + 1;

        switch (elements[k].type) {
            case 'R': top_row[col_mid] = 5; break;
            case 'C': top_row[col_mid] = 6; break;
            case 'L': top_row[col_mid] = 7; break;
            default:  top_row[col_mid] = 2; break;
        }
    }

    
    for (int k = 0; k < ne; k++) {
        int is_shunt = (strcmp(elements[k].n1, "0") == 0 || strcmp(elements[k].n2, "0") == 0);
        if (!is_shunt) {
            continue;
        }
        if (elements[k].type == 'V') {
            continue;
        }

        const char *named = strcmp(elements[k].n2, "0") ? elements[k].n2 : elements[k].n1;
        int idx = find_node(nodes, nnodes, named);
        if (idx < 0) {
            continue;
        }

        int col = col_of[idx];

       
        if (col != src_col && col != max_col)
            top_row[col] = 9;
    }

    
    for (int col = 0; col <= max_col; col++) {
        drawElement(DDR_addr,
            CIRC_ORIGIN_X + col * CIRC_CELL_SIZE,
            CIRC_ORIGIN_Y + CIRC_CELL_SIZE - CIRC_CELL_SIZE,
            top_row[col], c);
    }

    
    drawElement(DDR_addr,
        CIRC_ORIGIN_X + src_col * CIRC_CELL_SIZE,
        CIRC_ORIGIN_Y + 2*CIRC_CELL_SIZE - CIRC_CELL_SIZE,
        8, c);
    drawElement(DDR_addr,
        CIRC_ORIGIN_X + src_col * CIRC_CELL_SIZE,
        CIRC_ORIGIN_Y + 3*CIRC_CELL_SIZE - CIRC_CELL_SIZE,
        1, c);
    drawElement(DDR_addr,
        CIRC_ORIGIN_X + src_col * CIRC_CELL_SIZE,
        CIRC_ORIGIN_Y + 4*CIRC_CELL_SIZE - CIRC_CELL_SIZE,
        0, c);

    
    for (int k = 0; k < ne; k++) {
        int is_shunt = (strcmp(elements[k].n1, "0") == 0 ||
                        strcmp(elements[k].n2, "0") == 0);
        if (!is_shunt) continue;
        if (elements[k].type == 'V') continue;

        const char *named = strcmp(elements[k].n2, "0") ? elements[k].n2 : elements[k].n1;
        int idx = find_node(nodes, nnodes, named);
        if (idx < 0) continue;
        int col = col_of[idx];

        int elem_code;
        switch (elements[k].type) {
            case 'C': elem_code = 6; break;
            case 'L': elem_code = 7; break;
            case 'I': elem_code = 10; break;
            default:  elem_code = 5; break;
        }

        drawElement(DDR_addr, CIRC_ORIGIN_X + col*CIRC_CELL_SIZE, CIRC_ORIGIN_Y + 2*CIRC_CELL_SIZE - CIRC_CELL_SIZE, elem_code, c);
        drawElement(DDR_addr, CIRC_ORIGIN_X + col*CIRC_CELL_SIZE, CIRC_ORIGIN_Y + 3*CIRC_CELL_SIZE - CIRC_CELL_SIZE, 1, c);
        drawElement(DDR_addr, CIRC_ORIGIN_X + col*CIRC_CELL_SIZE, CIRC_ORIGIN_Y + 4*CIRC_CELL_SIZE - CIRC_CELL_SIZE, 0, c);
    }
    
    for (int col = 0; col <= max_col; col++) {
		if (top_row[col] == 9 ) {
			drawFilledRect(DDR_addr,
				CIRC_ORIGIN_X + col * CIRC_CELL_SIZE,
				CIRC_ORIGIN_Y,
				CIRC_CELL_SIZE, CIRC_CELL_SIZE, black);
			drawElement(DDR_addr,
				CIRC_ORIGIN_X + col * CIRC_CELL_SIZE,
				CIRC_ORIGIN_Y,
				9, c);
		}else if (top_row[col] == 4) {
			drawFilledRect(DDR_addr,
				CIRC_ORIGIN_X + col * CIRC_CELL_SIZE,
				CIRC_ORIGIN_Y,
				CIRC_CELL_SIZE, CIRC_CELL_SIZE, black);
			drawElement(DDR_addr,
				CIRC_ORIGIN_X + col * CIRC_CELL_SIZE,
				CIRC_ORIGIN_Y,
				4, c);
		}
	}
   
    if (probe_node && probe_node[0] != '\0') {
        int probe_ni = find_node(nodes, nnodes, probe_node);
        if (probe_ni >= 0 && col_of[probe_ni] != -1) {
            int dot_x = CIRC_ORIGIN_X + col_of[probe_ni] * CIRC_CELL_SIZE + CIRC_CELL_SIZE / 2;
            int dot_y = CIRC_ORIGIN_Y + CIRC_CELL_SIZE / 2;  
            drawFilledCircle(DDR_addr, dot_x, dot_y, 4, orange);
        }
    }
}

void clearScreen(int *DDR_addr)
{
    int *end = DDR_addr + 0x200000;
    for (; DDR_addr < end; DDR_addr++)
        *DDR_addr = 0x00000000;
}

void writeRGB(int *DDR_addr, int x, int y, color c)
{
	if (x < 0 || x >= X_MAX || y < 0 || y >= Y_MAX) return;
    char R = c.R & 0x0F;
    char G = c.G & 0x0F;
    char B = c.B & 0x0F;
    DDR_addr[1024 * y + x] = R * 0x100000 + G * 0x1000 + B * 0x10;
}

void setVideoMemAddr(volatile int *TFT_addr, int *DDR_addr)
{
    TFT_addr[0] = (int)DDR_addr;
}

void disableVGA(volatile int *TFT_addr) { TFT_addr[1] = 0x00; }
void enableVGA (volatile int *TFT_addr) { TFT_addr[1] = 0x01; }

int fontIndexFromAscii(unsigned char c)
{
    if (c >= 'A' && c <= 'Z') return c - 'A';
    if (c >= 'a' && c <= 'z') return c - 'z';
    if (c >= '0' && c <= '9') return (c - '0') + 26;
    if (c == ':') return 36;
    if (c == '.') return 37;
    if (c == '-') return 38;
    if (c == '+') return 39;
    return 40;
}

int32_t floatToQ1616(float x){ return (int32_t)(x * 65536.0f); }

float q1616ToFloat(int32_t x) { return (float)x / 65536.0f; }

void floatToAscii(float f, char *buf, int bufsize)
{
    if (bufsize < 2) return;

    int pos = 0;

    // sign
    if (f < 0) {
        if (pos < bufsize - 1) buf[pos++] = '-';
        f = -f;
    }

    // Integer part
    int int_part = (int)f;
    float frac   = f - int_part;

    
    int frac_part = (int)(frac * 100000.0f + 0.5f);

   
    if (frac_part >= 100000) {
        frac_part = 0;
        int_part += 1;
    }

    
    char temp[16];
    int tpos = 0;

    if (int_part == 0) {
        temp[tpos++] = '0';
    } else {
        while (int_part > 0 && tpos < (int)sizeof(temp) - 1) {
            temp[tpos++] = '0' + (int_part % 10);
            int_part /= 10;
        }
    }


    for (int i = tpos - 1; i >= 0 && pos < bufsize - 1; i--) {
        buf[pos++] = temp[i];
    }


    if (pos < bufsize - 1) buf[pos++] = '.';


    if (pos < bufsize - 6) {
        buf[pos++] = '0' + (frac_part / 10000);
        buf[pos++] = '0' + ((frac_part / 1000) % 10);
        buf[pos++] = '0' + ((frac_part / 100) % 10);
        buf[pos++] = '0' + ((frac_part / 10) % 10);
        buf[pos++] = '0' + (frac_part % 10);
    }


    buf[pos] = '\0';
}

uint32_t read_buttons(void)
{
    volatile uint32_t *btn_gpio = (volatile uint32_t *)BTN_GPIO_BASEADDR;
    return btn_gpio[0];
}

void debounce(void)
{
    for (volatile int i = 0; i < BTN_DEBOUNCE_DELAY; i++);
}


void redraw_plot_points(int *DDR_addr,
                        int32_t *mag_q, int32_t *phase_q, int selected)
{


	if (plot_is_voltage_sweep) {
		for (int i = 0; i < (POINTS_TO_PLOT_PARAM - 1); i++){
			color dot_c  = (i == selected) ? orange : green;
			int   radius = (i == selected) ? 4 : 2;

			int x1_v = PLOT_TOP_X + i * PLOT_LENGTH / (POINTS_TO_PLOT_PARAM - 1);
			int x2_v = PLOT_TOP_X + (i + 1) * PLOT_LENGTH / (POINTS_TO_PLOT_PARAM - 1);
			int y_mag1 = PLOT_TOP_Y + lin_mag_to_pixel_y(mag_q[i]);
			int y_mag2 = PLOT_TOP_Y + lin_mag_to_pixel_y(mag_q[i + 1]);

			drawLine(DDR_addr, x1_v, y_mag1, x2_v, y_mag2, green);
			drawFilledCircle(DDR_addr, x1_v, y_mag1, 4, black);
			drawFilledCircle(DDR_addr, x1_v, y_mag1, radius, dot_c);
		}
	} else {
		for (int i = 0; i < (POINTS_TO_PLOT - 1); i++) {
			int x1 = PLOT_TOP_X + i * PLOT_LENGTH / (POINTS_TO_PLOT - 1);
			int x2 = PLOT_TOP_X + (i + 1) * PLOT_LENGTH / (POINTS_TO_PLOT - 1);
			color dot_c  = (i == selected) ? orange : green;
			int   radius = (i == selected) ? 4 : 2;
			int y_phase1 = PLOT_BOT_Y + phase_to_pixel_y(phase_q[i]);
			int y_phase2 = PLOT_BOT_Y + phase_to_pixel_y(phase_q[i + 1]);

			int32_t mdb1 = to_dB_q(mag_q[i]);
			int32_t mdb2 = to_dB_q(mag_q[i + 1]);
			int y_mag1   = PLOT_TOP_Y + dB_to_pixel_y_mdb(mdb1);
			int y_mag2   = PLOT_TOP_Y + dB_to_pixel_y_mdb(mdb2);

			drawLine(DDR_addr, x1, y_phase1, x2, y_phase2, green);
			drawLine(DDR_addr, x1, y_mag1, x2, y_mag2, green);
			drawFilledCircle(DDR_addr, x1, y_phase1, 4, black);
			drawFilledCircle(DDR_addr, x1, y_mag1,   4, black);
			drawFilledCircle(DDR_addr, x1, y_phase1, radius, dot_c);
			drawFilledCircle(DDR_addr, x1, y_mag1,   radius, dot_c);
		}
	}


    if (plot_is_voltage_sweep) {



    	color c_last = ((POINTS_TO_PLOT_PARAM - 1) == selected) ? orange : green;
		int   radius_last = ((POINTS_TO_PLOT_PARAM - 1) == selected) ? 4 : 2;
		int x_last = PLOT_TOP_X + (POINTS_TO_PLOT_PARAM - 1) * PLOT_LENGTH / (POINTS_TO_PLOT_PARAM - 1);
        int y_mag_last = PLOT_TOP_Y + lin_mag_to_pixel_y(mag_q[POINTS_TO_PLOT_PARAM - 1]);

        drawFilledCircle(DDR_addr, x_last, y_mag_last, 4, black);
        drawFilledCircle(DDR_addr, x_last, y_mag_last, radius_last, c_last);

    } else {

    	color c_last = ((POINTS_TO_PLOT - 1) == selected) ? orange : green;
			int   radius_last = ((POINTS_TO_PLOT - 1) == selected) ? 4 : 2;
			int x_last = PLOT_TOP_X + (POINTS_TO_PLOT - 1) * PLOT_LENGTH / (POINTS_TO_PLOT - 1);
    	int y_phase_last = PLOT_BOT_Y + phase_to_pixel_y(phase_q[POINTS_TO_PLOT - 1]);
		int32_t mdb_last = to_dB_q(mag_q[POINTS_TO_PLOT - 1]);
		int y_mag_last   = PLOT_TOP_Y + dB_to_pixel_y_mdb(mdb_last);

		drawFilledCircle(DDR_addr, x_last, y_mag_last,   4, black);
		drawFilledCircle(DDR_addr, x_last, y_mag_last, radius_last, c_last);
		drawFilledCircle(DDR_addr, x_last, y_phase_last,   4, black);
		drawFilledCircle(DDR_addr, x_last, y_phase_last, radius_last, c_last);
    }
}


void draw_point_info(int *DDR_addr, int idx,
                     int32_t *mag_q, int32_t *phase_q)
{
    drawFilledRect(DDR_addr, COORD_DISPLAY_X, COORD_DISPLAY_Y,
                   COORD_CLEAR_W, COORD_CLEAR_H, black);

    char   buf[24];
    cursor curs = { .x = COORD_DISPLAY_X, .y = COORD_DISPLAY_Y };


    if (plot_is_voltage_sweep) {
        drawText(DDR_addr, &curs, "SRC:", white);
        floatToAscii(q1616ToFloat(plot_freqs_q[idx]), buf, sizeof(buf));
        drawText(DDR_addr, &curs, buf, white);
        drawText(DDR_addr, &curs, "V", white);
    } else {
        drawText(DDR_addr, &curs, "F:", white);
        floatToAscii(q1616ToFloat(plot_freqs_q[idx]), buf, sizeof(buf));
        drawText(DDR_addr, &curs, buf, white);
        drawText(DDR_addr, &curs, " HZ", white);
    }

    curs.x = COORD_DISPLAY_X;
    curs.y = COORD_DISPLAY_Y + 14;


    drawText(DDR_addr, &curs, "MAG:", white);
    if (plot_is_voltage_sweep) {

        int32_t mv = mag_q[idx];   
        floatToAscii(q1616ToFloat(mv), buf, sizeof(buf));
        drawText(DDR_addr, &curs, buf, white);
        drawText(DDR_addr, &curs, "V", white);
    } else {
        int32_t mdb = to_dB_q(mag_q[idx]);
        if (mdb < 0) { drawText(DDR_addr, &curs, "-", white); mdb = -mdb; }
        int whole = mdb / 1000;
        int frac  = mdb % 1000;
        floatToAscii((float)whole + (float)frac * 0.001f, buf, sizeof(buf));
        drawText(DDR_addr, &curs, buf, white);
        drawText(DDR_addr, &curs, "DB", white);
    }


    if (!plot_is_voltage_sweep) {
        drawText(DDR_addr, &curs, " PH:", white);
        int32_t ph_q = phase_q[idx];
        if (ph_q < 0) { 
            drawText(DDR_addr, &curs, "-", white); ph_q = -ph_q; 
        }
        floatToAscii(q1616ToFloat(ph_q), buf, sizeof(buf));
        drawText(DDR_addr, &curs, buf, white);
        drawText(DDR_addr, &curs, "DEG", white);
    }
}

int32_t to_dB_q(int32_t mag_q)
{
    if (mag_q <= 0) return -90000;

    int32_t idx = (int32_t)(((int64_t)mag_q * 500) >> 16);

    if (idx < 1)   {
        idx = 1;
    }
    if (idx > 500) {
        idx = 500;
    }
    return dB_table[idx];
}

int dB_to_pixel_y_mdb(int32_t mdb)
{
    if (mdb > 0) {     
        mdb = 0;
    }
    if (mdb < -45000) {
        mdb = -45000;
    }
    return (int)((-mdb * 130) / 45000);
}


int lin_mag_to_pixel_y(int32_t mag_q)
{
    if (mag_q < 0) {
        mag_q = 0;
    }
    int32_t fs = plot_lin_fullscale_q;
    if (fs <= 0) {
        fs = 1;
    }
    if (mag_q > fs) {
        mag_q = fs;
    }
    return PLOT_HEIGHT_BIG - (int)(((int64_t)mag_q * PLOT_HEIGHT_BIG) / fs);
}

static int32_t rad_to_deg_q1616(int32_t rad_q)
{
    return (int32_t)(((int64_t)rad_q * RAD_TO_DEG_Q) >> 16);
}

void int_to_str(int val, char *buf, int bufsize) {
    int pos = 0;
    if (val < 0) { 
        buf[pos++] = '-'; 
        val = -val;
    }
    char tmp[8]; int tpos = 0;
    if (val == 0) { 
        buf[pos++] = '0'; 
    }
    else { 
        while (val > 0 && tpos < 7) { 
            tmp[tpos++] = '0' + val % 10; 
            val /= 10; 
        } 
    }
    for (int i = tpos - 1; i >= 0 && pos < bufsize - 1; i--) {
        buf[pos++] = tmp[i];
    }
    buf[pos] = '\0';
}
