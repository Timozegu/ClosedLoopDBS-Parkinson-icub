/* Created by Language version: 7.7.0 */
/* VECTORIZED */
#define NRN_VECTORIZED 1
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "mech_api.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define nrn_init _nrn_init__SubTN
#define _nrn_initial _nrn_initial__SubTN
#define nrn_cur _nrn_cur__SubTN
#define _nrn_current _nrn_current__SubTN
#define nrn_jacob _nrn_jacob__SubTN
#define nrn_state _nrn_state__SubTN
#define _net_receive _net_receive__SubTN 
#define evaluate_fct evaluate_fct__SubTN 
#define states states__SubTN 
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define ena _p[0]
#define ena_columnindex 0
#define ek _p[1]
#define ek_columnindex 1
#define tezao _p[2]
#define tezao_columnindex 2
#define dbs _p[3]
#define dbs_columnindex 3
#define gnabar _p[4]
#define gnabar_columnindex 4
#define gkdrbar _p[5]
#define gkdrbar_columnindex 5
#define gl _p[6]
#define gl_columnindex 6
#define el _p[7]
#define el_columnindex 7
#define gcatbar _p[8]
#define gcatbar_columnindex 8
#define gcalbar _p[9]
#define gcalbar_columnindex 9
#define tau_d2 _p[10]
#define tau_d2_columnindex 10
#define gkabar _p[11]
#define gkabar_columnindex 11
#define gkcabar _p[12]
#define gkcabar_columnindex 12
#define ina _p[13]
#define ina_columnindex 13
#define ikD _p[14]
#define ikD_columnindex 14
#define ikA _p[15]
#define ikA_columnindex 15
#define ikAHP _p[16]
#define ikAHP_columnindex 16
#define icaT _p[17]
#define icaT_columnindex 17
#define icaL _p[18]
#define icaL_columnindex 18
#define ilk _p[19]
#define ilk_columnindex 19
#define idbs _p[20]
#define idbs_columnindex 20
#define periodo _p[21]
#define periodo_columnindex 21
#define h_inf _p[22]
#define h_inf_columnindex 22
#define tau_h _p[23]
#define tau_h_columnindex 23
#define m_inf _p[24]
#define m_inf_columnindex 24
#define tau_m _p[25]
#define tau_m_columnindex 25
#define n_inf _p[26]
#define n_inf_columnindex 26
#define tau_n _p[27]
#define tau_n_columnindex 27
#define p_inf _p[28]
#define p_inf_columnindex 28
#define q_inf _p[29]
#define q_inf_columnindex 29
#define tau_p _p[30]
#define tau_p_columnindex 30
#define tau_q _p[31]
#define tau_q_columnindex 31
#define eca _p[32]
#define eca_columnindex 32
#define c_inf _p[33]
#define c_inf_columnindex 33
#define tau_c _p[34]
#define tau_c_columnindex 34
#define d1_inf _p[35]
#define d1_inf_columnindex 35
#define tau_d1 _p[36]
#define tau_d1_columnindex 36
#define d2_inf _p[37]
#define d2_inf_columnindex 37
#define a_inf _p[38]
#define a_inf_columnindex 38
#define tau_a _p[39]
#define tau_a_columnindex 39
#define b_inf _p[40]
#define b_inf_columnindex 40
#define tau_b _p[41]
#define tau_b_columnindex 41
#define r_inf _p[42]
#define r_inf_columnindex 42
#define m _p[43]
#define m_columnindex 43
#define h _p[44]
#define h_columnindex 44
#define n _p[45]
#define n_columnindex 45
#define p _p[46]
#define p_columnindex 46
#define q _p[47]
#define q_columnindex 47
#define c _p[48]
#define c_columnindex 48
#define d1 _p[49]
#define d1_columnindex 49
#define d2 _p[50]
#define d2_columnindex 50
#define cai _p[51]
#define cai_columnindex 51
#define a _p[52]
#define a_columnindex 52
#define b _p[53]
#define b_columnindex 53
#define r _p[54]
#define r_columnindex 54
#define Dm _p[55]
#define Dm_columnindex 55
#define Dh _p[56]
#define Dh_columnindex 56
#define Dn _p[57]
#define Dn_columnindex 57
#define Dp _p[58]
#define Dp_columnindex 58
#define Dq _p[59]
#define Dq_columnindex 59
#define Dc _p[60]
#define Dc_columnindex 60
#define Dd1 _p[61]
#define Dd1_columnindex 61
#define Dd2 _p[62]
#define Dd2_columnindex 62
#define Dcai _p[63]
#define Dcai_columnindex 63
#define Da _p[64]
#define Da_columnindex 64
#define Db _p[65]
#define Db_columnindex 65
#define Dr _p[66]
#define Dr_columnindex 66
#define v _p[67]
#define v_columnindex 67
#define _g _p[68]
#define _g_columnindex 68
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static void _hoc_evaluate_fct(void);
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 
#define NMODL_TEXT 1
#if NMODL_TEXT
static const char* nmodl_file_text;
static const char* nmodl_filename;
extern void hoc_reg_nmodl_text(int, const char*);
extern void hoc_reg_nmodl_filename(int, const char*);
#endif

 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata() {
 Prop *_prop, *hoc_getdata_range(int);
 _prop = hoc_getdata_range(_mechtype);
   _setdata(_prop);
 hoc_retpushx(1.);
}
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 "setdata_SubTN", _hoc_setdata,
 "evaluate_fct_SubTN", _hoc_evaluate_fct,
 0, 0
};
 /* declare global and static user variables */
#define acai acai_SubTN
 double acai = 0.00518;
#define bcai bcai_SubTN
 double bcai = 0.002;
#define con con_SubTN
 double con = 12.8392;
#define cao cao_SubTN
 double cao = 2000;
#define k_r k_r_SubTN
 double k_r = -0.08;
#define k_b k_b_SubTN
 double k_b = 7.5;
#define k_a k_a_SubTN
 double k_a = -14.7;
#define k_d2 k_d2_SubTN
 double k_d2 = 0.02;
#define k_d1 k_d1_SubTN
 double k_d1 = 7.5;
#define k_c k_c_SubTN
 double k_c = -5;
#define k_q k_q_SubTN
 double k_q = 5.8;
#define k_p k_p_SubTN
 double k_p = -6.7;
#define k_n k_n_SubTN
 double k_n = -14;
#define k_h k_h_SubTN
 double k_h = 6.4;
#define k_m k_m_SubTN
 double k_m = -8;
#define sig_b2 sig_b2_SubTN
 double sig_b2 = 10;
#define sig_b1 sig_b1_SubTN
 double sig_b1 = -30;
#define sig_a sig_a_SubTN
 double sig_a = -0.5;
#define sig_d12 sig_d12_SubTN
 double sig_d12 = 20;
#define sig_d11 sig_d11_SubTN
 double sig_d11 = -15;
#define sig_c2 sig_c2_SubTN
 double sig_c2 = 15;
#define sig_c1 sig_c1_SubTN
 double sig_c1 = -20;
#define sig_q2 sig_q2_SubTN
 double sig_q2 = 16;
#define sig_q1 sig_q1_SubTN
 double sig_q1 = -15;
#define sig_p2 sig_p2_SubTN
 double sig_p2 = 15;
#define sig_p1 sig_p1_SubTN
 double sig_p1 = -10;
#define sig_n2 sig_n2_SubTN
 double sig_n2 = 50;
#define sig_n1 sig_n1_SubTN
 double sig_n1 = -40;
#define sig_h2 sig_h2_SubTN
 double sig_h2 = 16;
#define sig_h1 sig_h1_SubTN
 double sig_h1 = -15;
#define sig_m sig_m_SubTN
 double sig_m = -0.7;
#define tau_r tau_r_SubTN
 double tau_r = 2;
#define theta_r theta_r_SubTN
 double theta_r = 0.17;
#define tht_b2 tht_b2_SubTN
 double tht_b2 = -40;
#define tht_b1 tht_b1_SubTN
 double tht_b1 = -60;
#define tht_a tht_a_SubTN
 double tht_a = -40;
#define tau_b1 tau_b1_SubTN
 double tau_b1 = 200;
#define tau_b0 tau_b0_SubTN
 double tau_b0 = 0;
#define tau_a1 tau_a1_SubTN
 double tau_a1 = 1;
#define tau_a0 tau_a0_SubTN
 double tau_a0 = 1;
#define theta_b theta_b_SubTN
 double theta_b = -90;
#define theta_a theta_a_SubTN
 double theta_a = -45;
#define tht_d12 tht_d12_SubTN
 double tht_d12 = -20;
#define tht_d11 tht_d11_SubTN
 double tht_d11 = -40;
#define tht_c2 tht_c2_SubTN
 double tht_c2 = -50;
#define tht_c1 tht_c1_SubTN
 double tht_c1 = -27;
#define tau_d11 tau_d11_SubTN
 double tau_d11 = 500;
#define tau_d10 tau_d10_SubTN
 double tau_d10 = 400;
#define tau_c1 tau_c1_SubTN
 double tau_c1 = 10;
#define tau_c0 tau_c0_SubTN
 double tau_c0 = 45;
#define theta_d2 theta_d2_SubTN
 double theta_d2 = 0.1;
#define theta_d1 theta_d1_SubTN
 double theta_d1 = -60;
#define theta_c theta_c_SubTN
 double theta_c = -30.6;
#define tht_q2 tht_q2_SubTN
 double tht_q2 = -50;
#define tht_q1 tht_q1_SubTN
 double tht_q1 = -50;
#define tht_p2 tht_p2_SubTN
 double tht_p2 = -102;
#define tht_p1 tht_p1_SubTN
 double tht_p1 = -27;
#define tau_q1 tau_q1_SubTN
 double tau_q1 = 400;
#define tau_q0 tau_q0_SubTN
 double tau_q0 = 0;
#define tau_p1 tau_p1_SubTN
 double tau_p1 = 0.33;
#define tau_p0 tau_p0_SubTN
 double tau_p0 = 5;
#define theta_q theta_q_SubTN
 double theta_q = -85;
#define theta_p theta_p_SubTN
 double theta_p = -56;
#define tht_n2 tht_n2_SubTN
 double tht_n2 = -40;
#define tht_n1 tht_n1_SubTN
 double tht_n1 = -40;
#define tau_n1 tau_n1_SubTN
 double tau_n1 = 11;
#define tau_n0 tau_n0_SubTN
 double tau_n0 = 0;
#define theta_n theta_n_SubTN
 double theta_n = -41;
#define tht_h2 tht_h2_SubTN
 double tht_h2 = -50;
#define tht_h1 tht_h1_SubTN
 double tht_h1 = -50;
#define tht_m tht_m_SubTN
 double tht_m = -53;
#define tau_h1 tau_h1_SubTN
 double tau_h1 = 24.5;
#define tau_h0 tau_h0_SubTN
 double tau_h0 = 0;
#define tau_m1 tau_m1_SubTN
 double tau_m1 = 3;
#define tau_m0 tau_m0_SubTN
 double tau_m0 = 0.2;
#define theta_h theta_h_SubTN
 double theta_h = -45.5;
#define theta_m theta_m_SubTN
 double theta_m = -40;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "con_SubTN", "mV",
 "theta_m_SubTN", "mV",
 "theta_h_SubTN", "mV",
 "k_m_SubTN", "mV",
 "k_h_SubTN", "mV",
 "tau_m0_SubTN", "ms",
 "tau_m1_SubTN", "ms",
 "tau_h0_SubTN", "ms",
 "tau_h1_SubTN", "ms",
 "tht_m_SubTN", "mV",
 "tht_h1_SubTN", "mV",
 "tht_h2_SubTN", "mV",
 "sig_m_SubTN", "mV",
 "sig_h1_SubTN", "mV",
 "sig_h2_SubTN", "mV",
 "theta_n_SubTN", "mV",
 "k_n_SubTN", "mV",
 "tau_n0_SubTN", "ms",
 "tau_n1_SubTN", "ms",
 "tht_n1_SubTN", "mV",
 "tht_n2_SubTN", "mV",
 "sig_n1_SubTN", "mV",
 "sig_n2_SubTN", "mV",
 "theta_p_SubTN", "mV",
 "theta_q_SubTN", "mV",
 "k_p_SubTN", "mV",
 "k_q_SubTN", "mV",
 "tau_p0_SubTN", "ms",
 "tau_p1_SubTN", "ms",
 "tau_q0_SubTN", "ms",
 "tau_q1_SubTN", "ms",
 "tht_p1_SubTN", "mV",
 "tht_p2_SubTN", "mV",
 "tht_q1_SubTN", "mV",
 "tht_q2_SubTN", "mV",
 "sig_p1_SubTN", "mV",
 "sig_p2_SubTN", "mV",
 "sig_q1_SubTN", "mV",
 "sig_q2_SubTN", "mV",
 "theta_c_SubTN", "mV",
 "theta_d1_SubTN", "mV",
 "theta_d2_SubTN", "mV",
 "k_c_SubTN", "mV",
 "k_d1_SubTN", "mV",
 "k_d2_SubTN", "mV",
 "tau_c0_SubTN", "ms",
 "tau_c1_SubTN", "ms",
 "tau_d10_SubTN", "ms",
 "tau_d11_SubTN", "ms",
 "tht_c1_SubTN", "mV",
 "tht_c2_SubTN", "mV",
 "tht_d11_SubTN", "mV",
 "tht_d12_SubTN", "mV",
 "sig_c1_SubTN", "mV",
 "sig_c2_SubTN", "mV",
 "sig_d11_SubTN", "mV",
 "sig_d12_SubTN", "mV",
 "theta_a_SubTN", "mV",
 "theta_b_SubTN", "mV",
 "k_a_SubTN", "mV",
 "k_b_SubTN", "mV",
 "tau_a0_SubTN", "ms",
 "tau_a1_SubTN", "ms",
 "tau_b0_SubTN", "ms",
 "tau_b1_SubTN", "ms",
 "tht_a_SubTN", "mV",
 "tht_b1_SubTN", "mV",
 "tht_b2_SubTN", "mV",
 "sig_a_SubTN", "mV",
 "sig_b1_SubTN", "mV",
 "sig_b2_SubTN", "mV",
 "theta_r_SubTN", "mV",
 "k_r_SubTN", "mV",
 "tau_r_SubTN", "ms",
 "acai_SubTN", "cm2/mA/ms",
 "bcai_SubTN", "1/ms",
 "ena_SubTN", "mV",
 "ek_SubTN", "mV",
 "tezao_SubTN", "ms",
 "gnabar_SubTN", "S/cm2",
 "gkdrbar_SubTN", "S/cm2",
 "gl_SubTN", "S/cm2",
 "el_SubTN", "mV",
 "gcatbar_SubTN", "S/cm2",
 "gcalbar_SubTN", "S/cm2",
 "tau_d2_SubTN", "ms",
 "gkabar_SubTN", "S/cm2",
 "gkcabar_SubTN", "S/cm2",
 "ina_SubTN", "mA/cm2",
 "ikD_SubTN", "mA/cm2",
 "ikA_SubTN", "mA/cm2",
 "ikAHP_SubTN", "mA/cm2",
 "icaT_SubTN", "mA/cm2",
 "icaL_SubTN", "mA/cm2",
 "ilk_SubTN", "mA/cm2",
 "idbs_SubTN", "mA/cm2",
 "periodo_SubTN", "ms",
 "tau_h_SubTN", "ms",
 "tau_m_SubTN", "ms",
 "tau_n_SubTN", "ms",
 "tau_p_SubTN", "ms",
 "tau_q_SubTN", "ms",
 "eca_SubTN", "mV",
 "tau_c_SubTN", "ms",
 "tau_d1_SubTN", "ms",
 "tau_a_SubTN", "ms",
 "tau_b_SubTN", "ms",
 0,0
};
 static double a0 = 0;
 static double b0 = 0;
 static double cai0 = 0;
 static double c0 = 0;
 static double delta_t = 0.01;
 static double d20 = 0;
 static double d10 = 0;
 static double h0 = 0;
 static double m0 = 0;
 static double n0 = 0;
 static double p0 = 0;
 static double q0 = 0;
 static double r0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "cao_SubTN", &cao_SubTN,
 "con_SubTN", &con_SubTN,
 "theta_m_SubTN", &theta_m_SubTN,
 "theta_h_SubTN", &theta_h_SubTN,
 "k_m_SubTN", &k_m_SubTN,
 "k_h_SubTN", &k_h_SubTN,
 "tau_m0_SubTN", &tau_m0_SubTN,
 "tau_m1_SubTN", &tau_m1_SubTN,
 "tau_h0_SubTN", &tau_h0_SubTN,
 "tau_h1_SubTN", &tau_h1_SubTN,
 "tht_m_SubTN", &tht_m_SubTN,
 "tht_h1_SubTN", &tht_h1_SubTN,
 "tht_h2_SubTN", &tht_h2_SubTN,
 "sig_m_SubTN", &sig_m_SubTN,
 "sig_h1_SubTN", &sig_h1_SubTN,
 "sig_h2_SubTN", &sig_h2_SubTN,
 "theta_n_SubTN", &theta_n_SubTN,
 "k_n_SubTN", &k_n_SubTN,
 "tau_n0_SubTN", &tau_n0_SubTN,
 "tau_n1_SubTN", &tau_n1_SubTN,
 "tht_n1_SubTN", &tht_n1_SubTN,
 "tht_n2_SubTN", &tht_n2_SubTN,
 "sig_n1_SubTN", &sig_n1_SubTN,
 "sig_n2_SubTN", &sig_n2_SubTN,
 "theta_p_SubTN", &theta_p_SubTN,
 "theta_q_SubTN", &theta_q_SubTN,
 "k_p_SubTN", &k_p_SubTN,
 "k_q_SubTN", &k_q_SubTN,
 "tau_p0_SubTN", &tau_p0_SubTN,
 "tau_p1_SubTN", &tau_p1_SubTN,
 "tau_q0_SubTN", &tau_q0_SubTN,
 "tau_q1_SubTN", &tau_q1_SubTN,
 "tht_p1_SubTN", &tht_p1_SubTN,
 "tht_p2_SubTN", &tht_p2_SubTN,
 "tht_q1_SubTN", &tht_q1_SubTN,
 "tht_q2_SubTN", &tht_q2_SubTN,
 "sig_p1_SubTN", &sig_p1_SubTN,
 "sig_p2_SubTN", &sig_p2_SubTN,
 "sig_q1_SubTN", &sig_q1_SubTN,
 "sig_q2_SubTN", &sig_q2_SubTN,
 "theta_c_SubTN", &theta_c_SubTN,
 "theta_d1_SubTN", &theta_d1_SubTN,
 "theta_d2_SubTN", &theta_d2_SubTN,
 "k_c_SubTN", &k_c_SubTN,
 "k_d1_SubTN", &k_d1_SubTN,
 "k_d2_SubTN", &k_d2_SubTN,
 "tau_c0_SubTN", &tau_c0_SubTN,
 "tau_c1_SubTN", &tau_c1_SubTN,
 "tau_d10_SubTN", &tau_d10_SubTN,
 "tau_d11_SubTN", &tau_d11_SubTN,
 "tht_c1_SubTN", &tht_c1_SubTN,
 "tht_c2_SubTN", &tht_c2_SubTN,
 "tht_d11_SubTN", &tht_d11_SubTN,
 "tht_d12_SubTN", &tht_d12_SubTN,
 "sig_c1_SubTN", &sig_c1_SubTN,
 "sig_c2_SubTN", &sig_c2_SubTN,
 "sig_d11_SubTN", &sig_d11_SubTN,
 "sig_d12_SubTN", &sig_d12_SubTN,
 "theta_a_SubTN", &theta_a_SubTN,
 "theta_b_SubTN", &theta_b_SubTN,
 "k_a_SubTN", &k_a_SubTN,
 "k_b_SubTN", &k_b_SubTN,
 "tau_a0_SubTN", &tau_a0_SubTN,
 "tau_a1_SubTN", &tau_a1_SubTN,
 "tau_b0_SubTN", &tau_b0_SubTN,
 "tau_b1_SubTN", &tau_b1_SubTN,
 "tht_a_SubTN", &tht_a_SubTN,
 "tht_b1_SubTN", &tht_b1_SubTN,
 "tht_b2_SubTN", &tht_b2_SubTN,
 "sig_a_SubTN", &sig_a_SubTN,
 "sig_b1_SubTN", &sig_b1_SubTN,
 "sig_b2_SubTN", &sig_b2_SubTN,
 "theta_r_SubTN", &theta_r_SubTN,
 "k_r_SubTN", &k_r_SubTN,
 "tau_r_SubTN", &tau_r_SubTN,
 "acai_SubTN", &acai_SubTN,
 "bcai_SubTN", &bcai_SubTN,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(NrnThread*, _Memb_list*, int);
static void nrn_state(NrnThread*, _Memb_list*, int);
 static void nrn_cur(NrnThread*, _Memb_list*, int);
static void  nrn_jacob(NrnThread*, _Memb_list*, int);
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(NrnThread*, _Memb_list*, int);
static void _ode_matsol(NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[0]._i
 static void _ode_matsol_instance1(_threadargsproto_);
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "7.7.0",
"SubTN",
 "ena_SubTN",
 "ek_SubTN",
 "tezao_SubTN",
 "dbs_SubTN",
 "gnabar_SubTN",
 "gkdrbar_SubTN",
 "gl_SubTN",
 "el_SubTN",
 "gcatbar_SubTN",
 "gcalbar_SubTN",
 "tau_d2_SubTN",
 "gkabar_SubTN",
 "gkcabar_SubTN",
 0,
 "ina_SubTN",
 "ikD_SubTN",
 "ikA_SubTN",
 "ikAHP_SubTN",
 "icaT_SubTN",
 "icaL_SubTN",
 "ilk_SubTN",
 "idbs_SubTN",
 "periodo_SubTN",
 "h_inf_SubTN",
 "tau_h_SubTN",
 "m_inf_SubTN",
 "tau_m_SubTN",
 "n_inf_SubTN",
 "tau_n_SubTN",
 "p_inf_SubTN",
 "q_inf_SubTN",
 "tau_p_SubTN",
 "tau_q_SubTN",
 "eca_SubTN",
 "c_inf_SubTN",
 "tau_c_SubTN",
 "d1_inf_SubTN",
 "tau_d1_SubTN",
 "d2_inf_SubTN",
 "a_inf_SubTN",
 "tau_a_SubTN",
 "b_inf_SubTN",
 "tau_b_SubTN",
 "r_inf_SubTN",
 0,
 "m_SubTN",
 "h_SubTN",
 "n_SubTN",
 "p_SubTN",
 "q_SubTN",
 "c_SubTN",
 "d1_SubTN",
 "d2_SubTN",
 "cai_SubTN",
 "a_SubTN",
 "b_SubTN",
 "r_SubTN",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 69, _prop);
 	/*initialize range parameters*/
 	ena = 60;
 	ek = -90;
 	tezao = 7.6923;
 	dbs = 0;
 	gnabar = 0.049;
 	gkdrbar = 0.057;
 	gl = 0.00035;
 	el = -60;
 	gcatbar = 0.005;
 	gcalbar = 0.015;
 	tau_d2 = 130;
 	gkabar = 0.005;
 	gkcabar = 0.001;
 	_prop->param = _p;
 	_prop->param_size = 69;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 1, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _SubTN_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 69, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 SubTN /home/timozegu/MarmosetBase/MarmosetModel-main/SubTN.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "SubTN ion channels for single compartment model";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int evaluate_fct(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[12], _dlist1[12];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset = 0; {
   evaluate_fct ( _threadargscomma_ v ) ;
   Dh = ( h_inf - h ) / tau_h ;
   Dm = ( m_inf - m ) / tau_m ;
   Dn = ( n_inf - n ) / tau_n ;
   Dp = ( p_inf - p ) / tau_p ;
   Dq = ( q_inf - q ) / tau_q ;
   Dc = ( c_inf - c ) / tau_c ;
   Dd1 = ( d1_inf - d1 ) / tau_d1 ;
   Dd2 = ( d2_inf - d2 ) / tau_d2 ;
   Dcai = - acai * ( icaL + icaT ) - bcai * cai ;
   Da = ( a_inf - a ) / tau_a ;
   Db = ( b_inf - b ) / tau_b ;
   Dr = ( r_inf - r ) / tau_r ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
 evaluate_fct ( _threadargscomma_ v ) ;
 Dh = Dh  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_h )) ;
 Dm = Dm  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_m )) ;
 Dn = Dn  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_n )) ;
 Dp = Dp  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_p )) ;
 Dq = Dq  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_q )) ;
 Dc = Dc  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_c )) ;
 Dd1 = Dd1  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_d1 )) ;
 Dd2 = Dd2  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_d2 )) ;
 Dcai = Dcai  / (1. - dt*( ( - ( bcai )*( 1.0 ) ) )) ;
 Da = Da  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_a )) ;
 Db = Db  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_b )) ;
 Dr = Dr  / (1. - dt*( ( ( ( - 1.0 ) ) ) / tau_r )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) { {
   evaluate_fct ( _threadargscomma_ v ) ;
    h = h + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_h)))*(- ( ( ( h_inf ) ) / tau_h ) / ( ( ( ( - 1.0 ) ) ) / tau_h ) - h) ;
    m = m + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_m)))*(- ( ( ( m_inf ) ) / tau_m ) / ( ( ( ( - 1.0 ) ) ) / tau_m ) - m) ;
    n = n + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_n)))*(- ( ( ( n_inf ) ) / tau_n ) / ( ( ( ( - 1.0 ) ) ) / tau_n ) - n) ;
    p = p + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_p)))*(- ( ( ( p_inf ) ) / tau_p ) / ( ( ( ( - 1.0 ) ) ) / tau_p ) - p) ;
    q = q + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_q)))*(- ( ( ( q_inf ) ) / tau_q ) / ( ( ( ( - 1.0 ) ) ) / tau_q ) - q) ;
    c = c + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_c)))*(- ( ( ( c_inf ) ) / tau_c ) / ( ( ( ( - 1.0 ) ) ) / tau_c ) - c) ;
    d1 = d1 + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_d1)))*(- ( ( ( d1_inf ) ) / tau_d1 ) / ( ( ( ( - 1.0 ) ) ) / tau_d1 ) - d1) ;
    d2 = d2 + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_d2)))*(- ( ( ( d2_inf ) ) / tau_d2 ) / ( ( ( ( - 1.0 ) ) ) / tau_d2 ) - d2) ;
    cai = cai + (1. - exp(dt*(( - ( bcai )*( 1.0 ) ))))*(- ( ( - acai )*( ( icaL + icaT ) ) ) / ( ( - ( bcai )*( 1.0 ) ) ) - cai) ;
    a = a + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_a)))*(- ( ( ( a_inf ) ) / tau_a ) / ( ( ( ( - 1.0 ) ) ) / tau_a ) - a) ;
    b = b + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_b)))*(- ( ( ( b_inf ) ) / tau_b ) / ( ( ( ( - 1.0 ) ) ) / tau_b ) - b) ;
    r = r + (1. - exp(dt*(( ( ( - 1.0 ) ) ) / tau_r)))*(- ( ( ( r_inf ) ) / tau_r ) / ( ( ( ( - 1.0 ) ) ) / tau_r ) - r) ;
   }
  return 0;
}
 
static int  evaluate_fct ( _threadargsprotocomma_ double _lv ) {
   h_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_h ) / k_h ) ) ;
   m_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_m ) / k_m ) ) ;
   tau_h = tau_h0 + tau_h1 / ( exp ( - ( _lv - tht_h1 ) / sig_h1 ) + exp ( - ( _lv - tht_h2 ) / sig_h2 ) ) ;
   tau_m = tau_m0 + tau_m1 / ( 1.0 + exp ( - ( _lv - tht_m ) / sig_m ) ) ;
   n_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_n ) / k_n ) ) ;
   tau_n = tau_n0 + tau_n1 / ( exp ( - ( _lv - tht_n1 ) / sig_n1 ) + exp ( - ( _lv - tht_n2 ) / sig_n2 ) ) ;
   p_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_p ) / k_p ) ) ;
   q_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_q ) / k_q ) ) ;
   tau_p = tau_p0 + tau_p1 / ( exp ( - ( _lv - tht_p1 ) / sig_p1 ) + exp ( - ( _lv - tht_p2 ) / sig_p2 ) ) ;
   tau_q = tau_q0 + tau_q1 / ( exp ( - ( _lv - tht_q1 ) / sig_q1 ) + exp ( - ( _lv - tht_q2 ) / sig_q2 ) ) ;
   c_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_c ) / k_c ) ) ;
   d1_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_d1 ) / k_d1 ) ) ;
   d2_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_d2 ) / k_d2 ) ) ;
   tau_c = tau_c0 + tau_c1 / ( exp ( - ( _lv - tht_c1 ) / sig_c1 ) + exp ( - ( _lv - tht_c2 ) / sig_c2 ) ) ;
   tau_d1 = tau_d10 + tau_d11 / ( exp ( - ( _lv - tht_d11 ) / sig_d11 ) + exp ( - ( _lv - tht_d12 ) / sig_d12 ) ) ;
   a_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_a ) / k_a ) ) ;
   b_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_b ) / k_b ) ) ;
   tau_a = tau_a0 + tau_a1 / ( 1.0 + exp ( - ( _lv - tht_a ) / sig_a ) ) ;
   tau_b = tau_b0 + tau_b1 / ( exp ( - ( _lv - tht_b1 ) / sig_b1 ) + exp ( - ( _lv - tht_b2 ) / sig_b2 ) ) ;
   r_inf = 1.0 / ( 1.0 + exp ( ( _lv - theta_r ) / k_r ) ) ;
    return 0; }
 
static void _hoc_evaluate_fct(void) {
  double _r;
   double* _p; Datum* _ppvar; Datum* _thread; NrnThread* _nt;
   if (_extcall_prop) {_p = _extcall_prop->param; _ppvar = _extcall_prop->dparam;}else{ _p = (double*)0; _ppvar = (Datum*)0; }
  _thread = _extcall_thread;
  _nt = nrn_threads;
 _r = 1.;
 evaluate_fct ( _p, _ppvar, _thread, _nt, *getarg(1) );
 hoc_retpushx(_r);
}
 
static int _ode_count(int _type){ return 12;}
 
static void _ode_spec(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 12; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol_instance1(_threadargsproto_) {
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }
 
static void _ode_matsol(NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol_instance1(_threadargs_);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
  int _i; double _save;{
  a = a0;
  b = b0;
  c = c0;
  cai = cai0;
  d2 = d20;
  d1 = d10;
  h = h0;
  m = m0;
  n = n0;
  p = p0;
  q = q0;
  r = r0;
 {
   evaluate_fct ( _threadargscomma_ v ) ;
   m = m_inf ;
   h = h_inf ;
   n = n_inf ;
   p = p_inf ;
   q = q_inf ;
   c = c_inf ;
   d1 = d1_inf ;
   d2 = d2_inf ;
   a = a_inf ;
   b = b_inf ;
   r = r_inf ;
   cai = 0.005 ;
   periodo = 0.0 ;
   }
 
}
}

static void nrn_init(NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}
}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   eca = con * log ( cao / cai ) ;
   ina = gnabar * m * m * m * h * ( v - ena ) ;
   ikD = gkdrbar * pow( n , 4.0 ) * ( v - ek ) ;
   ikA = gkabar * a * a * b * ( v - ek ) ;
   ikAHP = gkcabar * r * r * ( v - ek ) ;
   icaT = gcatbar * p * p * q * ( v - eca ) ;
   icaL = gcalbar * c * c * d1 * d2 * ( v - eca ) ;
   ilk = gl * ( v - el ) ;
   if ( t >= periodo + tezao ) {
     periodo = periodo + tezao ;
     }
   if ( t >= periodo  && t <= periodo + 0.3 ) {
     idbs = - 0.3 * dbs ;
     }
   else {
     idbs = 0.0 ;
     }
   }
 _current += ilk;
 _current += icaT;
 _current += icaL;
 _current += ikD;
 _current += ikA;
 _current += ikAHP;
 _current += ina;
 _current += idbs;

} return _current;
}

static void nrn_cur(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}
 
}

static void nrn_jacob(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}
 
}

static void nrn_state(NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v = 0.0; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v=_v;
{
 {   states(_p, _ppvar, _thread, _nt);
  }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = h_columnindex;  _dlist1[0] = Dh_columnindex;
 _slist1[1] = m_columnindex;  _dlist1[1] = Dm_columnindex;
 _slist1[2] = n_columnindex;  _dlist1[2] = Dn_columnindex;
 _slist1[3] = p_columnindex;  _dlist1[3] = Dp_columnindex;
 _slist1[4] = q_columnindex;  _dlist1[4] = Dq_columnindex;
 _slist1[5] = c_columnindex;  _dlist1[5] = Dc_columnindex;
 _slist1[6] = d1_columnindex;  _dlist1[6] = Dd1_columnindex;
 _slist1[7] = d2_columnindex;  _dlist1[7] = Dd2_columnindex;
 _slist1[8] = cai_columnindex;  _dlist1[8] = Dcai_columnindex;
 _slist1[9] = a_columnindex;  _dlist1[9] = Da_columnindex;
 _slist1[10] = b_columnindex;  _dlist1[10] = Db_columnindex;
 _slist1[11] = r_columnindex;  _dlist1[11] = Dr_columnindex;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/home/timozegu/MarmosetBase/MarmosetModel-main/SubTN.mod";
static const char* nmodl_file_text = 
  "TITLE  SubTN ion channels for single compartment model\n"
  "\n"
  "\n"
  "\n"
  "UNITSON\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX SubTN\n"
  "	NONSPECIFIC_CURRENT ilk\n"
  "	NONSPECIFIC_CURRENT icaT,icaL,ikD,ikA,ikAHP,ina, idbs\n"
  "\n"
  "	RANGE gnabar, ena, m_inf, h_inf, tau_h, tau_m		   : fast sodium\n"
  "	RANGE gkdrbar, ek, n_inf, tau_n                   : delayed K rectifier\n"
  "	RANGE gl, el                                     : leak\n"
  "	RANGE gcatbar, p_inf, tau_p, q_inf, tau_q	       : T-type ca current\n"
  "	RANGE gcalbar, eca, c_inf, d1_inf, d2_inf, tau_c, tau_d1, tau_d2  : L-type ca current\n"
  "	RANGE gkabar, ek, a_inf, tau_a, b_inf, tau_b     : A-type K current\n"
  "	RANGE gkcabar, ek, r_inf                       : ca dependent AHP K current\n"
  "\n"
  "	RANGE periodo, tezao, cai, dbs\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mA) = (milliamp)\n"
  "	(mV) = (millivolt)\n"
  "	(S)  = (siemens)\n"
  "	(molar) = (1/liter)\n"
  "	(mM)	= (millimolar)\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	ena = 60	(mV)\n"
  "	ek = -90	(mV)\n"
  "	cao = 2000\n"
  "	con =12.8392 (mV)\n"
  "\n"
  "	tezao = 7.6923 (ms)\n"
  "	dbs\n"
  "\n"
  ":Fast Na channel\n"
  "	gnabar   = 49e-3 (S/cm2)\n"
  "	theta_m = -40 (mV)\n"
  "	theta_h = -45.5 (mV)\n"
  "	k_m = -8 (mV)\n"
  "	k_h = 6.4 (mV)\n"
  "	tau_m0 = 0.2 (ms)\n"
  "	tau_m1 = 3 (ms)\n"
  "	tau_h0 = 0 (ms)\n"
  "	tau_h1 = 24.5 (ms)\n"
  "	tht_m = -53 (mV)\n"
  "	tht_h1 = -50 (mV)\n"
  "	tht_h2 = -50 (mV)\n"
  "	sig_m = -0.7 (mV)\n"
  "	sig_h1 = -15 (mV)\n"
  "	sig_h2 = 16 (mV)\n"
  "\n"
  ": Delayed rectifier K\n"
  "	gkdrbar  = 57e-3	(S/cm2)\n"
  "	theta_n = -41 (mV)\n"
  "	k_n = -14 (mV)\n"
  "	tau_n0 = 0 (ms)\n"
  "	tau_n1 = 11 (ms)\n"
  "	tht_n1 = -40 (mV)\n"
  "	tht_n2 = -40 (mV)\n"
  "	sig_n1 = -40 (mV)\n"
  "	sig_n2 = 50 (mV)\n"
  "\n"
  ":Leakage current\n"
  "	gl	= 0.35e-3	(S/cm2)\n"
  "	el	= -60	(mV)\n"
  "\n"
  ":T-type ca current\n"
  "	gcatbar   = 5e-3 (S/cm2)\n"
  "	theta_p = -56 (mV)\n"
  "	theta_q = -85 (mV)\n"
  "	k_p = -6.7 (mV)\n"
  "	k_q = 5.8 (mV)\n"
  "	tau_p0 = 5 (ms)\n"
  "	tau_p1 = 0.33 (ms)\n"
  "	tau_q0 = 0 (ms)\n"
  "	tau_q1 = 400 (ms)\n"
  "	tht_p1 = -27 (mV)\n"
  "	tht_p2 = -102 (mV)\n"
  "	tht_q1 = -50 (mV)\n"
  "	tht_q2 = -50 (mV)\n"
  "	sig_p1 = -10 (mV)\n"
  "	sig_p2 = 15 (mV)\n"
  "	sig_q1 = -15 (mV)\n"
  "	sig_q2 = 16 (mV)\n"
  "\n"
  ":Ca L current\n"
  "	gcalbar   = 15e-3 (S/cm2)\n"
  "	theta_c = -30.6 (mV)\n"
  "	theta_d1 = -60 (mV)\n"
  "	theta_d2 = 0.1 (mV)\n"
  "	k_c = -5 (mV)\n"
  "	k_d1 = 7.5 (mV)\n"
  "	k_d2 = 0.02 (mV)\n"
  "	tau_c0 = 45 (ms)\n"
  "	tau_c1 = 10 (ms)\n"
  "	tau_d10 = 400 (ms)\n"
  "	tau_d11 = 500 (ms)\n"
  "	tht_c1 = -27 (mV)\n"
  "	tht_c2 = -50 (mV)\n"
  "	tht_d11 = -40 (mV)\n"
  "	tht_d12 = -20 (mV)\n"
  "	sig_c1 = -20 (mV)\n"
  "	sig_c2 = 15 (mV)\n"
  "	sig_d11 = -15 (mV)\n"
  "	sig_d12 = 20 (mV)\n"
  "	tau_d2 = 130 (ms)\n"
  "\n"
  ":A current\n"
  "	gkabar  = 5e-3	(S/cm2)\n"
  "	theta_a = -45 (mV)\n"
  "	theta_b = -90 (mV)\n"
  "	k_a = -14.7 (mV)\n"
  "	k_b = 7.5 (mV)\n"
  "	tau_a0 = 1 (ms)\n"
  "	tau_a1 = 1 (ms)\n"
  "	tau_b0 = 0 (ms)\n"
  "	tau_b1 = 200 (ms)\n"
  "	tht_a = -40 (mV)\n"
  "	tht_b1 = -60 (mV)\n"
  "	tht_b2 = -40 (mV)\n"
  "	sig_a = -0.5 (mV)\n"
  "	sig_b1 = -30 (mV)\n"
  "	sig_b2 = 10 (mV)\n"
  "\n"
  ":AHP current (Ca dependent K current)\n"
  "	gkcabar   = 1e-3 (S/cm2)\n"
  "	theta_r = 0.17 (mV)\n"
  "	k_r = -0.08 (mV)\n"
  "	tau_r = 2 (ms)\n"
  "\n"
  ":cai\n"
  "	acai=5.18e-3 (cm2/mA/ms)\n"
  "	bcai=2e-3 (1/ms)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v	(mV)\n"
  "	ina	(mA/cm2)\n"
  "	ikD	(mA/cm2)\n"
  "	ikA	(mA/cm2)\n"
  "	ikAHP(mA/cm2)\n"
  "	icaT(mA/cm2)\n"
  "	icaL (mA/cm2)\n"
  "	ilk	(mA/cm2)\n"
  "	idbs (mA/cm2)\n"
  "\n"
  "	periodo (ms)\n"
  "\n"
  ":Fast Na\n"
  "	h_inf\n"
  "	tau_h	(ms)\n"
  "	m_inf\n"
  "	tau_m	(ms)\n"
  "\n"
  ":Delayed rectifier\n"
  "	n_inf\n"
  "	tau_n	(ms)\n"
  "\n"
  ":ca T current\n"
  "	p_inf\n"
  "	q_inf\n"
  "	tau_p	(ms)\n"
  "	tau_q	(ms)\n"
  "	eca     (mV)   :calc from Nernst\n"
  "\n"
  ":ca L current\n"
  "	c_inf\n"
  "	tau_c	(ms)\n"
  "	d1_inf\n"
  "	tau_d1	(ms)\n"
  "	d2_inf\n"
  "\n"
  ":A current\n"
  "	a_inf\n"
  "	tau_a	(ms)\n"
  "	b_inf\n"
  "	tau_b	(ms)\n"
  "\n"
  ":AHP (Ca dependent K current)\n"
  "	r_inf\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	m h n\n"
  "	p q\n"
  "	c d1 d2\n"
  "	cai\n"
  "	a b r\n"
  "}\n"
  "\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE states METHOD cnexp\n"
  "\n"
  "	eca = con*log(cao/cai)\n"
  "\n"
  "	ina   = gnabar * m*m*m*h * (v - ena)\n"
  "	ikD   = gkdrbar * n^4 * (v - ek)\n"
  "\n"
  "	ikA   = gkabar * a*a*b * (v - ek)\n"
  "	ikAHP   = gkcabar *r*r* (v - ek)\n"
  "\n"
  "	icaT   = gcatbar * p*p*q * (v - eca)\n"
  "	icaL   = gcalbar * c*c*d1*d2 * (v - eca)\n"
  "\n"
  "	ilk = gl * (v - el)\n"
  "\n"
  "	if (t >= periodo + tezao){\n"
  "		periodo = periodo + tezao\n"
  "	}\n"
  "	if (t >= periodo && t <= periodo + 0.3) {\n"
  "        idbs = -0.3*dbs\n"
  "    } else{\n"
  "		idbs = 0\n"
  "    }\n"
  "}\n"
  "\n"
  "DERIVATIVE states {\n"
  "	evaluate_fct(v)\n"
  "	h' = (h_inf - h)/tau_h\n"
  "	m' = (m_inf - m)/tau_m\n"
  "	n' = (n_inf - n)/tau_n\n"
  "	p' = (p_inf - p)/tau_p\n"
  "	q' = (q_inf - q)/tau_q\n"
  "\n"
  "	c' = (c_inf - c)/tau_c\n"
  "	d1' = (d1_inf - d1)/tau_d1\n"
  "	d2' = (d2_inf - d2)/tau_d2\n"
  "\n"
  "	cai' =	-acai*(icaL+icaT)-bcai*cai\n"
  "\n"
  "	a' = (a_inf - a)/tau_a\n"
  "	b' = (b_inf - b)/tau_b\n"
  "\n"
  "	r' = (r_inf - r)/tau_r\n"
  "}\n"
  "\n"
  "UNITSOFF\n"
  "\n"
  "INITIAL {\n"
  "	evaluate_fct(v)\n"
  "	m = m_inf\n"
  "	h = h_inf\n"
  "	n = n_inf\n"
  "	p = p_inf\n"
  "	q = q_inf\n"
  "\n"
  "	c = c_inf\n"
  "	d1 = d1_inf\n"
  "	d2 = d2_inf\n"
  "\n"
  "	a = a_inf\n"
  "	b = b_inf\n"
  "\n"
  "	r = r_inf\n"
  "	cai= 0.005\n"
  "\n"
  "	periodo = 0\n"
  "}\n"
  "\n"
  "PROCEDURE evaluate_fct(v(mV)) {\n"
  ":Fast Na current\n"
  "	h_inf = 1/(1+exp((v-theta_h)/k_h))\n"
  "	m_inf = 1/(1+exp((v-theta_m)/k_m))\n"
  "	tau_h = tau_h0 + tau_h1/(exp(-(v-tht_h1)/sig_h1) + exp(-(v-tht_h2)/sig_h2))\n"
  "	tau_m = tau_m0 + tau_m1/(1+exp(-(v-tht_m)/sig_m))\n"
  "\n"
  ":Delayed rectifier K\n"
  "	n_inf = 1/(1+exp((v-theta_n)/k_n))\n"
  "	tau_n = tau_n0 + tau_n1/(exp(-(v-tht_n1)/sig_n1) + exp(-(v-tht_n2)/sig_n2))\n"
  "\n"
  ":Ca T current\n"
  "	p_inf = 1/(1+exp((v-theta_p)/k_p))\n"
  "	q_inf = 1/(1+exp((v-theta_q)/k_q))\n"
  "	tau_p = tau_p0 + tau_p1/(exp(-(v-tht_p1)/sig_p1) + exp(-(v-tht_p2)/sig_p2))\n"
  "	tau_q = tau_q0 + tau_q1/(exp(-(v-tht_q1)/sig_q1) + exp(-(v-tht_q2)/sig_q2))\n"
  "\n"
  ":Ca L current\n"
  "	c_inf = 1/(1+exp((v-theta_c)/k_c))\n"
  "	d1_inf = 1/(1+exp((v-theta_d1)/k_d1))\n"
  "	d2_inf = 1/(1+exp((v-theta_d2)/k_d2))\n"
  "	tau_c = tau_c0 + tau_c1/(exp(-(v-tht_c1)/sig_c1) + exp(-(v-tht_c2)/sig_c2))\n"
  "	tau_d1 = tau_d10 + tau_d11/(exp(-(v-tht_d11)/sig_d11) + exp(-(v-tht_d12)/sig_d12))\n"
  "\n"
  ":A current\n"
  "	a_inf = 1/(1+exp((v-theta_a)/k_a))\n"
  "	b_inf = 1/(1+exp((v-theta_b)/k_b))\n"
  "	tau_a = tau_a0 + tau_a1/(1+exp(-(v-tht_a)/sig_a))\n"
  "	tau_b = tau_b0 + tau_b1/(exp(-(v-tht_b1)/sig_b1) + exp(-(v-tht_b2)/sig_b2))\n"
  "\n"
  ":AHP current\n"
  "	r_inf = 1/(1+exp((v-theta_r)/k_r))\n"
  "}\n"
  "\n"
  "UNITSON\n"
  ;
#endif
