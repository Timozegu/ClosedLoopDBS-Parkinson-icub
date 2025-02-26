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
 
#define nrn_init _nrn_init__Str
#define _nrn_initial _nrn_initial__Str
#define nrn_cur _nrn_cur__Str
#define _nrn_current _nrn_current__Str
#define nrn_jacob _nrn_jacob__Str
#define nrn_state _nrn_state__Str
#define _net_receive _net_receive__Str 
#define evaluate_fct evaluate_fct__Str 
#define states states__Str 
 
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
#define em _p[2]
#define em_columnindex 2
#define gnabar _p[3]
#define gnabar_columnindex 3
#define gkdrbar _p[4]
#define gkdrbar_columnindex 4
#define gl _p[5]
#define gl_columnindex 5
#define el _p[6]
#define el_columnindex 6
#define gmbar _p[7]
#define gmbar_columnindex 7
#define ina _p[8]
#define ina_columnindex 8
#define irand _p[9]
#define irand_columnindex 9
#define ik _p[10]
#define ik_columnindex 10
#define ilk _p[11]
#define ilk_columnindex 11
#define im _p[12]
#define im_columnindex 12
#define alpha_h _p[13]
#define alpha_h_columnindex 13
#define beta_h _p[14]
#define beta_h_columnindex 14
#define alpha_m _p[15]
#define alpha_m_columnindex 15
#define beta_m _p[16]
#define beta_m_columnindex 16
#define alpha_n _p[17]
#define alpha_n_columnindex 17
#define beta_n _p[18]
#define beta_n_columnindex 18
#define alpha_p _p[19]
#define alpha_p_columnindex 19
#define beta_p _p[20]
#define beta_p_columnindex 20
#define m _p[21]
#define m_columnindex 21
#define h _p[22]
#define h_columnindex 22
#define n _p[23]
#define n_columnindex 23
#define p _p[24]
#define p_columnindex 24
#define cai _p[25]
#define cai_columnindex 25
#define cao _p[26]
#define cao_columnindex 26
#define nai _p[27]
#define nai_columnindex 27
#define nao _p[28]
#define nao_columnindex 28
#define ki _p[29]
#define ki_columnindex 29
#define ko _p[30]
#define ko_columnindex 30
#define Dm _p[31]
#define Dm_columnindex 31
#define Dh _p[32]
#define Dh_columnindex 32
#define Dn _p[33]
#define Dn_columnindex 33
#define Dp _p[34]
#define Dp_columnindex 34
#define Dcai _p[35]
#define Dcai_columnindex 35
#define Dcao _p[36]
#define Dcao_columnindex 36
#define Dnai _p[37]
#define Dnai_columnindex 37
#define Dnao _p[38]
#define Dnao_columnindex 38
#define Dki _p[39]
#define Dki_columnindex 39
#define Dko _p[40]
#define Dko_columnindex 40
#define v _p[41]
#define v_columnindex 41
#define _g _p[42]
#define _g_columnindex 42
 
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
 "setdata_Str", _hoc_setdata,
 "evaluate_fct_Str", _hoc_evaluate_fct,
 0, 0
};
 /* declare global and static user variables */
#define alpha_p1 alpha_p1_Str
 double alpha_p1 = 0.0003209;
#define alpha_n1 alpha_n1_Str
 double alpha_n1 = 0.032;
#define alpha_h1 alpha_h1_Str
 double alpha_h1 = 0.128;
#define alpha_m1 alpha_m1_Str
 double alpha_m1 = 0.32;
#define beta_p1 beta_p1_Str
 double beta_p1 = -0.0003209;
#define beta_n1 beta_n1_Str
 double beta_n1 = 0.5;
#define beta_h1 beta_h1_Str
 double beta_h1 = 4;
#define beta_m1 beta_m1_Str
 double beta_m1 = 0.28;
#define k_p k_p_Str
 double k_p = 9;
#define k_n k_n_Str
 double k_n = 5;
#define k_h k_h_Str
 double k_h = 18;
#define k_m k_m_Str
 double k_m = 4;
#define sig_p1 sig_p1_Str
 double sig_p1 = 9;
#define sig_n1 sig_n1_Str
 double sig_n1 = 40;
#define sig_h2 sig_h2_Str
 double sig_h2 = 5;
#define sig_m sig_m_Str
 double sig_m = 5;
#define thb_p thb_p_Str
 double thb_p = -30;
#define tht_p1 tht_p1_Str
 double tht_p1 = -30;
#define th_p th_p_Str
 double th_p = -30;
#define theta_p theta_p_Str
 double theta_p = 30;
#define tht_n1 tht_n1_Str
 double tht_n1 = 57;
#define th_n th_n_Str
 double th_n = -52;
#define theta_n theta_n_Str
 double theta_n = 52;
#define tht_h2 tht_h2_Str
 double tht_h2 = 27;
#define tht_m tht_m_Str
 double tht_m = -27;
#define th_mb th_mb_Str
 double th_mb = -27;
#define th_ma th_ma_Str
 double th_ma = -54;
#define theta_h theta_h_Str
 double theta_h = 50;
#define theta_m theta_m_Str
 double theta_m = 54;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "theta_m_Str", "mV",
 "theta_h_Str", "mV",
 "alpha_m1_Str", "ms/mV",
 "th_ma_Str", "mV",
 "k_m_Str", "mV",
 "k_h_Str", "mV",
 "beta_m1_Str", "ms/mV",
 "beta_h1_Str", "ms",
 "alpha_h1_Str", "ms/mV",
 "th_mb_Str", "mV",
 "tht_m_Str", "mV",
 "tht_h2_Str", "mV",
 "sig_m_Str", "mV",
 "sig_h2_Str", "mV",
 "theta_n_Str", "mV",
 "alpha_n1_Str", "ms/mV",
 "th_n_Str", "mV",
 "k_n_Str", "mV",
 "beta_n1_Str", "ms",
 "tht_n1_Str", "mV",
 "sig_n1_Str", "mV",
 "theta_p_Str", "mV",
 "alpha_p1_Str", "ms/mV",
 "th_p_Str", "mV",
 "k_p_Str", "mV",
 "tht_p1_Str", "mV",
 "beta_p1_Str", "ms/mV",
 "thb_p_Str", "mV",
 "sig_p1_Str", "mV",
 "ena_Str", "mV",
 "ek_Str", "mV",
 "em_Str", "mV",
 "gnabar_Str", "S/cm2",
 "gkdrbar_Str", "S/cm2",
 "gl_Str", "S/cm2",
 "el_Str", "mV",
 "gmbar_Str", "S/cm2",
 "cai_Str", "mM",
 "cao_Str", "mM",
 "nai_Str", "mM",
 "nao_Str", "mM",
 "ki_Str", "mM",
 "ko_Str", "mM",
 "ina_Str", "mA/cm2",
 "irand_Str", "mA/cm2",
 "ik_Str", "mA/cm2",
 "ilk_Str", "mA/cm2",
 "im_Str", "mA/cm2",
 "alpha_h_Str", "1/ms",
 "beta_h_Str", "1/ms",
 "alpha_m_Str", "1/ms",
 "beta_m_Str", "1/ms",
 "alpha_n_Str", "1/ms",
 "beta_n_Str", "1/ms",
 "alpha_p_Str", "1/ms",
 "beta_p_Str", "1/ms",
 0,0
};
 static double cao0 = 0;
 static double cai0 = 0;
 static double delta_t = 0.01;
 static double h0 = 0;
 static double ko0 = 0;
 static double ki0 = 0;
 static double m0 = 0;
 static double nao0 = 0;
 static double nai0 = 0;
 static double n0 = 0;
 static double p0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "theta_m_Str", &theta_m_Str,
 "theta_h_Str", &theta_h_Str,
 "alpha_m1_Str", &alpha_m1_Str,
 "th_ma_Str", &th_ma_Str,
 "k_m_Str", &k_m_Str,
 "k_h_Str", &k_h_Str,
 "beta_m1_Str", &beta_m1_Str,
 "beta_h1_Str", &beta_h1_Str,
 "alpha_h1_Str", &alpha_h1_Str,
 "th_mb_Str", &th_mb_Str,
 "tht_m_Str", &tht_m_Str,
 "tht_h2_Str", &tht_h2_Str,
 "sig_m_Str", &sig_m_Str,
 "sig_h2_Str", &sig_h2_Str,
 "theta_n_Str", &theta_n_Str,
 "alpha_n1_Str", &alpha_n1_Str,
 "th_n_Str", &th_n_Str,
 "k_n_Str", &k_n_Str,
 "beta_n1_Str", &beta_n1_Str,
 "tht_n1_Str", &tht_n1_Str,
 "sig_n1_Str", &sig_n1_Str,
 "theta_p_Str", &theta_p_Str,
 "alpha_p1_Str", &alpha_p1_Str,
 "th_p_Str", &th_p_Str,
 "k_p_Str", &k_p_Str,
 "tht_p1_Str", &tht_p1_Str,
 "beta_p1_Str", &beta_p1_Str,
 "thb_p_Str", &thb_p_Str,
 "sig_p1_Str", &sig_p1_Str,
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
"Str",
 "ena_Str",
 "ek_Str",
 "em_Str",
 "gnabar_Str",
 "gkdrbar_Str",
 "gl_Str",
 "el_Str",
 "gmbar_Str",
 0,
 "ina_Str",
 "irand_Str",
 "ik_Str",
 "ilk_Str",
 "im_Str",
 "alpha_h_Str",
 "beta_h_Str",
 "alpha_m_Str",
 "beta_m_Str",
 "alpha_n_Str",
 "beta_n_Str",
 "alpha_p_Str",
 "beta_p_Str",
 0,
 "m_Str",
 "h_Str",
 "n_Str",
 "p_Str",
 "cai_Str",
 "cao_Str",
 "nai_Str",
 "nao_Str",
 "ki_Str",
 "ko_Str",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
 	_p = nrn_prop_data_alloc(_mechtype, 43, _prop);
 	/*initialize range parameters*/
 	ena = 50;
 	ek = -100;
 	em = -100;
 	gnabar = 0.1;
 	gkdrbar = 0.08;
 	gl = 0.0001;
 	el = -67;
 	gmbar = 0.0026;
 	_prop->param = _p;
 	_prop->param_size = 43;
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 1, _prop);
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 "cai_Str", 1e-10,
 "cao_Str", 1e-10,
 "nai_Str", 1e-10,
 "nao_Str", 1e-10,
 "ki_Str", 1e-10,
 "ko_Str", 1e-10,
 0,0
};
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _Str_reg() {
	int _vectorized = 1;
  _initlists();
 	register_mech(_mechanism, nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init, hoc_nrnpointerindex, 1);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
 #if NMODL_TEXT
  hoc_reg_nmodl_text(_mechtype, nmodl_file_text);
  hoc_reg_nmodl_filename(_mechtype, nmodl_filename);
#endif
  hoc_register_prop_size(_mechtype, 43, 1);
  hoc_register_dparam_semantics(_mechtype, 0, "cvodeieq");
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 Str /mnt/c/Users/Timot/PycharmProjects/MarmosetBase/MarmosetModel-main/Str.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
 
#define FARADAY _nrnunit_FARADAY[_nrnunit_use_legacy_]
static double _nrnunit_FARADAY[2] = {0x1.78e555060882cp+16, 96485.3}; /* 96485.3321233100141 */
static int _reset;
static char *modelname = "All ion channels used in Str models";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
static int evaluate_fct(_threadargsprotocomma_ double);
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[4], _dlist1[4];
 static int states(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {int _reset = 0; {
   evaluate_fct ( _threadargscomma_ v ) ;
   Dm = alpha_m * ( 1.0 - m ) - beta_m * m ;
   Dh = alpha_h * ( 1.0 - h ) - beta_h * h ;
   Dn = alpha_n * ( 1.0 - n ) - beta_n * n ;
   Dp = alpha_p * ( 1.0 - p ) - beta_p * p ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) {
 evaluate_fct ( _threadargscomma_ v ) ;
 Dm = Dm  / (1. - dt*( ( alpha_m )*( ( ( - 1.0 ) ) ) - ( beta_m )*( 1.0 ) )) ;
 Dh = Dh  / (1. - dt*( ( alpha_h )*( ( ( - 1.0 ) ) ) - ( beta_h )*( 1.0 ) )) ;
 Dn = Dn  / (1. - dt*( ( alpha_n )*( ( ( - 1.0 ) ) ) - ( beta_n )*( 1.0 ) )) ;
 Dp = Dp  / (1. - dt*( ( alpha_p )*( ( ( - 1.0 ) ) ) - ( beta_p )*( 1.0 ) )) ;
  return 0;
}
 /*END CVODE*/
 static int states (double* _p, Datum* _ppvar, Datum* _thread, NrnThread* _nt) { {
   evaluate_fct ( _threadargscomma_ v ) ;
    m = m + (1. - exp(dt*(( alpha_m )*( ( ( - 1.0 ) ) ) - ( beta_m )*( 1.0 ))))*(- ( ( alpha_m )*( ( 1.0 ) ) ) / ( ( alpha_m )*( ( ( - 1.0 ) ) ) - ( beta_m )*( 1.0 ) ) - m) ;
    h = h + (1. - exp(dt*(( alpha_h )*( ( ( - 1.0 ) ) ) - ( beta_h )*( 1.0 ))))*(- ( ( alpha_h )*( ( 1.0 ) ) ) / ( ( alpha_h )*( ( ( - 1.0 ) ) ) - ( beta_h )*( 1.0 ) ) - h) ;
    n = n + (1. - exp(dt*(( alpha_n )*( ( ( - 1.0 ) ) ) - ( beta_n )*( 1.0 ))))*(- ( ( alpha_n )*( ( 1.0 ) ) ) / ( ( alpha_n )*( ( ( - 1.0 ) ) ) - ( beta_n )*( 1.0 ) ) - n) ;
    p = p + (1. - exp(dt*(( alpha_p )*( ( ( - 1.0 ) ) ) - ( beta_p )*( 1.0 ))))*(- ( ( alpha_p )*( ( 1.0 ) ) ) / ( ( alpha_p )*( ( ( - 1.0 ) ) ) - ( beta_p )*( 1.0 ) ) - p) ;
   }
  return 0;
}
 
static int  evaluate_fct ( _threadargsprotocomma_ double _lv ) {
   alpha_m = alpha_m1 * ( _lv - th_ma ) / ( 1.0 - exp ( ( - _lv - theta_m ) / k_m ) ) ;
   beta_m = beta_m1 * ( _lv - th_mb ) / ( - 1.0 + exp ( ( _lv - tht_m ) / sig_m ) ) ;
   alpha_h = alpha_h1 * exp ( ( - _lv - theta_h ) / k_h ) ;
   beta_h = beta_h1 / ( 1.0 + exp ( ( - _lv - tht_h2 ) / sig_h2 ) ) ;
   alpha_n = alpha_n1 * ( _lv - th_n ) / ( 1.0 - exp ( ( - _lv - theta_n ) / k_n ) ) ;
   beta_n = beta_n1 * ( exp ( ( - _lv - tht_n1 ) / sig_n1 ) ) ;
   alpha_p = alpha_p1 * ( _lv - th_p ) / ( 1.0 - exp ( ( - _lv - theta_p ) / k_p ) ) ;
   beta_p = beta_p1 * ( _lv - thb_p ) / ( 1.0 - exp ( ( _lv - tht_p1 ) / sig_p1 ) ) ;
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
 
static int _ode_count(int _type){ return 4;}
 
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
	for (_i=0; _i < 4; ++_i) {
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
  cao = cao0;
  cai = cai0;
  h = h0;
  ko = ko0;
  ki = ki0;
  m = m0;
  nao = nao0;
  nai = nai0;
  n = n0;
  p = p0;
 {
   evaluate_fct ( _threadargscomma_ v ) ;
   m = alpha_m / ( alpha_m + beta_m ) ;
   h = alpha_h / ( alpha_h + beta_h ) ;
   n = alpha_n / ( alpha_n + beta_n ) ;
   p = alpha_p / ( alpha_p + beta_p ) ;
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
   ilk = gl * ( v - el ) ;
   ina = gnabar * m * m * m * h * ( v - ena ) ;
   ik = gkdrbar * pow( n , 4.0 ) * ( v - ek ) ;
   im = gmbar * p * ( v - em ) ;
   irand = 0.0 * 2e-3 * sin ( t / 20.0 ) ;
   }
 _current += ilk;
 _current += im;
 _current += ik;
 _current += ina;
 _current += irand;

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
 _slist1[0] = m_columnindex;  _dlist1[0] = Dm_columnindex;
 _slist1[1] = h_columnindex;  _dlist1[1] = Dh_columnindex;
 _slist1[2] = n_columnindex;  _dlist1[2] = Dn_columnindex;
 _slist1[3] = p_columnindex;  _dlist1[3] = Dp_columnindex;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif

#if NMODL_TEXT
static const char* nmodl_filename = "/mnt/c/Users/Timot/PycharmProjects/MarmosetBase/MarmosetModel-main/Str.mod";
static const char* nmodl_file_text = 
  "TITLE  All ion channels used in Str models\n"
  "\n"
  "\n"
  "\n"
  "NEURON {\n"
  "	SUFFIX Str\n"
  "	NONSPECIFIC_CURRENT ilk,im,ik,ina, irand \n"
  "	RANGE gnabar, ena, alpha_m, alpha_h, beta_h, beta_m        : fast sodium\n"
  "	RANGE gkdrbar, ek, alpha_n, beta_n                 		   : delayed K rectifier\n"
  "	RANGE gl, el                                  	           : leak\n"
  "	RANGE gmbar,em, alpha_p, beta_p							   : m current(outward potassium current)\n"
  "}\n"
  "\n"
  "UNITS {\n"
  "	(mA) = (milliamp)\n"
  "	(mV) = (millivolt)\n"
  "	(S)  = (siemens)\n"
  "	(molar) = (1/liter)\n"
  "	(mM)	= (millimolar)\n"
  "	FARADAY = (faraday) (coulomb)  :units are really coulombs/mole\n"
  "}\n"
  "\n"
  "PARAMETER {\n"
  "	ena = 50	(mV)\n"
  "	ek = -100	(mV)\n"
  "	em = -100 	(mV)	\n"
  "\n"
  ":Fast Na channel\n"
  "	gnabar   = 100e-3 (S/cm2) \n"
  "	theta_m = 54 (mV)\n"
  "	theta_h = 50 (mV) \n"
  "	alpha_m1=0.32	(ms/mV)\n"
  "	th_ma=-54		(mV)\n"
  "	k_m = 4 	(mV)    \n"
  "	k_h = 18 	(mV)   \n"
  "	beta_m1 = 0.28 (ms/mV)\n"
  "	beta_h1 = 4 (ms) \n"
  "	alpha_h1=0.128	(ms/mV)\n"
  "\n"
  "	th_mb=-27	(mV)\n"
  "	tht_m = -27 (mV)\n"
  "	tht_h2 = 27 (mV)\n"
  "	sig_m = 5 (mV)\n"
  "	sig_h2 = 5 (mV)\n"
  "\n"
  ": delayed K rectifier \n"
  "	gkdrbar  = 80e-3	(S/cm2)  \n"
  "	theta_n = 52 (mV)\n"
  "	alpha_n1=0.032	(ms/mV)\n"
  "	th_n=-52		(mV)\n"
  "	k_n = 5 (mV)     \n"
  "	beta_n1 = 0.5 (ms) \n"
  "	tht_n1 = 57 (mV)\n"
  "	sig_n1 = 40 (mV)\n"
  "\n"
  ":Leakage current\n"
  "	gl	= 0.1e-3	(S/cm2)\n"
  "	el	= -67	(mV)\n"
  "\n"
  ":m current(outward potassium current)\n"
  "	gmbar   = 2.6e-3 (S/cm2)  :Healthy=2.6 ,PD=1.5\n"
  "	theta_p = 30 (mV)\n"
  "	alpha_p1=3.209e-4	(ms/mV)\n"
  "	th_p=-30	(mV)\n"
  "	k_p = 9 (mV)    \n"
  "	tht_p1 = -30 (mV)\n"
  "	beta_p1=-3.209e-4	(ms/mV)\n"
  "	thb_p=-30	(mV)\n"
  "	sig_p1 = 9 (mV)\n"
  "}\n"
  "\n"
  "ASSIGNED {\n"
  "	v	(mV)\n"
  "	ina	(mA/cm2)\n"
  "	irand (mA/cm2)\n"
  "	ik	(mA/cm2)   \n"
  "	ilk	(mA/cm2)\n"
  "	im	(mA/cm2)\n"
  "\n"
  ":Fast Na\n"
  "	alpha_h	(1/ms)\n"
  "	beta_h	(1/ms)\n"
  "	alpha_m (1/ms)\n"
  "	beta_m	(1/ms)\n"
  "\n"
  ":K rectifier\n"
  "	alpha_n	(1/ms)\n"
  "	beta_n	(1/ms)\n"
  "\n"
  ":m current(outward potassium current)\n"
  "	alpha_p	(1/ms)\n"
  "	beta_p	(1/ms)\n"
  "}\n"
  "\n"
  "STATE {\n"
  "	m h n p \n"
  "	cai (mM) <1e-10> : Mas que droga eh essa aqui?\n"
  "	cao (mM) <1e-10>\n"
  "	nai (mM) <1e-10>\n"
  "	nao (mM) <1e-10>\n"
  "	ki (mM) <1e-10>\n"
  "	ko (mM) <1e-10>\n"
  "}\n"
  "\n"
  "BREAKPOINT {\n"
  "	SOLVE states METHOD cnexp\n"
  "\n"
  "	ilk = gl * (v - el)					: leak\n"
  "	ina = gnabar * m*m*m*h * (v - ena)	: fast sodium\n"
  "	ik = gkdrbar * n^4 * (v - ek)		: delayed K rectifier\n"
  "	im =  gmbar* p * (v - em)			: m current(outward potassium current)\n"
  "	irand = 0*2e-3*sin(t/20)\n"
  "}\n"
  "\n"
  "DERIVATIVE states {   \n"
  "	evaluate_fct(v)\n"
  "	m' = alpha_m*(1-m)-beta_m*m\n"
  "	h' = alpha_h*(1-h)-beta_h*h\n"
  "	n' = alpha_n*(1-n)-beta_n*n\n"
  "	p' = alpha_p*(1-p)-beta_p*p\n"
  "}\n"
  "\n"
  "UNITSOFF\n"
  "\n"
  "INITIAL {\n"
  "	evaluate_fct(v)\n"
  "	m = alpha_m/(alpha_m+beta_m) \n"
  "	h = alpha_h/(alpha_h+beta_h) \n"
  "	n = alpha_n/(alpha_n+beta_n)  \n"
  "	p = alpha_p/(alpha_p+beta_p) \n"
  "}\n"
  "\n"
  "PROCEDURE evaluate_fct(v(mV)) { \n"
  "	\n"
  "	alpha_m = alpha_m1*(v-th_ma)/(1-exp((-v-theta_m)/k_m))\n"
  "	beta_m = beta_m1*(v-th_mb)/(-1+exp((v-tht_m)/sig_m)) \n"
  "\n"
  "	alpha_h = alpha_h1*exp((-v-theta_h)/k_h)\n"
  "	beta_h = beta_h1/(1 + exp((-v-tht_h2)/sig_h2)) \n"
  "\n"
  "	alpha_n = alpha_n1*(v-th_n)/(1-exp((-v-theta_n)/k_n))\n"
  "	beta_n = beta_n1*(exp((-v-tht_n1)/sig_n1))\n"
  "\n"
  "	alpha_p = alpha_p1*(v-th_p)/(1-exp((-v-theta_p)/k_p))\n"
  "	beta_p = beta_p1*(v-thb_p)/(1-exp((v-tht_p1)/sig_p1)) \n"
  "}\n"
  "\n"
  "UNITSON\n"
  ;
#endif
