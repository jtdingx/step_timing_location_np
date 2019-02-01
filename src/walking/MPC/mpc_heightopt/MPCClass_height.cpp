/*****************************************************************************
MPCClass_height.cpp

Description:    source file of MPCClass

@Version:   1.0
@Author:    Chengxu Zhou (zhouchengxu@gmail.com)
@Release:   Thu 02 Aug 2018 12:33:23 PM CEST
@Update:    Thu 02 Aug 2018 12:33:19 PM CEST
*****************************************************************************/
#include "MPC/MPCClass_height.h"
// #include "MPC/spline.h"
#include <cstdio>
#include <cstdlib>

#include <iostream>
#include <Eigen/Dense>
#include <math.h>
#include <fstream>
#include <time.h>
#include <vector>


using namespace Eigen;
using namespace std;


MPCClass::MPCClass()                    ///declaration function
	: QPBaseClass()
	, _robot_name("")
	, _robot_mass(0.0)
	, _lift_height(0.0)
	, _method_flag(0)
	,_n_end_walking(1)
	,_j_period(0)
{
  
}

////////////////step parameters input============================================================
void MPCClass::FootStepInputs(double stepwidth, double steplength, double stepheight)
{	
	_steplength.setConstant(steplength);
	_steplength(0) = 0;
	
	_stepwidth.setConstant(stepwidth);
	_stepwidth(0) = _stepwidth(0)/2;
	
	_stepheight.setConstant(stepheight);     
        _steplength(_footstepsnumber-1) = 0;
        _steplength(_footstepsnumber-2) = 0;
        _steplength(_footstepsnumber-3) = 0;
        _steplength(_footstepsnumber-4) = 0;	
        _steplength(_footstepsnumber-5) = 0;

	_lift_height_ref.setConstant(_lift_height);
        _lift_height_ref(_footstepsnumber-1) = 0;
        _lift_height_ref(_footstepsnumber-2) = 0;	
        _lift_height_ref(_footstepsnumber-3) = 0; 
        _lift_height_ref(_footstepsnumber-4) = 0; 	
}

/////////////////////// initialize all the variables============================================================
void MPCClass::Initialize()
{
 	// ==step loctions setup==
/*        _footx_ref = Eigen::VectorXd::Zero(_footstepsnumber);
	_footy_ref = Eigen::VectorXd::Zero(_footstepsnumber);
	_footz_ref = Eigen::VectorXd::Zero(_footstepsnumber);*/	
        _footx_ref.setZero();
	_footy_ref.setZero();
	_footz_ref.setZero();	
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
	  _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);   
	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
	}	
        
//       sampling time & step cycle	
//        _ts = Eigen::VectorXd::Zero(_footstepsnumber); //ssp time
	_ts.setConstant(_tstep);
	_td = 0.2*_ts;                                // dsp time
	
// 	_tx = Eigen::VectorXd::Zero(_footstepsnumber);   // start time for each step cycle
	_tx.setZero();   // start time for each step cycle	
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	  _tx(i) = round(_tx(i)/_dt)*_dt -0.00001;	  
	}	
	      
	_t.setLinSpaced(_nsum,_dt,_tx(_footstepsnumber-1));  ///sampling time sequence for entire step period
// 	_nsum = _t.size();                                                                /// number of whole control loop
// 	_nT = round(_ts(0)/_dt);                                                          /// the number of one step cycle		
        // ==initial parameters & matrix for MPC==
        _hcom = RobotParaClass::Z_C();                                                    /// intial comz
	_Hcom.setConstant(_hcom);                             /// for comz reference during the predictive window: matrix operations: height variance between support feet and body center	
	_ggg.setConstant(RobotParaClass::G());                                            //// acceleration of gravity: 1*1 matrix: matrix operations

	
// 	_nh = round(RobotParaClass::PreviewT()/_dt);	                      	          //// number of sampling time predictive window: <= 2*_nT;
// 	_Nt = 5*_nh + 3*_nstep;       /// the number of the variable of the optimization problem
	
//////////////==============================state variable============================================
// 	_zmpx_real = Eigen::VectorXd::Zero(_nsum); _zmpy_real = Eigen::VectorXd::Zero(_nsum);                              
// 	_comx = Eigen::VectorXd::Zero(_nsum); _comvx = Eigen::VectorXd::Zero(_nsum); _comax = Eigen::VectorXd::Zero(_nsum);
// 	_comy = Eigen::VectorXd::Zero(_nsum); _comvy = Eigen::VectorXd::Zero(_nsum); _comay = Eigen::VectorXd::Zero(_nsum);
// 	_comz = Eigen::VectorXd::Zero(_nsum); _comvz = Eigen::VectorXd::Zero(_nsum); _comaz = Eigen::VectorXd::Zero(_nsum);	
// 	_thetax = Eigen::VectorXd::Zero(_nsum); _thetavx = Eigen::VectorXd::Zero(_nsum); _thetaax = Eigen::VectorXd::Zero(_nsum);
// 	_thetay = Eigen::VectorXd::Zero(_nsum); _thetavy = Eigen::VectorXd::Zero(_nsum); _thetaay = Eigen::VectorXd::Zero(_nsum);
// 	_thetaz = Eigen::VectorXd::Zero(_nsum); _thetavz = Eigen::VectorXd::Zero(_nsum); _thetaaz = Eigen::VectorXd::Zero(_nsum);	
// 	_torquex_real = Eigen::VectorXd::Zero(_nsum); _torquey_real = Eigen::VectorXd::Zero(_nsum);  
	_zmpx_real.setZero(); _zmpy_real.setZero();                              
	_comx.setZero(); _comvx.setZero(); _comax.setZero();
	_comy.setZero(); _comvy.setZero(); _comay.setZero();
	_comz.setZero(); _comvz.setZero(); _comaz.setZero();	
	_thetax.setZero(); _thetavx.setZero(); _thetaax.setZero();
	_thetay.setZero(); _thetavy.setZero(); _thetaay.setZero();
	_thetaz.setZero(); _thetavz.setZero(); _thetaaz.setZero();	
	_torquex_real.setZero(); _torquey_real.setZero();  
          //// optimized CoM postion, foot trajectory, torse angle, footstep location;
// 	_CoM_position_optimal = Eigen::MatrixXd::Zero(3,_nsum);
// 	_torso_angle_optimal = Eigen::MatrixXd::Zero(3,_nsum);
// 	_L_foot_optition_optimal = Eigen::MatrixXd::Zero(3,_nsum);
// 	_R_foot_optition_optimal = Eigen::MatrixXd::Zero(3,_nsum);
// 	_foot_location_optimal = Eigen::MatrixXd::Zero(3,_footstepsnumber);
	_CoM_position_optimal.setZero();
	_torso_angle_optimal.setZero();
	_L_foot_optition_optimal.setZero();
	_R_foot_optition_optimal.setZero();
	_foot_location_optimal.setZero();
	
	//// state variable for mpc
// 	_xk = Eigen::MatrixXd::Zero(3,_nsum); 
// 	_yk = Eigen::MatrixXd::Zero(3,_nsum); 
// 	_zk = Eigen::MatrixXd::Zero(3,_nsum);
// 	_thetaxk = Eigen::MatrixXd::Zero(3,_nsum); 
// 	_thetayk = Eigen::MatrixXd::Zero(3,_nsum);
// 	_x_vacc_k = Eigen::VectorXd::Zero(_nsum); 
// 	_y_vacc_k = Eigen::VectorXd::Zero(_nsum);
// 	_z_vacc_k = Eigen::VectorXd::Zero(_nsum); 
// 	_thetax_vacc_k = Eigen::VectorXd::Zero(_nsum); 
// 	_thetay_vacc_k = Eigen::VectorXd::Zero(_nsum); 
	_xk.setZero(); 
	_yk.setZero(); 
	_zk.setZero();
	_thetaxk.setZero(); 
	_thetayk.setZero();
	_x_vacc_k.setZero(); 
	_y_vacc_k.setZero();
	_z_vacc_k.setZero(); 
	_thetax_vacc_k.setZero(); 
	_thetay_vacc_k.setZero(); 	
///////================================================================================================			
	_a << 1, _dt, pow(_dt,2)/2,    
	      0,   1,            _dt,
	      0,   0,              1;
	_b << pow(_dt,3)/6,    
	      pow(_dt,2)/2,
	               _dt;		
	
	_cp.setZero();
	_cp(0,0) = 1;
	_cv.setZero();
	_cv(0,1) = 1;
	_ca.setZero();
	_ca(0,2) = 1;		
		
	//predictive model matrixs: just calculated once
// 	_pps = Eigen::MatrixXd::Zero(_nh,3); _ppu = Eigen::MatrixXd::Zero(_nh,_nh);
// 	_pvs = Eigen::MatrixXd::Zero(_nh,3); _pvu = Eigen::MatrixXd::Zero(_nh,_nh);
// 	_pas = Eigen::MatrixXd::Zero(_nh,3); _pau = Eigen::MatrixXd::Zero(_nh,_nh);
	_pps.setZero(); _ppu.setZero();
	_pvs.setZero(); _pvu.setZero();
	_pas.setZero(); _pau.setZero();
	
        _pps = Matrix_ps(_a,_nh,_cp);
	_pvs = Matrix_ps(_a,_nh,_cv);
	_pas = Matrix_ps(_a,_nh,_ca);
	_ppu = Matrix_pu(_a,_b,_nh,_cp);
	_pvu = Matrix_pu(_a,_b,_nh,_cv);
	_pau = Matrix_pu(_a,_b,_nh,_ca);
	
	_pvu_2 = _pvu.transpose()*_pvu;
	_ppu_2 = _ppu.transpose()*_ppu;
	
	
	xyz1 = 0;  //flag for Indexfind function: 
	xyz2 = 1;
	_j_period = 0; // the number for step cycle indefind
	
        //footz refer:
	_Zsc.setZero();		
  	for (int i = 0; i < _nsum-1; i++) {	  
          Indexfind(_t(i),xyz1);	  
	  _Zsc(i) = _footz_ref(_j_period);   
	  _j_period = 0; 
	}		

        _yk.topRows(1).setConstant(_footy_ref(0)); 
	_zk.topRows(1).setConstant(_hcom);
		
	/// for footstep location reference generation: the foot-ref = _v_i*fx + _VV_i*_Lx_ref
// 	_v_i = Eigen::VectorXd::Zero(_nh);                    ///current step cycle
// 	_VV_i = Eigen::MatrixXd::Zero(_nh, _nstep);	      /// next step cycles: 2 cycles maximal	
	_v_i.setZero();                    ///current step cycle
	_VV_i.setZero();	      /// next step cycles: 2 cycles maximal
	
	// optimized footstep location
	_footx_real.setZero();  
	_footy_real.setZero(); 
	_footz_real.setZero();	
	_footxyz_real.setZero();	
	_footx_real_next.setZero(); 
	_footy_real_next.setZero(); 
	_footz_real_next.setZero();
	_footx_real_next1.setZero();  
	_footy_real_next1.setZero(); 
	_footz_real_next1.setZero();
	
	/// for foot trajectory generation
//         _Lfootx = Eigen::VectorXd::Zero(_nsum);
//         _Lfooty = Eigen::VectorXd::Zero(_nsum); _Lfooty.setConstant(_stepwidth(0)); _Lfootz = Eigen::VectorXd::Zero(_nsum); 
// 	_Lfootvx = Eigen::VectorXd::Zero(_nsum); _Lfootvy = Eigen::VectorXd::Zero(_nsum);_Lfootvz = Eigen::VectorXd::Zero(_nsum); 
// 	_Lfootax = Eigen::VectorXd::Zero(_nsum); _Lfootay = Eigen::VectorXd::Zero(_nsum);_Lfootaz = Eigen::VectorXd::Zero(_nsum);
// 	_Rfootx = Eigen::VectorXd::Zero(_nsum); 
//         _Rfooty = Eigen::VectorXd::Zero(_nsum); _Rfooty.setConstant(-_stepwidth(0));_Rfootz = Eigen::VectorXd::Zero(_nsum); 
// 	_Rfootvx = Eigen::VectorXd::Zero(_nsum); _Rfootvy = Eigen::VectorXd::Zero(_nsum);_Rfootvz = Eigen::VectorXd::Zero(_nsum); 
// 	_Rfootax = Eigen::VectorXd::Zero(_nsum); _Rfootay = Eigen::VectorXd::Zero(_nsum);_Rfootaz = Eigen::VectorXd::Zero(_nsum);
        _Lfootx.setZero();
        _Lfooty.setZero(); _Lfooty.setConstant(_stepwidth(0)); _Lfootz.setZero(); 
	_Lfootvx.setZero(); _Lfootvy.setZero();_Lfootvz.setZero(); 
	_Lfootax.setZero(); _Lfootay.setZero();_Lfootaz.setZero();
	_Rfootx.setZero(); 
        _Rfooty.setZero(); _Rfooty.setConstant(-_stepwidth(0));_Rfootz.setZero(); 
	_Rfootvx.setZero(); _Rfootvy.setZero();_Rfootvz.setZero(); 
	_Rfootax.setZero(); _Rfootay.setZero();_Rfootaz.setZero();	
	_ry_left_right = 0;
/////=========================================constraints initialize========================================
	_ZMP_ratio = 0.8;	
	  //vertical height constraints	
	_z_max=0.1;
	_z_min=-0.1;	
	
	if(_robot_name == "coman"){
	  _rad = 0.1; 	  
	  _footx_max=0.3;
	  _footx_min=-0.2;	  	  
	  /// zmp-constraints	
	  _zmpx_ub=0.07;  
	  _zmpx_lb=-0.03;
	  _zmpy_ub=0.05; 
	  _zmpy_lb=-0.05;		  	  
	}
	else if (_robot_name == "bigman")
	{
	  _rad = 0.2; 	  
	  _footx_max=0.5;
	  _footx_min=-0.2;	  	  
	  /// zmp-constraints	
	  _zmpx_ub=(RobotParaClass::FOOT_LENGTH()/2+RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio;  
	  _zmpx_lb=(-(RobotParaClass::FOOT_LENGTH()/2-RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio);
	  _zmpy_ub=(RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio); 
	  _zmpy_lb=(-RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio);		
	}
	else if (_robot_name == "cogimon")
        {	  
	  _rad = 0.2; 	  
	  _footx_max=0.4;
	  _footx_min=-0.2;	  	  
	  /// zmp-constraints	
	  _zmpx_ub=(RobotParaClass::FOOT_LENGTH()/2+RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio;  
	  _zmpx_lb=(-(RobotParaClass::FOOT_LENGTH()/2-RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio);
	  _zmpy_ub=(RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio); 
	  _zmpy_lb=(-RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio);	
        } 
        else {
	  DPRINTF("Errorrrrrrrr for IK\n");}		
	  
	_mass = _robot_mass; 		
	_j_ini = _mass* pow(_rad,2);		

	
	_footy_max=2*RobotParaClass::HALF_HIP_WIDTH() + 0.2; 
	_footy_min=RobotParaClass::HALF_HIP_WIDTH() - 0.03;
	
	// angle range
	_thetax_max=10*M_PI/180;  
	_thetax_min=-5*M_PI/180;
	_thetay_max=10*M_PI/180;  
	_thetay_min=-10*M_PI/180;
	
	// torque range
	_torquex_max=80/_j_ini; 
	_torquex_min=-60/_j_ini;
	_torquey_max=80/_j_ini;  
	_torquey_min=-80/_j_ini;	

	// swing foot velocity constraints	
	_footx_vmax=3;
	_footx_vmin=-2;
	_footy_vmax=2; 
	_footy_vmin=-2;	

	_fx= 0;    //current step location in local coordinate
	_fy= 0;
	_fxx_global= 0; //global footstep location in local coordinate
	_fyy_global= 0;		

///===========initiallize: preparation for MPC solution	
	// sulotion preparation	
// 	_V_ini = Eigen::VectorXd::Zero(_Nt);                                /// initialize optimal variable
	_V_ini.setZero();                                /// initialize optimal variable
        _V_inix.setZero();	
// 	_V_optimal = Eigen::MatrixXd::Zero(_Nt, _nsum);	      ///optimization det_V
	_V_optimal.setZero();	      ///optimization det_V
	
/*	_flag.setZero(_nsum);	        // 	 store n_vis: actual following step numbers under predictive window
	_flag_global.setZero(_nsum);    // store step cycle sequence*/	
	_flag.setZero();	        // 	 store n_vis: actual following step numbers under predictive window
	_flag_global.setZero();    // store step cycle sequence	
	
	_Lx_ref.setZero();             ///reference following footstep locations during the predictive window
	_Ly_ref.setZero(); 
	_Lz_ref.setZero();                    
	_comx_center_ref.setZero();
	_comy_center_ref.setZero();
	_comz_center_ref.setZero();	
	_thetax_center_ref.setZero(); 
	_thetay_center_ref.setZero();	
		  
	if(_robot_name == "coman"){
         //////// for methx ==2:reactive step + body inclination + height variance: for flat ground walking and up-down stairs: offline
	 _Rx = 1;           _Ry = 1;            _Rz =1;                     //com acceleration
	_alphax = 1;       _alphay = 1;        _alphaz = 100;               //com velocity
	_beltax = 5000;   _beltay = 10;        _beltaz = 20000000;          //com position
	_gamax =  10000000; _gamay = 10000000;  _gamaz = 200;               //footstep location
	_Rthetax = 1; _Rthetay = 1;                                         // theta acceleration
	_alphathetax =1; _alphathetay = 1;                                  // theta velocity
	_beltathetax = 10; _beltathetay = 10;	                            // theta postion
/*// // 	push recovery_test2:x:2:3,y:2:3: thetax:1:2,thetay:1:2  model2
// 	 _Rx = 10;           _Ry = 1;            _Rz =10;
// 	_alphax = 1000;       _alphay = 10;        _alphaz = 1000; 
// 	_beltax = 200000;     _beltay = 100000;     _beltaz = 20000000;
// 	_gamax =  10000000;    _gamay = 100000;  _gamaz = 200;
// 	_Rthetax = 10; _Rthetay = 10;
// 	_alphathetax =10000; _alphathetay = 1000;
// 	_beltathetax = 100000000; _beltathetay = 1000000;*/	
	}
	else if(_robot_name  == "bigman"){
         //////// for methx ==2:reactive step + body inclination + height variance: for flat ground walking and up-down stairs: offline
	 _Rx = 1;           _Ry = 1;            _Rz =1;
	_alphax = 1;       _alphay = 10;        _alphaz = 100; 
	_beltax = 100;   _beltay = 1000;        _beltaz = 20000000;
	_gamax =  50000000; _gamay = 1000000000;  _gamaz = 200;
	_Rthetax = 1; _Rthetay = 1;
	_alphathetax =1; _alphathetay = 1;
	_beltathetax = 1000; _beltathetay = 1000;
	}
	else if (_robot_name == "cogimon"){
	  //////// for methx ==2:reactive step + body inclination + height variance: for flat ground walking and up-down stairs: offline
	  _Rx = 1;           _Ry = 1;            _Rz =1;
	  _alphax = 1;       _alphay = 1;        _alphaz = 100; 
	  _beltax = 100;   _beltay = 100;        _beltaz = 20000000;
	  _gamax =  50000000; _gamay = 100000000;  _gamaz = 200;
	  _Rthetax = 1; _Rthetay = 1;
	  _alphathetax =1; _alphathetay = 1;
	  _beltathetax = 1000; _beltathetay = 1000;
        } 
	else
	{DPRINTF("Errorrrrrrrr for IK\n");}
	// time cost consumption======================mark
	_tcpu.setZero(_nsum);
	_tcpu_iterative.setZero(_nsum);
	_tcpu_prepara.setZero(_nsum);
	_tcpu_prepara2.setZero(_nsum);
	_tcpu_qp.setZero(_nsum);
	///////////////////=====================================//////////////////////////
	/// for offline calculation	
	_loop = 2;   /// loop number for SQP
///// next code just run once	
	A_unit.setIdentity(_nh,_nh);
	C_unit.setIdentity(_nstep,_nstep);		
	
  /////////// initialize each variable
        _bjxx_1 = 0; 
	_bjxx = 0; 
	_t_f.setZero();     ///predictive window time-period
	_bjx1 = 0;
	_bjx2 = 0;
        _mx = 0;
        _tnx.setZero();	  
	  
	_n_vis =0; 
	xxx = 0; 
	xxx1=0; 
	xxx2=0;	 
	
	// optimization objective function 	
	_WX.setZero();
	_WY.setZero();
	_WZ.setZero();
	_WthetaX.setZero();
	_WthetaY.setZero();
	_PHIX.setZero();
	_PHIY.setZero();
	_PHIZ.setZero();
	_Q_goal.setZero();
	_q_goal.setZero();
	_Q_goal1.setZero();
	_q_goal1.setZero();	
	
	_WX = _Rx*0.5 * A_unit + _alphax*0.5 * _pvu_2 + _beltax*0.5 * _ppu_2;	  
	_WY = _Ry/2 * A_unit + _alphay/2 * _pvu_2 + _beltay/2 * _ppu_2;
	_WZ = _Rz/2 * A_unit + _alphaz/2 * _pvu_2 + _beltaz/2 * _ppu_2;  
	_WthetaX = _Rthetax/2 * A_unit + _alphathetax/2 * _pvu_2 + _beltathetax/2 * _ppu_2;
	_WthetaY = _Rthetay/2 * A_unit + _alphathetay/2 * _pvu_2 + _beltathetay/2 * _ppu_2;
	_PHIX  = _gamax/2 * C_unit;
	_PHIY  = _gamay/2 * C_unit;
	_PHIZ  = _gamaz/2 * C_unit;
		
	_Q_goal.block<_nh, _nh>(0, 0) = _WX;
	_Q_goal.block<_nh, _nh>(_nh, _nh) = _WY;
	_Q_goal.block<_nh, _nh>(2*_nh, 2*_nh) = _WZ;
	_Q_goal.block<_nh, _nh>(3*_nh, 3*_nh) = _WthetaX;
	_Q_goal.block<_nh, _nh>(4*_nh, 4*_nh) = _WthetaY;
	_Q_goal.block<_nstep,_nstep>(5*_nh, 5*_nh) = _PHIX;
	_Q_goal.block<_nstep,_nstep>(5*_nh+_nstep, 5*_nh+_nstep) = _PHIY;
	_Q_goal.block<_nstep,_nstep>(5*_nh+2*_nstep, 5*_nh+2*_nstep) = _PHIZ;	  
      
	_Q_goal1 = 2 * _Q_goal;	
      
      
	// constraints
	_Sjx.setZero();
	_Sjy.setZero();
	_Sjz.setZero();
	_Sjthetax.setZero();
	_Sjthetay.setZero();
	_Sjx.block<_nh, _nh>(0, 0) = A_unit;
	_Sjy.block<_nh, _nh>(0, _nh) = A_unit;
	_Sjz.block<_nh, _nh>(0, 2*_nh) = A_unit;	
	_Sjthetax.block<_nh, _nh>(0, 3*_nh) = A_unit;
	_Sjthetay.block<_nh, _nh>(0, 4*_nh) = A_unit;
	
	_Sfx.setZero();
	_Sfy.setZero();
	_Sfz.setZero();	 	
	

	// ZMP boundary preparation
	_H_q_upx.setZero();
	_F_zmp_upx.setZero();
	_H_q_lowx.setZero();
	_F_zmp_lowx.setZero();
	_H_q_upy.setZero();
	_F_zmp_upy.setZero();
	_H_q_lowy.setZero();
	_F_zmp_lowy.setZero();

	_phi_i_x_up.setZero();
	_p_i_x_t_up.setZero();
	_del_i_x_up.setZero();
	_phi_i_x_low.setZero();
	_p_i_x_t_low.setZero();
	_del_i_x_low.setZero();
	_phi_i_y_up.setZero();
	_p_i_y_t_up.setZero();
	_del_i_y_up.setZero();
	_phi_i_y_low.setZero();
	_p_i_y_t_low.setZero();
	_del_i_y_low.setZero();	  

	// angle boundary preparation
	_q_upx.setZero();
	_qq_upx.setZero();
	_q_lowx.setZero();
	_qq_lowx.setZero();
	_q_upy.setZero();
	_qq_upy.setZero();
	_q_lowy.setZero();
	_qq_lowy.setZero();

	_qq1_upx.setZero();
	_qq1_lowx.setZero();
	_qq1_upy.setZero();
	_qq1_lowy.setZero();	  

	// torque bondary preparation
	_t_upx.setZero();
	_tt_upx.setZero();
	_t_lowx.setZero();
	_tt_lowx.setZero();
	_t_upy.setZero();
	_tt_upy.setZero();
	_t_lowy.setZero();
	_tt_lowy.setZero();

	_tt1_upx.setZero();
	_tt1_lowx.setZero();
	_tt1_upy.setZero();
	_tt1_lowy.setZero();

	// CoM height boundary preparation
	_H_h_upz.setZero();
	_F_h_upz.setZero();
	_H_h_lowz.setZero();
	_F_h_lowz.setZero();
	_delta_footz_up.setZero();
	_delta_footz_low.setZero();

	// CoM height acceleration boundary preparation
	_H_hacc_lowz.setZero();
	_F_hacc_lowz.setZero();
	_delta_footzacc_up.setZero();	  


	//swing foot velocity constraints
	_Footvx_max.setZero();
	_Footvx_min.setZero();
	_Footvy_max.setZero();
	_Footvy_min.setZero();
	_footubxv.setZero();
	_footlbxv.setZero();
	_footubyv.setZero();
	_footlbyv.setZero();
	
	// foot location constraints: be careful that the step number is change: so should be intialized in each whole loop
	_H_q_footx_up.setZero();
	_F_foot_upx.setZero();
	_H_q_footx_low.setZero();
	_F_foot_lowx.setZero();
	_H_q_footy_up.setZero();
	_F_foot_upy.setZero();
	_H_q_footy_low.setZero();
	_F_foot_lowy.setZero();	
	
	

	// foot vertical location-equality constraints
	_H_q_footz.setZero(1, _Nt);
	_F_footz.setZero(1, 1);

	// CoMZ height-equality constraints
	_h_h.setZero();
	_hhhx.setZero();	  

	// body inclination-equality constraints
	_a_hx.setZero();
	_a_hxx.setZero();
	_a_hy.setZero();
	_a_hyy.setZero();


	// foot location constraints
	_Sfoot.setZero();
	_Sfoot(0) = -1;
	_Sfoot(1) = 1;
	  
	// offline calulated the ZMP constraints coefficient==================================
	  //////////initiallize vector
	vector <Eigen::Matrix<double,_Nt, _Nt>> x_offline1(_nh)  ;
	for (int j=0;j<_nh; j++)
	{
	  x_offline1[j]= Eigen::Matrix<double,_Nt, _Nt>::Zero();
	}

	ZMPx_constraints_offfline = x_offline1;
	ZMPy_constraints_offfline = x_offline1;		
	_phi_i_x_up_est = x_offline1;
	_phi_i_y_up_est = x_offline1;
	
	vector <Eigen::Matrix<double,_nh, _Nt>> x_offline2(_nh)  ;
	for (int j=0;j<_nh; j++)
	{
	  x_offline2[j]= Eigen::Matrix<double,_nh, _Nt>::Zero();
	}	

	ZMPx_constraints_half = x_offline2;	
	ZMPy_constraints_half = x_offline2;
		
	for(int jxx=1; jxx<=_nh; jxx++)
	{
	  _Si.setZero();
	  _Si(0,jxx-1) = 1;
	  // ZMP constraints	      		 
         ZMPx_constraints_offfline[jxx-1] = (_Si * _ppu * _Sjx).transpose() * _Si * _pau * _Sjz - (_Si * _pau * _Sjx).transpose() * _Si * _ppu * _Sjz;
	 ZMPx_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _pau * _Sjz;
	  	  
         ZMPy_constraints_offfline[jxx-1] = (_Si * _ppu * _Sjy).transpose() * _Si * _pau * _Sjz - (_Si * _pau * _Sjy).transpose() * _Si * _ppu * _Sjz;
	 ZMPy_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _pau * _Sjz;
	      
	}
        

        _Footx_global_relative =0;
        _Footy_global_relative =0;	

	// boundary initialzation	  
	_Si.setZero();	  

	_ZMPx_constraints_half2.setZero();	 
	_ZMPy_constraints_half2.setZero();
	_phi_i_x_up1.setZero();
	_phi_i_y_up1.setZero();
	
	CoMMM_ZMP_foot.setZero();
	
	///QP initiallize
	if (_method_flag ==0)
	{
	  
	  int nVars = _Nt;
	  int nEqCon = 1+3*_nh;
	  int nIneqCon = 5*_nh + 4*_nstep+4;  
	  resizeQP(nVars, nEqCon, nIneqCon);		   
        }
	else if (_method_flag ==1)
	{
	  int nVars = _Nt;
	  int nEqCon = 1+_nh;
	  int nIneqCon = 13*_nh + 4*_nstep +4;
	  resizeQP(nVars, nEqCon, nIneqCon);	   
	}
	else
	{
	  int nVars = _Nt;
	  int nEqCon = 1;
	  int nIneqCon = 15*_nh + 4*_nstep +4;
	  resizeQP(nVars, nEqCon, nIneqCon);		   
	}

	_t_end_walking = _tx(_footstepsnumber-1)- 9*_tstep/4;  // ending time when normal walking ends
	_n_end_walking = round(_t_end_walking/_dt);	
	_comy_matrix_inv.setZero();	
}


/////////////////////// local coordinate CoM solution---modified---------------------------------
void MPCClass::CoM_foot_trajection_generation_local(int i, Eigen::Matrix<double,18,1> estimated_state, Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,double lamda, bool _stopwalking)
{
 
       if (i<_n_end_walking)   ///////normal walking
       {
	  /// modified the footy_min
	  if (i==(round(2*_ts(1)/_dt))+1) ///update the footy_limit
	  {
	      _footy_min = RobotParaClass::FOOT_WIDTH()+0.01;
	  }
  ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////	
  // 	================ iterative calulcation: predictive control_tracking with time-varying height+angular momentum	  
	    clock_t t_start,t_start1, t_start2, t_start3, t_start4,t_finish,t_finish1;	  	    
	    /// run once
	    ///////////////////////////////////////////////////////
	    ////////////////////////////////////// time_clock0////////////////////////
	    t_start = clock();		    

	    Indexfind((i-1)*_dt,xyz1);	  	      //// step cycle number when (i-1)*dt fall into      
	    _bjxx_1 = _j_period+1;
	    _j_period = 0;
	    
	    
	    Indexfind(i*_dt,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
	    _bjxx = _j_period+1;  //coincidence with matlab 
	    _j_period = 0;	  
	    
	    // com_center_ref = ZMP_center_ref = v_i*f + V_i*L_ref
	    //solve the following steps: 1.5s may couve 2 or three  steps, so  one/two following steps	    
	    _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
	    
	    Indexfind(_t_f(0),xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
	    _bjx1 = _j_period+1;
	    _j_period = 0;
	    
	    Indexfind(_t_f(_nh-1),xyz1);           /// step cycle number when (i+_nh)*dt fall into : current sampling time
	    _bjx2 = _j_period+1;
	    _j_period = 0;	  
	    	    
	    ////================================================================
	    /// judge if stop walking enable: if (from _bjx1 step, reference step length and step height will be set to be zero)
	    if(_stopwalking)  
	    {    
	      for (int i_t = _bjx1; i_t < _footstepsnumber; i_t++) {
		_steplength(i_t) = 0;	
		_footx_ref(i_t) = _footx_ref(i_t-1) + _steplength(i_t-1); 	      
	      }	  
	    }
	    
	    _mx = _bjx2 - _bjx1 +1;
            /// find out the relative postion of step cycle switching time to the predictive window 	    
	    for (int j=1;j<_mx; j++)
	    {
	      Indexfind(_tx(_bjx1+j-1),xyz2);
	      _tnx(j-1) = _j_period; 
	      _j_period = 0;	      
	    }

	    _v_i.setZero();
	    _VV_i.setZero();
	    xxx = _tnx(0);
	    // be careful that the position is from 0;;;;;         
	    if (fabs(_tnx(0) - _nT) <=0.00001)
	    {
	      _n_vis =2;
	      for (int jjj = 1; jjj <= _mx; jjj++)
	      {
		
		if (jjj == 1)
		{
		  _VV_i.block(0, 0, xxx, 1).setOnes();
		}
		else
		{	
		  xxx1 = _nh-_tnx(0);
		  _VV_i.block(xxx, 1, xxx1, 1).setOnes();		
		}
	      }	    
	    }
	    else
	    {	
	      _v_i.segment(0, xxx).setOnes();	      
	      if (abs(_mx - 2) <=0.00001)
	      {
		_n_vis = 1;
		_VV_i.block(xxx, 0, _nh -xxx, 1).setOnes();
	      }
	      else
	      {
		_n_vis = 2;
		xxx2 = _nh -_tnx(1);
		_VV_i.block<_nT, 1>(xxx, 0).setOnes();
		_VV_i.block(_tnx(1), 1, xxx2, 1).setOnes();	      	      
	      }
	    }
	    
  
	    _flag(i-1,0)= _n_vis;	    
	    _flag_global(i-1,0) = _bjxx;
	    _flag_global(i,0) = _bjx1;	  

  ////////////////////////////////////////////////////////////////////////////////////	  
  //============================================================//	  
	  
  
	    
  //////////////////// relative state: 	  
	    ///// pass the actual stage into the control loop	  	  	      
	    if (i>1)
	    {
	      _Footx_global_relative = _xk(0,i-1) + _fxx_global;
	      _Footy_global_relative = _yk(0,i-1) + _fyy_global;	    
	    }
	    // current foot location
	    _fx =0;
	    _fy = 0;	  	    
	    _fxx_global = _footx_real(_bjxx-1);
	    _fyy_global = _footy_real(_bjxx-1);	    

	    if (_n_vis ==1)
	    {
	      _Lx_ref(0) = _footx_ref(_bjx2-1) - _fxx_global;
	      _Ly_ref(0) = _footy_ref(_bjx2-1) - _fyy_global;
	      _Lz_ref(0) = _footz_ref(_bjx2-1);
	      _Lx_ref(1) = 0;
	      _Ly_ref(1) = 0;
	      _Lz_ref(1) = 0;	    
	    }
	    else
	    {
	      _Lx_ref(0) = _footx_ref(_bjx2-2) - _fxx_global;
	      _Ly_ref(0) = _footy_ref(_bjx2-2) - _fyy_global;
	      _Lz_ref(0) = _footz_ref(_bjx2-2);
	      _Lx_ref(1) = _footx_ref(_bjx2-1) - _fxx_global;
	      _Ly_ref(1) = _footy_ref(_bjx2-1) - _fyy_global;
	      _Lz_ref(1) = _footz_ref(_bjx2-1);	    
	    }	    
	  // com_center_ref
	    _comx_center_ref = _v_i*_fx + _VV_i*_Lx_ref;
	    _comy_center_ref = _v_i*_fy + _VV_i*_Ly_ref;
	    _comz_center_ref = _Zsc.segment<_nh>(i) + _Hcom;
	    
	    /// hot start
	    if (i==1)
	    {
	      _V_ini(5*_nh) = _footx_ref(1);
	      _V_ini(5*_nh+1) = _footy_ref(1);	    
	    }
	    else
	    {
	      _V_ini.topRows(5*_nh) = _V_optimal.block<5*_nh, 1>(0, i-2);
	      if (_n_vis > _flag(i-2))
	      { 
		_V_ini(_Nt -1-1) = _V_optimal(5*_nh+6-1, i-2);
		_V_ini(_Nt -3-1) = _V_optimal(5*_nh+6-2-1, i-2);
		_V_ini(_Nt -5-1) = _V_optimal(5*_nh+6-4-1, i-2); 
	      }
	      else
	      {
		if (_n_vis < _flag(i-2))
		{ 
		  _V_ini(_Nt-1) = _V_optimal(5*_nh+6-1, i-2);
		  _V_ini(_Nt -1-1) = _V_optimal(5*_nh+6-2-1, i-2);
		  _V_ini(_Nt -2-1) = _V_optimal(5*_nh+6-4-1, i-2); 
		}	   
		else
		{
		  if (_n_vis ==1)
		  { 
		    _V_ini(_Nt-1) = _V_optimal(5*_nh+6-1, i-2);
		    _V_ini(_Nt -1-1) = _V_optimal(5*_nh+6-2-1, i-2);
		    _V_ini(_Nt -2-1) = _V_optimal(5*_nh+6-4-1, i-2); 
		  }
		  else
		  {
		    _V_ini.bottomRows(6) = _V_optimal.block<6, 1>(5*_nh+6-6, i-2);
		  }
		}
	      }
	    }
	      	  
  // 	      // relative state switch	      
	    if (i>1)
	    {
	      if (_flag_global(i-2,0) < _flag_global(i-1,0) )
	      {
		/// reference relative state switch
	      
		_xk(0,i-1) = _Footx_global_relative - _fxx_global; 
		_yk(0,i-1) = _Footy_global_relative - _fyy_global;
		
	      }

	    }
	    //////////////////////////////////////////////////////////////////////////============================================================//////////////////////////////////////////
	    // foot location constraints: be careful that the step number is change: so should be intialized in each whole loop
	    _H_q_footx_up.setZero();
	    _F_foot_upx.setZero();
	    _H_q_footx_low.setZero();
	    _F_foot_lowx.setZero();
	    _H_q_footy_up.setZero();
	    _F_foot_upy.setZero();
	    _H_q_footy_low.setZero();
	    _F_foot_lowy.setZero();
	    

	    // boundary initialzation
	    _Sfx.setZero();
	    _Sfy.setZero();
	    _Sfz.setZero();
	    
	    if (_n_vis ==1)
	    {
	      _Sfx(0,5*_nh) = 1;
	      _Sfy(0,5*_nh+_nstep) = 1;
	      _Sfz(0,5*_nh+2*_nstep) = 1;
	    }  
	    else
	    {
	      _Sfx(0,5*_nh) = 1;
	      _Sfx(1,5*_nh+1) = 1;
	      _Sfy(0,5*_nh+_nstep) = 1;
	      _Sfy(1,5*_nh+_nstep+1) = 1;
	      _Sfz(0,5*_nh+2*_nstep) = 1;	 
	      _Sfz(1,5*_nh+2*_nstep+1) = 1;	
	      
	    }
		
		  
	    

	    //SQP MOdels	 
	      _q_goal.block<_nh, 1>(0, 0) = _alphax * _pvu.transpose() * _pvs * _xk.col(i-1) + _beltax * _ppu.transpose() * _pps * _xk.col(i-1) - _beltax * _ppu.transpose() * _comx_center_ref;
	      _q_goal.block<_nh, 1>(_nh, 0) = _alphay * _pvu.transpose() * _pvs * _yk.col(i-1) + _beltay * _ppu.transpose() * _pps * _yk.col(i-1) - _beltay * _ppu.transpose() * _comy_center_ref;
	      _q_goal.block<_nh, 1>(2*_nh, 0) = _alphaz * _pvu.transpose() * _pvs * _zk.col(i-1) + _beltaz * _ppu.transpose() * _pps * _zk.col(i-1) - _beltaz * _ppu.transpose() * _comz_center_ref;
	      _q_goal.block<_nh, 1>(3*_nh, 0) = _alphathetax * _pvu.transpose() * _pvs * _thetaxk.col(i-1) + _beltathetax * _ppu.transpose() * _pps * _thetaxk.col(i-1) - _beltathetax * _ppu.transpose() * _thetax_center_ref;
	      _q_goal.block<_nh, 1>(4*_nh, 0) = _alphathetay * _pvu.transpose() * _pvs * _thetayk.col(i-1) + _beltathetay * _ppu.transpose() * _pps * _thetayk.col(i-1) - _beltathetay * _ppu.transpose() * _thetay_center_ref;
	      _q_goal.block<_nstep, 1>(5*_nh, 0) = -_gamax * _Lx_ref;
	      _q_goal.block<_nstep, 1>(5*_nh+_nstep, 0) = -_gamay * _Ly_ref;
	      _q_goal.block<_nstep, 1>(5*_nh+2*_nstep, 0) = -_gamaz * _Lz_ref;

	      
		
	    ///// the following code only run once in each loop   
	      for(int jxx=1; jxx<=_nh; jxx++)
	      {
		_Si.setZero();
		_Si(0,jxx-1) = 1;
		
		// ZMP constraints
		// x-ZMP upper boundary                                      
		_p_i_x_t_up.col(jxx-1) = _mass * (((_Si * _pps * _xk.col(i-1)).transpose() *_Si*_pau*_Sjz + (_Si*_pas*_zk.col(i-1)).transpose()*_Si*_ppu*_Sjx + _ggg*_Si*_ppu*_Sjx - ((_Si * _pps * _zk.col(i-1)).transpose() *_Si*_pau*_Sjx + _Si * _pas * _xk.col(i-1)* _Si* _ppu* _Sjz) + _Zsc.row(i+jxx-1)*_Si*_pau*_Sjx - ((_Si * _pas * _zk.col(i-1)).transpose() *_Si*_VV_i*_Sfx + (_Si * _v_i * _fx).transpose() *_Si*_pau*_Sjz) - _ggg*_Si*_VV_i*_Sfx - _zmpx_ub*_Si*_pau*_Sjz).transpose()) - (_j_ini * _Si*_pau * _Sjthetay).transpose();		
		_del_i_x_up.col(jxx-1) = _mass * ((_Si * _pps * _xk.col(i-1)).transpose() *_Si*_pas*_zk.col(i-1) + _ggg*_Si * _pps * _xk.col(i-1) - (_Si * _pas * _xk.col(i-1)).transpose() *_Si*_pps*_zk.col(i-1) + (_Si * _pas * _xk.col(i-1)).transpose() *_Zsc.row(i+jxx-1) - (_Si * _v_i * _fx).transpose() *_Si*_pas*_zk.col(i-1) - _ggg *_Si * _v_i * _fx - _zmpx_ub*_Si*_pas*_zk.col(i-1) - _ggg *_zmpx_ub) - _j_ini * _Si*_pas * _thetayk.col(i-1);


		// x-ZMP low boundary
		_p_i_x_t_low.col(jxx-1) = (_p_i_x_t_up.col(jxx-1).transpose() + _mass * _zmpx_ub*_Si*_pau*_Sjz - _mass * _zmpx_lb*_Si*_pau*_Sjz).transpose();	      
		_del_i_x_low.col(jxx-1) = _del_i_x_up.col(jxx-1) +_mass*_zmpx_ub*_Si*_pas*_zk.col(i-1)+  _mass * _ggg*_zmpx_ub - _mass*_zmpx_lb*_Si*_pas*_zk.col(i-1)-_mass * _ggg*_zmpx_lb;
		
		// y-ZMP upper boundary
    		_p_i_y_t_up.col(jxx-1) = _mass * (((_Si * _pps * _yk.col(i-1)).transpose() *_Si*_pau*_Sjz + (_Si*_pas*_zk.col(i-1)).transpose()*_Si*_ppu*_Sjy + _ggg*_Si*_ppu*_Sjy - ((_Si * _pps * _zk.col(i-1)).transpose() *_Si*_pau*_Sjy + _Si * _pas * _yk.col(i-1)* _Si* _ppu* _Sjz) + _Zsc.row(i+jxx-1)*_Si*_pau*_Sjy - ((_Si * _pas * _zk.col(i-1)).transpose() *_Si*_VV_i*_Sfy + (_Si * _v_i * _fy).transpose() *_Si*_pau*_Sjz) - _ggg*_Si*_VV_i*_Sfy - _zmpy_ub*_Si*_pau*_Sjz).transpose()) + (_j_ini * _Si*_pau * _Sjthetax).transpose();
		_del_i_y_up.col(jxx-1) = _mass * ((_Si * _pps * _yk.col(i-1)).transpose() *_Si*_pas*_zk.col(i-1) + _ggg*_Si * _pps * _yk.col(i-1) - (_Si * _pas * _yk.col(i-1)).transpose() *_Si*_pps*_zk.col(i-1) + (_Si * _pas * _yk.col(i-1)).transpose() *_Zsc.row(i+jxx-1) - (_Si * _v_i * _fy).transpose() *_Si*_pas*_zk.col(i-1) - _ggg *_Si * _v_i * _fy - _zmpy_ub*_Si*_pas*_zk.col(i-1) - _ggg *_zmpy_ub); + _j_ini * _Si*_pas * _thetaxk.col(i-1);	      
	      
		// y-ZMP low boundary
		_phi_i_y_low = _phi_i_y_up;  
		_p_i_y_t_low.col(jxx-1) = (_p_i_y_t_up.col(jxx-1).transpose() + _mass * _zmpy_ub*_Si*_pau*_Sjz - _mass * _zmpy_lb*_Si*_pau*_Sjz).transpose();	      
		_del_i_y_low.col(jxx-1) = _del_i_y_up.col(jxx-1) +_mass*_zmpy_ub*_Si*_pas*_zk.col(i-1)+  _mass * _ggg*_zmpy_ub - _mass*_zmpy_lb*_Si*_pas*_zk.col(i-1)-_mass * _ggg*_zmpy_lb;	      	      	     

		
		
		//angle range constraints
		_q_upx.row(jxx-1) = _Si* _ppu* _Sjthetax;
		_q_lowx.row(jxx-1) = -_q_upx.row(jxx-1);	         

		_qq1_upx.row(jxx-1) = _Si* _pps* _thetaxk.col(i-1);
		_qq1_upx(jxx-1,0) = _qq1_upx(jxx-1,0)-_thetax_max;
		
		_q_upy.row(jxx-1) = _Si* _ppu* _Sjthetay;
		_q_lowy.row(jxx-1) = -_q_upy.row(jxx-1);	
		
		_qq1_upy.row(jxx-1) = _Si* _pps* _thetayk.col(i-1);
		_qq1_upy(jxx-1,0) = _qq1_upy(jxx-1,0)-_thetay_max;    

		//torque range constraints
		_t_upx.row(jxx-1) = _Si* _pau* _Sjthetax;
		_t_lowx.row(jxx-1) = -_t_upx.row(jxx-1);
		
		_tt1_upx.row(jxx-1) = _Si* _pas* _thetaxk.col(i-1);
		_tt1_upx(jxx-1,0) = _tt1_upx(jxx-1,0)-_torquex_max;
		
		_t_upy.row(jxx-1) = _Si* _pau* _Sjthetay;
		_t_lowy.row(jxx-1) = -_t_upy.row(jxx-1);	 
		
		_tt1_upy.row(jxx-1) = _Si* _pas* _thetayk.col(i-1);
		_tt1_upy(jxx-1,0) = _tt1_upy(jxx-1,0)-_torquey_max;		
		
		// body height constraints
		_H_h_upz.row(jxx-1) = _Si* _ppu* _Sjz;
		_H_h_lowz.row(jxx-1) = -_Si* _ppu* _Sjz;   	
		_delta_footz_up.row(jxx-1) = _Si*_pps*_zk.col(i-1) - _comz_center_ref.row(jxx-1) ;
		_delta_footz_up(jxx-1,0) = _delta_footz_up(jxx-1,0) - _z_max;
		
		// body height acceleration constraints	      
		_H_hacc_lowz.row(jxx-1) = -_Si* _pau* _Sjz;   
		_delta_footzacc_up.row(jxx-1) = _Si*_pas*_zk.col(i-1) + _ggg;
	      }

  ///       only one-time caluation	  
	    _ZMPx_constraints_half2 = (_VV_i* _Sfx).transpose();
	    _ZMPy_constraints_half2 = (_VV_i* _Sfy).transpose();	    

	    for(int jxx=1; jxx<=_nh; jxx++)
	    {
	      // ZMP constraints
	      // x-ZMP upper boundary
	      _phi_i_x_up1 = ZMPx_constraints_offfline[jxx-1] + _ZMPx_constraints_half2 * ZMPx_constraints_half[jxx-1];	      
	      _phi_i_x_up_est[jxx-1] = _mass * (_phi_i_x_up1 + _phi_i_x_up1.transpose())/2;
    
	      // y-ZMP upper boundary
	      _phi_i_y_up1 = ZMPy_constraints_offfline[jxx-1] + _ZMPy_constraints_half2 * ZMPy_constraints_half[jxx-1];
	      _phi_i_y_up_est[jxx-1] = _mass * (_phi_i_y_up1 + _phi_i_y_up1.transpose())/2;   
	    }
    
	    // constraints: only once 
	    _Footvx_max = _Sfx.row(0);
	    _Footvx_min = -_Sfx.row(0);
	    _Footvy_max = _Sfy.row(0);
	    _Footvy_min = -_Sfy.row(0);	  		    
	    ///////////// equality equation	    
	    //equality constraints
	    _H_q_footz = _Sfz.row(0);	    
	    _h_h = _ppu * _Sjz;
	    
	    _a_hx = _ppu * _Sjthetax;
	    _a_hy = _ppu * _Sjthetay;
	   
	    // SEQUENCE QUADARTIC PROGRAMMING: lOOP_until the maximal loops reaches	
	    // SEQUENCE QUADARTIC PROGRAMMING: lOOP_until the maximal loops reaches	
	    t_start1 = clock();
    
	  // hot start	    
	    for (int xxxx=1; xxxx <= _loop; xxxx++)
	    {	
    
	      _q_goal1 = _Q_goal1 * _V_ini + _q_goal;	  
		      
  ///////////// inequality equation   
	      t_start2 = clock();
	      
	      /// time consuming process	    
	      
	      for(int jxx=1; jxx<=_nh; jxx++)
	      {
		// ZMP constraints
		_phi_i_x_up = _phi_i_x_up_est[jxx-1];
		_H_q_upx.row(jxx-1) = (2*_phi_i_x_up*_V_ini + _p_i_x_t_up.col(jxx-1)).transpose(); 
		_F_zmp_upx.row(jxx-1) = -((_V_ini.transpose() * _phi_i_x_up + _p_i_x_t_up.col(jxx-1).transpose()) * _V_ini + _del_i_x_up.col(jxx-1)); 

		// x-ZMP low boundary
		_phi_i_x_low = _phi_i_x_up;	      	      
		_H_q_lowx.row(jxx-1) = (- _p_i_x_t_low.col(jxx-1) + _p_i_x_t_up.col(jxx-1)).transpose() - _H_q_upx.row(jxx-1);  
		_F_zmp_lowx.row(jxx-1) = (_p_i_x_t_low.col(jxx-1)-_p_i_x_t_up.col(jxx-1)).transpose() * _V_ini + _del_i_x_low.col(jxx-1) -  _del_i_x_up.col(jxx-1)-_F_zmp_upx.row(jxx-1); 
		
		
		// y-ZMP upper boundary	      
		_phi_i_y_up = _phi_i_y_up_est[jxx-1];	      
		_H_q_upy.row(jxx-1) = (2*_phi_i_y_up*_V_ini + _p_i_y_t_up.col(jxx-1)).transpose(); 
		_F_zmp_upy.row(jxx-1) = -((_V_ini.transpose() * _phi_i_y_up + _p_i_y_t_up.col(jxx-1).transpose()) * _V_ini + _del_i_y_up.col(jxx-1)); 
		
		// y-ZMP low boundary
		_phi_i_y_low = _phi_i_y_up;  	      	      
		_H_q_lowy.row(jxx-1) = (- _p_i_y_t_low.col(jxx-1) + _p_i_y_t_up.col(jxx-1)).transpose() - _H_q_upy.row(jxx-1); 
		_F_zmp_lowy.row(jxx-1) = (_p_i_y_t_low.col(jxx-1)-_p_i_y_t_up.col(jxx-1)).transpose() * _V_ini + _del_i_y_low.col(jxx-1) - _del_i_y_up.col(jxx-1)-_F_zmp_upy.row(jxx-1);      	      
	    

		
		//angle range constraints
		_qq_upx.row(jxx-1) = -(_q_upx.row(jxx-1)* _V_ini + _qq1_upx.row(jxx-1));
		_qq_lowx.row(jxx-1) = - _qq_upx.row(jxx-1);	 
		_qq_lowx(jxx-1,0) = _thetax_max - _thetax_min + _qq_lowx(jxx-1,0);	 
				
		_qq_upy.row(jxx-1) = -(_q_upy.row(jxx-1)* _V_ini + _qq1_upy.row(jxx-1));
		_qq_lowy.row(jxx-1) = - _qq_upy.row(jxx-1);
		_qq_lowy(jxx-1,0) = _thetay_max - _thetay_min + _qq_lowy(jxx-1,0);

		//torque range constraints	      
		_tt_upx.row(jxx-1) = -(_t_upx.row(jxx-1)* _V_ini +  _tt1_upx.row(jxx-1));
		_tt_lowx.row(jxx-1) = - _tt_upx.row(jxx-1);	
		_tt_lowx(jxx-1,0) = _torquex_max - _torquex_min + _tt_lowx(jxx-1,0);	
		
		_tt_upy.row(jxx-1) = -(_t_upy.row(jxx-1)* _V_ini +  _tt1_upy.row(jxx-1));
		_tt_lowy.row(jxx-1) = - _tt_upy.row(jxx-1);		      
		_tt_lowy(jxx-1,0) = _torquey_max - _torquey_min + _tt_lowy(jxx-1,0);	
		
		// body height constraints	      
		_F_h_upz.row(jxx-1) = -(_H_h_upz.row(jxx-1)*_V_ini + _delta_footz_up.row(jxx-1));
		_F_h_lowz.row(jxx-1) = -_F_h_upz.row(jxx-1);	      
		_F_h_lowz(jxx-1,0) = _z_max - _z_min +_F_h_lowz(jxx-1,0);
		
		// body height acceleration constraints	      
		_F_hacc_lowz.row(jxx-1) = (-_H_hacc_lowz.row(jxx-1)*_V_ini + _delta_footzacc_up.row(jxx-1));	      	      
	      }

	      
	      
	      
	      
	      
	      t_start3 = clock();
	      
	      // foot location constraints
	      if (_n_vis == 1)  //one next steo
	      {
		_H_q_footx_up.row(0) = _Sfx.row(0);
		_F_foot_upx.row(0) = -(_H_q_footx_up.row(0) * _V_ini); 
		_F_foot_upx(0,0) = _F_foot_upx(0,0) +_fx + _footx_max; 
		
		_H_q_footx_low.row(0) = -_Sfx.row(0);
		_F_foot_lowx.row(0) = (_H_q_footx_up.row(0) * _V_ini);
		_F_foot_lowx(0,0) = _F_foot_lowx(0,0)-_fx- _footx_min;
		
		
		// footy location constraints
		if (_bjxx % 2 == 0) //odd
		{
		  _H_q_footy_up.row(0) = _Sfy.row(0);
		  _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
		  _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy - _footy_min; 
		  
		  _H_q_footy_low.row(0) = -_Sfy.row(0);
		  _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini);	
		  _F_foot_lowy(0,0) = _F_foot_lowy(0,0) -_fy + _footy_max;	
		}
		else
		{
		  _H_q_footy_up.row(0) = _Sfy.row(0);
		  _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
		  _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy + _footy_max; 
		  _H_q_footy_low.row(0) = -_Sfy.row(0);
		  _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini );
		  _F_foot_lowy(0,0) = _F_foot_lowy(0,0)-_fy - _footy_min;
		}	 
		
		
	      }
	      else   //two next steps
	      {
		_H_q_footx_up.row(0) = _Sfx.row(0);
		_F_foot_upx.row(0) = -(_H_q_footx_up.row(0) * _V_ini); 
		_F_foot_upx(0,0) = _F_foot_upx(0,0) +_fx + _footx_max;
		_H_q_footx_low.row(0) = -_Sfx.row(0);
		_F_foot_lowx.row(0) = (_H_q_footx_up.row(0) * _V_ini);
		_F_foot_lowx(0,0) = _F_foot_lowx(0,0)-_fx - _footx_min;
		
		// footy location constraints
		if (_bjxx % 2 == 0) //odd
		{
		  _H_q_footy_up.row(0) = _Sfy.row(0);
		  _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
		  _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy - _footy_min;
		  _H_q_footy_low.row(0) = -_Sfy.row(0);
		  _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini);
		  _F_foot_lowy(0,0) = _F_foot_lowy(0,0)-_fy + _footy_max;
		}
		else
		{
		  _H_q_footy_up.row(0) = _Sfy.row(0);
		  _F_foot_upy.row(0) = -(_H_q_footy_up.row(0) * _V_ini); 
		  _F_foot_upy(0,0) = _F_foot_upy(0,0)+_fy + _footy_max;
		  _H_q_footy_low.row(0) = -_Sfy.row(0);
		  _F_foot_lowy.row(0) = (_H_q_footy_up.row(0) * _V_ini);
		  _F_foot_lowy(0,0) = _F_foot_lowy(0,0)-_fy - _footy_min;
		}
		
		// the next two steps 
		_H_q_footx_up.row(1) = _Sfoot* _Sfx;
		_F_foot_upx.row(1) = -(_H_q_footx_up.row(1) * _V_ini); 	 
		_F_foot_upx(1,0) = _F_foot_upx(1,0) +_footx_max; 
		
		_H_q_footx_low.row(1) = -_Sfoot* _Sfx;
		_F_foot_lowx.row(1) = (_H_q_footx_up.row(1) * _V_ini);
		_F_foot_lowx(1,0) = (_F_foot_lowx(1,0) - _footx_min);
		
		// footy location constraints
		if (_bjxx % 2 == 0) //odd
		{
		  _H_q_footy_up.row(1) = _Sfoot* _Sfy;
		  _F_foot_upy.row(1) = -(_H_q_footy_up.row(1) * _V_ini); 
		  _F_foot_upy(1,0) = _F_foot_upy(1,0) + _footy_max; 
		  
		  _H_q_footy_low.row(1) = -_H_q_footy_up.row(1);
		  _F_foot_lowy.row(1) = (_H_q_footy_up.row(1) * _V_ini);
		  _F_foot_lowy(1,0) = _F_foot_lowy(1,0) - _footy_min;		  
		}
		else
		{
		  _H_q_footy_up.row(1) = _Sfoot* _Sfy;
		  _F_foot_upy.row(1) = -(_H_q_footy_up.row(1) * _V_ini); 
		  _F_foot_upy(1,0) = _F_foot_upy(1,0) - _footy_min; 
		  
		  _H_q_footy_low.row(1) = -_H_q_footy_up.row(1);
		  _F_foot_lowy.row(1) = (_H_q_footy_up.row(1) * _V_ini);
		  _F_foot_lowy(1,0) = _F_foot_lowy(1,0) + _footy_max;		  
		}	      	     	      
	      }

	      
	      //swing foot veloctiy boundary
	      if (i ==1)
	      {
		_footubxv = -(_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));
		_footubxv(0,0) = _footubxv(0,0)  + _footx_max;
		
		_footlbxv = (_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));
		_footlbxv(0,0) = _footlbxv(0,0) - _footx_min;		
		
		_footubyv = -(_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));
		_footubyv(0,0) = _footubyv(0,0) +_footy_max;		
		
		_footlbyv = (_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));
		_footlbyv(0,0) = _footlbyv(0,0) - _footy_min;		
		
	      }
	      else
	      {
		if (fabs(i*_dt - _tx(_bjxx-1))<=0.01)
		{		
		  _Footvx_max.setZero();  _Footvx_min.setZero();  _Footvy_max.setZero();  _Footvy_min.setZero();		  
		  _footubxv.setZero(); _footlbxv.setZero(); _footubyv.setZero(); _footlbyv.setZero();			
		}
		else
		{
		  _footubxv = -(_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));
		  _footubxv(0,0) = _footubxv(0,0) + _footx_vmax*_dt;
		  _footlbxv = (_Sfx.row(0) * _V_ini - _footx_real_next.row(i+_nT-2));
		  _footlbxv(0,0) = _footlbxv(0,0) - _footx_vmin*_dt;		  
		  _footubyv = -(_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));
		  _footubyv(0,0) = _footubyv(0,0) + _footy_vmax*_dt;		  
		  _footlbyv = (_Sfy.row(0) * _V_ini - _footy_real_next.row(i+_nT-2));	
		  _footlbyv(0,0) = _footlbyv(0,0) - _footy_vmin*_dt;		  
		}
	      }


	    ///////////// equality equation	    
	    //equality constraints
	    _F_footz = _Sfz.row(0)*_V_ini - _Lz_ref.row(0);	    
	    _hhhx = _h_h*_V_ini + _pps * _zk.col(i-1) - _Hcom;	

	    _a_hxx = _a_hx * _V_ini + _pps * _thetaxk.col(i-1);
	    _a_hyy = _a_hy * _V_ini + _pps * _thetayk.col(i-1);		    	      
  //////////////////////////////===========////////////////////////////////////////////////////	    
  // 	    // quadratic program GetSolution
	      t_start4 = clock();
	      
	      if (_method_flag ==0)
	      {
		solve_reactive_step(); ///merely the reactive steps
	      }
	      else
	      {
		if (_method_flag ==1)
		{
		  solve_reactive_step_body_inclination(); /// the reactive steps + body inclination
		}
		else
		{
		  solve_reactive_step_body_inclination_CoMz(); /// the reactive steps + body inclination + height variance
		}
	      }

	      
	      t_finish1 = clock();
	      _tcpu_prepara(0,i-1) = (double)(t_finish1 - t_start2)/CLOCKS_PER_SEC ;
    
	      _tcpu_prepara2(0,i-1) = (double)(t_finish1 - t_start3)/CLOCKS_PER_SEC ;
	      _tcpu_qp(0,i-1) = (double)(t_finish1 - t_start4)/CLOCKS_PER_SEC ;	    
	    }

	    
	    
	    t_finish = clock();	  
  /////////////////////////////////////////////////////////
  /////////===================================================%%%%%	  
	  // results postprocessed:	  
	    if (_n_vis == 1)
	    {
	      _V_optimal.block< 5*_nh, 1>(0, i-1)  = _V_ini.topRows(5*_nh);
	      _V_optimal.block< 2, 1>(5*_nh, i-1)  = _V_inix.setConstant(_V_ini(5*_nh,0));
	      _V_optimal.block< 2, 1>(5*_nh+2, i-1)  = _V_inix.setConstant(_V_ini(5*_nh+1,0));
	      _V_optimal.block< 2, 1>(5*_nh+4, i-1)  = _V_inix.setConstant(_V_ini(5*_nh+2,0));	      
	    }
	    else
	    {
	      _V_optimal.col(i-1) = _V_ini;
	    }
	    
	    //next step location
	    _x_vacc_k.col(i-1) = _V_ini.row(0);
	    _footx_real.row(_bjxx) = _V_ini.row(5*_nh);
	    _footx_real_next.row(i+_nT -1) = _V_ini.row(5*_nh);
	    _xk.col(i) = _a * _xk.col(i-1)  + _b* _x_vacc_k.col(i-1);
	    _comx(0,i)=_xk(0,i); _comvx(0,i) = _xk(1,i); _comax(0,i)=_xk(2,i); 	  
	    
	    _y_vacc_k.col(i-1) = _V_ini.row(0+_nh);
	    _footy_real.row(_bjxx) = _V_ini.row(5*_nh + _nstep);
	    _footy_real_next.row(i+_nT -1) = _V_ini.row(5*_nh + _nstep);
	    _yk.col(i) = _a * _yk.col(i-1)  + _b* _y_vacc_k.col(i-1);
	    _comy(0,i)=_yk(0,i); _comvy(0,i) = _yk(1,i); _comay(0,i)=_yk(2,i); 
	    
	    
	    
	    _comx(0,i) +=  _fxx_global;
	    _comy(0,i) +=  _fyy_global;	  
	    _footx_real(_bjxx) = _footx_real(_bjxx) + _fxx_global;
	    _footy_real(_bjxx) = _footy_real(_bjxx) + _fyy_global;	  	  
	    _footx_real_next1(i+_nT -1) = _footx_real_next(i+_nT -1) + _fxx_global;	  
	    _footy_real_next1(i+_nT -1) = _footy_real_next(i+_nT -1) + _fyy_global;

	    
	    
	    _z_vacc_k.col(i-1) = _V_ini.row(0+2*_nh);
	    _footz_real.row(_bjxx) = _V_ini.row(5*_nh + 2*_nstep);
	    _footz_real_next.row(i+_nT -1) = _V_ini.row(5*_nh + 2*_nstep);	  
	    _zk.col(i) = _a * _zk.col(i-1)  + _b* _z_vacc_k.col(i-1);
	    _comz(0,i)=_zk(0,i); _comvz(0,i) = _zk(1,i); _comaz(0,i)=_zk(2,i); 

	    _thetax_vacc_k.col(i-1) = _V_ini.row(0+3*_nh);
	    _thetaxk.col(i) = _a * _thetaxk.col(i-1)  + _b* _thetax_vacc_k.col(i-1);
	    _thetax(0,i) = _thetaxk(0,i); _thetavx(0,i) = _thetaxk(1,i); _thetaax(0,i)=_thetaxk(2,i); 	


	    _thetay_vacc_k.col(i-1) = _V_ini.row(0+4*_nh);
	    _thetayk.col(i) = _a * _thetayk.col(i-1)  + _b* _thetay_vacc_k.col(i-1);
	    _thetay(0,i) = _thetayk(0,i); _thetavy(0,i) = _thetayk(1,i); _thetaay(0,i)=_thetayk(2,i); 	
	    
	    
	    // reference relative state    
	    /// /// relative state to the actual foot lcoation: very good
	    if (_bjxx % 2 == 0)  // odd : left support
	    {
	      estimated_state(0,0) =  estimated_state(0,0) - _Lfoot_location_feedback(0);
	      estimated_state(3,0) =  estimated_state(3,0) - _Lfoot_location_feedback(1);	 	    
	    }
	    else
	    {
	      estimated_state(0,0) =  estimated_state(0,0) - _Rfoot_location_feedback(0);
	      estimated_state(3,0) =  estimated_state(3,0) - _Rfoot_location_feedback(1);	  
	    }
  //////============================================================================================================================	      
  ////////////////////////////// state modified:====================================================================================

	    if (_method_flag <2)
	    {
	      estimated_state(6,0) =  _comz(0,i-1);  
	      estimated_state(7,0) =  _comvz(0,i-1);  
	      estimated_state(8,0) =  _comaz(0,i-1);
	    }	  
	  
	    
	    if (_method_flag <=0)
	    {
	      estimated_state(9,0) =  _thetax(0,i-1);  
	      estimated_state(10,0) =  _thetavx(0,i-1);  
	      estimated_state(11,0) =  _thetaax(0,i-1);	  
	      estimated_state(12,0) =  _thetay(0,i-1);  
	      estimated_state(13,0) =  _thetavy(0,i-1);  
	      estimated_state(14,0) =  _thetaay(0,i-1);	    
	    }
		
		
		
		
  /*	  ///////////////================state feedback: determined by ratio parameter: lamda==============================////
	    /// model0================COMX+COMY feedback

	    _xk(0,i) = (estimated_state(0,0)+2*_xk(0,i))/3;             
	    _xk(1,i) = (estimated_state(1,0)+2*_xk(1,i))/3; 
	    _xk(2,i) = (estimated_state(2,0)+2*_xk(2,i))/3;
	    

	    if (i<round(2*_ts(1)/_dt))
	    {
		
	    _yk(0,i) = (estimated_state(3,0)+24*_yk(0,i))/25; _yk(1,i) = (estimated_state(4,0)+24*_yk(1,i))/25;
	    _yk(2,i) = (estimated_state(5,0)+24*_yk(2,i))/25;		    
	      
	    }
	    else
	    {
		
	    _yk(0,i) = (estimated_state(3,0)+2*_yk(0,i))/3; _yk(1,i) = (estimated_state(4,0)+2*_yk(1,i))/3;
	    _yk(2,i) = (estimated_state(5,0)+2*_yk(2,i))/3;		    
	    }

	    ///model1===================COMX+COMY + thetax+ thetay feedback
	    _thetaxk(0,i) = (estimated_state(9,0)+1*_thetaxk(0,i))/2;
	    _thetaxk(1,i) = (estimated_state(10,0)+1*_thetaxk(1,i))/2; 
	    _thetaxk(2,i) = (estimated_state(11,0)+1*_thetaxk(2,i))/2;	  
	    _thetayk(0,i) = (estimated_state(12,0)+1*_thetayk(0,i))/2; 
	    _thetayk(1,i) = (estimated_state(13,0)+1*_thetayk(1,i))/2; 
	    _thetayk(2,i) = (estimated_state(14,0)+1*_thetayk(2,i))/2;	
	    
	    //model2====================COMX+COMY + thetax+ thetay + CoMz feedback
	    _zk(0,i) = (estimated_state(6,0)+2*_zk(0,i))/3; 
	    _zk(1,i) = (estimated_state(7,0)+2*_zk(1,i))/3; 
	    _zk(2,i) = (estimated_state(8,0)+2*_zk(2,i))/3;*/	
	    
	    
	    
	    
	    
    

  ////////////////===============================================================================================	  
	  /// next two sample time:	actually the preictive value is not reliable  
	    _comx(0,i+1) = _comx(0,i) + _dt * _comvx(0,i); 	  	  
	    _comy(0,i+1) = _comy(0,i) + _dt * _comvy(0,i); 	 
	    _comz(0,i+1) = _comz(0,i) + _dt * _comvz(0,i); 	 
	    _thetax(0,i+1) = _thetax(0,i)+ _dt * _thetavx(0,i); 	
	    _thetay(0,i+1) = _thetay(0,i)+ _dt * _thetavy(0,i); 	
	    
	    
	    _torquex_real.col(i) = _j_ini * _thetaax.col(i);
	    _torquey_real.col(i) = _j_ini * _thetaay.col(i);
	    
	    _zmpx_real(0,i) = _comx(0,i) - (_comz(0,i) - _Zsc(i))/(_comaz(0,i)+_ggg(0))*_comax(0,i) - _j_ini * _thetavy(0,i)/(_mass * (_ggg(0) + _comaz(0,i)));
	    _zmpy_real(0,i) = _comy(0,i) - (_comz(0,i) - _Zsc(i))/(_comaz(0,i)+_ggg(0))*_comay(0,i) + _j_ini * _thetavx(0,i)/(_mass * (_ggg(0) + _comaz(0,i)));
	    
	    t_finish = clock();
	    _tcpu(0,i-1) = (double)(t_finish - t_start)/CLOCKS_PER_SEC ;
	    _tcpu_iterative(0,i-1) = (double)(t_finish - t_start1)/CLOCKS_PER_SEC ;
	    
	    _footxyz_real.row(0) = _footx_real.transpose();
	    _footxyz_real.row(1) = _footy_real.transpose();	  
	    _footxyz_real.row(2) = _footz_real.transpose();
	  
	  
	  if (i>=1)
	  {	  
	    _Rfootx(0) = _Rfootx(1);
	    _Lfootx(0) = _Lfootx(1);
	    _Rfooty(0) = _Rfooty(1);
	    _Lfooty(0) = _Lfooty(1);
	    _comx(0) = _comx(1);	
	    _comy(0) = _comy(1);
	    _comz(0) = _comz(1);	  
	  }	 
	  
	 
	 
	 
	 
	 
      }
       else
       {
	 if(i <= _n_end_walking+round(_tstep/_dt/2)){
	   Eigen::Matrix<double, 4, 1> _comy_temp;
	   _comy_temp.setZero();
	   _comy_temp(0) = _comy(0,_n_end_walking-2);
	   _comy_temp(1) = _comy(0,_n_end_walking-1);
	   _comy_temp(2) = 0;
	   _comy_temp(3) = 0;
	      
	   
	   Eigen::Matrix<double, 1, 4> _t_temp;
	   

	   _t_temp(0) = pow(i-_n_end_walking+1, 3);
	   _t_temp(1) = pow(i-_n_end_walking+1, 2);
	   _t_temp(2) = pow(i-_n_end_walking+1, 1);
	   _t_temp(3) = pow(i-_n_end_walking+1, 0);
	   
	   Eigen::Matrix<double, 1, 4> _t_temp1;

	   _t_temp1(0) = pow(i-_n_end_walking+2, 3);
	   _t_temp1(1) = pow(i-_n_end_walking+2, 2);
	   _t_temp1(2) = pow(i-_n_end_walking+2, 1);
	   _t_temp1(3) = pow(i-_n_end_walking+2, 0);	   
	   
	   
	   Eigen::Matrix<double, 4, 4> _comy_matrix;
	   
	   int ix_temp1 = -1;
	   int ix_temp2 = 0;	   
	   int ix_temp3 = round(_tstep/_dt)/2+1;
	   
	  
	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((ix_temp1 - ix_temp2)*pow(ix_temp1 - ix_temp3, 2));
	  abx2 = ((ix_temp1 - ix_temp2)*pow(ix_temp2 - ix_temp3, 2));
	  abx3 =(pow(ix_temp1 - ix_temp3, 2)*pow(ix_temp2 - ix_temp3, 2));
	  abx4 = ((ix_temp1 - ix_temp3)*(ix_temp2 - ix_temp3));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (ix_temp1 + ix_temp2 - 2*ix_temp3)/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(ix_temp2 + 2*ix_temp3)/ abx1;
	  AAA_inv(1,1) = (ix_temp1 + 2*ix_temp3)/ abx2;
	  AAA_inv(1,2) = -(pow(ix_temp1, 2) + ix_temp1*ix_temp2 + pow(ix_temp2, 2) - 3*pow(ix_temp3, 2))/ abx3;
	  AAA_inv(1,3) = -(ix_temp1 + ix_temp2 + ix_temp3)/ abx4;
	  
	  AAA_inv(2,0) = (ix_temp3*(2*ix_temp2 + ix_temp3))/ abx1;
	  AAA_inv(2,1) = -(ix_temp3*(2*ix_temp1 + ix_temp3))/ abx2;
	  AAA_inv(2,2) = (ix_temp3*(2*pow(ix_temp1, 2) + 2*ix_temp1*ix_temp2 - 3*ix_temp3*ix_temp1 + 2*pow(ix_temp2, 2) - 3*ix_temp3*ix_temp2))/ abx3;
	  AAA_inv(2,3) = (ix_temp1*ix_temp2 + ix_temp1*ix_temp3 + ix_temp2*ix_temp3)/ abx4;
	  
	  AAA_inv(3,0) = -(ix_temp2*pow(ix_temp3, 2))/ abx1;
	  AAA_inv(3,1) = (ix_temp1*pow(ix_temp3, 2))/ abx2;
	  AAA_inv(3,2) = (ix_temp1*ix_temp2*(ix_temp1*ix_temp2 - 2*ix_temp1*ix_temp3 - 2*ix_temp2*ix_temp3 + 3*pow(ix_temp3, 2)))/ abx3;
	  AAA_inv(3,3) = -(ix_temp1*ix_temp2*ix_temp3)/ abx4;
	  	  
	  	   
	   
	   
	   
	   
	   _comy_matrix_inv = AAA_inv * _comy_temp;
	   

	   
	   _comy.col(i) = _t_temp* _comy_matrix_inv;
	   _comy.col(i+1) = _t_temp1* _comy_matrix_inv;	   
	   
	   _comx(0,i) = _comx(0,_n_end_walking-1);
	   _comx(0,i+1) = _comx(0,_n_end_walking-1);
	   _comz(0,i) = _comz(0,_n_end_walking-1);
	   _comz(0,i+1) = _comz(0,_n_end_walking-1);	   
	   _thetax(0,i) = _thetax(0,_n_end_walking-1);
	   _thetax(0,i+1) = _thetax(0,_n_end_walking-1);	   
	   _thetay(0,i) = _thetay(0,_n_end_walking-1);
	   _thetay(0,i+1) = _thetay(0,_n_end_walking-1);


	   
	  _footx_real_next.row(i+_nT -1)=_footx_real_next.row(_n_end_walking-1+_nT -1)	;
	  _footy_real_next.row(i+_nT -1)=_footy_real_next.row(_n_end_walking-1-_nT -1)	;	   
	   
	   for (int jxxx = _bjxx+1; jxxx<_footstepsnumber; jxxx++){
	   	    _footx_real(jxxx) = _footx_real(_bjxx) ;
	   	    _footy_real(jxxx) = _footy_real(_bjxx-2);		     
	     
	  }
	    _footxyz_real.row(0) = _footx_real.transpose();
	    _footxyz_real.row(1) = _footy_real.transpose();	  
	    _footxyz_real.row(2) = _footz_real.transpose();
  
	   
	}
	 else
	 {
	   
	   _comy(0,i) = _comy(0,_n_end_walking+round(_tstep/_dt/2));
	   _comy(0,i+1) = _comy(0,_n_end_walking+round(_tstep/_dt/2));
	   
	   _comx(0,i) = _comx(0,_n_end_walking-1);
	   _comx(0,i+1) = _comx(0,_n_end_walking-1);
	   _comz(0,i) = _comz(0,_n_end_walking-1);
	   _comz(0,i+1) = _comz(0,_n_end_walking-1);	   
	   _thetax(0,i) = _thetax(0,_n_end_walking-1);
	   _thetax(0,i+1) = _thetax(0,_n_end_walking-1);	   
	   _thetay(0,i) = _thetay(0,_n_end_walking-1);
	   _thetay(0,i+1) = _thetay(0,_n_end_walking-1);
	   
	  _footx_real_next.row(i+_nT -1)=_footx_real_next.row(_n_end_walking-1+_nT -1)	;
	  _footy_real_next.row(i+_nT -1)=_footy_real_next.row(_n_end_walking-1-_nT -1)	;	   
	   
	   for (int jxxx = _bjxx+1; jxxx<_footstepsnumber; jxxx++){
	   	    _footx_real(jxxx) = _footx_real(_bjxx) ;
	   	    _footy_real(jxxx) = _footy_real(_bjxx-2);		     
	     
	  }
	    _footxyz_real.row(0) = _footx_real.transpose();
	    _footxyz_real.row(1) = _footy_real.transpose();	  
	    _footxyz_real.row(2) = _footz_real.transpose();
   
	}
	 
       }
        
  


}
//////////////////////////// modified
int MPCClass::Indexfind(double goalvari, int xyz)
{
        _j_period = 0;
	if (xyz<0.05)
	{
	  while (goalvari >= _tx(_j_period))
	  {
	    _j_period++;
	  }
	  
	  _j_period = _j_period-1;	  
	}
	else
	{
	  while ( fabs(goalvari - _t_f(_j_period)) >0.0001 )
	  {
	    _j_period++;
	  }
	  	  
	}	  
	
// 	return j;
  
}

////////////////////////////modified
int MPCClass::is_sigular(int num)
{
	if ( num & 1)
	{	  
	  return 1; //sigular 
	}
	else
	{
	  return 0;	// odd  	 
	}	   
}

///// only walking once when initialize
Eigen::MatrixXd  MPCClass::Matrix_ps(Eigen::Matrix<double,3,3> a, int nh,Eigen::RowVector3d cxps)
{
//   Eigen::MatrixXd matrixps(nh,3);
  Eigen::MatrixXd matrixps;
  matrixps.setZero(nh,3);  
  
  
  
  Eigen::MatrixXd A;
//   A.setIdentity(a.rows(),a.cols());
  
  for (int i = 0; i < nh; i++) {
    A.setIdentity(a.rows(),a.cols());
    for (int j = 1; j < i+2; j++)
    {
      A = A*a;
    }  
    
     matrixps.middleRows(i, 1)= cxps * A;      
  }
    
  return matrixps;
}


Eigen::MatrixXd MPCClass::Matrix_pu(Eigen::Matrix<double,3,3> a, Eigen::Matrix<double,3,1> b, int nh, Eigen::RowVector3d cxpu)
{
  Eigen::MatrixXd matrixpu;
  matrixpu.setZero(nh,nh);
  
  Eigen::MatrixXd A;
  Eigen::MatrixXd Tempxx;
  
  
  for (int i = 1; i < nh+1; i++) {
    for (int j = 1; j < i+1; j++)
    { 
      A.setIdentity(a.rows(),a.cols());      
      if (j==i)
      {
	Tempxx = cxpu * A * b;
	matrixpu(i-1,j-1) = Tempxx(0,0);
      }
      else
      {	
	for (int k = 1; k < i-j+1; k++)
	{
	  A = A*a;
	}
	Tempxx = cxpu * A * b;
	matrixpu(i-1,j-1) = Tempxx(0,0);
      }          
    }       
  }
    
  return matrixpu;  
}

///DATA SAVING:modified=========================================================
void MPCClass::File_wl()
{
        
// 	CoMMM_ZMP_foot.setZero();
	CoMMM_ZMP_foot.block<1,_nsum>(0, 0) = _comx;
	CoMMM_ZMP_foot.block<1,_nsum>(1, 0) = _comy;	
	CoMMM_ZMP_foot.block<1,_nsum>(2, 0) = _comz;	
	CoMMM_ZMP_foot.block<1,_nsum>(3, 0) = _zmpx_real;	
	CoMMM_ZMP_foot.block<1,_nsum>(4, 0) = _zmpy_real;
	CoMMM_ZMP_foot.block<1,_nsum>(5, 0) = _thetax;	
	CoMMM_ZMP_foot.block<1,_nsum>(6, 0) = _thetay;	
	CoMMM_ZMP_foot.block<1,_nsum>(7, 0) = _torquex_real;
	CoMMM_ZMP_foot.block<1,_nsum>(8, 0) = _torquey_real;
	CoMMM_ZMP_foot.block<1,_nsum>(9, 0) = _footx_real_next1.transpose();	
	CoMMM_ZMP_foot.block<1,_nsum>(10, 0) = _footy_real_next1.transpose();	
	CoMMM_ZMP_foot.block<1,_nsum>(11, 0) = _footz_real_next.transpose();
	
	CoMMM_ZMP_foot.block<1,_nsum>(12, 0) = _Lfootx;	
	CoMMM_ZMP_foot.block<1,_nsum>(13, 0) = _Lfooty;	
	CoMMM_ZMP_foot.block<1,_nsum>(14, 0) = _Lfootz;
	CoMMM_ZMP_foot.block<1,_nsum>(15, 0) = _Rfootx;	
	CoMMM_ZMP_foot.block<1,_nsum>(16, 0) = _Rfooty;	
	CoMMM_ZMP_foot.block<1,_nsum>(17, 0) = _Rfootz;

	CoMMM_ZMP_foot.block<1,_nsum>(18, 0) = _comvx;
	CoMMM_ZMP_foot.block<1,_nsum>(19, 0) = _comax;
	
	CoMMM_ZMP_foot.block<1,_nsum>(20, 0) = _comvy;	
	CoMMM_ZMP_foot.block<1,_nsum>(21, 0) = _comay;
	
	CoMMM_ZMP_foot.block<1,_nsum>(22, 0) = _comvz;	
	CoMMM_ZMP_foot.block<1,_nsum>(23, 0) = _comaz;	

	CoMMM_ZMP_foot.block<1,_nsum>(24, 0) = _thetavx;
	CoMMM_ZMP_foot.block<1,_nsum>(25, 0) = _thetaax;
	
	CoMMM_ZMP_foot.block<1,_nsum>(26, 0) = _thetavy;	
	CoMMM_ZMP_foot.block<1,_nsum>(27, 0) = _thetaay;

	
	
	
	
  
	std::string fileName = "C++_NMPC2018_3robut3_runtime.txt" ;
	std::ofstream outfile( fileName.c_str() ) ; // file name and the operation type. 
       
        for(int i=0; i<_tcpu.rows(); i++){
           for(int j=0; j<_tcpu.cols(); j++){
                 outfile << (double) _tcpu(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();
	
        for(int i=0; i<_tcpu.rows(); i++){
           for(int j=0; j<_tcpu.cols(); j++){
                 outfile << (double) _tcpu(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();	


	std::string fileName1 = "C++_NMPC2018_3robut3_optimal_trajectory.txt" ;
	std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.        
	
        for(int i=0; i<CoMMM_ZMP_foot.rows(); i++){
           for(int j=0; j<CoMMM_ZMP_foot.cols(); j++){
                 outfile1 << (double) CoMMM_ZMP_foot(i,j) << " " ; 
           }
           outfile1 << std::endl;       // a   newline
        }
        outfile1.close();	
	
	
	
}

///// three model MPC solution :modified================================================================
void MPCClass::solve_reactive_step()
{
/*  int nVars = _Nt;
  int nEqCon = 1+3*_nh;
  int nIneqCon = 5*_nh + 4*_nstep+4;  
  resizeQP(nVars, nEqCon, nIneqCon);	*/    

  _G = _Q_goal1;
  _g0 = _q_goal1;
  _X = _V_ini;
	    
  

  _CI.block<_Nt,_nh>(0,0) = _H_q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,_nh) = _H_q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,2*_nh) = _H_q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,3*_nh) = _H_q_lowy.transpose() * (-1); 
  _CI.block<_Nt,_nh>(0,4*_nh) = _H_hacc_lowz.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh) = _H_q_footx_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+_nstep) = _H_q_footx_low.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+2*_nstep) = _H_q_footy_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+3*_nstep) = _H_q_footy_low.transpose() * (-1);  

  
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep) = _Footvx_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep+1) = _Footvx_min.transpose() * (-1);
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep+2) = _Footvy_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,5*_nh+4*_nstep+3) = _Footvy_min.transpose() * (-1);

  

  
  _ci0.block(0, 0,_nh,1) = _F_zmp_upx;
  _ci0.block(_nh, 0,_nh,1) = _F_zmp_lowx;
  _ci0.block(2*_nh, 0,_nh,1) = _F_zmp_upy;
  _ci0.block(3*_nh, 0,_nh,1) = _F_zmp_lowy;
  _ci0.block(4*_nh, 0,_nh,1) = _F_hacc_lowz;
   
  _ci0.block(5*_nh, 0,_nstep,1) = _F_foot_upx;
  _ci0.block(5*_nh+_nstep, 0,_nstep,1) = _F_foot_lowx;
  _ci0.block(5*_nh+2*_nstep, 0,_nstep,1) = _F_foot_upy;
  _ci0.block(5*_nh+3*_nstep, 0,_nstep,1) = _F_foot_lowy;
   
  _ci0.block(5*_nh+4*_nstep, 0,1,1) = _footubxv;
  _ci0.block(5*_nh+4*_nstep+1, 0,1,1) = _footlbxv;
  _ci0.block(5*_nh+4*_nstep+2, 0,1,1) = _footubyv;
  _ci0.block(5*_nh+4*_nstep+3, 0,1,1) = _footlbyv;

  
  _CE.block(0,0, _Nt,1) = _H_q_footz.transpose();
  _CE.block(0,1, _Nt,_nh) = _h_h.transpose();
  _CE.block(0,_nh+1, _Nt,_nh) = _a_hx.transpose();
  _CE.block(0,2*_nh+1, _Nt,_nh) = _a_hy.transpose();
  
  _ce0.block(0,0, 1,1) = _F_footz;
  _ce0.block(1,0, _nh,1) = _hhhx;  
  _ce0.block(1+_nh,0, _nh,1) = _a_hxx;  
  _ce0.block(1+2*_nh,0, _nh,1) = _a_hyy;    
  
  
  Solve();  

}

void MPCClass::solve_reactive_step_body_inclination()
{
/*  int nVars = _Nt;
  int nEqCon = 1+_nh;
  int nIneqCon = 13*_nh + 4*_nstep +4;
  resizeQP(nVars, nEqCon, nIneqCon);*/	    

  _G = _Q_goal1;
  _g0 = _q_goal1;
  _X = _V_ini;

  
  _CI.block<_Nt,_nh>(0,0) = _H_q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,_nh) = _H_q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,2*_nh) = _H_q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,3*_nh) = _H_q_lowy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,4*_nh) = _H_hacc_lowz.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh) = _H_q_footx_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+_nstep) = _H_q_footx_low.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+2*_nstep) = _H_q_footy_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,5*_nh+3*_nstep) = _H_q_footy_low.transpose() * (-1);	    

  _CI.block<_Nt,_nh>(0,5*_nh+4*_nstep) = _q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,6*_nh+4*_nstep) = _q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,7*_nh+4*_nstep) = _q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,8*_nh+4*_nstep) = _q_lowy.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,9*_nh+4*_nstep) = _t_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,10*_nh+4*_nstep) = _t_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,11*_nh+4*_nstep) = _t_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,12*_nh+4*_nstep) = _t_lowy.transpose() * (-1);
  
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep) = _Footvx_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep+1) = _Footvx_min.transpose() * (-1);
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep+2) = _Footvy_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,13*_nh+4*_nstep+3) = _Footvy_min.transpose() * (-1);

  

  
  _ci0.block(0, 0,_nh,1) = _F_zmp_upx;
  _ci0.block(_nh, 0,_nh,1) = _F_zmp_lowx;
  _ci0.block(2*_nh, 0,_nh,1) = _F_zmp_upy;
  _ci0.block(3*_nh, 0,_nh,1) = _F_zmp_lowy; 
  _ci0.block(4*_nh, 0,_nh,1) = _F_hacc_lowz;
  _ci0.block(5*_nh, 0,_nstep,1) = _F_foot_upx;
  _ci0.block(5*_nh+_nstep, 0,_nstep,1) = _F_foot_lowx;
  _ci0.block(5*_nh+2*_nstep, 0,_nstep,1) = _F_foot_upy;
  _ci0.block(5*_nh+3*_nstep, 0,_nstep,1) = _F_foot_lowy;	    
  _ci0.block(5*_nh+4*_nstep, 0,_nh,1) = _qq_upx;
  _ci0.block(6*_nh+4*_nstep, 0,_nh,1) = _qq_lowx;
  _ci0.block(7*_nh+4*_nstep, 0,_nh,1) = _qq_upy;
  _ci0.block(8*_nh+4*_nstep, 0,_nh,1) = _qq_lowy;
  
  _ci0.block(9*_nh+4*_nstep, 0,_nh,1) = _tt_upx;
  _ci0.block(10*_nh+4*_nstep, 0,_nh,1) = _tt_lowx;
  _ci0.block(11*_nh+4*_nstep, 0,_nh,1) = _tt_upy;
  _ci0.block(12*_nh+4*_nstep, 0,_nh,1) = _tt_lowy;
  
  _ci0.block(13*_nh+4*_nstep, 0,1,1) = _footubxv;
  _ci0.block(13*_nh+4*_nstep+1, 0,1,1) = _footlbxv;
  _ci0.block(13*_nh+4*_nstep+2, 0,1,1) = _footubyv;
  _ci0.block(13*_nh+4*_nstep+3, 0,1,1) = _footlbyv;

  
  _CE.block(0,0, _Nt,1) = _H_q_footz.transpose();
  _CE.block(0,1, _Nt,_nh) = _h_h.transpose();
  
  _ce0.block(0,0, 1,1) = _F_footz;
  _ce0.block(1,0, _nh,1) = _hhhx;  

  Solve();  

}

void MPCClass::solve_reactive_step_body_inclination_CoMz()
{
/*  int nVars = _Nt;
  int nEqCon = 1;
  int nIneqCon = 15*_nh + 4*_nstep +4;
  resizeQP(nVars, nEqCon, nIneqCon);	*/    

  _G = _Q_goal1;
  _g0 = _q_goal1;
  _X = _V_ini;

  
  _CI.block<_Nt,_nh>(0,0) = _H_q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,_nh) = _H_q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,2*_nh) = _H_q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,3*_nh) = _H_q_lowy.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,4*_nh) = _H_h_upz.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,5*_nh) = _H_h_lowz.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,6*_nh) = _H_hacc_lowz.transpose() * (-1);
  
  _CI.block<_Nt,_nstep>(0,7*_nh) = _H_q_footx_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,7*_nh+_nstep) = _H_q_footx_low.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,7*_nh+2*_nstep) = _H_q_footy_up.transpose() * (-1);
  _CI.block<_Nt,_nstep>(0,7*_nh+3*_nstep) = _H_q_footy_low.transpose() * (-1);	    

  _CI.block<_Nt,_nh>(0,7*_nh+4*_nstep) = _q_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,8*_nh+4*_nstep) = _q_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,9*_nh+4*_nstep) = _q_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,10*_nh+4*_nstep,_Nt,_nh) = _q_lowy.transpose() * (-1);
  
  _CI.block<_Nt,_nh>(0,11*_nh+4*_nstep) = _t_upx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,12*_nh+4*_nstep) = _t_lowx.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,13*_nh+4*_nstep) = _t_upy.transpose() * (-1);
  _CI.block<_Nt,_nh>(0,14*_nh+4*_nstep) = _t_lowy.transpose() * (-1);
  
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep) = _Footvx_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep+1) = _Footvx_min.transpose() * (-1);
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep+2) = _Footvy_max.transpose() * (-1);
  _CI.block<_Nt,1>(0,15*_nh+4*_nstep+3) = _Footvy_min.transpose() * (-1);

  

  
  _ci0.block(0, 0,_nh,1) = _F_zmp_upx;
  _ci0.block(_nh, 0,_nh,1) = _F_zmp_lowx;
  _ci0.block(2*_nh, 0,_nh,1) = _F_zmp_upy;
  _ci0.block(3*_nh, 0,_nh,1) = _F_zmp_lowy;
  
  _ci0.block(4*_nh, 0,_nh,1) = _F_h_upz;
  _ci0.block(5*_nh, 0,_nh,1) = _F_h_lowz;
  
  _ci0.block(6*_nh, 0,_nh,1) = _F_hacc_lowz;
  
  _ci0.block(7*_nh, 0,_nstep,1) = _F_foot_upx;
  _ci0.block(7*_nh+_nstep, 0,_nstep,1) = _F_foot_lowx;
  _ci0.block(7*_nh+2*_nstep, 0,_nstep,1) = _F_foot_upy;
  _ci0.block(7*_nh+3*_nstep, 0,_nstep,1) = _F_foot_lowy;	    

  _ci0.block(7*_nh+4*_nstep, 0,_nh,1) = _qq_upx;
  _ci0.block(8*_nh+4*_nstep, 0,_nh,1) = _qq_lowx;
  _ci0.block(9*_nh+4*_nstep, 0,_nh,1) = _qq_upy;
  _ci0.block(10*_nh+4*_nstep, 0,_nh,1) = _qq_lowy;
  
  _ci0.block(11*_nh+4*_nstep, 0,_nh,1) = _tt_upx;
  _ci0.block(12*_nh+4*_nstep, 0,_nh,1) = _tt_lowx;
  _ci0.block(13*_nh+4*_nstep, 0,_nh,1) = _tt_upy;
  _ci0.block(14*_nh+4*_nstep, 0,_nh,1) = _tt_lowy;
  
  _ci0.block(15*_nh+4*_nstep, 0,1,1) = _footubxv;
  _ci0.block(15*_nh+4*_nstep+1, 0,1,1) = _footlbxv;
  _ci0.block(15*_nh+4*_nstep+2, 0,1,1) = _footubyv;
  _ci0.block(15*_nh+4*_nstep+3, 0,1,1) = _footlbyv;

  
  
  
  
  
  _CE = _H_q_footz.transpose();
  _ce0 = _F_footz;
  
  
  
  Solve();  

}

void MPCClass::Solve()
{
// min 0.5 * x G x + g0 x
// _s.t.
// 		CE^T x + ce0 = 0
// 		CI^T x + ci0 >= 0
		solveQP();
		if (_X.rows() == _Nt)
		{
		  _V_ini += _X;
		}

}
/////////////////////////////////////////////////////////////////////////////////=================swing foot trajectory==============================================////////////////////////
/////////////============================================================================================================================////////////////////////////////////////////////
//// foot trajectory solve--------polynomial ================================================
void MPCClass::Foot_trajectory_solve(int j_index,bool _stopwalking)
{
  // maximal swing foot height: 
//   double  Footz_ref = _lift_height;
  
  //////////////external push mode==========
//   double  Footz_ref = 0.1;  
  ///// stairs :============================
//    double  Footz_ref = 0.07;  //0.02m
//     double  Footz_ref = 0.09;  //0.05m  
//    double  Footz_ref = 0.1;      //0.06m 
 
//    double  Footz_ref = 0.12;      //0.07m 
//    double  Footz_ref = 0.13;      //0.08m,0.09m 
//    double  Footz_ref = 0.14;      //0.1m,

//// judge if stop  
        if(_stopwalking)  
	{
	  
	  for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
	    _lift_height_ref(i_t) = 0;  
	  }	  

	}  
  
  
  
  _footxyz_real(1,0) = -_stepwidth(0);
  
//   foot trajectory generation:
  if (_bjx1 >= 2)
  {
//     cout << "_bjx1 >= 2"<<endl;
    if (_bjx1 % 2 == 0)           //odd:left support
    {
//     no change on the left support location
      _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);    
  
      /// right swing
      if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double support
      {
// 	cout << "dsp"<<endl;
	_Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
	_Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
	_Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
	
	_Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
	_Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
	_Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);	
	
	
      }
      else
      {
	
// 	cout << "ssp"<<endl;
	//initial state and final state and the middle state
	double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
	Eigen::Vector3d t_plan;
	t_plan(0) = t_des - _dt;
	t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
	t_plan(2) = _ts(_bjx1-1) + 0.0001;
	
	if (abs(t_des - _ts(_bjx1-1)) <= (_dt + 0.0005))
	{
	  _Rfootx(j_index) = _footxyz_real(0,_bjxx); 
	  _Rfooty(j_index) = _footxyz_real(1,_bjxx);
	  _Rfootz(j_index) = _footxyz_real(2,_bjxx); 	
	  
	  _Rfootx(j_index+1) = _footxyz_real(0,_bjxx); 
	  _Rfooty(j_index+1) = _footxyz_real(1,_bjxx);
	  _Rfootz(j_index+1) = _footxyz_real(2,_bjxx); 	  
	  
	}
	else
	{
	  Eigen::Matrix<double,7,7> AAA;
	  AAA.setZero();
	  Eigen::Matrix<double, 1, 7> aaaa;
	  aaaa.setZero();
	  
	  aaaa(0) = 6*pow(t_plan(0), 5);   aaaa(1) =  5*pow(t_plan(0), 4);  aaaa(2) =  4*pow(t_plan(0), 3);   aaaa(3) =  3*pow(t_plan(0), 2);
	  aaaa(4) = 2*pow(t_plan(0), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
	  AAA.row(0) = aaaa;
	  
	  aaaa(0) = 30*pow(t_plan(0), 4);  aaaa(1) =  20*pow(t_plan(0), 3);  aaaa(2) =  12*pow(t_plan(0), 2);   aaaa(3) =  6*pow(t_plan(0), 1);
	  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
	  AAA.row(1) = aaaa;
	  
	  aaaa(0) = pow(t_plan(0), 6);     aaaa(1) =  pow(t_plan(0), 5);     aaaa(2) =  pow(t_plan(0), 4);   aaaa(3) =  pow(t_plan(0), 3);
	  aaaa(4) = pow(t_plan(0), 2);     aaaa(5) = pow(t_plan(0), 1);      aaaa(6) =  1;	  
	  AAA.row(2) = aaaa;
	  
	  aaaa(0) = pow(t_plan(1), 6);     aaaa(1) =  pow(t_plan(1), 5);     aaaa(2) =  pow(t_plan(1), 4);   aaaa(3) =  pow(t_plan(1), 3);
	  aaaa(4) = pow(t_plan(1), 2);     aaaa(5) = pow(t_plan(1), 1);      aaaa(6) =  1;
	  AAA.row(3) = aaaa;
	  
	  aaaa(0) = pow(t_plan(2), 6);     aaaa(1) =  pow(t_plan(2), 5);     aaaa(2) =  pow(t_plan(2), 4);   aaaa(3) =  pow(t_plan(2), 3);
	  aaaa(4) = pow(t_plan(2), 2);     aaaa(5) = pow(t_plan(2), 1);      aaaa(6) =  1;
	  AAA.row(4) = aaaa;	  
	  
	  aaaa(0) = 6*pow(t_plan(2), 5);   aaaa(1) =  5*pow(t_plan(2), 4);  aaaa(2) =  4*pow(t_plan(2), 3);   aaaa(3) =  3*pow(t_plan(2), 2);
	  aaaa(4) = 2*pow(t_plan(2), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
	  AAA.row(5) = aaaa;
	  
	  aaaa(0) = 30*pow(t_plan(2), 4);  aaaa(1) =  20*pow(t_plan(2), 3);  aaaa(2) =  12*pow(t_plan(2), 2);   aaaa(3) =  6*pow(t_plan(2), 1);
	  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
	  AAA.row(6) = aaaa;
	  
	  Eigen::Matrix<double,7,7> AAA_inv;
// 	  AAA_inv.setZero(); 
	  AAA_inv = AAA.inverse();
	 	  
	  Eigen::Matrix<double, 1, 7> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
	  t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
	  

	  Eigen::Matrix<double, 1, 7> t_a_planv;
	  t_a_planv.setZero();
	  t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
	  t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
	  
	  
	  Eigen::Matrix<double, 1, 7> t_a_plana;
	  t_a_plana.setZero();
	  t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
	  t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
	  
// 	  cout<<"AAA="<<endl<<AAA<<endl;
// 	  cout <<"AAA_inverse="<<endl<<AAA.inverse()<<endl;	  
// 	  cout <<"t_des="<<endl<<t_des<<endl;
// 	  cout <<"t_plan="<<endl<<t_plan<<endl;
	  
	  ////////////////////////////////////////////////////////////////////////////
	  Eigen::Matrix<double, 7, 1> Rfootx_plan;
	  Rfootx_plan.setZero();	
	  Rfootx_plan(0) = _Rfootvx(j_index-1);     Rfootx_plan(1) = _Rfootax(j_index-1); Rfootx_plan(2) = _Rfootx(j_index-1); Rfootx_plan(3) = _Lfootx(j_index);
	  Rfootx_plan(4) = _footxyz_real(0,_bjxx);  Rfootx_plan(5) = 0;                   Rfootx_plan(6) = 0;
	  
	  
	  Eigen::Matrix<double, 7, 1> Rfootx_co;
	  Rfootx_co.setZero();
	  Rfootx_co = AAA_inv * Rfootx_plan;
	  
	  _Rfootx(j_index) = t_a_plan * Rfootx_co;
	  _Rfootvx(j_index) = t_a_planv * Rfootx_co;
	  _Rfootax(j_index) = t_a_plana * Rfootx_co;
	  
	  /////////////////////////////////////////////////////////////////////////////
	  if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
	  {
	    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
	  }
	  
	  Eigen::Matrix<double, 7, 1> Rfooty_plan;
	  Rfooty_plan.setZero();	
	  Rfooty_plan(0) = _Rfootvy(j_index-1);     Rfooty_plan(1) = _Rfootay(j_index-1); Rfooty_plan(2) = _Rfooty(j_index-1); Rfooty_plan(3) = _ry_left_right;
	  Rfooty_plan(4) = _footxyz_real(1,_bjxx);  Rfooty_plan(5) = 0;                   Rfooty_plan(6) = 0;	    
	  
	  Eigen::Matrix<double, 7, 1> Rfooty_co;
	  Rfooty_co.setZero();
	  Rfooty_co = AAA_inv * Rfooty_plan;
	  
	  _Rfooty(j_index) = t_a_plan * Rfooty_co;
	  _Rfootvy(j_index) = t_a_planv * Rfooty_co;
	  _Rfootay(j_index) = t_a_plana * Rfooty_co;	
	  
	  
	  //////////////////////////////////////////////////////////
	  Eigen::Matrix<double, 7, 1> Rfootz_plan;
	  Rfootz_plan.setZero();	
	  Rfootz_plan(0) = _Rfootvz(j_index-1);     Rfootz_plan(1) = _Rfootaz(j_index-1); Rfootz_plan(2) = _Rfootz(j_index-1); Rfootz_plan(3) = _Lfootz(j_index)+_lift_height_ref(_bjx1-1);
	  Rfootz_plan(4) = _footxyz_real(2,_bjxx);  Rfootz_plan(5) = 0;                   Rfootz_plan(6) = 0.001;	
	  
	  Eigen::Matrix<double, 7, 1> Rfootz_co;
	  Rfootz_co.setZero();
	  Rfootz_co = AAA_inv * Rfootz_plan;
	  
	  _Rfootz(j_index) = t_a_plan * Rfootz_co;
	  _Rfootvz(j_index) = t_a_planv * Rfootz_co;
	  _Rfootaz(j_index) = t_a_plana * Rfootz_co;	
	 
	  
	  _Rfootx(j_index+1) = _Rfootx(j_index)+_dt * _Rfootvx(j_index);
	  _Rfooty(j_index+1) = _Rfooty(j_index)+_dt * _Rfootvy(j_index);
	  _Rfootz(j_index+1) = _Rfootz(j_index)+_dt * _Rfootvz(j_index);
	  
	  
	  	  
	  
	  
	}
      }
 
      
    }
    
    
    else                       //right support
    {
//       no change on right support
      _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);      
  
      /// left swing
      if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1))  // j_index and _bjx1 coincident with matlab: double suppot
      {
// 	cout << "dsp"<<endl;
	_Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
	_Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
	_Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);

	_Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
	_Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
	_Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
	
      }
      else
      {
// 	cout << "ssp"<<endl;
	//initial state and final state and the middle state
	double t_des = (j_index +1 - round(_tx(_bjx1-1)/_dt) +1)*_dt;
	Eigen::Vector3d t_plan;
	t_plan(0) = t_des - _dt;
	t_plan(1) = (_td(_bjx1-1) + _ts(_bjx1-1))/2 + 0.0001;
	t_plan(2) = _ts(_bjx1-1) + 0.0001;
	
	if (abs(t_des - _ts(_bjx1-1)) <= (_dt + 0.0005))
	{
	  
	  _Lfootx(j_index) = _footxyz_real(0,_bjxx); 
	  _Lfooty(j_index) = _footxyz_real(1,_bjxx);
	  _Lfootz(j_index) = _footxyz_real(2,_bjxx); 

	  _Lfootx(j_index+1) = _footxyz_real(0,_bjxx); 
	  _Lfooty(j_index+1) = _footxyz_real(1,_bjxx);
	  _Lfootz(j_index+1) = _footxyz_real(2,_bjxx); 
	  
	}
	else
	{
	  Eigen::Matrix<double, 7, 7> AAA;
	  AAA.setZero();
	  Eigen::Matrix<double, 1, 7> aaaa;
	  aaaa.setZero();
	  
	  aaaa(0) = 6*pow(t_plan(0), 5);   aaaa(1) =  5*pow(t_plan(0), 4);  aaaa(2) =  4*pow(t_plan(0), 3);   aaaa(3) =  3*pow(t_plan(0), 2);
	  aaaa(4) = 2*pow(t_plan(0), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
	  AAA.row(0) = aaaa;
	  
	  aaaa(0) = 30*pow(t_plan(0), 4);  aaaa(1) =  20*pow(t_plan(0), 3);  aaaa(2) =  12*pow(t_plan(0), 2);   aaaa(3) =  6*pow(t_plan(0), 1);
	  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
	  AAA.row(1) = aaaa;
	  
	  aaaa(0) = pow(t_plan(0), 6);     aaaa(1) =  pow(t_plan(0), 5);     aaaa(2) =  pow(t_plan(0), 4);   aaaa(3) =  pow(t_plan(0), 3);
	  aaaa(4) = pow(t_plan(0), 2);     aaaa(5) = pow(t_plan(0), 1);      aaaa(6) =  1;	  
	  AAA.row(2) = aaaa;
	  
	  aaaa(0) = pow(t_plan(1), 6);     aaaa(1) =  pow(t_plan(1), 5);     aaaa(2) =  pow(t_plan(1), 4);   aaaa(3) =  pow(t_plan(1), 3);
	  aaaa(4) = pow(t_plan(1), 2);     aaaa(5) = pow(t_plan(1), 1);      aaaa(6) =  1;
	  AAA.row(3) = aaaa;
	  
	  aaaa(0) = pow(t_plan(2), 6);     aaaa(1) =  pow(t_plan(2), 5);     aaaa(2) =  pow(t_plan(2), 4);   aaaa(3) =  pow(t_plan(2), 3);
	  aaaa(4) = pow(t_plan(2), 2);     aaaa(5) = pow(t_plan(2), 1);      aaaa(6) =  1;
	  AAA.row(4) = aaaa;	  
	  
	  aaaa(0) = 6*pow(t_plan(2), 5);   aaaa(1) =  5*pow(t_plan(2), 4);  aaaa(2) =  4*pow(t_plan(2), 3);   aaaa(3) =  3*pow(t_plan(2), 2);
	  aaaa(4) = 2*pow(t_plan(2), 1);   aaaa(5) = 1;                     aaaa(6) =  0;
	  AAA.row(5) = aaaa;
	  
	  aaaa(0) = 30*pow(t_plan(2), 4);  aaaa(1) =  20*pow(t_plan(2), 3);  aaaa(2) =  12*pow(t_plan(2), 2);   aaaa(3) =  6*pow(t_plan(2), 1);
	  aaaa(4) = 2;                     aaaa(5) = 0;                      aaaa(6) =  0;
	  AAA.row(6) = aaaa;
	  
          
	  Eigen::Matrix<double,7,7> AAA_inv;
// 	  AAA_inv.setZero(); 
	  AAA_inv = AAA.inverse();
	  
	  Eigen::Matrix<double, 1, 7> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_des, 6);   t_a_plan(1) = pow(t_des, 5);   t_a_plan(2) = pow(t_des, 4);  t_a_plan(3) = pow(t_des, 3);
	  t_a_plan(4) = pow(t_des, 2);   t_a_plan(5) = pow(t_des, 1);   t_a_plan(6) = 1;
	  

	  Eigen::Matrix<double, 1, 7> t_a_planv;
	  t_a_planv.setZero();
	  t_a_planv(0) = 6*pow(t_des, 5);   t_a_planv(1) = 5*pow(t_des, 4);   t_a_planv(2) = 4*pow(t_des, 3);  t_a_planv(3) = 3*pow(t_des, 2);
	  t_a_planv(4) = 2*pow(t_des, 1);   t_a_planv(5) = 1;                 t_a_planv(6) = 0;
	  
	  
	  Eigen::Matrix<double, 1, 7> t_a_plana;
	  t_a_plana.setZero();
	  t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
	  t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
	  
	  
	  ////////////////////////////////////////////////////////////////////////////
	  Eigen::Matrix<double, 7, 1> Lfootx_plan;
	  Lfootx_plan.setZero();	
	  Lfootx_plan(0) = _Lfootvx(j_index-1);     Lfootx_plan(1) = _Lfootax(j_index-1); Lfootx_plan(2) = _Lfootx(j_index-1); Lfootx_plan(3) = _Rfootx(j_index);
	  Lfootx_plan(4) = _footxyz_real(0,_bjxx);  Lfootx_plan(5) = 0;                   Lfootx_plan(6) = 0;	  
	  
	  
	  Eigen::Matrix<double, 7, 1> Lfootx_co;
	  Lfootx_co.setZero();
	  Lfootx_co = AAA_inv * Lfootx_plan;
	  
	  _Lfootx(j_index) = t_a_plan * Lfootx_co;
	  _Lfootvx(j_index) = t_a_planv * Lfootx_co;
	  _Lfootax(j_index) = t_a_plana * Lfootx_co;
	  
	  /////////////////////////////////////////////////////////////////////////////
	  if ((j_index +1 - round(_tx(_bjx1-1)/_dt))*_dt < _td(_bjx1-1)+_dt)
	  {
	    _ry_left_right = (_footxyz_real(1,_bjxx) + _footxyz_real(1,_bjxx-2))/2;
	  }
	  
	  Eigen::Matrix<double, 7, 1> Lfooty_plan;
	  Lfooty_plan.setZero();	
	  Lfooty_plan(0) = _Lfootvy(j_index-1);     Lfooty_plan(1) = _Lfootay(j_index-1); Lfooty_plan(2) = _Lfooty(j_index-1); Lfooty_plan(3) = _ry_left_right;
	  Lfooty_plan(4) = _footxyz_real(1,_bjxx);  Lfooty_plan(5) = 0;                   Lfooty_plan(6) = 0;		  
	  
	  
	  Eigen::Matrix<double, 7, 1> Lfooty_co;
	  Lfooty_co.setZero();
	  Lfooty_co = AAA_inv * Lfooty_plan;
	  
	  _Lfooty(j_index) = t_a_plan * Lfooty_co;
	  _Lfootvy(j_index) = t_a_planv * Lfooty_co;
	  _Lfootay(j_index) = t_a_plana * Lfooty_co;	
	  
	  
	  //////////////////////////////////////////////////////////
	  Eigen::Matrix<double, 7, 1> Lfootz_plan;
	  Lfootz_plan.setZero();		
	  Lfootz_plan(0) = _Lfootvz(j_index-1);     Lfootz_plan(1) = _Lfootaz(j_index-1); Lfootz_plan(2) = _Lfootz(j_index-1); Lfootz_plan(3) = _Rfootz(j_index)+_lift_height_ref(_bjx1-1);
	  Lfootz_plan(4) = _footxyz_real(2,_bjxx);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0.001;		  
	  
	  
	  Eigen::Matrix<double, 7, 1> Lfootz_co;
	  Lfootz_co.setZero();
	  Lfootz_co = AAA_inv * Lfootz_plan;
	  
	  _Lfootz(j_index) = t_a_plan * Lfootz_co;
	  _Lfootvz(j_index) = t_a_planv * Lfootz_co;
	  _Lfootaz(j_index) = t_a_plana * Lfootz_co;
	  
	  
	  _Lfootx(j_index+1) = _Lfootx(j_index)+_dt * _Lfootvx(j_index);
	  _Lfooty(j_index+1) = _Lfooty(j_index)+_dt * _Lfootvy(j_index);
	  _Lfootz(j_index+1) = _Lfootz(j_index)+_dt * _Lfootvz(j_index);
	   
	  
	  
	}
      }

    }
      
  }
  else
  {
    _Rfooty(j_index) = -_stepwidth(0);
    _Lfooty(j_index) = _stepwidth(0);
  }
    
// 	cout << "Rfooty_generated:"<<_Rfooty(j_index)<<endl;
  
}

///////////////////////// ODE - sampling time maximal ========================================
int MPCClass::Get_maximal_number_reference()
{
  return (_nsum -_nh-1);
}

int MPCClass::Get_maximal_number(double dtx)
{
  
  return (_nsum -_nh-1)*floor(_dt/dtx);
}
////====================================================================================================================
/////////////////////////// using the lower-level control-loop  sampling time as the reference: every 5ms;  at the same time: just using the next one position + next one velocity

Vector3d MPCClass::XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
  //reference com position
        _CoM_position_optimal.row(0) = _comx;
	_CoM_position_optimal.row(1) = _comy;
	_CoM_position_optimal.row(2) = _comz;
	_comz(0) = RobotParaClass::Z_C();
	_comz(1) = RobotParaClass::Z_C();
	_comz(2) = RobotParaClass::Z_C();
	_comz(3) = RobotParaClass::Z_C();
	_comz(4) = RobotParaClass::Z_C();
	
	
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
	  t_int = floor(walktime / (_dt / dt_sample) );

	  ///// chage to be relative time
	  double t_cur;
	  t_cur = walktime * dt_sample ;
	  

	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// 	  Eigen::MatrixXd AAA1;	
// 
// 	  AAA1.setZero(4,4);	
// 	  AAA1(0,0) = pow(t_plan(0), 3); AAA1(0,1) = pow(t_plan(0), 2); AAA1(0,2) = pow(t_plan(0), 1); AAA1(0,3) = pow(t_plan(0), 0); 
// 	  AAA1(1,0) = pow(t_plan(1), 3); AAA1(1,1) = pow(t_plan(1), 2); AAA1(1,2) = pow(t_plan(1), 1); AAA1(1,3) = pow(t_plan(0), 0); 
// 	  AAA1(2,0) = pow(t_plan(2), 3); AAA1(2,1) = pow(t_plan(2), 2); AAA1(2,2) = pow(t_plan(2), 1); AAA1(2,3) = pow(t_plan(0), 0); 
// 	  AAA1(3,0) = 3*pow(t_plan(2), 2); AAA1(3,1) = 2*pow(t_plan(2), 1); AAA1(3,2) = pow(t_plan(2), 0); AAA1(3,3) = 0;  


	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  	  
	  
	  

	
	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 = _CoM_position_optimal.col(t_int);
// 	  x13 = _CoM_position_optimal.col(t_int+1);
	  
	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _comvx(t_int);	  
	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _comvy(t_int);	  
	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _comvz(t_int);	  
	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

  
	  
	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);	
	  
	}

 	return com_inte;
	
}

Vector3d MPCClass::XGetSolution_body_inclination(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
  //reference com position
        _torso_angle_optimal.row(0) = _thetax;
	_torso_angle_optimal.row(1) = _thetay;
	_torso_angle_optimal.row(2) = _thetaz;
	
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
	  t_int = floor(walktime / (_dt / dt_sample) );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// 	  Eigen::MatrixXd AAA;	
// 
// 	  AAA.setZero(4,4);	
// 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
	  
	  
	
	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);   t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 = _torso_angle_optimal.col(t_int);
	  x13 = _torso_angle_optimal.col(t_int+1);
	  
	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _thetavx(t_int);	  
	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _thetavy(t_int);	  
	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _thetavz(t_int);	  
	  com_inte(2) = t_a_plan *(AAA_inv)*temp;

	  
	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);	
	  
	}

 	return com_inte;
	
  
  
  
}


Vector3d MPCClass::XGetSolution_Foot_positionR(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
	
        _R_foot_optition_optimal.row(0) = _Rfootx;
	_R_foot_optition_optimal.row(1) = _Rfooty;
	_R_foot_optition_optimal.row(2) = _Rfootz;
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
	  t_int = floor(walktime / (_dt / dt_sample) );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// 	  Eigen::MatrixXd AAA;
// 
// 	  
// 	  AAA.setZero(4,4);	
// 	  AAA(0,0) = pow(t_plan(0), 3); AAA(0,1) = pow(t_plan(0), 2); AAA(0,2) = pow(t_plan(0), 1); AAA(0,3) = pow(t_plan(0), 0); 
// 	  AAA(1,0) = pow(t_plan(1), 3); AAA(1,1) = pow(t_plan(1), 2); AAA(1,2) = pow(t_plan(1), 1); AAA(1,3) = pow(t_plan(0), 0); 
// 	  AAA(2,0) = pow(t_plan(2), 3); AAA(2,1) = pow(t_plan(2), 2); AAA(2,2) = pow(t_plan(2), 1); AAA(2,3) = pow(t_plan(0), 0); 
// 	  AAA(3,0) = 3*pow(t_plan(2), 2); AAA(3,1) = 2*pow(t_plan(2), 1); AAA(3,2) = pow(t_plan(2), 0); AAA(3,3) = 0;  


	  
	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
	  	  
	  

	
	  
	  
	  Eigen::Matrix<double, 1, 4> t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);  
	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1);  
	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 =  _R_foot_optition_optimal.col(t_int);
	  x13 =  _R_foot_optition_optimal.col(t_int+1);
	  
	  
	  Eigen::Matrix<double, 4, 1>  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0); temp(3) = _Rfootvx(t_int);	  
	  com_inte(0) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1); temp(3) = _Rfootvy(t_int);

// 	  cout << "Rfooty:"<<temp<<endl;
	  com_inte(1) = t_a_plan * (AAA_inv)*temp;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); temp(3) = _Rfootvz(t_int);	  
	  com_inte(2) = t_a_plan *(AAA_inv)*temp;
	  
////=====================================================////////////////////////////////////////////////
// 	  //////spline for Rfoot_
	  
	  
	  
	  
	  
	  
	  
	  
	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);

// 	  com_inte = Rfoot_IN.col(walktime);	  
	}
	
// 	cout << "Rfooty_generated:"<<com_inte(1)<<endl;

 	return com_inte;
	
	
	
	
}

Vector3d MPCClass::XGetSolution_Foot_positionL(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
        _L_foot_optition_optimal.row(0) = _Lfootx;
	_L_foot_optition_optimal.row(1) = _Lfooty;
	_L_foot_optition_optimal.row(2) = _Lfootz;
	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
	  t_int = floor(walktime / (_dt / dt_sample) );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);


/*	  Eigen::MatrixXd  AAAaaa; /// should be Marix4d
	  AAAaaa.setZero(4,4);
	  
	  AAAaaa(0,0) = pow(t_plan(0), 3); AAAaaa(0,1) = pow(t_plan(0), 2); AAAaaa(0,2) = pow(t_plan(0), 1); AAAaaa(0,3) = pow(t_plan(0), 0); 
	  AAAaaa(1,0) = pow(t_plan(1), 3); AAAaaa(1,1) = pow(t_plan(1), 2); AAAaaa(1,2) = pow(t_plan(1), 1); AAAaaa(1,3) = pow(t_plan(1), 0); 
	  AAAaaa(2,0) = pow(t_plan(2), 3); AAAaaa(2,1) = pow(t_plan(2), 2); AAAaaa(2,2) = pow(t_plan(2), 1); AAAaaa(2,3) = pow(t_plan(2), 0); 
         // AAAaaa(3,0) = pow(t_plan(3), 3); AAAaaa(3,1) = pow(t_plan(3), 2); AAAaaa(3,2) = pow(t_plan(3), 1); AAAaaa(3,3) = pow(t_plan(3), 0); /// using the next to positioin would cause over-fitting	  
	  AAAaaa(3,0) = 3*pow(t_plan(2), 2); AAAaaa(3,1) = 2*pow(t_plan(2), 1); AAAaaa(3,2) = pow(t_plan(2), 0); AAAaaa(3,3) = 0.0;  	  
 */	  
//       MatrixXd.inverse( != Matrix4d (under Xd =4. so write the inverse of Matrix4d explicitly): for the time being)
	  
	  
	  Eigen::Matrix4d AAA_inv;
	  
	  double abx1, abx2, abx3, abx4;
	  abx1 = ((t_plan(0) - t_plan(1))*pow(t_plan(0) - t_plan(2), 2));
	  abx2 = ((t_plan(0) - t_plan(1))*pow(t_plan(1) - t_plan(2), 2));
	  abx3 =(pow(t_plan(0) - t_plan(2), 2)*pow(t_plan(1) - t_plan(2), 2));
	  abx4 = ((t_plan(0) - t_plan(2))*(t_plan(1) - t_plan(2)));
	  

	  AAA_inv(0,0) = 1/ abx1;
	  AAA_inv(0,1) =  -1/ abx2;
	  AAA_inv(0,2) = (t_plan(0) + t_plan(1) - 2*t_plan(2))/ abx3;
	  AAA_inv(0,3) = 1/ abx4;
	  
	  AAA_inv(1,0) = -(t_plan(1) + 2*t_plan(2))/ abx1;
	  AAA_inv(1,1) = (t_plan(0) + 2*t_plan(2))/ abx2;
	  AAA_inv(1,2) = -(pow(t_plan(0), 2) + t_plan(0)*t_plan(1) + pow(t_plan(1), 2) - 3*pow(t_plan(2), 2))/ abx3;
	  AAA_inv(1,3) = -(t_plan(0) + t_plan(1) + t_plan(2))/ abx4;
	  
	  AAA_inv(2,0) = (t_plan(2)*(2*t_plan(1) + t_plan(2)))/ abx1;
	  AAA_inv(2,1) = -(t_plan(2)*(2*t_plan(0) + t_plan(2)))/ abx2;
	  AAA_inv(2,2) = (t_plan(2)*(2*pow(t_plan(0), 2) + 2*t_plan(0)*t_plan(1) - 3*t_plan(2)*t_plan(0) + 2*pow(t_plan(1), 2) - 3*t_plan(2)*t_plan(1)))/ abx3;
	  AAA_inv(2,3) = (t_plan(0)*t_plan(1) + t_plan(0)*t_plan(2) + t_plan(1)*t_plan(2))/ abx4;
	  
	  AAA_inv(3,0) = -(t_plan(1)*pow(t_plan(2), 2))/ abx1;
	  AAA_inv(3,1) = (t_plan(0)*pow(t_plan(2), 2))/ abx2;
	  AAA_inv(3,2) = (t_plan(0)*t_plan(1)*(t_plan(0)*t_plan(1) - 2*t_plan(0)*t_plan(2) - 2*t_plan(1)*t_plan(2) + 3*pow(t_plan(2), 2)))/ abx3;
	  AAA_inv(3,3) = -(t_plan(0)*t_plan(1)*t_plan(2))/ abx4;
	  	  
	  
	  
	  
	  
	
// // 	  Eigen::Matrix4d  AAAaaa1_inv=AAA_inv;
	  
// 	  cout<< t_plan<<endl;
// 	  cout<< AAAaaa<<endl;
// 	  cout<< AAA_inv - AAAaaa1_inv<<endl;
// 	  cout<< AAA_inv - AAAaaa1.inverse()<<endl;
	  
	  Eigen::RowVector4d t_a_plan;
	  t_a_plan.setZero();
	  t_a_plan(0) = pow(t_cur-( t_cur - 2*dt_sample), 3);  
	  t_a_plan(1) = pow(t_cur-( t_cur - 2*dt_sample), 2);   
	  t_a_plan(2) = pow(t_cur-( t_cur - 2*dt_sample), 1); 
	  t_a_plan(3) = pow(t_cur-( t_cur - 2*dt_sample), 0); 


	  
	  // COM&&foot trajectory interpolation
	  
	  Eigen::Vector3d  x10;
	  Eigen::Vector3d  x11;
	  Eigen::Vector3d  x12;
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 =  _L_foot_optition_optimal.col(t_int);
// 	  x13 =  _L_foot_optition_optimal.col(t_int+1); /// using next two position would caused over-fitting
	  
	  
	  Eigen::Vector4d  temp;
	  temp.setZero();
	  temp(0) = x10(0); temp(1) = x11(0); temp(2) = x12(0);
	  temp(3) = _Lfootvx(t_int);
// 	  	  temp(3) = x13(0);
	  Eigen::Vector4d tmp111 = AAA_inv*temp;
	  com_inte(0) = t_a_plan * tmp111;
	  temp(0) = x10(1); temp(1) = x11(1); temp(2) = x12(1);
	  temp(3) = _Lfootvy(t_int);	 
/*          temp(3) = x13(1);	*/  
	  tmp111 = AAA_inv*temp;  
	  com_inte(1) = t_a_plan * tmp111;
	  temp(0) = x10(2); temp(1) = x11(2); temp(2) = x12(2); 
	  temp(3) = _Lfootvz(t_int);	
// 	  temp(3) = x13(2);
	  tmp111 = AAA_inv*temp;	  
	  com_inte(2) = t_a_plan * tmp111;

////================================================not use spline.h=====////////////////////////////////////////////////


	  
	}
	else
	{
	  com_inte(0) = body_in3(0);	  
	  com_inte(1) = body_in3(1);	  	  
	  com_inte(2) = body_in3(2);
	}

 	return com_inte;
	
	cout << "Lfooty_generated:"<<com_inte(1)<<endl;
}






