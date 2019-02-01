/*****************************************************************************
MPCClass.cpp

Description:    source file of MPCClass

@Version:   1.0
@Author:    Chengxu Zhou (zhouchengxu@gmail.com)
@Release:   Thu 02 Aug 2018 12:33:23 PM CEST
@Update:    Thu 02 Aug 2018 12:33:19 PM CEST
*****************************************************************************/
#include "MPC/MPCClass.h"
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
{
  
}


void MPCClass::FootStepInputs( double stepwidth, double steplength, double stepheight)
{	
	_steplength.setConstant(steplength);
	_steplength(0) = 0;

	
	
	_stepwidth.setConstant(stepwidth);
	_stepwidth(0) = _stepwidth(0)/2;	
	
	_stepheight.setConstant(stepheight);
	
	_lift_height_ref.setConstant(_lift_height);

}


void MPCClass::Initialize()
{
        // ==step parameters initialize==: given by the inputs
        ////NP initialization for step timing optimization
       //// reference steplength and stepwidth for timing optimization  
 	// ==step loctions setup==
        _Lxx_ref = _steplength;
        _Lyy_ref = _stepwidth;	   
        // local coordinate
	_Lyy_ref(0) = 0;
	for (int j =0; j<_footstepsnumber;j++)
	{
	  _Lyy_ref(j) = (int)pow(-1,j)*_stepwidth(j);
	}
  
// 	reference footstep locations setup
        _footx_ref.setZero();
	_footy_ref.setZero();
	_footz_ref.setZero();		
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _footx_ref(i) = _footx_ref(i-1) + _steplength(i-1);
	  _footy_ref(i) = _footy_ref(i-1) + (int)pow(-1,i-1)*_stepwidth(i-1);   
	  _footz_ref(i) = _footz_ref(i-1) + _stepheight(i-1);
	}
	_footx_offline = _footx_ref;
	_footy_offline = _footy_ref;
	_footz_offline = _footz_ref;
	
// 	cout<<"_footy_ref:"<<_footy_ref<<endl;
	
	// == step cycle setup
	_ts.setConstant(_tstep);
	_ts(5) = 0.3;
	_ts(6) = 0.3;	
	_ts(7) = 0.3;	
	_td = 0.2*_ts;
	_tx.setZero();
  	for (int i = 1; i < _footstepsnumber; i++) {
 	  _tx(i) = _tx(i-1) + _ts(i-1);
	  _tx(i) = round(_tx(i)/_dt)*_dt -0.000001;	  
	}	

	//whole sampling time sequnece       
	_t.setLinSpaced(round(_tx(_footstepsnumber-1)/_dt),_dt,_tx(_footstepsnumber-1));

	//parameters
        _hcom = RobotParaClass::Z_C();	
//         _hcom = 0.4668;
	_g = RobotParaClass::G();	
	_Wn = sqrt(_g/_hcom);
	
	_Wndt = _Wn*_dt;	
	
        // COM state
	_comx.setZero(); _comvx.setZero(); _comax.setZero();
	_comy.setZero(); _comvy.setZero(); _comay.setZero();
	_comz.setConstant(_hcom); _comvz.setZero(); _comaz.setZero();
        ///actual steplengh,stepwidth and walking period
	_Lxx_ref_real.setZero(); 
	_Lyy_ref_real.setZero(); 
	_Ts_ref_real.setZero();
	

	

	
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% parameters for first MPC-step timing adjustment and next one step location
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%	
	_px.setZero(); _py.setZero(); _pz.setZero();
	_zmpvx.setZero(); _zmpvy.setZero(); 
	_COMx_is.setZero(); _COMx_es.setZero(); _COMvx_is.setZero(); 
	_COMy_is.setZero(); _COMy_es.setZero(); _COMvy_is.setZero();
	_comx_feed.setZero(); _comvx_feed.setZero(); _comax_feed.setZero();
	_comy_feed.setZero(); _comvy_feed.setZero(); _comay_feed.setZero();
	
	_Vari_ini.setZero(); //Lxx,Lyy,Tr1,Tr2,Lxx1,Lyy1,Tr11,Tr21;
	_vari_ini.setZero();


	
	
	
	
	//step timing constraints:
	_t_min = 0.4; _t_max = 2;
	
	// swing foot velocity constraints	
	_footx_vmax = 15;
	_footx_vmin = -15;
	_footy_vmax = 15;
	_footy_vmin = -15;		
	
	// CoM acceleration velocity:
	_comax_max = 6.5;
	_comax_min = -6.5;
	_comay_max = 6;
	_comay_min = -6;	

	
	if(_robot_name == "coman"){
	  _rad = 0.1; 	  
	  	  
	//weight coefficient
	_aax = 5000000;          _aay =5000000;
	_aaxv =1000;             _aayv=1000;
	_bbx = 50000000;         _bby =50000000;
	_rr1 = 5000000000;       _rr2 =5000000000; 
	_aax1 =100;             _aay1 =100;
	_bbx1 =10;              _bby1 =10;
	_rr11 =1000;            _rr21 =1000; 	  
	  
/*	  /// zmp-constraints	
	  _zmpx_ub=0.07;  
	  _zmpx_lb=-0.03;
	  _zmpy_ub=0.05; 
	  _zmpy_lb=-0.05;*/	

        //  foot location constraints 
	_footx_max=0.3;
	_footx_min=-0.2;
// 	_footy_max=2*RobotParaClass::HALF_HIP_WIDTH() + 0.2; 
// 	_footy_min=RobotParaClass::HALF_HIP_WIDTH() - 0.03;
	}
	else if (_robot_name == "bigman")
	{
	  _rad = 0.2; 	
	  
	//weight coefficient
	_aax = 5000000;          _aay =5000000;
	_aaxv =1000;             _aayv=1000;
	_bbx = 50000000;         _bby =50000000;
	_rr1 = 5000000000;       _rr2 =5000000000; 
	_aax1 =100;             _aay1 =100;
	_bbx1 =10;              _bby1 =10;
	_rr11 =1000;            _rr21 =1000; 	  
	  	  
/*	  /// zmp-constraints	
	  _zmpx_ub=(RobotParaClass::FOOT_LENGTH()/2+RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio;  
	  _zmpx_lb=(-(RobotParaClass::FOOT_LENGTH()/2-RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio);
	  _zmpy_ub=(RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio); 
	  _zmpy_lb=(-RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio);*/	

        //  foot location constraints 
	_footx_max=0.5;
	_footx_min=-0.2;
// 	_footy_max=2*RobotParaClass::HALF_HIP_WIDTH() + 0.2; 
// 	_footy_min=RobotParaClass::HALF_HIP_WIDTH() - 0.03;

	}
	else if (_robot_name == "cogimon")
        {	  
	  _rad = 0.2; 	 
	  
	//weight coefficient
	_aax = 5000000;          _aay =5000000;
	_aaxv =1000;             _aayv=1000;
	_bbx = 50000000;         _bby =50000000;
	_rr1 = 5000000000;       _rr2 =5000000000; 
	_aax1 =100;             _aay1 =100;
	_bbx1 =10;              _bby1 =10;
	_rr11 =1000;            _rr21 =1000; 	  	  

/*	  /// zmp-constraints	
	  _zmpx_ub=(RobotParaClass::FOOT_LENGTH()/2+RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio;  
	  _zmpx_lb=(-(RobotParaClass::FOOT_LENGTH()/2-RobotParaClass::HIP_TO_ANKLE_X_OFFSET())*_ZMP_ratio);
	  _zmpy_ub=(RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio); 
	  _zmpy_lb=(-RobotParaClass::FOOT_WIDTH()/2*_ZMP_ratio);*/

        //  foot location constraints 
	_footx_max=0.4;
	_footx_min=-0.2;
// 	_footy_max=2*RobotParaClass::HALF_HIP_WIDTH() + 0.2; 
// 	_footy_min=RobotParaClass::HALF_HIP_WIDTH() - 0.03; 
	  
        } 
        else {
	  DPRINTF("Errorrrrrrrr for IK\n");}		
	
	
	_footy_max=2*RobotParaClass::HALF_HIP_WIDTH() + 0.2; 
	_footy_min=RobotParaClass::HALF_HIP_WIDTH() - 0.03; 	
	
	_mass = _robot_mass; 
	_j_ini = _mass* pow(_rad,2);	
	
	///external force
	_FX =160;  _FY =120;
	_t_last = 0.5;
	_det_xa = _FX/_mass;  _det_ya = _FY/_mass; 
	_det_xv = _det_xa*_t_last; _det_yv = _det_ya*_t_last;
	_det_xp = pow(_det_xv,2)/(2*_det_xa); _det_yp = pow(_det_yv,2)/(2*_det_ya);
	
	
	
	
	_tcpu.setZero();
	
	_n_loop_omit = 2*round(_tstep/_dt);
	
	xyz0 = -1; //flag for find function 
	xyz1 = 0;  
	xyz2 = 1;
		
	_periond_i = 0;  /// period number falls into which cycle 
	_ki = 0;
	_k_yu = 0;
	_Tk = 0; 
	
	/// remaining time boundaries
	_tr1_min=0;   _tr2_min=0;  _tr1_max=0;  _tr2_max=0;
	_tr11_min=0;  _tr21_min=0; _tr11_max=0; _tr21_max=0;	
	
	/// selection matrix for variables
        _SS1.setZero(); _SS1(0) =1;
	_SS2.setZero(); _SS2(1) =1;
	_SS3.setZero(); _SS3(2) =1;
	_SS4.setZero(); _SS4(3) =1;
	_SS5.setZero(); _SS5(4) =1;
	_SS6.setZero(); _SS6(5) =1;
	_SS7.setZero(); _SS7(6) =1;
	_SS8.setZero(); _SS8(7) =1;

/*	cout << _SS1<<endl;	cout << _SS2<<endl;
	cout << _SS3<<endl;	cout << _SS4<<endl;
	cout << _SS5<<endl;	cout << _SS6<<endl;
	cout << _SS7<<endl;
	cout << _SS8<<endl;*/		
	
	
	_comvx_endref.setZero();
	_comvy_endref.setZero();
	
	_AxO.setZero();_BxO.setZero();_Cx.setZero(); 
	_Axv.setZero();_Bxv.setZero();_Cxv.setZero();
	_AyO.setZero();_ByO.setZero();_Cy.setZero(); 
	_Ayv.setZero();_Byv.setZero();_Cyv.setZero();	

	_SQ_goal0.setZero();	
	_SQ_goal.setZero();
	_SQ_goal1.setZero();
	_SQ_goal20.setZero();
	_SQ_goal2.setZero();	
	_SQ_goal3.setZero();	
	_Sq_goal.setZero();
	_Sq_goal1.setZero();	
	_Sq_goal2.setZero();
	_Sq_goal3.setZero();	
	_Ax.setZero(); 	_Ay.setZero();
	_Bx.setZero();  _By.setZero();
	_ixi.setZero(); _iyi.setZero();
	
	/// remaining time constraints: inequality constraints
	_trx1_up.setZero();  _trx1_lp.setZero();
	_trx2_up.setZero();  _trx2_lp.setZero();	
	_trx3_up.setZero();  _trx3_lp.setZero();	
	_trx4_up.setZero();  _trx4_lp.setZero();	
	_det_trx1_up.setZero();  _det_trx1_lp.setZero();
	_det_trx2_up.setZero();  _det_trx2_lp.setZero();	
	_det_trx3_up.setZero();  _det_trx3_lp.setZero();	
	_det_trx4_up.setZero();  _det_trx4_lp.setZero();	
	
	_trx.setZero();  _det_trx.setZero();
	
	//// tr1&tr2:equation constraints
	_trx12.setZero();     _trx121.setZero();	
	_det_trx12.setZero(); _det_trx121.setZero();		
	_trxx.setZero();      _det_trxx.setZero();	
	

	
	_h_lx_up.setZero();  _h_lx_lp.setZero(); _h_ly_up.setZero(); _h_ly_lp.setZero();
	_h_lx_up1.setZero(); _h_lx_lp1.setZero(); _h_ly_up1.setZero(); _h_ly_lp1.setZero();
	_det_h_lx_up.setZero();_det_h_lx_lp.setZero();_det_h_ly_up.setZero();_det_h_ly_lp.setZero();
	_det_h_lx_up1.setZero();_det_h_lx_lp1.setZero();_det_h_ly_up1.setZero();_det_h_ly_lp1.setZero();
	_h_lx_upx.setZero(); _det_h_lx_upx.setZero();
	
	// swing foot velocity constraints
	_h_lvx_up.setZero();  _h_lvx_lp.setZero(); _h_lvy_up.setZero(); _h_lvy_lp.setZero();
	_h_lvx_up1.setZero(); _h_lvx_lp1.setZero(); _h_lvy_up1.setZero(); _h_lvy_lp1.setZero();
	_det_h_lvx_up.setZero();_det_h_lvx_lp.setZero();_det_h_lvy_up.setZero();_det_h_lvy_lp.setZero();
	_det_h_lvx_up1.setZero();_det_h_lvx_lp1.setZero();_det_h_lvy_up1.setZero();_det_h_lvy_lp1.setZero();	
	_h_lvx_upx.setZero(); _det_h_lvx_upx.setZero();	
	
	// CoM acceleration boundary
	_AA = 0; _CCx=0; _BBx=0; _CCy=0; _BBy=0;_AA1x=0;_AA2x=0;_AA3x=0;_AA1y=0;_AA2y=0;_AA3y=0;
	_CoM_lax_up.setZero();  _CoM_lax_lp.setZero();  _CoM_lay_up.setZero();  _CoM_lay_lp.setZero();
	_det_CoM_lax_up.setZero();  _det_CoM_lax_lp.setZero();  _det_CoM_lay_up.setZero();  _det_CoM_lay_lp.setZero();
	_CoM_lax_upx.setZero();
	_det_CoM_lax_upx.setZero();		
	
	
	
	/// CoM velocity_inremental boundary
	_VAA=0; _VCCx=0; _VBBx=0; _VCCy=0; _VBBy=0;_VAA1x=0;_VAA2x=0;_VAA3x=0;_VAA1y=0;_VAA2y=0;_VAA3y=0;
	_CoM_lvx_up.setZero();  _CoM_lvx_lp.setZero();  _CoM_lvy_up.setZero();  _CoM_lvy_lp.setZero();
	_det_CoM_lvx_up.setZero();  _det_CoM_lvx_lp.setZero();  _det_CoM_lvy_up.setZero();  _det_CoM_lvy_lp.setZero();
	_CoM_lvx_upx.setZero();
	_det_CoM_lvx_upx.setZero();	
	
	
	
	/// CoM initial velocity_ boundary
	_VAA1x1=0;_VAA2x1=0;_VAA3x1=0;_VAA1y1=0;_VAA2y1=0;_VAA3y1=0;
	_CoM_lvx_up1.setZero();  _CoM_lvx_lp1.setZero();  _CoM_lvy_up1.setZero();  _CoM_lvy_lp1.setZero();
	_det_CoM_lvx_up1.setZero();  _det_CoM_lvx_lp1.setZero();  _det_CoM_lvy_up1.setZero();  _det_CoM_lvy_lp1.setZero();
	_CoM_lvx_upx1.setZero();
	_det_CoM_lvx_upx1.setZero();	
	
	
	///////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	///%%%%%%%%%%%%% foot trajectory geneartion
	//////////////%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        _t_f.setZero();	
	_bjx1 = 0;
	_bjxx = 0;
	_footxyz_real.setZero();
	
	
	_Lfootx.setZero(); _Lfooty.setConstant(_stepwidth(0));_Lfootz.setZero(); _Lfootvx.setZero(); _Lfootvy.setZero();_Lfootvz.setZero(); 
	_Lfootax.setZero(); _Lfootay.setZero();_Lfootaz.setZero();
	_Rfootx.setZero(); _Rfooty.setConstant(-_stepwidth(0));_Rfootz.setZero(); _Rfootvx.setZero(); _Rfootvy.setZero();_Rfootvz.setZero(); 
	_Rfootax.setZero(); _Rfootay.setZero();_Rfootaz.setZero();	
	_ry_left_right = 0;	
	
	
	 _CoM_position_optimal.setZero();
	 _torso_angle_optimal.setZero();
	 _L_foot_optition_optimal.setZero();
	 _R_foot_optition_optimal.setZero();
	 _foot_location_optimal.setZero();	
	
	
	
	
	
	
	
	
	
	
	
	 
	 
	
	
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% pamameters for second MPC- ZMP movement, height variance and body inclination
	// %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
	// %% robot parameters		
	_zmpx_real.setZero(_nsum); _zmpy_real.setZero(_nsum);	
	_thetax.setZero(_nsum); _thetavx.setZero(_nsum); _thetaax.setZero(_nsum);
	_thetay.setZero(_nsum); _thetavy.setZero(_nsum); _thetaay.setZero(_nsum);
	_thetaz.setZero(_nsum); _thetavz.setZero(_nsum); _thetaaz.setZero(_nsum);

	
	_torquex_real.setZero(_nsum); _torquey_real.setZero(_nsum);



	 
	
	_xk.setZero(3,_nsum); _yk.setZero(3,_nsum); _zk.setZero(3,_nsum);
	_thetaxk.setZero(3,_nsum); _thetayk.setZero(3,_nsum);
	_x_vacc_k.setZero(_nsum); _y_vacc_k.setZero(_nsum); _z_vacc_k.setZero(_nsum); 
	_thetax_vacc_k.setZero(_nsum); _thetay_vacc_k.setZero(_nsum); 
	
        // ==initial parameters for MPC==
	_ggg.setConstant(1, 9.8);

	_Hcom1.setConstant(_nh,1,_hcom);
        	
	_a.setZero(3,3);
	_a << 1, _dt, pow(_dt,2)/2,    
	      0,   1,            _dt,
	      0,   0,              1;
	_b.setZero(3,1);
	_b << pow(_dt,3)/6,    
	      pow(_dt,2)/2,
	               _dt;	
	_c.setZero(1,3);
	_c << 1,0,(-1)*_hcom /_g;	
	
	_cp.setZero(1,3);
	_cp(0,0) = 1;
	_cv.setZero(1,3);
	_cv(0,1) = 1;
	_ca.setZero(1,3);
	_ca(0,2) = 1;	
		

	//vertical height constraints
	_z_max.setConstant(_nsum,0.1);
	_z_min.setConstant(_nsum,-0.1);	
	
        //footz refer: height of step
	_Zsc.setZero(_nsum,1);		



	
  	for (int i = 0; i < _nsum-1; i++) {
          Indexfind(_t(i),xyz1);	  
	  _Zsc(i) = _footz_ref(_j_period);   
	  _j_period = 0;   
	}		

        _yk.topRows(1).setConstant(_footy_ref(0)); 
	_zk.topRows(1).setConstant(_hcom);
	

	//predictive model
	_pps.setZero(_nh,3); _ppu.setZero(_nh,_nh);
	_pvs.setZero(_nh,3); _pvu.setZero(_nh,_nh);
	_pas.setZero(_nh,3); _pau.setZero(_nh,_nh);

        _pps = Matrix_ps(_a,_nh,_cp);
	_pvs = Matrix_ps(_a,_nh,_cv);
	_pas = Matrix_ps(_a,_nh,_ca);

	_ppu = Matrix_pu(_a,_b,_nh,_cp);
	_pvu = Matrix_pu(_a,_b,_nh,_cv);
	_pau = Matrix_pu(_a,_b,_nh,_ca);

	
	_footx_real.setZero(_footstepsnumber);  _footy_real.setZero(_footstepsnumber); _footz_real.setZero(_footstepsnumber);
	

	



	
	_footx_real_next.setZero(_nsum);  _footy_real_next.setZero(_nsum); _footz_real_next.setZero(_nsum);
	_footx_real_next1.setZero(_nsum);  _footy_real_next1.setZero(_nsum); _footz_real_next1.setZero(_nsum);
	
	


	
	
	
	

	
	_fx.setZero(1);
	_fy.setZero(1);

	_fxx_global.setZero(1);
	_fyy_global.setZero(1);	
	
	
	/// zmp-constraints
// 	_zmpx_ub.setConstant(_nsum,0.07);  _zmpx_lb.setConstant(_nsum,-0.03);
// 	_zmpy_ub.setConstant(_nsum,0.05); _zmpy_lb.setConstant(_nsum,-0.05);
// 	
	_zmpx_ub.setConstant(_nsum,0.07);  _zmpx_lb.setConstant(_nsum,-0.03);
	_zmpy_ub.setConstant(_nsum,0.05); _zmpy_lb.setConstant(_nsum,-0.05);
		
	
	// com-support range
	_comx_max.setConstant(1,0.06);
	_comx_min.setConstant(1,-0.04);  
	_comy_max.setConstant(1,0.6);  
	_comy_min.setConstant(1,0.02);
	
	// angle range
	_thetax_max.setConstant(1,10*M_PI/180);  
	_thetax_min.setConstant(1,-5*M_PI/180);
	_thetay_max.setConstant(1,10*M_PI/180);  
	_thetay_min.setConstant(1,-10*M_PI/180);
	
	// torque range
	_torquex_max.setConstant(1,80/_j_ini); 
	_torquex_min.setConstant(1,-60/_j_ini);
	_torquey_max.setConstant(1,80/_j_ini);  
	_torquey_min.setConstant(1,-80/_j_ini);	

	
	
	
///===========initiallize: preparation for MPC solution
// 	_nstep = 2;
// 	_Nt = 5*_nh + 3*_nstep;	
	
	// sulotion preparation		
	_V_optimal.setZero(_Nt, _nsum);	
	_Lx_ref.setZero(_nstep); _Ly_ref.setZero(_nstep); _Lz_ref.setZero(_nstep);
	_V_ini.setZero(_Nt,1);
	_comx_center_ref.setZero(_nh,1);
	_comy_center_ref.setZero(_nh,1);
	_comz_center_ref.setZero(_nh,1);
	
	_thetax_center_ref.setZero(_nh,1); 
	_thetay_center_ref.setZero(_nh,1);	
	
        
// 	 store n_vis
	_flag.setZero(_nsum);
	
	_flag_global.setZero(_nsum);
	
// 	parameters for objective function======================	
	 _Rx = 1;           _Ry = 1;            _Rz =1;
	_alphax = 1;       _alphay = 1;        _alphaz = 10; 
	_beltax = 5000;   _beltay = 10;     _beltaz = 20000000;
	_gamax =  10000000; _gamay = 10000000;  _gamaz = 200;
	_Rthetax = 1; _Rthetay = 1;
	_alphathetax = 100; _alphathetay = 100;
	_beltathetax = 500000; _beltathetay = 500000;

	
	// time cost consumption
	_tcpu_iterative.setZero(_nsum);
	_tcpu_prepara.setZero(_nsum);
	_tcpu_prepara2.setZero(_nsum);
	_tcpu_qp.setZero(_nsum);

	
	_pvu_2 = _pvu.transpose()*_pvu;
	_ppu_2 = _ppu.transpose()*_ppu;

	
	_loop = 2;

///////////////////////////////////////////////////////////////
//////////// next code block just run once	
	  A_unit.setIdentity(_nh,_nh);
	  C_unit.setIdentity(_nstep,_nstep);		
	

	  // optimization objective function 	
	  _WX.setZero(_nh,_nh);
	  _WY.setZero(_nh,_nh);
	  _WZ.setZero(_nh,_nh);
	  _WthetaX.setZero(_nh,_nh);
	  _WthetaY.setZero(_nh,_nh);
	  _PHIX.setZero(_nstep,_nstep);
	  _PHIY.setZero(_nstep,_nstep);
	  _PHIZ.setZero(_nstep,_nstep);
	  _Q_goal.setZero(_Nt,_Nt);
	  _q_goal.setZero(_Nt,1);
	  _Q_goal1.setZero(_Nt,_Nt);
	  _q_goal1.setZero(_Nt,1);	
	  
	  _WX = _Rx*0.5 * A_unit + _alphax*0.5 * _pvu_2 + _beltax*0.5 * _ppu_2;	  
	  _WY = _Ry/2 * A_unit + _alphay/2 * _pvu_2 + _beltay/2 * _ppu_2;
	  _WZ = _Rz/2 * A_unit + _alphaz/2 * _pvu_2 + _beltaz/2 * _ppu_2;  
	  _WthetaX = _Rthetax/2 * A_unit + _alphathetax/2 * _pvu_2 + _beltathetax/2 * _ppu_2;
	  _WthetaY = _Rthetay/2 * A_unit + _alphathetay/2 * _pvu_2 + _beltathetay/2 * _ppu_2;
	  _PHIX  = _gamax/2 * C_unit;
	  _PHIY  = _gamay/2 * C_unit;
	  _PHIZ  = _gamaz/2 * C_unit;
		  
	  _Q_goal.block<_nh, _nh>(0, 0) = _WX;
	  _Q_goal.block< Dynamic, Dynamic>(_nh, _nh,_nh, _nh) = _WY;
	  _Q_goal.block< Dynamic, Dynamic>(2*_nh, 2*_nh,_nh, _nh) = _WZ;
	  _Q_goal.block< Dynamic, Dynamic>(3*_nh, 3*_nh,_nh, _nh) = _WthetaX;
	  _Q_goal.block< Dynamic, Dynamic>(4*_nh, 4*_nh,_nh, _nh) = _WthetaY;
	  _Q_goal.block< Dynamic, Dynamic>(5*_nh, 5*_nh,_nstep,_nstep) = _PHIX;
	  _Q_goal.block< Dynamic, Dynamic>(5*_nh+_nstep, 5*_nh+_nstep,_nstep, _nstep) = _PHIY;
	  _Q_goal.block< Dynamic, Dynamic>(5*_nh+2*_nstep, 5*_nh+2*_nstep,_nstep, _nstep) = _PHIZ;	  
	
	  _Q_goal1 = 2 * _Q_goal;	
	
	
	  // constraints
	  _Sjx.setZero(_nh,_Nt);
	  _Sjy.setZero(_nh,_Nt);
	  _Sjz.setZero(_nh,_Nt);
	  _Sjthetax.setZero(_nh,_Nt);
	  _Sjthetay.setZero(_nh,_Nt);
	  _Sjx.block<_nh, _nh>(0, 0) = A_unit;
	  _Sjy.block<_nh, _nh>(0, _nh) = A_unit;
	  _Sjz.block<_nh, _nh>(0, 2*_nh) = A_unit;	
	  _Sjthetax.block<_nh, _nh>(0, 3*_nh) = A_unit;
	  _Sjthetay.block<_nh, _nh>(0, 4*_nh) = A_unit;
	  
	  // ZMP boundary preparation
	  _H_q_upx.setZero(_nh,_Nt);
	  _F_zmp_upx.setZero(_nh,1);
	  _H_q_lowx.setZero(_nh,_Nt);
	  _F_zmp_lowx.setZero(_nh,1);
	  _H_q_upy.setZero(_nh,_Nt);
	  _F_zmp_upy.setZero(_nh,1);
	  _H_q_lowy.setZero(_nh,_Nt);
	  _F_zmp_lowy.setZero(_nh,1);

	  _phi_i_x_up.setZero(_Nt,_Nt);
	  _p_i_x_t_up.setZero(_Nt,_nh);
	  _del_i_x_up.setZero(1,_nh);
	  _phi_i_x_low.setZero(_Nt,_Nt);
	  _p_i_x_t_low.setZero(_Nt,_nh);
	  _del_i_x_low.setZero(1,_nh);
	  _phi_i_y_up.setZero(_Nt,_Nt);
	  _p_i_y_t_up.setZero(_Nt,_nh);
	  _del_i_y_up.setZero(1,_nh);
	  _phi_i_y_low.setZero(_Nt,_Nt);
	  _p_i_y_t_low.setZero(_Nt,_nh);
	  _del_i_y_low.setZero(1,_nh);	  
	  
	  // angle boundary preparation
	  _q_upx.setZero(_nh,_Nt);
	  _qq_upx.setZero(_nh,1);
	  _q_lowx.setZero(_nh,_Nt);
	  _qq_lowx.setZero(_nh,1);
	  _q_upy.setZero(_nh,_Nt);
	  _qq_upy.setZero(_nh,1);
	  _q_lowy.setZero(_nh,_Nt);
	  _qq_lowy.setZero(_nh,1);
	  
	  _qq1_upx.setZero(_nh,1);
	  _qq1_lowx.setZero(_nh,1);
	  _qq1_upy.setZero(_nh,1);
	  _qq1_lowy.setZero(_nh,1);	  

	  // torque bondary preparation
	  _t_upx.setZero(_nh,_Nt);
	  _tt_upx.setZero(_nh,1);
	  _t_lowx.setZero(_nh,_Nt);
	  _tt_lowx.setZero(_nh,1);
	  _t_upy.setZero(_nh,_Nt);
	  _tt_upy.setZero(_nh,1);
	  _t_lowy.setZero(_nh,_Nt);
	  _tt_lowy.setZero(_nh,1);
	  
	  _tt1_upx.setZero(_nh,1);
	  _tt1_lowx.setZero(_nh,1);
	  _tt1_upy.setZero(_nh,1);
	  _tt1_lowy.setZero(_nh,1);
	  
	  // CoM height boundary preparation
	  _H_h_upz.setZero(_nh,_Nt);
	  _F_h_upz.setZero(_nh,1);
	  _H_h_lowz.setZero(_nh,_Nt);
	  _F_h_lowz.setZero(_nh,1);
	  _delta_footz_up.setZero(_nh,1);
	  _delta_footz_low.setZero(_nh,1);

	  // CoM height acceleration boundary preparation
	  _H_hacc_lowz.setZero(_nh,_Nt);
	  _F_hacc_lowz.setZero(_nh,1);
	  _delta_footzacc_up.setZero(_nh,1);	  

	  
	  //swing foot velocity constraints
	  _Footvx_max.setZero(1,_Nt);
	  _Footvx_min.setZero(1,_Nt);
	  _Footvy_max.setZero(1,_Nt);
	  _Footvy_min.setZero(1,_Nt);
	  _footubxv.setZero(1,1);
	  _footlbxv.setZero(1,1);
	  _footubyv.setZero(1,1);
	  _footlbyv.setZero(1,1);
	  
	  // foot vertical location-equality constraints
	  _H_q_footz.setZero(1, _Nt);
	  _F_footz.setZero(1, 1);
	  
	  // CoMZ height-equality constraints
	  _h_h.setZero(_nh, _Nt);
	  _F_footz.setZero(_nh, 1);	  

	  // body inclination-equality constraints
	  _a_hx.setZero(_nh, _Nt);
	  _a_hxx.setZero(_nh, 1);
	  _a_hy.setZero(_nh, _Nt);
	  _a_hyy.setZero(_nh, 1);
	  
	  
	  // foot location constraints_ki, _k_yu
	  _Sfoot.setZero(1,2);
	  _Sfoot(0,0) = -1;
	  _Sfoot(0,1) = 1;
	  
	  _S1.setZero(1,_nh);
	  _S1(0,0) = 1;	  
	  
	// offline calulated the ZMP constraints coefficient
	vector <Eigen::MatrixXd> x_offline1(_nh)  ;
	  
	  
	ZMPx_constraints_offfline = x_offline1;
// 	vector <Eigen::MatrixXd> ZMPy_constraints_offfline(_nh);
	ZMPy_constraints_offfline = x_offline1;
	
	
// 	vector <Eigen::MatrixXd> ZMPx_constraints_half(_nh);
// 	vector <Eigen::MatrixXd> ZMPy_constraints_half(_nh);
	ZMPx_constraints_half = x_offline1;
	
	ZMPy_constraints_half = x_offline1;
	
	
	for(int jxx=1; jxx<=_nh; jxx++)
	{
	  _Si.setZero(1,_nh);
	  _Si(0,jxx-1) = 1;
	  // ZMP constraints	      
		 
         ZMPx_constraints_offfline[jxx-1] = (_Si * _ppu * _Sjx).transpose() * _Si * _pau * _Sjz - (_Si * _pau * _Sjx).transpose() * _Si * _ppu * _Sjz;
	 ZMPx_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _pau * _Sjz;
	  
	  
         ZMPy_constraints_offfline[jxx-1] = (_Si * _ppu * _Sjy).transpose() * _Si * _pau * _Sjz - (_Si * _pau * _Sjy).transpose() * _Si * _ppu * _Sjz;
	 ZMPy_constraints_half[jxx-1] = - (_Si).transpose() * _Si * _pau * _Sjz;
	      
	}

   
  
  
}



void MPCClass::step_timing_opti_loop(int i,Eigen::Matrix<double,18,1> estimated_state, Eigen::Vector3d _Rfoot_location_feedback, Eigen::Vector3d _Lfoot_location_feedback,double lamda, bool _stopwalking)
{
  cout <<"i:"<<i<<endl;
  
  clock_t _t_start,_t_finish;
  _t_start = clock();
  
  //// step cycle number when (i+1)*dt fall into: attention that _tstep+1 falls ionto the next cycle      
  Indexfind((i+1)*_dt,xyz0);	   /// next one sampling time
  _periond_i = _j_period+1;      ///coincident with Matlab
  _j_period = 0;  
  
//   cout <<"k:"<<_periond_i<<endl;	

  

  //ZMP & ZMPv
  _px(0,i) = _footx_ref(_periond_i-1,0); _zmpvx(0,i) = 0; 
  _py(0,i) = _footy_ref(_periond_i-1,0); _zmpvy(0,i) = 0; 
  
//   cout<<"_py:"<<_py(0,i)<<endl;  
  
  ///remaining step timing for the next stepping
  _ki = round(_tx(_periond_i-1,0)/_dt);
  _k_yu = i-_ki;                
  _Tk = _ts(_periond_i-1) - _k_yu*_dt; 
  
  //// reference remaining time and step length and step width
  _Lxx_refx = _footx_offline(_periond_i)-_footx_ref(_periond_i-1);        
  _Lyy_refy = _footy_offline(_periond_i)-_footy_ref(_periond_i-1); //%% tracking the step location
  _Lxx_refx1 = _footx_offline(_periond_i+1)-_footx_offline(_periond_i);        
  _Lyy_refy1 = _footy_offline(_periond_i+1)-_footy_offline(_periond_i); // tracking the step location
    
  _tr1_ref = cosh(_Wn*_Tk);        _tr2_ref = sinh(_Wn*_Tk);
  _tr1_ref1 = cosh(_Wn*_ts(_periond_i));  _tr2_ref1 = sinh(_Wn*_ts(_periond_i));  
  
/*   cout <<_Lxx_refx<<endl;
  cout <<_Lyy_refy<<endl; 
 cout <<_Lxx_refx1<<endl;
  cout <<_Lyy_refy1<<endl;  
  */ 
  
  // warm start
  if (i==1)
  {
    _vari_ini << _Lxx_refx,
                 _Lyy_refy,
		 _tr1_ref,
		 _tr2_ref,
		 _Lxx_refx1,
                 _Lyy_refy1,
		 _tr1_ref1,
		 _tr2_ref1;
  }
  else
  {
    _vari_ini = _Vari_ini.col(i-1);
  }
  
  
// step timing upper&lower boundaries  modification
  if ((_t_min -_k_yu*_dt)>=0.0001)
  {
    _tr1_min = cosh(_Wn*(_t_min-_k_yu*_dt));
    _tr2_min = sinh(_Wn*(_t_min-_k_yu*_dt));    
  }
  else
  {
    _tr1_min = cosh(_Wn*(0.0001));
    _tr2_min = sinh(_Wn*(0.0001));      
  }
 
  
 
  _tr1_max = cosh(_Wn*(_t_max-_k_yu*_dt));
  _tr2_max = sinh(_Wn*(_t_max-_k_yu*_dt));
  
  _tr11_min = cosh(_Wn*_t_min);
  _tr21_min = sinh(_Wn*_t_min);     
  _tr11_max = cosh(_Wn*_t_max);
  _tr21_max = sinh(_Wn*_t_max);    
    
//   cout <<_tr11_min<<endl;
//   cout <<_tr21_min<<endl;   
//   cout <<_tr11_max<<endl;
//   cout <<_tr21_max<<endl;  

  if (i==1)
  {
        _COMx_is(_periond_i-1) = _comx_feed(i-1)-_footx_ref(_periond_i-1);  
	_COMx_es.col(_periond_i-1) = _SS1*_vari_ini*0.5;  
	_COMvx_is(_periond_i-1)= (_COMx_es(_periond_i-1)-_COMx_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
        _COMy_is(_periond_i-1) = _comy_feed(i-1)-_footy_ref(_periond_i-1);  
	_COMy_es.col(_periond_i-1) = _SS2*_vari_ini*0.5;  
	_COMvy_is(_periond_i-1)= (_COMy_es(_periond_i-1)-_COMy_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);  
        _comvx_endref= _Wn*_COMx_is(_periond_i-1)*_SS4*_vari_ini + _COMvx_is(_periond_i-1)*_SS3*_vari_ini;
        _comvy_endref= _Wn*_COMy_is(_periond_i-1)*_SS4*_vari_ini + _COMvy_is(_periond_i-1)*_SS3*_vari_ini;     
  }

/*  cout <<_comvx_endref<<endl;
  cout <<_comvy_endref<<endl;  */ 
//   cout <<_COMvy_is<<endl;
 
  
// SEQUENCE QUADARTIC PROGRAMMING-step timing &step location optimization
  for (int xxxx=1; xxxx<=2; xxxx++)
  { 
    //// be careful the  divide / (one of the factor should be double: type)
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// // %% optimal programme formulation/OBJECTIVE FUNCTION:
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%    
// // %%%%% Lx0 = Lxk(:,i);Ly0 = Lyk(:,i);tr10 = Tr1k(:,i); tr20 = Tr2k(:,i);   
    step_timing_object_function(i);

// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// // %% constraints
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
    step_timing_constraints(i);
    
    ///// generated CoM final position
    ////number of inequality constraints: 8+8+8+4+4+4
    solve_stepping_timing();
    
    if (_X.rows() == 8)
    {
      _vari_ini += _X;
    }   
    
    
  }
  
  
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// // %% results postprocession
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
  
  _Vari_ini.col(i) = _vari_ini;  
  
//   cout <<"_vari_ini:"<<endl<<_vari_ini<<endl;
  
  
// update the optimal parameter in the real-time: at the result, the effect of optimization reduced gradually
  _Lxx_ref(_periond_i-1) = _SS1*_vari_ini;
  _Lyy_ref(_periond_i-1) = _SS2*_vari_ini;
  _ts(_periond_i-1) = _k_yu*_dt+ log((_SS3+_SS4)*_vari_ini)/_Wn;   //check log function
  
  _Lxx_ref(_periond_i) = _SS5*_vari_ini;
  _Lyy_ref(_periond_i) = _SS6*_vari_ini;
  _ts(_periond_i) = log((_SS7+_SS8)*_vari_ini)/_Wn;    
  
  
  _Lxx_ref_real(i) = _Lxx_ref(_periond_i-1);
  _Lyy_ref_real(i) = _Lyy_ref(_periond_i-1);
  _Ts_ref_real(i) = _ts(_periond_i-1);  
  
/*  cout <<"_Lxx_ref_real:"<<endl<<_Lxx_ref_real<<endl;
  cout <<"_Lxx_ref_real:"<<endl<<_Lxx_ref_real<<endl; 
  cout <<"_Ts_ref_real:"<<endl<<_Ts_ref_real<<endl;  */   
  

  _COMx_is(_periond_i-1) = _comx_feed(i-1)-_footx_ref(_periond_i-1);  
  _COMx_es.col(_periond_i-1) = _SS1*_vari_ini*0.5;  
  _COMvx_is(_periond_i-1)= (_COMx_es(_periond_i-1)-_COMx_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);
  _COMy_is(_periond_i-1) = _comy_feed(i-1)-_footy_ref(_periond_i-1);  
  _COMy_es.col(_periond_i-1) = _SS2*_vari_ini*0.5;  
  _COMvy_is(_periond_i-1)= (_COMy_es(_periond_i-1)-_COMy_is(_periond_i-1)*_SS3*_vari_ini)/(1/_Wn *_SS4*_vari_ini);    

/*  cout <<"_Lxx_ref_real:"<<endl<<_Lxx_ref_real<<endl;
  cout <<"_Lxx_ref_real:"<<endl<<_Lxx_ref_real<<endl; 
  cout <<"_Ts_ref_real:"<<endl<<_Ts_ref_real<<endl;  */   
  
  
// update walking period and step location 
  for (int jxx = _periond_i+1; jxx<=_footstepsnumber; jxx++)
  {
    _tx(jxx-1) = _tx(jxx-2)+_ts(jxx-2);
  }
  
  _footx_ref(_periond_i)=_footx_ref(_periond_i-1)+_SS1*_vari_ini;
  _footy_ref(_periond_i)=_footy_ref(_periond_i-1)+_SS2*_vari_ini;    

  _comvx_endref= _Wn*_COMx_is(_periond_i-1)*_SS4*_vari_ini + _COMvx_is(_periond_i-1)*_SS3*_vari_ini;
  _comvy_endref= _Wn*_COMy_is(_periond_i-1)*_SS4*_vari_ini + _COMvy_is(_periond_i-1)*_SS3*_vari_ini;     
      
// update CoM state  
   
  _comx(i)= _COMx_is(_periond_i-1)*cosh(_Wndt) + _COMvx_is(_periond_i-1)*1/_Wn*sinh(_Wndt)+_px(i);
  _comy(i)= _COMy_is(_periond_i-1)*cosh(_Wndt) + _COMvy_is(_periond_i-1)*1/_Wn*sinh(_Wndt)+_py(i);           
  _comvx(i)= _Wn*_COMx_is(_periond_i-1)*sinh(_Wndt) + _COMvx_is(_periond_i-1)*cosh(_Wndt);
  _comvy(i)= _Wn*_COMy_is(_periond_i-1)*sinh(_Wndt) + _COMvy_is(_periond_i-1)*cosh(_Wndt);     
  _comax(i)= pow(_Wn,2)*_COMx_is(_periond_i-1)*cosh(_Wndt) + _COMvx_is(_periond_i-1)*_Wn*sinh(_Wndt);
  _comay(i)= pow(_Wn,2)*_COMy_is(_periond_i-1)*cosh(_Wndt) + _COMvy_is(_periond_i-1)*_Wn*sinh(_Wndt);   


//  external disturbances
  _comx_feed(i) = _comx(i);    
  _comvx_feed(i) = _comvx(i);
  _comax_feed(i) = _comax(i);
  _comy_feed(i) = _comy(i);    
  _comvy_feed(i) = _comvy(i);  
  _comay_feed(i) = _comay(i);  
  
//  external disturbances

//     if (i==40)
//     {
//       _comx_feed(i) = _comx(i)+_det_xp/16;    
//       _comvx_feed(i) = _comvx(i)+_det_xv;
//       _comy_feed(i) = _comy(i)+_det_yp/16;    
//       _comvy_feed(i) = _comvy(i)+_det_yv;       
//     }

  
  _t_finish = clock();  
  
  _tcpu(0,i-1) = (double)(_t_finish - _t_start)/CLOCKS_PER_SEC;
  
  
  
  _t_f.setLinSpaced(_nh,(i+1)*_dt, (i+_nh)*_dt);
  
  Indexfind(i*_dt,xyz1);                   //// step cycle number when (i)*dt fall into : current sampling time
  _bjxx = _j_period+1;  //coincidence with matlab 
  _j_period = 0;  
  
  Indexfind(_t_f(0),xyz1);                /// step cycle number when (i+1)*dt fall into : current sampling time
  _bjx1 = _j_period+1;
  _j_period = 0;  
  
  _td = 0.2* _ts;
  
  _footxyz_real.row(0) = _footx_ref.transpose();
  _footxyz_real.row(1) = _footy_ref.transpose();  
  _footxyz_real.row(2) = _footz_ref.transpose();    
  
  
}



int MPCClass::Indexfind(double goalvari, int xyz)
{
  _j_period = 0;
  
  if (xyz<-0.5)
  {
      while (goalvari > (_tx(_j_period))+0.0001)
      {
	_j_period++;
      }    
      _j_period = _j_period-1;	    
  }
  else
  {
  
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

      
  }

}


void MPCClass::step_timing_object_function(int i)
{
    _AxO(0) = _comx_feed(i-1)-_footx_ref(_periond_i-1); _BxO(0) = _comvx_feed(i-1)/_Wn; _Cx(0,0) =-0.5*_Lxx_refx; 
    _Axv = _Wn*_AxO;  _Bxv = _Wn*_BxO; _Cxv=_comvx_endref; 
    _AyO(0) = _comy_feed(i-1)-_footy_ref(_periond_i-1); _ByO(0) = _comvy_feed(i-1)/_Wn; _Cy(0,0) =-0.5*_Lyy_refy; 
    _Ayv = _Wn*_AyO;  _Byv = _Wn*_ByO; _Cyv=_comvy_endref;    
    
    _SQ_goal0(0,0)=0.5*_bbx;
    _SQ_goal0(1,1)=0.5*_bby;
    _SQ_goal0(2,2)=0.5*(_rr1+_aax*_AxO(0,0)*_AxO(0,0)+_aay*_AyO(0,0)*_AyO(0,0)+_aaxv*_Axv(0,0)*_Axv(0,0)+_aayv*_Ayv(0,0)*_Ayv(0,0));
    _SQ_goal0(2,3)=0.5*(     _aax*_AxO(0,0)*_BxO(0,0)+_aay*_AyO(0,0)*_ByO(0,0)+_aaxv*_Axv(0,0)*_Bxv(0,0)+_aayv*_Ayv(0,0)*_Byv(0,0));
    _SQ_goal0(3,2)=0.5*(     _aax*_BxO(0,0)*_AxO(0,0)+_aay*_ByO(0,0)*_AyO(0,0)+_aaxv*_Bxv(0,0)*_Axv(0,0)+_aayv*_Byv(0,0)*_Ayv(0,0));
    _SQ_goal0(3,3)=0.5*(_rr2+_aax*_BxO(0,0)*_BxO(0,0)+_aay*_ByO(0,0)*_ByO(0,0)+_aaxv*_Bxv(0,0)*_Bxv(0,0)+_aayv*_Byv(0,0)*_Byv(0,0));    
    _SQ_goal0(4,4)=0.5*_bbx1; 
    _SQ_goal0(5,5)=0.5*_bby1; 
    _SQ_goal0(6,6)=0.5*_rr11; 
    _SQ_goal0(7,7)=0.5*_rr21; 
    
    _SQ_goal = (_SQ_goal0+_SQ_goal0.transpose())/2.0;  
    _Sq_goal << -_bbx*_Lxx_refx,
	      -_bby*_Lyy_refy,
	      -_rr1*_tr1_ref+_aax*_AxO(0,0)*_Cx(0,0)+_aay*_AyO(0,0)*_Cy(0,0)+_aaxv*_Axv(0,0)*_Cxv(0,0)+_aayv*_Ayv(0,0)*_Cyv(0,0),
	      -_rr2*_tr2_ref+_aax*_BxO(0,0)*_Cx(0,0)+_aay*_ByO(0,0)*_Cy(0,0)+_aaxv*_Bxv(0,0)*_Cxv(0,0)+_aayv*_Byv(0,0)*_Cyv(0,0),
	      -_bbx1*_Lxx_refx1,
	      -_bby1*_Lyy_refy1,
	      -_rr11*_tr1_ref1,
	      -_rr21*_tr2_ref1;   
	      
    _SQ_goal1 = 2 * _SQ_goal;
    _Sq_goal1 = 2 * _SQ_goal * _vari_ini + _Sq_goal;
    
/*  cout <<"_SQ_goal1:"<<_SQ_goal1<<endl;
  cout <<"_Sq_goal1:"<<_Sq_goal1<<endl;   */   
     
    
    
    _Ax= _SS3.transpose()*_AxO*_SS7+_SS4.transpose()*_BxO*_SS7-_SS1.transpose()*_SS7+_SS4.transpose()*_AxO*_SS8+_SS3.transpose()*_BxO*_SS8;   
    _Bx(0,0)= -0.5*_Lxx_refx1; 
    _Ay= _SS3.transpose()*_AyO*_SS7+_SS4.transpose()*_ByO*_SS7-_SS2.transpose()*_SS7+_SS4.transpose()*_AyO*_SS8+_SS3.transpose()*_ByO*_SS8;   
    _By(0,0)= -0.5*_Lyy_refy1;         ///check!!!!!!!!

    _ixi = _vari_ini.transpose()*_Ax*_vari_ini;
    _iyi = _vari_ini.transpose()*_Ay*_vari_ini;
    _SQ_goal20= _aax1/2.0*2*( 2*(_ixi(0,0)*_Ax.transpose() + 2*_Ax*_vari_ini*(_Ax*_vari_ini).transpose()) + 2*(2*_Bx(0,0)*_Ax.transpose()))+_aay1/2.0*2*( 2*(_iyi(0,0)*_Ay.transpose() + 2*_Ay*_vari_ini*(_Ay*_vari_ini).transpose()) + 2*(2*_By(0,0)*_Ay.transpose()));

    _SQ_goal2 = (_SQ_goal20.transpose()+_SQ_goal20)/2.0;    
    _Sq_goal2= _aax1/2.0*2*(2*_Ax*_vari_ini)*(_vari_ini.transpose()*_Ax*_vari_ini+_Bx)  +  _aay1/2.0*2*(2*_Ay*_vari_ini)*(_vari_ini.transpose()*_Ay*_vari_ini+_By);    
    
    _SQ_goal3 = _SQ_goal1+_SQ_goal2;
    _Sq_goal3 = _Sq_goal1+_Sq_goal2;   
  
/*  cout <<"_Ax:"<<endl<<_Ax<<endl;
  cout <<"_Ay:"<<endl<<_Ay<<endl;   */    

/*  cout <<"_SQ_goal3:"<<endl<<_SQ_goal3<<endl;
  cout <<"_Sq_goal3:"<<endl<<_Sq_goal3<<endl;  */     
  
	// update your _G, _g0 ..... matices
	// _G, _g0, _CE, _ce0, _CI, _ci0, _X
}

void MPCClass::step_timing_constraints(int i)
{
  
// // %% constraints
// // %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 
// // %% remaining time constraints: inequality constraints    
    _trx1_up = _SS3;
    _det_trx1_up = -(_SS3*_vari_ini);
    _det_trx1_up(0,0) +=  _tr1_max; 

    _trx1_lp = -_SS3;
    _det_trx1_lp = -(-_SS3*_vari_ini); 
    _det_trx1_lp(0,0) -= _tr1_min;  	

    _trx2_up = _SS4;
    _det_trx2_up = -(_SS4*_vari_ini);
    _det_trx2_up(0,0) += _tr2_max;	

    _trx2_lp = -_SS4;
    _det_trx2_lp = -(-_SS4*_vari_ini);  
    _det_trx2_lp(0,0) -= _tr2_min;  	

    _trx3_up = _SS7;
    _det_trx3_up = -(_SS7*_vari_ini);
    _det_trx3_up(0,0) += _tr11_max;	

    _trx3_lp = -_SS7;
    _det_trx3_lp = -(-_SS7*_vari_ini);        
    _det_trx3_lp(0,0) -= _tr11_min;    
    
    _trx4_up = _SS8;
    _det_trx4_up = -(_SS8*_vari_ini);
    _det_trx4_up(0,0) += _tr21_max;
    
    _trx4_lp = -_SS8;
    _det_trx4_lp = -(-_SS8*_vari_ini);      
    _det_trx4_lp(0,0) -= _tr21_min;  
    
    _trx.row(0) = _trx1_up; _trx.row(1) = _trx1_lp; _trx.row(2) = _trx2_up; _trx.row(3) = _trx2_lp;
    _trx.row(4) = _trx3_up; _trx.row(5) = _trx3_lp; _trx.row(6) = _trx4_up; _trx.row(7) = _trx4_lp;	
    
    _det_trx.row(0) = _det_trx1_up; _det_trx.row(1) = _det_trx1_lp; _det_trx.row(2) = _det_trx2_up; _det_trx.row(3) = _det_trx2_lp;
    _det_trx.row(4) = _det_trx3_up; _det_trx.row(5) = _det_trx3_lp; _det_trx.row(6) = _det_trx4_up; _det_trx.row(7) = _det_trx4_lp;		
    
    
//     cout <<"_trx:"<<endl<<_trx<<endl;
//     cout <<"_det_trx:"<<endl<<_det_trx<<endl;  
  
  
// tr1 & tr2: equation constraints:   
    _trx12 = (2*(_SS3.transpose()*_SS3-_SS4.transpose()*_SS4)*_vari_ini).transpose();
    _det_trx12 = -(_vari_ini.transpose()*(_SS3.transpose()*_SS3-_SS4.transpose()*_SS4)*_vari_ini);
    _det_trx12(0,0) +=1; 

    _trx121 = (2*(_SS7.transpose()*_SS7-_SS8.transpose()*_SS8)*_vari_ini).transpose();
    _det_trx121 = -(_vari_ini.transpose()*(_SS7.transpose()*_SS7-_SS8.transpose()*_SS8)*_vari_ini);
    _det_trx121(0,0) +=1; 

    _trxx.row(0) = _trx12;   _trxx.row(1) = _trx121;
    _det_trxx.row(0) = _det_trx12; _det_trxx.row(1) = _det_trx121;    

    
/*    cout <<"_trxx:"<<endl<<_trxx<<endl;
    cout <<"_det_trxx:"<<endl<<_det_trxx<<endl;  */    

//  foot location constraints      
    if (_periond_i % 2 == 0)
    {
//       _footy_max = -0.2;
//       _footy_min = -0.6;
      
	  if (i>=(round(2*_ts(1)/_dt))+1) ///update the footy_limit
	  {
	      
            _footy_min=-(2*RobotParaClass::HALF_HIP_WIDTH() + 0.2); 
// 	    _footy_max=-(RobotParaClass::HALF_HIP_WIDTH() - 0.03); 
	    _footy_max = -(RobotParaClass::FOOT_WIDTH()+0.01);
	  }
	  else
	  {
	    _footy_min=-(2*RobotParaClass::HALF_HIP_WIDTH() + 0.2); 
	    _footy_max=-(RobotParaClass::HALF_HIP_WIDTH() - 0.03); 	    
	  }
      
      
      
    }
    else
    {
//       _footy_max = 0.6;
//       _footy_min = 0.2; 
       if (i>=(round(2*_ts(1)/_dt))+1)
       {
	_footy_max=2*RobotParaClass::HALF_HIP_WIDTH() + 0.2; 
// 	_footy_min=RobotParaClass::HALF_HIP_WIDTH() - 0.03; 
	_footy_min =RobotParaClass::FOOT_WIDTH()+0.01;
      }
       else
       {
	_footy_max=2*RobotParaClass::HALF_HIP_WIDTH() + 0.2; 
	_footy_min=  RobotParaClass::HALF_HIP_WIDTH() - 0.03; 	 
      }
      
      
    }
    // only the next one step
    _h_lx_up = _SS1;
    _det_h_lx_up = -(_SS1*_vari_ini);
    _det_h_lx_up(0,0) += _footx_max;

    _h_lx_lp = -_SS1;
    _det_h_lx_lp = -(-_SS1*_vari_ini); 
    _det_h_lx_lp(0,0) -= _footx_min;

    _h_ly_up = _SS2;
    _det_h_ly_up = -(_SS2*_vari_ini);
    _det_h_ly_up(0,0) += _footy_max;

    _h_ly_lp = -_SS2;
    _det_h_ly_lp = -(-_SS2*_vari_ini);
    _det_h_ly_lp(0,0) -= _footy_min;    

    _h_lx_up1 = _SS5;
    _det_h_lx_up1 = -(_SS5*_vari_ini);
    _det_h_lx_up1(0,0) += _footx_max;    

    _h_lx_lp1 = -_SS5;
    _det_h_lx_lp1 = -(-_SS5*_vari_ini);
    _det_h_lx_lp1(0,0) -= _footx_min;     

    _h_ly_up1 = _SS6;
    _det_h_ly_up1 = -(_SS6*_vari_ini);
    _det_h_ly_up1(0,0) -= _footy_min;
    
    _h_ly_lp1 = -_SS6;
    _det_h_ly_lp1 = -(-_SS6*_vari_ini); 
    _det_h_ly_lp1(0,0) += _footy_max;     

    _h_lx_upx.row(0)= _h_lx_up;    _h_lx_upx.row(1)= _h_lx_lp;   _h_lx_upx.row(2)= _h_ly_up;     _h_lx_upx.row(3)= _h_ly_lp;
    _h_lx_upx.row(4)= _h_lx_up1;   _h_lx_upx.row(5)= _h_lx_lp1;  _h_lx_upx.row(6)= _h_ly_up1;    _h_lx_upx.row(7)= _h_ly_lp1;
    _det_h_lx_upx.row(0)=_det_h_lx_up;  _det_h_lx_upx.row(1)=_det_h_lx_lp;  _det_h_lx_upx.row(2)=_det_h_ly_up;  _det_h_lx_upx.row(3)=_det_h_ly_lp;
    _det_h_lx_upx.row(4)=_det_h_lx_up1; _det_h_lx_upx.row(5)=_det_h_lx_lp1; _det_h_lx_upx.row(6)=_det_h_ly_up1; _det_h_lx_upx.row(7)=_det_h_ly_lp1;    

 
/*    cout <<"_h_lx_upx:"<<endl<<_h_lx_upx<<endl;
    cout <<"_det_h_lx_upx:"<<endl<<_det_h_lx_upx<<endl;   */   
    
    
// swing foot velocity boundary
    if (_k_yu ==0)
    {
	_h_lvx_up.setZero();  _h_lvx_lp.setZero(); _h_lvy_up.setZero(); _h_lvy_lp.setZero();
	_h_lvx_up1.setZero(); _h_lvx_lp1.setZero(); _h_lvy_up1.setZero(); _h_lvy_lp1.setZero();
	_det_h_lvx_up.setZero();_det_h_lvx_lp.setZero();_det_h_lvy_up.setZero();_det_h_lvy_lp.setZero();
	_det_h_lvx_up1.setZero();_det_h_lvx_lp1.setZero();_det_h_lvy_up1.setZero();_det_h_lvy_lp1.setZero();    
    }                
    else
    {   
	_h_lvx_up = _SS1;
	_det_h_lvx_up(0,0) = -(-_footx_vmax*_dt);

	_h_lvx_lp = -_SS1;
	_det_h_lvx_lp(0,0) = -(+_footx_vmin*_dt); 

	_h_lvy_up = _SS2;
	_det_h_lvy_up(0,0) = -(-_footy_vmax*_dt);

	_h_lvy_lp = -_SS2;
	_det_h_lvy_lp(0,0) = -(+_footy_vmin*_dt);  

	_h_lvx_up1 = _SS5;
	_det_h_lvx_up1(0,0) = -(-_footx_vmax*_dt);

	_h_lvx_lp1 = -_SS5;
	_det_h_lvx_lp1(0,0) = -(+_footx_vmin*_dt); 

	_h_lvy_up1 = _SS6;
	_det_h_lvy_up1(0,0) = -(-_footy_vmax*_dt);

	_h_lvy_lp1 = -_SS6;
	_det_h_lvy_lp1(0,0) = -(+_footy_vmin*_dt);           
    }                                   
    _h_lvx_upx.row(0)= _h_lvx_up;    _h_lvx_upx.row(1)= _h_lvx_lp;  _h_lvx_upx.row(2)= _h_lvy_up;   _h_lvx_upx.row(3)= _h_lvy_lp;
    _h_lvx_upx.row(4)= _h_lvx_up1;   _h_lvx_upx.row(5)= _h_lvx_lp1; _h_lvx_upx.row(6)= _h_lvy_up1;  _h_lvx_upx.row(7)= _h_lvy_lp1;
    _det_h_lvx_upx.row(0)=_det_h_lvx_up; _det_h_lvx_upx.row(1)=_det_h_lvx_lp; _det_h_lvx_upx.row(2)=_det_h_lvy_up; _det_h_lvx_upx.row(3)=_det_h_lvy_lp;
    _det_h_lvx_upx.row(4)=_det_h_lvx_up1;_det_h_lvx_upx.row(5)=_det_h_lvx_lp1;_det_h_lvx_upx.row(6)=_det_h_lvy_up1;_det_h_lvx_upx.row(7)=_det_h_lvy_lp1;    

/*    cout <<"_h_lvx_upx:"<<endl<<_h_lvx_upx<<endl;
    cout <<"_det_h_lvx_upx:"<<endl<<_det_h_lvx_upx<<endl;   */    
    
// CoM accelearation boundary    
    
    _AA= _Wn*sinh(_Wn*_dt); _CCx = _comx_feed(0,i-1)-_footx_ref(_periond_i-1,0); _BBx = pow(_Wn,2)*_CCx*cosh(_Wn*_dt); 
		            _CCy = _comy_feed(0,i-1)-_footy_ref(_periond_i-1,0); _BBy = pow(_Wn,2)*_CCy*cosh(_Wn*_dt);

    _AA1x = _AA*_Wn; _AA2x = -2* _AA*_CCx*_Wn;  _AA3x = 2*_BBx; 
    _AA1y = _AA*_Wn; _AA2y = -2* _AA*_CCy*_Wn;  _AA3y = 2*_BBy;


    _CoM_lax_up = _AA1x*_SS1+_AA2x*_SS3+(_AA3x-2*_comax_max)*_SS4;
    _det_CoM_lax_up = -(_AA1x*_SS1+_AA2x*_SS3+(_AA3x-2*_comax_max)*_SS4)*_vari_ini;

    _CoM_lax_lp = -_AA1x*_SS1-_AA2x*_SS3-(_AA3x-2*_comax_min)*_SS4;
    _det_CoM_lax_lp = -(-_AA1x*_SS1-_AA2x*_SS3-(_AA3x-2*_comax_min)*_SS4)*_vari_ini; 

    _CoM_lay_up = _AA1y*_SS2+_AA2y*_SS3+(_AA3y-2*_comay_max)*_SS4;
    _det_CoM_lay_up = -(_AA1y*_SS2+_AA2y*_SS3+(_AA3y-2*_comay_max)*_SS4)*_vari_ini;

    _CoM_lay_lp = -_AA1y*_SS2-_AA2y*_SS3-(_AA3y-2*_comay_min)*_SS4;
    _det_CoM_lay_lp = -(-_AA1y*_SS2-_AA2y*_SS3-(_AA3y-2*_comay_min)*_SS4)*_vari_ini;      
    
    _CoM_lax_upx.row(0) = _CoM_lax_up; _CoM_lax_upx.row(1) = _CoM_lax_lp; 
    _CoM_lax_upx.row(2) = _CoM_lay_up; _CoM_lax_upx.row(3) = _CoM_lay_lp;
    _det_CoM_lax_upx.row(0) = _det_CoM_lax_up;  _det_CoM_lax_upx.row(1) = _det_CoM_lax_lp; 
    _det_CoM_lax_upx.row(2) = _det_CoM_lay_up;  _det_CoM_lax_upx.row(3) = _det_CoM_lay_lp;
    
//     cout <<"_CoM_lax_upx:"<<endl<<_CoM_lax_upx<<endl;
//     cout <<"_det_CoM_lax_upx:"<<endl<<_det_CoM_lax_upx<<endl;       
    
    
    
//   CoM velocity_inremental boundary  
    _VAA= cosh(_Wn*_dt); _VCCx = _comx_feed(0,i-1)-_footx_ref(_periond_i-1,0); _VBBx = _Wn*_VCCx*sinh(_Wn*_dt); 
		         _VCCy = _comy_feed(0,i-1)-_footy_ref(_periond_i-1,0); _VBBy = _Wn*_VCCy*sinh(_Wn*_dt);

    _VAA1x = _VAA*_Wn; _VAA2x = -2* _VAA*_VCCx*_Wn; _VAA3x = 2*_VBBx - 2*_comvx_feed(0,i-1); 
    _VAA1y = _VAA*_Wn; _VAA2y = -2* _VAA*_VCCy*_Wn; _VAA3y = 2*_VBBy - 2*_comvy_feed(0,i-1);

    _CoM_lvx_up = _VAA1x*_SS1+_VAA2x*_SS3+(_VAA3x-2*_comax_max*_dt)*_SS4;
    _det_CoM_lvx_up = -(_VAA1x*_SS1+_VAA2x*_SS3+(_VAA3x-2*_comax_max*_dt)*_SS4)*_vari_ini;

    _CoM_lvx_lp = -_VAA1x*_SS1-_VAA2x*_SS3-(_VAA3x-2*_comax_min*_dt)*_SS4;
    _det_CoM_lvx_lp = -(-_VAA1x*_SS1-_VAA2x*_SS3-(_VAA3x-2*_comax_min*_dt)*_SS4)*_vari_ini; 

    _CoM_lvy_up = _VAA1y*_SS2+_VAA2y*_SS3+(_VAA3y-2*_comay_max*_dt)*_SS4;
    _det_CoM_lvy_up = -(_VAA1y*_SS2+_VAA2y*_SS3+(_VAA3y-2*_comay_max*_dt)*_SS4)*_vari_ini;

    _CoM_lvy_lp = -_VAA1y*_SS2-_VAA2y*_SS3-(_VAA3y-2*_comay_min*_dt)*_SS4;
    _det_CoM_lvy_lp = -(-_VAA1y*_SS2-_VAA2y*_SS3-(_VAA3y-2*_comay_min*_dt)*_SS4)*_vari_ini;      
    
    _CoM_lvx_upx.row(0) = _CoM_lvx_up; _CoM_lvx_upx.row(1) = _CoM_lvx_lp; 
    _CoM_lvx_upx.row(2) = _CoM_lvy_up; _CoM_lvx_upx.row(3) = _CoM_lvy_lp;
    _det_CoM_lvx_upx.row(0) = _det_CoM_lvx_up;  _det_CoM_lvx_upx.row(1) = _det_CoM_lvx_lp; 
    _det_CoM_lvx_upx.row(2) = _det_CoM_lvy_up;  _det_CoM_lvx_upx.row(3) = _det_CoM_lvy_lp;  
    
/*    cout <<"_CoM_lvx_upx:"<<endl<<_CoM_lvx_upx<<endl;
    cout <<"_det_CoM_lvx_upx:"<<endl<<_det_CoM_lvx_upx<<endl; */          
    
    
    
//   CoM intial velocity boundary: check   
    _VAA1x1 = _Wn; _VAA2x1 = -2*_VCCx*_Wn; _VAA3x1 = - 2*_comvx_feed(0,i-1); 
    _VAA1y1 = _Wn; _VAA2y1 = -2*_VCCy*_Wn; _VAA3y1 = - 2*_comvy_feed(0,i-1);

    /// modified!!!
    _CoM_lvx_up1 = _VAA1x1*_SS1+_VAA2x1*_SS3+(_VAA3x1-2*_comax_max*_dt)*_SS4;
    _det_CoM_lvx_up1 = -(_VAA1x1*_SS1+_VAA2x1*_SS3+(_VAA3x1-2*_comax_max*_dt/2.0)*_SS4)*_vari_ini;

    _CoM_lvx_lp1 = -_VAA1x1*_SS1-_VAA2x1*_SS3-(_VAA3x1-2*_comax_min*_dt)*_SS4;
    _det_CoM_lvx_lp1 = -(-_VAA1x1*_SS1-_VAA2x1*_SS3-(_VAA3x1-2*_comax_min*_dt/2.0)*_SS4)*_vari_ini; 

    _CoM_lvy_up1 = _VAA1y1*_SS2+_VAA2y1*_SS3+(_VAA3y1-2*_comay_max*_dt)*_SS4;
    _det_CoM_lvy_up1 = -(_VAA1y1*_SS2+_VAA2y1*_SS3+(_VAA3y1-2*_comay_max*_dt/2.0)*_SS4)*_vari_ini;

    _CoM_lvy_lp1 = -_VAA1y1*_SS2-_VAA2y1*_SS3-(_VAA3y1-2*_comay_min*_dt)*_SS4;
    _det_CoM_lvy_lp1 = -(-_VAA1y1*_SS2-_VAA2y1*_SS3-(_VAA3y1-2*_comay_min*_dt/2.0)*_SS4)*_vari_ini;      
    
    _CoM_lvx_upx1.row(0) = _CoM_lvx_up1; _CoM_lvx_upx1.row(1) = _CoM_lvx_lp1; 
    _CoM_lvx_upx1.row(2) = _CoM_lvy_up1; _CoM_lvx_upx1.row(3) = _CoM_lvy_lp1;
    _det_CoM_lvx_upx1.row(0) = _det_CoM_lvx_up1;  _det_CoM_lvx_upx1.row(1) = _det_CoM_lvx_lp1; 
    _det_CoM_lvx_upx1.row(2) = _det_CoM_lvy_up1;  _det_CoM_lvx_upx1.row(3) = _det_CoM_lvy_lp1;      
    
//     cout <<"_CoM_lvx_upx1:"<<endl<<_CoM_lvx_upx1<<endl;
//     cout <<"_det_CoM_lvx_upx1:"<<endl<<_det_CoM_lvx_upx1<<endl; 
      
  
  
  
}




Eigen::MatrixXd  MPCClass::Matrix_ps(Eigen::MatrixXd a, int nh,Eigen::MatrixXd cxps)
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


Eigen::MatrixXd MPCClass::Matrix_pu(Eigen::MatrixXd a, Eigen::MatrixXd b, int nh, Eigen::MatrixXd cxpu)
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






void MPCClass::solve_stepping_timing()
{
  int nVars = 8;
  int nEqCon = 2;
  int nIneqCon = 24 + 12;
  resizeQP(nVars, nEqCon, nIneqCon);	    

  _G = _SQ_goal3;
  _g0 = _Sq_goal3;
  _X = _vari_ini;

// min 0.5 * x G x + g0 x
// _s.t.
// 		CE^T x + ce0 = 0   ///// equality constraints
// 		CI^T x + ci0 >= 0  //// inequality constraints
  _CI.block<8,8>(0,0) = _trx.transpose() * (-1);
  _CI.block<8,8>(0,8) = _h_lx_upx.transpose() * (-1);
  _CI.block<8,8>(0,16) = _h_lvx_upx.transpose() * (-1);
  _CI.block<8,4>(0,24) = _CoM_lax_upx.transpose() * (-1);
  _CI.block<8,4>(0,28) = _CoM_lvx_upx.transpose() * (-1);
  _CI.block<8,4>(0,32) = _CoM_lvx_upx1.transpose() * (-1);
  
  _ci0.block<8,1>(0, 0) = _det_trx;
  _ci0.block<8,1>(8, 0) = _det_h_lx_upx;
  _ci0.block<8,1>(16, 0) = _det_h_lvx_upx;
  _ci0.block<4,1>(24, 0) = _det_CoM_lax_upx;
  _ci0.block<4,1>(28, 0) = _det_CoM_lvx_upx;
  _ci0.block<4,1>(32, 0) = _det_CoM_lvx_upx1;

  
  _CE = _trxx.transpose()*(-1);
  _ce0 = _det_trxx;
  
  
  
  Solve();  

}

void MPCClass::Solve()
{
// min 0.5 * x G x + g0 x
// _s.t.
// 		CE^T x + ce0 = 0
// 		CI^T x + ci0 >= 0
		solveQP();

}


//// each time
void MPCClass::Foot_trajectory_solve(int j_index, bool _stopwalking)
{
  

	    
  double  Footz_ref = 0.05;
  _footxyz_real(1,0) = -_stepwidth(0);
  
  
//// judge if stop  
  if(_stopwalking)  
  {
    
    for (int i_t = _bjx1+1; i_t < _footstepsnumber; i_t++) {	  
      _lift_height_ref(i_t) = 0;  
    }	  

  }    
  
  
//   foot trajectory generation:
  if (_bjx1 >= 2)
  {
//     cout << "_bjx1 >= 2"<<endl;
    if (_bjx1 % 2 == 0)           //odd:left support
    {
//       cout << "left support"<<endl;
      _Lfootx(j_index) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfooty(j_index) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfootz(j_index) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      _Lfootx(j_index+1) = _Lfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfooty(j_index+1) = _Lfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Lfootz(j_index+1) = _Lfootz(round(_tx(_bjx1-1)/_dt) -1-1);      
      
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
	  t_a_plana.setZero(7);
	  t_a_plana(0) = 30*pow(t_des, 4);   t_a_plana(1) = 20*pow(t_des, 3);   t_a_plana(2) = 12*pow(t_des, 2);  t_a_plana(3) = 6*pow(t_des, 1);
	  t_a_plana(4) = 2;                  t_a_plana(5) = 0;                  t_a_plana(6) = 0;
	  
// 	  cout <<"AAA="<<endl<<AAA<<endl;
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
	  Rfootz_plan(4) = _footxyz_real(2,_bjxx);  Rfootz_plan(5) = 0;                   Rfootz_plan(6) = 0;	
	  
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
//       cout << "right support"<<endl;
      _Rfootx(j_index) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfooty(j_index) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfootz(j_index) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);
      
      _Rfootx(j_index+1) = _Rfootx(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfooty(j_index+1) = _Rfooty(round(_tx(_bjx1-1)/_dt) -1-1);
      _Rfootz(j_index+1) = _Rfootz(round(_tx(_bjx1-1)/_dt) -1-1);      
      
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
	  
// 	  cout <<"AAA="<<endl<<AAA<<endl;
// 	  cout <<"AAA_inverse="<<endl<<AAA.inverse()<<endl;	  
// 	  cout <<"t_des="<<endl<<t_des<<endl;
// 	  cout <<"t_plan="<<endl<<t_plan<<endl;
	  
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
	  Lfootz_plan(0) = _Lfootvz(j_index-1);     Lfootz_plan(1) = _Lfootaz(j_index-1); Lfootz_plan(2) = _Lfootz(j_index-1); Lfootz_plan(3) = _Rfootz(j_index)+Footz_ref;
	  Lfootz_plan(4) = _footxyz_real(2,_bjxx);  Lfootz_plan(5) = 0;                   Lfootz_plan(6) = 0;		  
	  
	  
	  Eigen::VectorXd Lfootz_co;
	  Lfootz_co.setZero(7);
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
    

  
}



int MPCClass::Get_maximal_number_reference()
{
  int nsum_max;
  nsum_max = (_nsum -_n_loop_omit-1);  
  return nsum_max;
}

int MPCClass::Get_maximal_number(double dtx)
{
  int nsum_max;
  nsum_max = (_nsum -_n_loop_omit-1)*floor(_dt/dtx);
  
  return nsum_max;
}



////====================================================================================================================
/////////////////////////// using the lower-level control-loop  sampling time as the reference: every 5ms;  at the same time: just using the next one position + next one velocity

Vector3d MPCClass::XGetSolution_CoM_position(int walktime, double dt_sample, Eigen::Vector3d body_in1, Eigen::Vector3d body_in2, Eigen::Vector3d body_in3)
{
  //reference com position
        _CoM_position_optimal.row(0) = _comx;
	_CoM_position_optimal.row(1) = _comy;
	_CoM_position_optimal.row(2) = _comz;



	
	Vector3d com_inte;	
	
	if (walktime>=2)
	{
	  int t_int; 
// 	  t_int = floor(walktime / (_dt / dt_sample) );
	  t_int = floor(walktime* dt_sample/ _dt);	  
          cout <<"T_int_inter:"<<t_int<<endl;
	  ///// chage to be relative time
	  double t_cur;
	  t_cur = walktime * dt_sample ;
	  

	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);
          
// 	  cout << "t_plan1:"<<endl<<t_plan<<endl;
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
	  	  
	  	  
/*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;*/
	  

	
	  
	  
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
	  t_int = floor(walktime* dt_sample/ _dt  );

	  
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
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 = _torso_angle_optimal.col(t_int);
// 	  x13 = _torso_angle_optimal.col(t_int+1);
	  
	  
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
	  t_int = floor(walktime* dt_sample/ _dt  );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

// 	  cout << "t_plan2:"<<endl<<t_plan<<endl;
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
	  	  
	  
/*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;	*/  	  
	  

	
	  
	  
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
// 	  Eigen::Vector3d  x13;


/*	  x10(0) = COM_IN(0,walktime-2); x10(1) = COM_IN(1,walktime-2); x10(2) = COM_IN(2,walktime-2);
	  x11(0) = COM_IN(0,walktime-1); x11(1) = COM_IN(1,walktime-1); x11(2) = COM_IN(2,walktime-1);	 */ 
	  x10 = body_in1; 
	  x11 = body_in2;  
	  x12 =  _R_foot_optition_optimal.col(t_int);
//	  x13 =  _R_foot_optition_optimal.col(t_int+1);
	  
	  
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
	  t_int = floor(walktime* dt_sample/ _dt  );

	  
	  double t_cur;
	  t_cur = walktime * dt_sample;
	  

	  
	  Eigen::Matrix<double, 4, 1> t_plan;
	  t_plan.setZero();
	  t_plan(0) = t_cur - 2*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(1) = t_cur - 1*dt_sample-( t_cur - 2*dt_sample);
	  t_plan(2) = (t_int + 1) *_dt-( t_cur - 2*dt_sample);
	  t_plan(3) = (t_int + 2) *_dt-( t_cur - 2*dt_sample);

//           cout << "t_plan3:"<<endl<<t_plan<<endl;
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
	  	  
/*	  cout << "AAA_inv:"<<endl<<AAA_inv<<endl;	*/  
	  
	  
	  
	
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




void MPCClass::File_wl_steptiming()
{
        Eigen::MatrixXd CoM_ZMP_foot;
	CoM_ZMP_foot.setZero(24,_comx.cols());
	CoM_ZMP_foot.block< Dynamic, Dynamic>(0, 0,1,_comx.cols()) = _px;
	CoM_ZMP_foot.block< Dynamic, Dynamic>(1, 0,1,_comx.cols()) = _py;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(2, 0,1,_comx.cols()) = _pz;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(3, 0,1,_comx.cols()) = _comx;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(4, 0,1,_comx.cols()) = _comy;
	CoM_ZMP_foot.block< Dynamic, Dynamic>(5, 0,1,_comx.cols()) = _comvx;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(6, 0,1,_comx.cols()) = _comvy;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(7, 0,1,_comx.cols()) = _comax;
	CoM_ZMP_foot.block< Dynamic, Dynamic>(8, 0,1,_comx.cols()) = _comay;
	CoM_ZMP_foot.block< Dynamic, Dynamic>(9, 0,1,_comx.cols()) = _comx_feed;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(10, 0,1,_comx.cols()) = _comy_feed;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(11, 0,1,_comx.cols()) = _comvx_feed;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(12, 0,1,_comx.cols()) = _comvy_feed;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(13, 0,1,_comx.cols()) = _comax_feed;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(14, 0,1,_comx.cols()) = _comay_feed;
	CoM_ZMP_foot.block< Dynamic, Dynamic>(15, 0,1,_comx.cols()) = _Lxx_ref_real;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(16, 0,1,_comx.cols()) = _Lyy_ref_real;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(17, 0,1,_comx.cols()) = _Ts_ref_real;
	CoM_ZMP_foot.block< Dynamic, Dynamic>(18, 0,1,_comx.cols()) = _Lfootx;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(19, 0,1,_comx.cols()) = _Lfooty;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(20, 0,1,_comx.cols()) = _Lfootz;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(21, 0,1,_comx.cols()) = _Rfootx;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(22, 0,1,_comx.cols()) = _Rfooty;	
	CoM_ZMP_foot.block< Dynamic, Dynamic>(23, 0,1,_comx.cols()) = _Rfootz;		
  
	std::string fileName = "C++_NP_step_timing_runtime.txt" ;
	std::ofstream outfile( fileName.c_str() ) ; // file name and the operation type. 
       
        for(int i=0; i<_tcpu.rows(); i++){
           for(int j=0; j<_tcpu.cols(); j++){
                 outfile << (double) _tcpu(i,j) << " " ; 
           }
           outfile << std::endl;       // a   newline
        }
        outfile.close();
		


	std::string fileName1 = "C++_NP_3robut3_optimal_trajectory.txt" ;
	std::ofstream outfile1( fileName1.c_str() ) ; // file name and the operation type.        
	
        for(int i=0; i<CoM_ZMP_foot.rows(); i++){
           for(int j=0; j<CoM_ZMP_foot.cols(); j++){
                 outfile1 << (double) CoM_ZMP_foot(i,j) << " " ; 
           }
           outfile1 << std::endl;       // a   newline
        }
        outfile1.close();	
	
  
  
}

