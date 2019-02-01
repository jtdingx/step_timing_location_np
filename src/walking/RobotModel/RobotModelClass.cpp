/*****************************************************************************
RobotModelClass.cpp

Description:  scource file of RobotModelClass

@Version: 1.0
@Author:  Chengxu Zhou (zhouchengxu@gmail.com)
@Release: 2015/10/01
@Update:  Tue 12 Apr 2016 12:03:22 PM CEST
*****************************************************************************/
#include "RobotModelClass.h"

#include <rbdl/addons/urdfreader/urdfreader.h>
// typedef std::map<std::string, double> JointPositionMap;

namespace qpOASES {
const double INFTY = 1.0e20;
}

#ifdef USE_KDL
KDLModelClass::KDLModelClass()
  : nDOF(0)
  , nC_self_pair(0)
  , nC_Obs_pair(0)
  , IsRftSwing(0.0)
  , IsLftSwing(0.0)
  , We(1.0)
  , scalar(0.0)
  , scalar2(0.0)
{
}

void KDLModelClass::InitKDL(const RobotParaClass& robotpara)
{
  dt = robotpara.dt;

  std::cout << "loading urdf to kdl from " << robotpara.urdf_path << std::endl;
  if (!kdl_parser::treeFromFile(robotpara.urdf_path.c_str(), _kdl_tree)) {
    std::cerr << "Error loading urdf model to kdl" << std::endl;
  };

  RThighChain.reset(new RobotChainClass(robotpara, _kdl_tree, RIGHT_THIGH, dt));
  RCalfChain.reset(new RobotChainClass(robotpara, _kdl_tree, RIGHT_CALF, dt));
  RFootChain.reset(new RobotChainClass(robotpara, _kdl_tree, RIGHT_FOOT, dt));
  LThighChain.reset(new RobotChainClass(robotpara, _kdl_tree, LEFT_THIGH, dt));
  LCalfChain.reset(new RobotChainClass(robotpara, _kdl_tree, LEFT_CALF, dt));
  LFootChain.reset(new RobotChainClass(robotpara, _kdl_tree, LEFT_FOOT, dt));
  RHandChain.reset(new RobotChainClass(robotpara, _kdl_tree, RIGHT_HAND, dt));
  LHandChain.reset(new RobotChainClass(robotpara, _kdl_tree, LEFT_HAND, dt));


  FootP1 = KDL::Frame(KDL::Vector(0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                                  0.5 * RobotParaClass::FOOT_WIDTH(),
                                  -RobotParaClass::ANKLE_HEIGHT()));
  FootP2 = KDL::Frame(KDL::Vector(0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                                  -0.5 * RobotParaClass::FOOT_WIDTH(),
                                  -RobotParaClass::ANKLE_HEIGHT()));
  FootP3 = KDL::Frame(KDL::Vector(-0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                                  0.5 * RobotParaClass::FOOT_WIDTH(),
                                  -RobotParaClass::ANKLE_HEIGHT()));
  FootP4 = KDL::Frame(KDL::Vector(-0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                                  -0.5 * RobotParaClass::FOOT_WIDTH(),
                                  -RobotParaClass::ANKLE_HEIGHT()));

  double leg_radius = 0.05;
  AnkleProjection = KDL::Frame(KDL::Vector(0.0, 0.0, -RobotParaClass::ANKLE_HEIGHT() + 0.5 * leg_radius));

  RThighLowLine.setPara("RThighLowLine", RThighChain->EndLinkName, leg_radius);
  RCalfLine.setPara("RCalfLine", RCalfChain->EndLinkName, leg_radius);
  LThighLowLine.setPara("LThighLowLine", LThighChain->EndLinkName, leg_radius);
  LCalfLine.setPara("LCalfLine", LCalfChain->EndLinkName, leg_radius);

  RAnkleLine.setPara("RAnkleLine", LFootChain->EndLinkName, leg_radius);
  LAnkleLine.setPara("LAnkleLine", LFootChain->EndLinkName, leg_radius);

  double foot_edge_radius = 0.01;
  RFootLine1.setPara("RFootLine1", RFootChain->EndLinkName, foot_edge_radius);
  RFootLine2.setPara("RFootLine2", RFootChain->EndLinkName, foot_edge_radius);
  RFootLine3.setPara("RFootLine3", RFootChain->EndLinkName, foot_edge_radius);
  RFootLine4.setPara("RFootLine4", RFootChain->EndLinkName, foot_edge_radius);
  LFootLine1.setPara("LFootLine1", LFootChain->EndLinkName, foot_edge_radius);
  LFootLine2.setPara("LFootLine2", LFootChain->EndLinkName, foot_edge_radius);
  LFootLine3.setPara("LFootLine3", LFootChain->EndLinkName, foot_edge_radius);
  LFootLine4.setPara("LFootLine4", LFootChain->EndLinkName, foot_edge_radius);


  /*
  Foot Description:
        P1   L1   P2          P1   L1   P2
         ----------            ----------
         |        |            |        |
      L4 |        | L2      L4 |        | L2
         |        |            |        |
         | Left   |            | Right  |
         ----------            ----------
        P4   L3   P3          P4   L3   P3
  */

  SelfCollisionPair.push_back(LinkPair(RThighLowLine, LThighLowLine));
  SelfCollisionPair.push_back(LinkPair(RThighLowLine, LCalfLine));
  // SelfCollisionPair.push_back(LinkPair(RThighLowLine, LFootLine2));

  SelfCollisionPair.push_back(LinkPair(RCalfLine, LThighLowLine));
  SelfCollisionPair.push_back(LinkPair(RCalfLine, LCalfLine));
  SelfCollisionPair.push_back(LinkPair(RCalfLine, LFootLine1));
  SelfCollisionPair.push_back(LinkPair(RCalfLine, LFootLine2));
  SelfCollisionPair.push_back(LinkPair(RCalfLine, LFootLine3));
  SelfCollisionPair.push_back(LinkPair(RCalfLine, LAnkleLine));

  SelfCollisionPair.push_back(LinkPair(RAnkleLine, LCalfLine));
  SelfCollisionPair.push_back(LinkPair(RAnkleLine, LAnkleLine));
  SelfCollisionPair.push_back(LinkPair(RAnkleLine, LFootLine1));
  SelfCollisionPair.push_back(LinkPair(RAnkleLine, LFootLine2));
  SelfCollisionPair.push_back(LinkPair(RAnkleLine, LFootLine3));

  SelfCollisionPair.push_back(LinkPair(RFootLine1, LCalfLine));
  // SelfCollisionPair.push_back(LinkPair(RFootLine1, LFootLine1));
  // SelfCollisionPair.push_back(LinkPair(RFootLine1, LFootLine2));
  SelfCollisionPair.push_back(LinkPair(RFootLine1, LFootLine3));
  // SelfCollisionPair.push_back(LinkPair(RFootLine1, LFootLine4));
  SelfCollisionPair.push_back(LinkPair(RFootLine1, LAnkleLine));

  // SelfCollisionPair.push_back(LinkPair(RFootLine2, LFootLine1));
  // SelfCollisionPair.push_back(LinkPair(RFootLine2, LFootLine2));
  // SelfCollisionPair.push_back(LinkPair(RFootLine2, LFootLine3));
  // SelfCollisionPair.push_back(LinkPair(RFootLine2, LFootLine4));

  SelfCollisionPair.push_back(LinkPair(RFootLine3, LCalfLine));
  SelfCollisionPair.push_back(LinkPair(RFootLine3, LFootLine1));
  // SelfCollisionPair.push_back(LinkPair(RFootLine3, LFootLine2));
  // SelfCollisionPair.push_back(LinkPair(RFootLine3, LFootLine3));
  // SelfCollisionPair.push_back(LinkPair(RFootLine3, LFootLine4));
  SelfCollisionPair.push_back(LinkPair(RFootLine3, LAnkleLine));

  // SelfCollisionPair.push_back(LinkPair(RFootLine4, LThighLowLine));
  SelfCollisionPair.push_back(LinkPair(RFootLine4, LCalfLine));
  // SelfCollisionPair.push_back(LinkPair(RFootLine4, LFootLine1));
  SelfCollisionPair.push_back(LinkPair(RFootLine4, LFootLine2));
  // SelfCollisionPair.push_back(LinkPair(RFootLine4, LFootLine3));
  // SelfCollisionPair.push_back(LinkPair(RFootLine4, LFootLine4));
  SelfCollisionPair.push_back(LinkPair(RFootLine4, LAnkleLine));

  nDOF = RFootChain->nDOF + LFootChain->nDOF;
  nC_self_pair = SelfCollisionPair.size();
  nC_Obs_pair = 0 * 8 * 2;

  _A_LBn_eigen = Eigen::MatrixXd::Zero(nC_self_pair, nDOF);
  _lbA_LBn_eigen = Eigen::VectorXd::Zero(nC_self_pair);
  // _ubA_LBn_eigen = Eigen::VectorXd::Zero(nC_self_pair);

  std::cout << "The number of DOF is " << nDOF << std::endl;
  std::cout << "The number of collision pairs is " << nC_self_pair << std::endl;


  std::vector<double> qall_temp(25, 0);
  // qall_temp[4] = -0.5*M_PI;
  // this->Update(qall_temp);
  // RThighChain->Update(qall_temp);
  // RCalfChain->Update(qall_temp);
  // RFootChain->Update(qall_temp);
  // LThighChain->Update(qall_temp);
  // LCalfChain->Update(qall_temp);
  // LFootChain->Update(qall_temp);
  // this->FK();
  UpdateKDL(qall_temp);

  std::cout << "Finish init KDLModelClass." << std::endl;
  std::cout << "=====================================================\n\n\n" << std::endl;

}

void KDLModelClass::UpdateKDL(const std::vector<double>& qall_msr)
{
  RThighChain->Update(qall_msr);
  RCalfChain->Update(qall_msr);
  RFootChain->Update(qall_msr);
  LThighChain->Update(qall_msr);
  LCalfChain->Update(qall_msr);
  LFootChain->Update(qall_msr);
  RHandChain->Update(qall_msr);
  LHandChain->Update(qall_msr);
  RThighFrame = RThighChain->ChainFK();
  RCalfFrame = RCalfChain-> ChainFK();
  RFootFrame = RFootChain-> ChainFK();
  LThighFrame = LThighChain->ChainFK();
  LCalfFrame = LCalfChain-> ChainFK();
  LFootFrame = LFootChain-> ChainFK();


  RThighLowLine.Update(vKDLtoEigen(RThighFrame.p), vKDLtoEigen(RCalfFrame.p));
  RCalfLine.Update(vKDLtoEigen(RCalfFrame.p), vKDLtoEigen(RFootFrame.p));
  LThighLowLine.Update(vKDLtoEigen(LThighFrame.p), vKDLtoEigen(LCalfFrame.p));
  LCalfLine.Update(vKDLtoEigen(LCalfFrame.p), vKDLtoEigen(LFootFrame.p));

  RFootLine1.Update(vKDLtoEigen((RFootFrame * FootP1).p), vKDLtoEigen((RFootFrame * FootP2).p));
  RFootLine2.Update(vKDLtoEigen((RFootFrame * FootP2).p), vKDLtoEigen((RFootFrame * FootP3).p));
  RFootLine3.Update(vKDLtoEigen((RFootFrame * FootP3).p), vKDLtoEigen((RFootFrame * FootP4).p));
  RFootLine4.Update(vKDLtoEigen((RFootFrame * FootP4).p), vKDLtoEigen((RFootFrame * FootP1).p));
  LFootLine1.Update(vKDLtoEigen((LFootFrame * FootP1).p), vKDLtoEigen((LFootFrame * FootP2).p));
  LFootLine2.Update(vKDLtoEigen((LFootFrame * FootP2).p), vKDLtoEigen((LFootFrame * FootP3).p));
  LFootLine3.Update(vKDLtoEigen((LFootFrame * FootP3).p), vKDLtoEigen((LFootFrame * FootP4).p));
  LFootLine4.Update(vKDLtoEigen((LFootFrame * FootP4).p), vKDLtoEigen((LFootFrame * FootP1).p));

  RAnkleLine.Update(vKDLtoEigen(RFootFrame.p), vKDLtoEigen((RFootFrame * AnkleProjection).p));
  LAnkleLine.Update(vKDLtoEigen(LFootFrame.p), vKDLtoEigen((LFootFrame * AnkleProjection).p));

  SelfCollisionCheck();
}

void KDLModelClass::SelfCollisionCheck()
{
  for (int i = 0; i < nC_self_pair; ++i) {
    // COUT(SelfCollisionPair[i].IsTooClose(), IsRftSwing, IsLftSwing);
    if (SelfCollisionPair[i].IsTooClose() && (IsRftSwing || IsLftSwing)) {
      // if (SelfCollisionPair[i].IsTooClose()) {
      Eigen::MatrixXd jaco = this->getJacobian(SelfCollisionPair[i].S1.EndLinkName);
      SelfCollisionPair[i].S1.UpdateJacobian(jaco);
      jaco = this->getJacobian(SelfCollisionPair[i].S2.EndLinkName);
      SelfCollisionPair[i].S2.UpdateJacobian(jaco);
      SelfCollisionPair[i].Update();
      // Eigen::RowVectorXd A_row = SelfCollisionPair[i].A_row();
      _A_LBn_eigen.row(i) = SelfCollisionPair[i].A_row();
      _lbA_LBn_eigen(i) = SelfCollisionPair[i].B_row();

      // // disable Collision Constraints for support leg. NOT correct.
      // if(IsLftSwing){
      //   _A_LBn_eigen.row(i).head(6).setZero();
      // }
      // else if(IsRftSwing){
      //   _A_LBn_eigen.row(i).tail(6).setZero();
      // }
      // // _ubA_LBn_eigen(i) = qpOASES::INFTY;
      // SelfCollisionPair[i].PrintSelf();
    }
    else {
      // _A_LBn_eigen.block(i+nC_Jaco,0,1,nDOF) = SelfCollisionPair[i].A_row();
      _A_LBn_eigen.row(i) = Eigen::MatrixXd::Zero(1, nDOF);
      _lbA_LBn_eigen(i) = 0.0;
      // _ubA_LBn_eigen(i) = 0.0;

    }

    // if(int(IsLftSwing)*SelfCollisionPair[25].IsTooClose()){
    //   // std::cout<<SelfCollisionPair[25].minDis<<std::endl;
    //   SelfCollisionPair[25].PrintSelf();
    // }
    // SelfCollisionPair[25].PrintSelf();
  }
}


void KDLModelClass::WhichFoot(const int& which, const double& percent)
{
  if (which == -1) { // Left Swing
    IsRftSwing = 0.0;
    IsLftSwing = 1.0;
  }
  else if (which == 1) { // Right Swing
    IsRftSwing = 1.0;
    IsLftSwing = 0.0;
  }
  else {
    IsRftSwing = 0.0;
    IsLftSwing = 0.0;
    // sum_e.assign(sum_e.size(), 0.0);
  }

  // double Lgain = 1e2;
  // double Hgain = 1e4;

  // suitable for cross leg walk
  // double Lgain = 1e5;
  // double Hgain = 1e9;

  double Lgain = 1e-4;
  double Hgain = 1e0;

  double BeginSS = 0.1;
  double EndSS = 0.7;
  double Interval = 0.1;
  FadeOutRatio(which, percent, BeginSS, EndSS, Interval);
  We = setWeight(scalar, Lgain, Hgain);

  // // //---  for footplacement modification in X
  // // Lgain = 2e1;
  // // vecW(0) = vecW(6) = setWeight(scalar, Lgain, Hgain);
  // dXref[0] = We;
  // dXref[1] = scalar;
  // dXref[2] = scalar2;

  // int nV = _qpsolverLB->getNV();
  // const int nD2 = 0.5 * nDOF;
  // double k = 1.0 * scalar;
  // double a1 = 1.0;
  // _g_LBn_eigen = Eigen::MatrixXd::Zero(nV, 1);
  // if (IsRftSwing == 1.0) {
  //   std::vector<double> Rft_mani = CalcManipulabilityGradient(RFootChain);
  //   for (int i = 0; i < nD2; ++i) {
  //     _g_LBn_eigen(i) = a1 * Rft_mani[i];
  //   }
  //   // // _g_LBn_eigen(1) -= (1-a1)*2.0; // hip roll

  //   // // std::cout<<" Rft_mani: ";
  //   // // for (auto i : Rft_mani)  std::cout << i <<'\t';
  //   // // std::cout<<std::endl;

  //   // // for support leg z modification
  //   // _lb_LBn[nDOF + 2] = -4.0;
  //   // _ub_LBn[nDOF + 2] = 0.0;
  //   // _lb_LBn[nDOF + 8] = 0.2 * scalar * EnableSS;
  //   // _ub_LBn[nDOF + 8] = 4.0;
  // }

  // if (IsLftSwing == 1.0) {
  //   std::vector<double> Lft_mani = CalcManipulabilityGradient(LFootChain);
  //   for (int i = 0; i < nD2; ++i) {
  //     _g_LBn_eigen(i + nD2) = a1 * Lft_mani[i];
  //   }
  //   // // _g_LBn_eigen(1+nD2) += (1-a1)*2.0;

  //   // // for support leg z modification
  //   // _lb_LBn[nDOF + 2] = 0.2 * scalar * EnableSS;
  //   // _ub_LBn[nDOF + 2] = 4.0;
  //   // _lb_LBn[nDOF + 8] = -4.0;
  //   // _ub_LBn[nDOF + 8] = 0.0;
  // }
  // _g_LBn_eigen *= -2 * k;
}

double KDLModelClass::FadeOutRatio(double whichfoot, double percent, double BeginSS, double EndSS, double Interval)
{
  scalar2 = scalar = 0.0;

  if (whichfoot != 0) {
    if (percent <= (BeginSS + Interval)) {
      scalar = Poly3(0.0, 0.0, BeginSS, 0.0, 1.0, BeginSS + Interval, percent);
      if (scalar > 0.0) {scalar2 = 1.0;};
    }
    else {
      scalar = Poly3(0.0, 1.0, EndSS - 1.0 * Interval, 0.0, 0.0, EndSS, percent);
      if (scalar > 0.9) {scalar2 = 1.0;};
    }
  }

  return scalar;
}

void KDLModelClass::UpdateCollisionInEq(Eigen::MatrixXd& CI, Eigen::Vector3d& ci)
{
  // CI*x+ci>=0


}

double KDLModelClass::CalcManipulability(const int& body_id)
{
  Eigen::MatrixXd jaco = this->getJacobian(body_id);
  double manipulability = std::sqrt((jaco * jaco.transpose()).determinant());

  // if(manipulability==0.0){
  //   return qpOASES::INFTY;
  // }
  // else{
  //   return k/manipulability;
  // }
  return manipulability;
}

double KDLModelClass::CalcManipulability(const Eigen::MatrixXd& jaco)
{
  return std::sqrt((jaco * jaco.transpose()).determinant());
}

Eigen::MatrixXd KDLModelClass::getJacobian(const int& body_id)
{
  if (body_id == RThighChain->EndLinkName) {
    return RThighChain->Jacobian();
  }
  else if (body_id == RCalfChain->EndLinkName) {
    return RCalfChain->Jacobian();
  }
  else if (body_id == RThighChain->EndLinkName) {
    return RThighChain->Jacobian();
  }
  else if (body_id == RFootChain->EndLinkName) {
    return RFootChain->Jacobian();
  }
  else if (body_id == LThighChain->EndLinkName) {
    return LThighChain->Jacobian();
  }
  else if (body_id == LCalfChain->EndLinkName) {
    return LCalfChain->Jacobian();
  }
  else if (body_id == LFootChain->EndLinkName) {
    return LFootChain->Jacobian();
  }
  else if (body_id == LHandChain->EndLinkName) {
    return LHandChain->Jacobian();
  }
  else if (body_id == RHandChain->EndLinkName) {
    return RHandChain->Jacobian();
  }
  else {
    std::cerr << "not defined jacobian in kdl....body_id is " << body_id << std::endl;
    assert(!body_id);
  }
}

Eigen::MatrixXd KDLModelClass::getJdot(const int& body_id)
{
  if (body_id == RIGHT_FOOT) {
    return RFootChain->Jdot();
  }
  else if (body_id == LEFT_FOOT) {
    return LFootChain->Jdot();
  }
  else if (body_id == RIGHT_HAND) {
    return RHandChain->Jdot();
  }
  else if (body_id == LEFT_HAND) {
    return LHandChain->Jdot();
  }
  else {
    std::cerr << "not defined Jdot in kdl.....\n";
    assert(!body_id);
  }
}

Eigen::Vector6d KDLModelClass::getJdotQdot(const int& body_id)
{
  if (body_id == RIGHT_FOOT) {
    return RFootChain->Jdotqdot();
  }
  else if (body_id == LEFT_FOOT) {
    return LFootChain->Jdotqdot();
  }
  else if (body_id == RIGHT_HAND) {
    return RHandChain->Jdotqdot();
  }
  else if (body_id == LEFT_HAND) {
    return LHandChain->Jdotqdot();
  }
  else {
    std::cerr << "not defined JdotQdot in kdl.....\n";
    assert(!body_id);
  }
}

double KDLModelClass::Poly3(double ds0, double s0, double t0, double dsf, double sf, double tf, double time)
{
  double a0 = s0;
  double a1 = ds0;
  double a2 = (3.0 * (sf - s0) - (2.0 * ds0 + dsf) * (tf - t0) ) /  ( (tf - t0) * (tf - t0) );
  double a3 = (-2.0 * (sf - s0) + (ds0 + dsf) * (tf - t0) ) / ( (tf - t0) * (tf - t0) * (tf - t0) ) ;
  double t = time - t0;
  double s;
  if (time <= t0) {
    s = s0;
  }
  else if (time >= tf) {
    s = sf;
  }
  else {
    s = a0 + a1 * t + a2 * t * t + a3 * t * t * t;
  }
  return s;
}

#endif
//=====================================================================

namespace RigidBodyDynamics {
namespace Utils {

void UpdateBodyCOM(Model &model, const unsigned int id, Math::Vector3d com)
{
  model.mBodies[id].mCenterOfMass = com;
  model.Ic[id].h = com;
  model.I[id].h = com;
};

void CalcCentroidalProperty(Model &model,
                            const Math::VectorNd &q,
                            const Math::VectorNd &qdot,
                            double &mass,
                            Math::Vector3d &com,
                            Math::Vector3d *com_velocity,
                            Math::Vector3d *angular_momentum,
                            Math::MatrixNd *centroidal_momentum_matrix,
                            bool update_kinematics)
{
  if (update_kinematics) {
    UpdateKinematicsCustom(model, &q, &qdot, NULL);
  }

  for (size_t i = 1; i < model.mBodies.size(); i++) {
    model.Ic[i] = model.I[i];
  }

  Math::SpatialRigidBodyInertia Itot(0., Math::Vector3d(0., 0., 0.), Math::Matrix3d::Zero(3, 3));

  for (size_t i = model.mBodies.size() - 1; i > 0; i--) {
    unsigned int lambda = model.lambda[i];

    if (lambda != 0) {
      model.Ic[lambda] = model.Ic[lambda] + model.X_lambda[i].applyTranspose(model.Ic[i]);
    }
    else {
      Itot = Itot + model.X_lambda[i].applyTranspose(model.Ic[i]);
    }
  }

  mass = Itot.m;
  com = Itot.h / mass;

  if (centroidal_momentum_matrix) {
    std::vector<Math::SpatialTransform> X_G = model.X_lambda;
    X_G[0] = Math::SpatialTransform(Math::Matrix3d::Identity(3, 3), -com);
    for (size_t i = 1; i < model.mBodies.size(); i++) {
      unsigned int lambda = model.lambda[i];

      X_G[i] = model.X_lambda[i] * X_G[lambda];
      centroidal_momentum_matrix->col(i - 1) = X_G[i].toMatrixTranspose() * model.Ic[i].toMatrix() * model.S[i];
    }
  }

  Math::VectorNd momentum(6);

  if (com_velocity) {
    momentum = (*centroidal_momentum_matrix) * qdot;
    *com_velocity = momentum.tail(3) * (1 / mass);
  }

  if (angular_momentum) {
    *angular_momentum = momentum.head(3);
  }
};
}

}

void RBDLModelClass::InitRBDL(const RobotParaClass& robotpara)
{
  std::cout << "\n\n\n============= Initializing RBDLModelClass ============" << std::endl;
  dt = robotpara.dt;

  rbdl240 = (2 << 16) + (4 << 8) + 0;
  rbdl250 = (2 << 16) + (5 << 8) + 0;

  if (RBDL_API_VERSION == rbdl250) {
    COUT("RBDL version 2.5.0 is loaded.");
  }
  else if (RBDL_API_VERSION == rbdl240) {
    COUT("RBDL version 2.4.0 is loaded.");
  }
  else {
    COUT("RBDL version is not correct!\n\n\n\n\n\n\n\n\n\n\n");
    assert(0);
  }

  // _rbdl_model = new RBDL::Model();
  std::cout << "loading urdf to rbdl from " << robotpara.urdf_path << std::endl;
  bool IsVerbose = false;

  if (RBDL_API_VERSION == rbdl250) {
    bool IsFloating = true; // for rbdl 2.5.0
    if (!RBDL::Addons::URDFReadFromFile (robotpara.urdf_path.c_str(), &_rbdl_model, IsFloating, IsVerbose)) {
      std::cerr << "Error loading urdf model to rbdl" << std::endl;
    }
  }
  else if (RBDL_API_VERSION == rbdl240) {
    if (!RBDL::Addons::URDFReadFromFile (robotpara.urdf_path.c_str(), &_rbdl_model, IsVerbose)) {
      std::cerr << "Error loading urdf model to rbdl" << std::endl;
    }
  }

  _rbdl_model.gravity.set (0., 0., -robotpara.g);

  // for (std::map<std::string, unsigned int>::iterator it = _rbdl_model->mBodyNameMap.begin(); it != _rbdl_model->mBodyNameMap.end(); ++it) {
  for (auto it : _rbdl_model.mBodyNameMap) {
    if (it.first != "ROOT" && it.first != "Waist_Translate" && it.second < 50 && it.second >= 0) {
      // if (1) {
      if ((robotpara.getLinkName(it.first) < 50) && (robotpara.getLinkName(it.first) >= 0)) {
        _map_LinkName_to_rbdl[robotpara.getLinkName(it.first)] = it.second;
        if (IsVerbose) {
          // if (true) {
          std::cout << it.first << " => " << it.second << " _map_LinkName_to_rbdl " << _map_LinkName_to_rbdl.find(robotpara.getLinkName(it.first))->first << " => " << _map_LinkName_to_rbdl.find(robotpara.getLinkName(it.first))->second << '\n';
        }
        if (robotpara.getLinkName(it.first) != PELVIS) {
          _map_JointName_to_rbdl[robotpara.getJointName(robotpara.getLink(it.first)->parent_joint->name)] = it.second - 1; // -1 is because in rbdl the parent joints id is the same as child links id, since PELVIS has no parent joint except the base_joint, therefore the parent joint id should -1.
          // if (IsVerbose) {
          if (true) {
            std::cout << it.first << " => " << it.second << " ==> " << robotpara.getJointName(_map_JointName_to_rbdl.find(robotpara.getJointName(robotpara.getLink(it.first)->parent_joint->name))->first) << " -=> " << _map_JointName_to_rbdl.find(robotpara.getJointName(robotpara.getLink(it.first)->parent_joint->name))->first << " +=> " << _map_JointName_to_rbdl.find(robotpara.getJointName(robotpara.getLink(it.first)->parent_joint->name))->second << '\n';
          }
        }
      }
    }
  }

  // if (IsVerbose) {
  if (true) {
    for (auto it : _map_JointName_to_rbdl) {
      COUT(robotpara.getJointName(it.first), it.first, it.second);
      // COUT(_rbdl_model.multdof3_S[it.second], "\n");
    }
  }

  // update the feet com for COMAN
  if (robotpara.name == "coman" || robotpara.name == "bigman") {
    RBDL::Math::Vector3d foot_com(robotpara.ankle_x_offset, 0.0, -robotpara.ankle_height + 0.5 * robotpara.foot_height);
    RBDL::Utils::UpdateBodyCOM(_rbdl_model, _map_LinkName_to_rbdl[LEFT_FOOT], foot_com);
    RBDL::Utils::UpdateBodyCOM(_rbdl_model, _map_LinkName_to_rbdl[RIGHT_FOOT], foot_com);
  }

  nDoF_floating = _rbdl_model.dof_count;
  nLink_floating = _rbdl_model.mBodies.size();
  std::cout << "loaded rbdl model with dof " << _rbdl_model.dof_count << "\t link num " << _map_LinkName_to_rbdl.size() << "\t joint num " << _map_JointName_to_rbdl.size()  << std::endl;

  std::cout << "_rbdl_model.mBodies.size() " << _rbdl_model.mBodies.size()  << std::endl;

  if (!robotpara.IsOnlyLowerBody) {
    assert((_rbdl_model.dof_count - 6) == robotpara.JOINT_NUM());

    if (RBDL_API_VERSION == rbdl250) {
      assert((_rbdl_model.mBodies.size() - 2) == robotpara.LINK_NUM()); // for rbdl 2.5.0
    }
    else if (RBDL_API_VERSION == rbdl240) {
     assert((_rbdl_model.mBodies.size() - 6) == robotpara.LINK_NUM()); // for rbdl 2.4.0
    }
  }

  if (IsVerbose) {
    for (auto it : _rbdl_model.mBodyNameMap) {
      COUT(it.first, it.second, _rbdl_model.lambda[it.second]); // lambda is The id of the parents body.
      COUT(_rbdl_model.I[it.second].m, "        ", _rbdl_model.I[it.second].h.transpose());
      COUT(" ");
    }
    COUT("_rbdl_model.fixed_body_discriminator", _rbdl_model.fixed_body_discriminator);
    // std::cout << RBDL::Utils::GetModelDOFOverview(_rbdl_model) << std::endl;
    // std::cout << RBDL::Utils::GetModelHierarchy(_rbdl_model) << std::endl;
    std::cout << RBDL::Utils::GetNamedBodyOriginsOverview(_rbdl_model) << std::endl;
  }

  Jcom =  RBDL::Math::MatrixNd::Zero(3, nDoF_floating);
  dJcom =  RBDL::Math::MatrixNd::Zero(3, nDoF_floating);
  _CMM =  RBDL::Math::MatrixNd::Zero(6, nDoF_floating);
  _dCMM =  RBDL::Math::MatrixNd::Zero(6, nDoF_floating);
  _CMM_old =  RBDL::Math::MatrixNd::Zero(6, nDoF_floating);
  _H =  RBDL::Math::MatrixNd::Zero(nDoF_floating, nDoF_floating);
  _C = RBDL::Math::VectorNd::Zero(nDoF_floating);

  q_all_fixed = RBDL::Math::VectorNd::Zero(nDoF_floating);
  dq_all_fixed = RBDL::Math::VectorNd::Zero(nDoF_floating);

  q_all_floating = RBDL::Math::VectorNd::Zero(nDoF_floating);
  q_all_floating_old = RBDL::Math::VectorNd::Zero(nDoF_floating);
  dq_all_floating = RBDL::Math::VectorNd::Zero(nDoF_floating);
  ddq_all_floating = RBDL::Math::VectorNd::Zero(nDoF_floating);
  tau_all_floating = RBDL::Math::VectorNd::Zero(nDoF_floating);
  com_floating = RBDL::Math::Vector3d::Zero();
  mass = 0.0;
  _ankle_offset << 0, 0, -RobotParaClass::ANKLE_HEIGHT();
  q_all_floating[2] = robotpara.waist_height;
  // q_all_floating[2] = robotpara.full_leg;

  // q_all_floating[4+getRBDLJointID(RIGHT_HIP_PITCH)] = DEGTORAD(-20);
  // q_all_floating[4+getRBDLJointID(RIGHT_KNEE_PITCH)] = DEGTORAD(40);
  // q_all_floating[4+getRBDLJointID(RIGHT_FOOT_PITCH)] = DEGTORAD(-20);

  // COUT("q_all_floating",q_all_floating.transpose());

  // q_all_floating[0] = 3;
  // q_all_floating[1] = -2;
  // q_all_floating[2] = 5;
  // q_all_floating[3] = DEGTORAD(-90);
  // q_all_floating[4] = DEGTORAD(-90);
  // q_all_floating[5] = DEGTORAD(-90);
  // q_all_floating[13] = DEGTORAD(-90);
  // COUT("getRBDLJointID(RIGHT_HIP_ROLL)]:",getRBDLJointID(RIGHT_HIP_ROLL));
  // q_all_floating[getRBDLJointID(RIGHT_HIP_ROLL)] = DEGTORAD(-90);
  // q_all_floating[getRBDLJointID(RIGHT_HIP_PITCH)] = DEGTORAD(-90);
  // q_all_floating[_map_JointName_to_rbdl[RIGHT_FOOT_PITCH]] = DEGTORAD(30);
  // q_all_floating[0] = 100;
  // q_all_floating[1] = -30;
  // q_all_floating[3] = DEGTORAD(45);
  // q_all_floating[4] = DEGTORAD(45);
  // q_all_floating[5] = DEGTORAD(45);
  CalcCenterOfMass(true);
  // InverseDynamics();
  // getJacobian(RIGHT_FOOT);
  // COUT(getLocalBodyFrame(RIGHT_FOOT));
  // COUT(InertiaMatrix(),"\n");
  // COUT(NonlinearEffects(),"\n");
  //
  // COUT("lsole", lsole);
  // COUT("lankle", lankle);
  // COUT("rsole", rsole);
  // COUT("rankle", rankle);

  // COUT("\n getGloballBodyPosition(RIGHT_FOOT): ");
  // COUT(getGloballBodyPosition(RIGHT_FOOT).transpose());

  // com_ft = Rot_World_to_Pelvis * (com_floating - 0.5 * (getGloballBodyPosition(LEFT_FOOT) + getGloballBodyPosition(RIGHT_FOOT)));
  // DPRINTF("The robot COM is at %.3f %.3f %.4f m.\n", com_ft[0], com_ft[1], com_ft[2]);
  // com_ft = Rot_World_to_Pelvis * (getGloballBodyPosition(PELVIS) - 0.5 * (getGloballBodyPosition(LEFT_FOOT) + getGloballBodyPosition(RIGHT_FOOT)));
  // DPRINTF("The pelvis COM is at %.3f %.3f %.4f m.\n", com_ft[0], com_ft[1], com_ft[2]);


  // RBDL::Math::SpatialTransform Rpelvis = _rbdl_model.X_base[_map_LinkName_to_rbdl[PELVIS]];
  // RBDL::Math::SpatialTransform Rfoot = _rbdl_model.X_base[_map_LinkName_to_rbdl[RIGHT_FOOT]];
  // COUT("\n Rpelvis: ");
  // COUT(Rpelvis);
  // COUT("\n Rot_World_to_Pelvis: ");
  // COUT(Rot_World_to_Pelvis);
  // // COUT("\n Rpelvis.inverse(): ");
  // // COUT(Rpelvis.inverse());
  // // COUT("\n Rfoot: ");
  // // COUT(Rfoot);
  // // COUT("\n Rpelvis.inverse()*Rfoot: ");
  // // COUT(Rpelvis.inverse()*Rfoot);
  // // COUT("\n Rfoot*Rpelvis.inverse(): "); // this combination is correct in position, but rotation needs to transpose
  // // COUT(Rfoot * Rpelvis.inverse());
  // // COUT("\n Rfoot.inverse()*Rpelvis: ");
  // // COUT(Rfoot.inverse()*Rpelvis);
  // // COUT("\n Rpelvis*Rfoot.inverse(): ");
  // // COUT(Rpelvis * Rfoot.inverse());

  // RBDL::Math::SpatialTransform rsole_pose_ = CalcFrameTransform(getRBDLBodyID(RIGHT_FOOT), Eigen::Matrix3d::Identity(), ankle_offset_);


  // RBDL::Math::SpatialTransform local_origin_ = CalcFrameTransform(getRBDLBodyID(PELVIS), Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, 0));

  // Eigen::Vector3d rsole_pose_local = Rot_World_to_Pelvis * (rsole_pose_.r - local_origin_.r);

  // COUT("\n local_origin_: ");
  // COUT(local_origin_);
  // COUT("\n rsole_pose_: ");
  // COUT(rsole_pose_);
  // COUT("\n rsole_pose_local: ");
  // COUT(rsole_pose_local.transpose());

  // for (auto it : _rbdl_model.mBodyNameMap) {
  //  if (it.first != "ROOT") {
  //    if (!IsVerbose) {
  //      std::cout << it.first << " => " << it.second << " _map_LinkName_to_rbdl " << _map_LinkName_to_rbdl.find(robotpara.getLinkName(it.first))->first << " => " << _map_LinkName_to_rbdl.find(robotpara.getLinkName(it.first))->second << '\n';
  //      COUT(_rbdl_model.X_base[it.second]);
  //      COUT(" ");
  //    }
  //  }
  // }
  // if (!IsVerbose) {
  //   for (auto it : _map_JointName_to_rbdl) {
  //     COUT(robotpara.getJointName(it.first), it.first, it.second);
  //   }
  // }
  // int i = 0;
  // for (auto it : _rbdl_model.mBodies) {
  //   COUT(i++,it.mMass);
  // }
  // COUT(centroidal_momentum_matrix());


  // -------------------------------------------------------------------
  // construct contact points
  // -------------------------------------------------------------------
  /*
  Foot Description:
      P1   L1   P2          P1   L1   P2
       ----------            ----------
       |        |            |        |
    L4 |        | L2      L4 |        | L2
       |        |            |        |
       | Left   |            | Right  |
       ----------            ----------
      P4   L3   P3          P4   L3   P3
  */

  Eigen::Vector3d FootP1(0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                         0.5 * RobotParaClass::FOOT_WIDTH(),
                         -RobotParaClass::ANKLE_HEIGHT());
  Eigen::Vector3d FootP2(0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                         -0.5 * RobotParaClass::FOOT_WIDTH(),
                         -RobotParaClass::ANKLE_HEIGHT());
  Eigen::Vector3d FootP3(-0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                         0.5 * RobotParaClass::FOOT_WIDTH(),
                         -RobotParaClass::ANKLE_HEIGHT());
  Eigen::Vector3d FootP4(-0.5 * RobotParaClass::FOOT_LENGTH() + RobotParaClass::ANKLE_X_OFFSET(),
                         -0.5 * RobotParaClass::FOOT_WIDTH(),
                         -RobotParaClass::ANKLE_HEIGHT());
  Eigen::Vector3d axis_x(1., 0., 0.);
  Eigen::Vector3d axis_y(0., 1., 0.);
  Eigen::Vector3d axis_z(0., 0., 1.);

  int lft_id = getRBDLBodyID(LEFT_FOOT);
  constraint_set_lft.AddConstraint(lft_id, FootP1, axis_z, "left_p1_z");
  constraint_set_lft.AddConstraint(lft_id, FootP2, axis_z, "left_p2_z");
  constraint_set_lft.AddConstraint(lft_id, FootP3, axis_z, "left_p3_z");
  constraint_set_lft.AddConstraint(lft_id, FootP4, axis_z, "left_p4_z");
  constraint_set_lft.Bind (_rbdl_model);


  int rft_id = getRBDLBodyID(RIGHT_FOOT);
  constraint_set_rft.AddConstraint(rft_id, FootP1, axis_z, "right_p1_z");
  constraint_set_rft.AddConstraint(rft_id, FootP2, axis_z, "right_p2_z");
  constraint_set_rft.AddConstraint(rft_id, FootP3, axis_z, "right_p3_z");
  constraint_set_rft.AddConstraint(rft_id, FootP4, axis_z, "right_p4_z");
  constraint_set_rft.Bind (_rbdl_model);


  constraint_set_bothft.AddConstraint(lft_id, FootP1, axis_x, "left_p1_x");
  constraint_set_bothft.AddConstraint(lft_id, FootP2, axis_x, "left_p2_x");
  constraint_set_bothft.AddConstraint(lft_id, FootP3, axis_x, "left_p3_x");
  constraint_set_bothft.AddConstraint(lft_id, FootP4, axis_x, "left_p4_x");
  constraint_set_bothft.AddConstraint(rft_id, FootP1, axis_x, "right_p1_x");
  constraint_set_bothft.AddConstraint(rft_id, FootP2, axis_x, "right_p2_x");
  constraint_set_bothft.AddConstraint(rft_id, FootP3, axis_x, "right_p3_x");
  constraint_set_bothft.AddConstraint(rft_id, FootP4, axis_x, "right_p4_x");

  constraint_set_bothft.AddConstraint(lft_id, FootP1, axis_y, "left_p1_y");
  constraint_set_bothft.AddConstraint(lft_id, FootP2, axis_y, "left_p2_y");
  constraint_set_bothft.AddConstraint(lft_id, FootP3, axis_y, "left_p3_y");
  constraint_set_bothft.AddConstraint(lft_id, FootP4, axis_y, "left_p4_y");
  constraint_set_bothft.AddConstraint(rft_id, FootP1, axis_y, "right_p1_y");
  constraint_set_bothft.AddConstraint(rft_id, FootP2, axis_y, "right_p2_y");
  constraint_set_bothft.AddConstraint(rft_id, FootP3, axis_y, "right_p3_y");
  constraint_set_bothft.AddConstraint(rft_id, FootP4, axis_y, "right_p4_y");

  constraint_set_bothft.AddConstraint(lft_id, FootP1, axis_z, "left_p1_z");
  constraint_set_bothft.AddConstraint(lft_id, FootP2, axis_z, "left_p2_z");
  constraint_set_bothft.AddConstraint(lft_id, FootP3, axis_z, "left_p3_z");
  constraint_set_bothft.AddConstraint(lft_id, FootP4, axis_z, "left_p4_z");
  constraint_set_bothft.AddConstraint(rft_id, FootP1, axis_z, "right_p1_z");
  constraint_set_bothft.AddConstraint(rft_id, FootP2, axis_z, "right_p2_z");
  constraint_set_bothft.AddConstraint(rft_id, FootP3, axis_z, "right_p3_z");
  constraint_set_bothft.AddConstraint(rft_id, FootP4, axis_z, "right_p4_z");

  // constraint_set_bothft.Bind (_rbdl_model);

  std::cout << "Finish init RBDLModelClass." << std::endl;
  std::cout << "=====================================================\n\n\n" << std::endl;
}

void RBDLModelClass::UpdateRBDL(const std::vector<double>& qall_msr)
{
  q_all_floating_old = q_all_floating;
  dq_all_floating_old = dq_all_floating;
  vJointNameToRBDL(qall_msr, q_all_floating); // without the first 6 floating base DoF
  dq_all_floating = (q_all_floating - q_all_floating_old) / dt;
  ddq_all_floating = (dq_all_floating - dq_all_floating_old) / dt;

  CalcCenterOfMass(true);
  // InverseDynamics();

  // _CMM = centroidal_momentum_matrix();
  _CMM = CalcCMM();
  _dCMM = (_CMM - _CMM_old) / dt;
  _CMM_old = _CMM;
  Jcom = _CMM.topRows(3) / mass;
  dJcom = _dCMM.topRows(3) / mass;
}

void RBDLModelClass::UpdateRBDL(const Eigen::VectorXd& qall_msr)
{
  q_all_floating_old = q_all_floating;
  dq_all_floating_old = dq_all_floating;
  q_all_floating.segment(6, qall_msr.size()) = qall_msr;
  // vJointNameToRBDL(qall_msr, q_all_floating); // without the first 6 floating base DoF
  dq_all_floating = (q_all_floating - q_all_floating_old) / dt;
  ddq_all_floating = (dq_all_floating - dq_all_floating_old) / dt;

  CalcCenterOfMass(true);
  // InverseDynamics();

  // _CMM = centroidal_momentum_matrix();
//   _CMM = CalcCMM();
//   _dCMM = (_CMM - _CMM_old) / dt;
//   _CMM_old = _CMM;
//   Jcom = _CMM.topRows(3) / mass;
//   dJcom = _dCMM.topRows(3) / mass;
}

void RBDLModelClass::CalcCenterOfMass(const bool & IsUpdateKinematics)
{
  // RBDL::Math::Vector3d com_velocity, angular_momentum;
  q_all_fixed.tail(nDoF_floating - 6) = q_all_floating.tail(nDoF_floating - 6);
  dq_all_fixed.tail(nDoF_floating - 6) = dq_all_floating.tail(nDoF_floating - 6);
  RBDL::Utils::CalcCenterOfMass(_rbdl_model, q_all_fixed, dq_all_fixed, mass, com_hip, &dcom_hip, &angular_momentum, false);
  RBDL::Utils::CalcCenterOfMass(_rbdl_model, q_all_floating, dq_all_floating, mass, com_floating, &com_velocity, &angular_momentum, false); // com in base_pos which is changing, so com_floating is not used anywhere else for now.

  if(IsUpdateKinematics){
      RBDL::UpdateKinematics(_rbdl_model, q_all_floating, dq_all_floating, ddq_all_floating);
  }
  // COUT("inital com in rbdl. ", com_floating.transpose());
  // COUT("mass in rbdl. ", mass);
  // COUT("com_velocity ", com_velocity.transpose());
  // COUT("angular_momentum ", angular_momentum.transpose());

  // CalcPotentialEnergy
  // RBDL::Math::Vector3d g = - RBDL::Math::Vector3d (_rbdl_model.gravity[0], _rbdl_model.gravity[1], _rbdl_model.gravity[2]);
  Ep = mass * com_floating.dot(-_rbdl_model.gravity);

  // CalcKineticEnergy
  double result = 0.0;
  for (size_t i = 1; i < _rbdl_model.mBodies.size(); i++) {
    result += 0.5 * _rbdl_model.v[i].transpose() * (_rbdl_model.I[i] * _rbdl_model.v[i]);
  }
  Ek = result;

  Rot_World_to_Pelvis = _rbdl_model.X_base[_map_LinkName_to_rbdl[PELVIS]].E;
  local_origin_PelvisProj = CalcFrameTransform(getRBDLBodyID(PELVIS), Eigen::Matrix3d::Identity(), Eigen::Vector3d(0, 0, -RobotParaClass::FULL_LEG()));

  lankle = getLocalBodyFrame(LEFT_FOOT);
  rankle = getLocalBodyFrame(RIGHT_FOOT);
  lwrist = getLocalBodyFrame(LEFT_HAND);
  rwrist = getLocalBodyFrame(RIGHT_HAND);
  lsole = CalcFrameTransform(getRBDLBodyID(LEFT_FOOT), Eigen::Matrix3d::Identity(), _ankle_offset, getRBDLBodyID(PELVIS));
  rsole = CalcFrameTransform(getRBDLBodyID(RIGHT_FOOT), Eigen::Matrix3d::Identity(), _ankle_offset, getRBDLBodyID(PELVIS));

  com_ft = Rot_World_to_Pelvis * (com_hip - 0.5 * (lsole.r + rsole.r));
  // com_hip = Rot_World_to_Pelvis * (com_floating - _rbdl_model.X_base[_map_LinkName_to_rbdl[PELVIS]].r);

  // std::cout << "kine_energy = " << Ek << " pot_energy =  " << Ep << " mass = " << mass << " com = " << com_floating.transpose() << std::endl;
}

void RBDLModelClass::UpdateKinematicsOnce()
{
  // RBDL::UpdateKinematicsCustom(_rbdl_model, &q_all_floating, NULL, NULL);
  RBDL::UpdateKinematics(_rbdl_model, q_all_floating, dq_all_floating, ddq_all_floating);
  lsole = CalcFrameTransform(getRBDLBodyID(LEFT_FOOT), Eigen::Matrix3d::Identity(), _ankle_offset, getRBDLBodyID(PELVIS));
  rsole = CalcFrameTransform(getRBDLBodyID(RIGHT_FOOT), Eigen::Matrix3d::Identity(), _ankle_offset, getRBDLBodyID(PELVIS));
}

Eigen::Vector3d RBDLModelClass::PositionFromGlobal2Hip(const Eigen::Vector3d& from)
{
  return Rot_World_to_Pelvis * (from - getGloballBodyPosition(PELVIS));
}

Eigen::Vector3d RBDLModelClass::VelocityFromGlobal2Hip(const Eigen::Vector3d& vel_from)
{
  return Rot_World_to_Pelvis * (vel_from - getGloballBodyVelocity(PELVIS));
}

void RBDLModelClass::InverseDynamics()
{
  // dq_all_floating = (q_all_floating - q_all_floating_old) / dt;

  RBDL::InverseDynamics (_rbdl_model,
                         q_all_floating,     // inputs
                         dq_all_floating,  // inputs
                         ddq_all_floating, // inputs
                         tau_all_floating,   // output
                         NULL // optional -- later -- external forces std::vector<Math::SpatialVector> *f_ext =
                        );
  // q_all_floating_old = q_all_floating;
// for (auto it : _map_JointName_to_rbdl)
// {
//  COUT(robotpara.getJointName(it.first), tau_all_floating[it.second]);
// }
}

Eigen::VectorXd RBDLModelClass::InverseDynamicsContacts(RBDL::ConstraintSet & CS)
{
  //Retrieve sizes
  int sizeConstraints = CS.size();

  //Compute full H, G matrix and C, gamma
  //vectors into the constraint set
  RBDL::CalcContactSystemVariables(_rbdl_model, q_all_floating, dq_all_floating, Eigen::VectorXd::Zero(nDoF_floating), CS);

  //Build matrix system
  Eigen::MatrixXd A = Eigen::MatrixXd::Zero(nDoF_floating + sizeConstraints, nDoF_floating + sizeConstraints);
  Eigen::VectorXd b = Eigen::VectorXd::Zero(nDoF_floating + sizeConstraints);
  A.block(0, 0, nDoF_floating, nDoF_floating) = CS.H;
  A.block(nDoF_floating, 0, sizeConstraints, nDoF_floating) = CS.G;
  A.block(0, nDoF_floating, nDoF_floating, sizeConstraints) = CS.G.transpose();
  b.segment(0, nDoF_floating) = ddq_all_floating;
  b.segment(nDoF_floating, sizeConstraints) = -CS.force;

  //Compute and retrieve DOF torques
  Eigen::VectorXd tmpX = A * b;
  // COUT(tmpX.segment(0, nDoF_floating).transpose());
  // COUT(CS.C.transpose());
  return tmpX.segment(0, nDoF_floating) + CS.C;
}

void RBDLModelClass::ForwardDynamicsContacts(RBDL::ConstraintSet & CS)
{
  RBDL::ForwardDynamicsContactsDirect(_rbdl_model,
                                      q_all_floating,     // inputs
                                      dq_all_floating,  // inputs
                                      tau_all_floating,   // inputs
                                      CS,
                                      ddq_all_floating // output
                                     );

  for (int i = 0; i < CS.body.size(); i++) {
    Eigen::Vector3d point_acceleration = RBDL::CalcPointAcceleration (_rbdl_model, q_all_floating, dq_all_floating, ddq_all_floating, CS.body[i], CS.point[i]);
    COUT(CS.name[i], point_acceleration.transpose());
  }
  COUT("");

}

// void RBDLModelClass::vJointNameToRBDL(const std::vector<double>& from, RBDL::Math::VectorNd& to_rbdl)
// {
//   for (auto it : _map_JointName_to_rbdl) {
//     to_rbdl[it.second] = from[it.first];
//   }
// }

// void RBDLModelClass::vRBDLToJointName(const RBDL::Math::VectorNd& from_rbdl, std::vector<double>& to)
// {
//   for (auto it : _map_JointName_to_rbdl) {
//     to[it.first] = from_rbdl[it.second];
//   }
// }

// template <typename T>
// void RBDLModelClass::getRequiredTorques(T& torque_all)
// {
//   InverseDynamics();
//   vRBDLToJointName(tau_all_floating, torque_all, 0, torque_all.size() - 1);
// }

Eigen::MatrixXd RBDLModelClass::getJacobian(const int& body_id, const bool & IsFullJacobian, const bool & IsUpdateKinematics)
{
  RBDL::Math::MatrixNd Jaco = RBDL::Math::MatrixNd::Zero (6, nDoF_floating);
  RBDL::CalcBodySpatialJacobian(_rbdl_model,
                                q_all_floating,
                                getRBDLBodyID(body_id),
                                Jaco, // output
                                IsUpdateKinematics);

  // swap rbdl Jacobian [rotation; translation] to [translation; rotation]
  if (IsFullJacobian) {
    RBDL::Math::MatrixNd temp = RBDL::Math::MatrixNd::Zero (6, nDoF_floating);
    temp.topRows(3) = Jaco.bottomRows(3);
    temp.bottomRows(3) = Jaco.topRows(3);
    return temp;
  }
  else {
    Eigen::Matrix<double, 6, 6> footJaco;
    Eigen::Matrix<double, 6, 7> handJaco;
    if (body_id == RIGHT_FOOT) {
      footJaco.block(0, 0, 3, 6) = Jaco.block(3, getRBDLJointID(RIGHT_HIP_PITCH), 3, 6);
      footJaco.block(3, 0, 3, 6) = Jaco.block(0, getRBDLJointID(RIGHT_HIP_PITCH), 3, 6);
      return footJaco;
    }
    else if (body_id == LEFT_FOOT) {
      footJaco.block(0, 0, 3, 6) = Jaco.block(3, getRBDLJointID(LEFT_HIP_PITCH), 3, 6);
      footJaco.block(3, 0, 3, 6) = Jaco.block(0, getRBDLJointID(LEFT_HIP_PITCH), 3, 6);
      return footJaco;
    }
    else if (body_id == RIGHT_HAND) {
      handJaco.block(0, 0, 3, 7) = Jaco.block(3, getRBDLJointID(RIGHT_SHOULDER_PITCH), 3, 7);
      handJaco.block(3, 0, 3, 7) = Jaco.block(0, getRBDLJointID(RIGHT_SHOULDER_PITCH), 3, 7);
      return handJaco;
    }
    else if (body_id == LEFT_HAND) {
      handJaco.block(0, 0, 3, 7) = Jaco.block(3, getRBDLJointID(LEFT_SHOULDER_PITCH), 3, 7);
      handJaco.block(3, 0, 3, 7) = Jaco.block(0, getRBDLJointID(LEFT_SHOULDER_PITCH), 3, 7);
      return handJaco;
    }
    else {
      std::cerr << "not defined jacobian in rbdl.....\n";
      assert(!body_id);
    }
  }
}

Eigen::MatrixXd RBDLModelClass::CalcFrameJacobian(
  unsigned int tar_body_id,
  RBDL::Math::Matrix3d tar_body_Rotation_tar_frame,
  RBDL::Math::Vector3d tar_body_Translation_tar_frame,
  unsigned int ref_body_id,
  RBDL::Math::Matrix3d ref_body_Rotation_ref_frame,
  RBDL::Math::Vector3d ref_body_Translation_ref_frame,
  bool IsUpdateKinematics)
{
  RBDL::Math::MatrixNd Jaco = RBDL::Math::MatrixNd::Zero (6, nDoF_floating);
  Jaco.setZero();
  RBDL::CalcBodySpatialJacobian( _rbdl_model, q_all_floating, getRBDLBodyID(tar_body_id), Jaco, IsUpdateKinematics );

  RBDL::Math::Matrix3d tar_body_Rotation_world = RBDL::CalcBodyWorldOrientation( _rbdl_model, q_all_floating, getRBDLBodyID(tar_body_id), false );
  RBDL::Math::Matrix3d ref_body_Rotation_world = RBDL::CalcBodyWorldOrientation( _rbdl_model, q_all_floating, (ref_body_id), false );
  RBDL::Math::Matrix3d ref_body_Rotation_tar_body = ref_body_Rotation_world * ( tar_body_Rotation_world.transpose() );
  RBDL::Math::Matrix3d ref_frame_Rotation_ref_body = ref_body_Rotation_ref_frame.transpose();
  RBDL::Math::Matrix3d ref_frame_Rotation_tar_body = ref_frame_Rotation_ref_body * ref_body_Rotation_tar_body;
  RBDL::Math::SpatialTransform spatial_transform( ref_frame_Rotation_tar_body, tar_body_Translation_tar_frame );

  Jaco = spatial_transform.toMatrix() * Jaco;
  RBDL::Math::MatrixNd temp = RBDL::Math::MatrixNd::Zero (6, nDoF_floating);
  temp.topRows(3) = Jaco.bottomRows(3);
  temp.bottomRows(3) = Jaco.topRows(3);
  return temp;
}


RBDL::Math::SpatialTransform RBDLModelClass::CalcFrameTransform(
  unsigned int tar_body_id,
  RBDL::Math::Matrix3d tar_body_Rotation_tar_frame,
  RBDL::Math::Vector3d tar_body_Translation_tar_frame,
  unsigned int ref_body_id,
  RBDL::Math::Matrix3d ref_body_Rotation_ref_frame,
  RBDL::Math::Vector3d ref_body_Translation_ref_frame
)
{
  // traslation
  RBDL::Math::Vector3d base_Translation_tar_frame = RBDL::CalcBodyToBaseCoordinates(_rbdl_model, q_all_floating, tar_body_id, tar_body_Translation_tar_frame, false);
  RBDL::Math::Vector3d base_Translation_ref_frame = RBDL::CalcBodyToBaseCoordinates(_rbdl_model, q_all_floating, ref_body_id, ref_body_Translation_ref_frame, false);
  RBDL::Math::Vector3d ref_frame_Translation_tar_frame = base_Translation_tar_frame - base_Translation_ref_frame;

  // rotation
  RBDL::Math::Matrix3d base_Rotation_tar_body = RBDL::CalcBodyWorldOrientation(_rbdl_model, q_all_floating, tar_body_id, false).transpose();
  RBDL::Math::Matrix3d base_Rotation_ref_body = RBDL::CalcBodyWorldOrientation(_rbdl_model, q_all_floating, ref_body_id, false).transpose();
  RBDL::Math::Matrix3d base_Rotation_tar_frame = base_Rotation_tar_body * tar_body_Rotation_tar_frame;
  RBDL::Math::Matrix3d base_Rotation_ref_frame = base_Rotation_ref_body * ref_body_Rotation_ref_frame;
  RBDL::Math::Matrix3d ref_frame_Rotation_tar_frame = base_Rotation_ref_frame.transpose() * base_Rotation_tar_frame;

  return RBDL::Math::SpatialTransform(ref_frame_Rotation_tar_frame, ref_frame_Translation_tar_frame);
}



RBDL::Math::SpatialTransform RBDLModelClass::getLocalBodyFrame(const int& link_name)
{
  RBDL::Math::SpatialTransform local_body_frame = _rbdl_model.X_base[_map_LinkName_to_rbdl[link_name]] * _rbdl_model.X_base[_map_LinkName_to_rbdl[PELVIS]].inverse();

  // it seems rbdl/X_base rotation matrix E is from world to body, not body to world, therefore need transpose it before use
  return RBDL::Math::SpatialTransform(local_body_frame.E.transpose(), local_body_frame.r);
};

Eigen::Matrix3d RBDLModelClass::getGloballBodyFrame(const int& link_name)
{
  // RBDL::Math::Matrix3d tar_body_Rotation_world = RBDL::CalcBodyWorldOrientation ( _rbdl_model, q_all_floating, getRBDLBodyID(link_name), false);

  // COUT("\ntar_body_Rotation_world:\n", tar_body_Rotation_world);
  // COUT("\nX_base:\n", _rbdl_model.X_base[_map_LinkName_to_rbdl[link_name]].E);  // they are the same

  return  _rbdl_model.X_base[_map_LinkName_to_rbdl[link_name]].E.transpose();
};

Eigen::Vector3d RBDLModelClass::getGloballBodyPosition(const int& link_name)
{
  return  _rbdl_model.X_base[_map_LinkName_to_rbdl[link_name]].r;
};

Eigen::Vector3d RBDLModelClass::getGloballBodyVelocity(const int& link_name)
{
  return RBDL::CalcPointVelocity(_rbdl_model,
                                 q_all_floating,
                                 dq_all_floating,
                                 _map_LinkName_to_rbdl[link_name],
                                 Eigen::Vector3d::Zero(), false);
};

const RBDL::Math::Matrix3d& RBDLModelClass::getOrientationFromWorldToBody(const int& link_name)
{
  return _rbdl_model.X_base[_map_LinkName_to_rbdl[link_name]].E;
};

const RBDL::Math::MatrixNd& RBDLModelClass::InertiaMatrix()
{
  _H.setZero();
  RBDL::CompositeRigidBodyAlgorithm (_rbdl_model, q_all_floating, _H, false);
  return _H;
};

const RBDL::Math::VectorNd& RBDLModelClass::NonlinearEffects()
{
  _C.setZero();
  RBDL::NonlinearEffects (_rbdl_model, q_all_floating, dq_all_floating, _C);
  return _C;
};

Eigen::MatrixXd RBDLModelClass::getJdot(const int& body_id, const bool & IsFullJacobian)
{
  // update to J_q_old first, then update back to original status
  RBDL::Math::VectorNd temp = q_all_floating;
  q_all_floating = q_all_floating_old;
  Eigen::MatrixXd J_q_old = getJacobian(body_id, true, true);
  q_all_floating = temp;
  Eigen::MatrixXd J_q = getJacobian(body_id, true, true);

  // Eigen::MatrixXd Jdot = RBDL::Math::MatrixNd::Zero (6, nDoF_floating);

  // for (int l = 0; l < 6; l++)
  //   for (int c = 0; c < J_q.cols(); c++)
  //     Jdot(l, c) = (J_q(l, c) - J_q_old(l, c)) / dt;

  Eigen::MatrixXd Jdot = (J_q - J_q_old) / dt;

  if (IsFullJacobian) {
    return Jdot;
  }
  else {
    Eigen::Matrix<double, 6, 6> footJaco;
    Eigen::Matrix<double, 6, 7> handJaco;
    if (body_id == RIGHT_FOOT) {
      footJaco = Jdot.block(0, getRBDLJointID(RIGHT_HIP_PITCH), 6, 6);
      return footJaco;
    }
    else if (body_id == LEFT_FOOT) {
      footJaco = Jdot.block(0, getRBDLJointID(LEFT_HIP_PITCH), 6, 6);
      return footJaco;
    }
    else if (body_id == RIGHT_HAND) {
      handJaco = Jdot.block(0, getRBDLJointID(RIGHT_SHOULDER_PITCH), 6, 7);
      return handJaco;
    }
    else if (body_id == LEFT_HAND) {
      handJaco = Jdot.block(0, getRBDLJointID(LEFT_SHOULDER_PITCH), 6, 7);
      return handJaco;
    }
    else {
      std::cerr << "not defined getJdot in rbdl.....\n";
      assert(!body_id);
    }
  }
}

Eigen::VectorXd RBDLModelClass::getJdotQdot(const int& body_id, const bool & IsFullJacobian)
{
  if (IsFullJacobian) {
    return getJdot(body_id, IsFullJacobian) * dq_all_floating;
  }
  else {
    if (body_id == RIGHT_FOOT) {
      return getJdot(body_id, IsFullJacobian) * dq_all_floating.segment(getRBDLJointID(RIGHT_HIP_PITCH), 6);
    }
    else if (body_id == LEFT_FOOT) {
      return getJdot(body_id, IsFullJacobian) * dq_all_floating.segment(getRBDLJointID(LEFT_HIP_PITCH), 6);
    }
    else if (body_id == RIGHT_HAND) {
      return getJdot(body_id, IsFullJacobian) * dq_all_floating.segment(getRBDLJointID(RIGHT_SHOULDER_PITCH), 10);
    }
    else if (body_id == LEFT_HAND) {
      return getJdot(body_id, IsFullJacobian) * dq_all_floating.segment(getRBDLJointID(LEFT_SHOULDER_PITCH), 10);
    }
    else {
      std::cerr << "not defined getJdotQdot in rbdl.....\n";
      assert(!body_id);
    }
  }
}

// Eigen::VectorXd RBDLModelClass::sortVecFromRBDLtoJointName(const Eigen::VectorXd& from)
// {
//   int size = RobotParaClass::JOINT_NUM();
//   assert(from.size() == size);
//   Eigen::VectorXd to = Eigen::VectorXd::Zero(size);

//   for (int joint_name = 0; joint_name < size; ++joint_name) {
//     to[joint_name] = from[getRBDLJointID(joint_name)];
//   }

//   return to;
// };

//=====================================================================

/* T, U, V are 3 by 3b, b is # of bodies
 * P = [T 0; U V]
 * T = [m_0*I, m_1*I, ... ]
 * U = [m_0* cross(r_0), ... ]
 * V = [similarity_transform(I_0) ... ]
 */
Eigen::MatrixXd RBDLModelClass::compute_P()
{
  Eigen::MatrixXd P = Eigen::MatrixXd::Zero(6, 6 * _rbdl_model.mBodies.size());
  double m;
  Eigen::Vector3d r;
  Eigen::Matrix3d I, rot;

  for (int i = 0; i < nLink_floating; i++) {
    m = _rbdl_model.mBodies[i].mMass;
    P.block(0, 3 * i, 3, 3) = m * Eigen::Matrix3d::Identity();

    r = _rbdl_model.mBodies[i].mCenterOfMass - com_floating;
    P.block(3, 3 * i, 3, 3) = m * SkewSymmetric(r);

    I = _rbdl_model.mBodies[i].mInertia;
    rot = _rbdl_model.X_base[i].E.transpose();

    P.block(3, 3 * i + 3 * nLink_floating, 3, 3) = rot * I * rot.transpose();
  }
  return P;
}

/*
 * J is 6b by u,
 * top half block are all positions, bottom half are all rotation
 * each block is the body's com jacobian
 */
Eigen::MatrixXd RBDLModelClass::compute_J()
{
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6 * _rbdl_model.mBodies.size(), _rbdl_model.dof_count);

  RBDL::Math::MatrixNd Jaco = RBDL::Math::MatrixNd::Zero (6, nDoF_floating);
  for (int i = 0; i < nLink_floating; i++) {

    RBDL::CalcBodySpatialJacobian(_rbdl_model,
                                  q_all_floating,
                                  i,
                                  Jaco, // output
                                  false);

    J.block(3 * i, 0, 3, nDoF_floating) = Jaco.bottomRows(3);
    J.block(3 * nLink_floating + 3 * i, 0, 3, nDoF_floating) = Jaco.topRows(3);
  }

  return J;
}

Eigen::MatrixXd RBDLModelClass::centroidal_momentum_matrix()
{
  return compute_P() * compute_J();
}

Eigen::MatrixXd RBDLModelClass::CalcCMM()
{
  RBDL::Math::MatrixNd cmm = RBDL::Math::MatrixNd::Zero(6, nDoF_floating);
  RBDL::Utils::CalcCentroidalProperty(_rbdl_model, q_all_floating, dq_all_floating, mass, com_floating, &com_velocity, &angular_momentum, &cmm, false);

  RBDL::Math::MatrixNd temp = RBDL::Math::MatrixNd::Zero (6, nDoF_floating);
  temp.topRows(3) = cmm.bottomRows(3);
  temp.bottomRows(3) = cmm.topRows(3);
  return temp;
}